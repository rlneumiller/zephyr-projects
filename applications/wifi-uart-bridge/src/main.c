 // WiFi to UART Bridge
 // Bridges WiFi and UART interfaces for remote access to 
 // UART-connected 3D printers and other UART devices over WiFi.
 //////////////////////////////////////////////////////////////////////////////
 
 // This application connects to a WiFi network, starts a TCP server, and 
 // forwards data between the TCP connection and a UART interface. It also 
 // hosts a simple web server for status monitoring.
 // The application uses Zephyr's WiFi and networking APIs, UART driver, 
 // and ring buffers for efficient data handling.
 // Note: This is a simplified example for demonstration purposes and may need 
 // enhancements for production use, such as error handling, security, and 
 // support for multiple clients.
 //////////////////////////////////////////////////////////////////////////////

#include <zephyr/sys/atomic.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

LOG_MODULE_REGISTER(wifi_uart_bridge, LOG_LEVEL_INF);

// Configuration
#define WIFI_SSID "2.4-voda"
#define WIFI_PRE_SHARED_KEY  "5E7HAwbB4K" // If you live nearby you're welcome to use our WiFi, but please be mindful of bandwidth usage as it's a residential connection!
#define BRIDGE_PORT 8080
#define RING_BUF_SIZE 1024
#define TCP_SERVER_STACK_SIZE 4096
#define TCP_SERVER_PRIORITY 7
//#define CMD_RESPONSE_BUF_SIZE 2048
#define CMD_RECV_TIMEOUT_MS 5000

// Globals
RING_BUF_DECLARE(uart_rx_ringbuf, RING_BUF_SIZE);
RING_BUF_DECLARE(tcp_rx_ringbuf, RING_BUF_SIZE);
//RING_BUF_DECLARE(cmd_response_ringbuf, CMD_RESPONSE_BUF_SIZE);

static void log_printer_data(uint8_t *buf, size_t len)
{
	static char line_buf[128];
	static size_t line_pos = 0;

	for (size_t i = 0; i < len; i++) {
		if (buf[i] == '\r' || buf[i] == '\n') {
			if (line_pos > 0) {
				line_buf[line_pos] = '\0';
				// Look for temperature reports like "T:21.0 /0.0 B:21.0 /0.0"
				if (strstr(line_buf, "T:") && strstr(line_buf, "B:")) {
					LOG_INF("Printer says: %s", line_buf);
				} else {
					LOG_INF("Printer sent to Octoprint: %s", line_buf);
				}
				line_pos = 0;
			}
		} else if (line_pos < sizeof(line_buf) - 1) {
			line_buf[line_pos++] = buf[i];
		}
	}
}

// Use the dedicated hardware UART (uart0 via bridge-uart alias) for the bridge,
// leaving usb_serial for the system console and logs.
#define UART_DEVICE_NODE DT_ALIAS(bridge_uart)

const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
int client_socket = -1;
uint32_t last_printer_activity_ms = 0;

//bool command_response_pending = false; // accessed by web server when sending M115

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;
static K_SEM_DEFINE(wifi_connected, 0, 1);
static K_SEM_DEFINE(ipv4_address_obtained, 0, 1);

// TCP server thread resources
K_THREAD_STACK_DEFINE(tcp_server_stack, TCP_SERVER_STACK_SIZE);
static struct k_thread tcp_server_thread_data;

// keep last IPv4 string so we can print it after a reset/boot
char device_ip[NET_IPV4_ADDR_LEN];

bool wifi_is_connected;
static int wifi_connect_result = -ETIMEDOUT;

/* Watchdog: periodically check WiFi connectivity and reconnect if needed */
#define WIFI_WATCHDOG_INTERVAL_MS 30000

/* TCP connectivity check (to detect “captive portal / no upstream” cases). */
#define WIFI_WATCHDOG_TCP_CHECK_ADDR "1.1.1.1"
#define WIFI_WATCHDOG_TCP_CHECK_PORT 53
#define WIFI_WATCHDOG_TCP_CHECK_TIMEOUT_MS 1500

/* RSSI check: force reconnect if signal is too weak (or missing). */
#define WIFI_WATCHDOG_MIN_RSSI_DBM (-75)

static atomic_t wifi_reconnect_in_progress = ATOMIC_INIT(0);
static struct k_work_delayable wifi_watchdog_work;

static const char *wifi_rssi_quality(int rssi)
{
	if (rssi >= -50) {
		return "excellent";
	} else if (rssi >= -60) {
		return "very good";
	} else if (rssi >= -67) {
		return "good";
	} else if (rssi >= -70) {
		return "fair";
	} else if (rssi >= -80) {
		return "weak";
	} else {
		return "poor";
	}
}

static bool wifi_check_rssi(struct net_if *iface)
{
	struct wifi_iface_status status = {0};
	int ret;

	if (!iface) {
		return false;
	}

	ret = net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status, sizeof(status));
	if (ret != 0) {
		LOG_DBG("WiFi watchdog: could not get iface status (%d)", ret);
		return false;
	}

	/* If we are not associated, the RSSI value is not meaningful. */
	if (status.state < WIFI_STATE_ASSOCIATED) {
		LOG_DBG("WiFi watchdog: iface state %s (not associated)", wifi_state_txt(status.state));
		return false;
	}

	LOG_DBG("WiFi watchdog: RSSI %d dBm (%s)", status.rssi, wifi_rssi_quality(status.rssi));
	return (status.rssi > WIFI_WATCHDOG_MIN_RSSI_DBM);
}

static const char *wifi_disconnect_reason_str(int reason)
{
	/* These are common IEEE 802.11 disconnection reason codes. */
	switch (reason) {
	case 0:
		return "UNSPECIFIED";
	case 1:
		return "UNSPECIFIED";
	case 2:
		return "PREV_AUTH_NOT_VALID";
	case 3:
		return "DEAUTH_LEAVING";
	case 4:
		return "DISASSOC_DUE_TO_INACTIVITY";
	case 5:
		return "DISASSOC_AP_BUSY";
	case 6:
		return "CLASS2_FRAME_FROM_NONAUTH_STA";
	case 7:
		return "CLASS3_FRAME_FROM_NONASSOC_STA";
	case 8:
		return "DISASSOC_STA_HAS_LEFT";
	case 9:
		return "STA_REQ_ASSOC_WITHOUT_AUTH";
	case 10:
		return "UNSPECIFIED_QOS_REASON";
	case 11:
		return "NOT_AUTHED";
	case 12:
		return "NOT_ASSOCED";
	case 13:
		return "ASSOC_DENIED_REASON_UNSPEC";
	case 14:
		return "ASSOC_DENIED_NOT_SUPPORTED";
	case 15:
		return "ASSOC_DENIED_AP_BUSY";
	case 16:
		return "ASSOC_DENIED_SHORT_SLOT_TIME";
	case 17:
		return "ASSOC_DENIED_DSSS_PARAM";
	case 18:
		return "ASSOC_DENIED_NO_SHORT_PREAMBLE";
	case 19:
		return "ASSOC_DENIED_NO_HT";
	case 23:
		return "DISASSOC_STA_INITIATED";
	default:
		return "UNKNOWN_REASON";
	}
}

// Internal temperature sensor device
static const struct device *temp_sensor = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(coretemp));

static bool wifi_check_tcp_connectivity(void)
{
	struct sockaddr_in remote = {0};
	int sock;
	struct timeval tv = {
		.tv_sec = WIFI_WATCHDOG_TCP_CHECK_TIMEOUT_MS / 1000,
		.tv_usec = (WIFI_WATCHDOG_TCP_CHECK_TIMEOUT_MS % 1000) * 1000,
	};

	if (net_addr_pton(AF_INET, WIFI_WATCHDOG_TCP_CHECK_ADDR, &remote.sin_addr) != 0) {
		return false;
	}
	remote.sin_family = AF_INET;
	remote.sin_port = htons(WIFI_WATCHDOG_TCP_CHECK_PORT);

	sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock < 0) {
		return false;
	}

	zsock_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	zsock_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

	int ret = zsock_connect(sock, (struct sockaddr *)&remote, sizeof(remote));
	zsock_close(sock);
	return (ret == 0);
}

static void log_wifi_iface_state(struct net_if *iface)
{
	struct wifi_iface_status status = {0};

	if (!iface) {
		return;
	}

	if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status, sizeof(status)) == 0) {
		LOG_INF("WiFi state: %s", wifi_state_txt(status.state));
	}
}

static void read_and_log_chip_temperature(void)
{
	if (temp_sensor == NULL || !device_is_ready(temp_sensor)) {
		LOG_DBG("Temperature sensor not available");
		return;
	}

	struct sensor_value temperature;
	int ret = sensor_sample_fetch(temp_sensor);
	if (ret != 0) {
		LOG_DBG("Failed to fetch temperature: %d", ret);
		return;
	}

	ret = sensor_channel_get(temp_sensor, SENSOR_CHAN_DIE_TEMP, &temperature);
	if (ret == 0) {
		int temp_int = temperature.val1;
		int temp_frac = temperature.val2 / 300000; // Convert to 2 decimal places
		LOG_INF("ESP32-C3 SOC internal Temp: %d.%02d°C", temp_int, temp_frac);

	} else {
		LOG_DBG("Failed to read temperature: %d", ret);
	}
}

// UART ISR 
static void uart_isr(const struct device *dev, void *user_data)
{
	if (!uart_irq_update(dev)) {
		return;
	}

	if (uart_irq_rx_ready(dev)) {
		uint8_t buffer[64];
		int len = uart_fifo_read(dev, buffer, sizeof(buffer));
		if (len > 0) {
			last_printer_activity_ms = k_uptime_get_32();
			ring_buf_put(&uart_rx_ringbuf, buffer, len);
			// if (command_response_pending) {
			// 	ring_buf_put(&cmd_response_ringbuf, buffer, len);
			// }
		}
	}

	if (uart_irq_tx_ready(dev)) {
		uint8_t tx_data[64];
		uint32_t size = ring_buf_get(&tcp_rx_ringbuf, tx_data, sizeof(tx_data));
		if (size > 0) {
			int written = uart_fifo_fill(dev, tx_data, size);
			if (written < size) {
				LOG_WRN("UART TX buffer full, dropping %d bytes", size - written);
				// Generally the commands we transmit will be quite small, 
				// so we'll skip implementing retries for now to keep it simple.
			}
		} else {
			uart_irq_tx_disable(dev);
		}
	}
}

// WiFi Management
static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status;

	if (cb->info == NULL || cb->info_length < sizeof(struct wifi_status)) {
		LOG_WRN("WiFi connect event without status info");
		wifi_connect_result = -EINVAL;
		k_sem_give(&wifi_connected);
		return;
	}

	status = (const struct wifi_status *)cb->info;
	if (status->status) {
		LOG_ERR("Connection request failed (%d)", status->status);
		wifi_is_connected = false;
		wifi_connect_result = status->status;
	} else {
		LOG_INF("Connected to WiFi");
		wifi_is_connected = true;
		wifi_connect_result = 0;
	}

	k_sem_give(&wifi_connected);
}

static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status;

	wifi_is_connected = false;
	device_ip[0] = '\0';

	if (cb->info == NULL || cb->info_length < sizeof(struct wifi_status)) {
		LOG_WRN("WiFi disconnect event");
		return;
	}

	status = (const struct wifi_status *)cb->info;
	if (status->status) {
		LOG_WRN("WiFi disconnected (reason %d = %s)", status->status,
			 wifi_disconnect_reason_str(status->status));
	} else {
		LOG_WRN("WiFi disconnected (reason 0: %s)",
			 wifi_disconnect_reason_str(status->status));
	}

	/* Trigger a reconnect ASAP via the watchdog. */
	k_work_schedule(&wifi_watchdog_work, K_NO_WAIT);
}

static void handle_ipv4_result(struct net_if *iface)
{
	int i;
	for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		if (iface->config.ip.ipv4->unicast[i].ipv4.addr_type == NET_ADDR_DHCP) {
			char buf[NET_IPV4_ADDR_LEN];
			net_addr_ntop(AF_INET,
					  &iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr,
					  buf, sizeof(buf));
			// Remember it for later and print now
			strncpy(device_ip, buf, sizeof(device_ip) - 1);
			device_ip[sizeof(device_ip) - 1] = '\0';
			LOG_INF("handle_ipv4_result: Our device IPv4 address: %s", buf);
			k_sem_give(&ipv4_address_obtained);
			break;
		}
	}
}

/* Forward declarations */
static void wifi_connect(void);

static void wifi_watchdog_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	LOG_INF("WiFi watchdog tick");

	struct net_if *iface = net_if_get_default();
	bool need_reconnect = false;

	/* Log the current RSSI and association state for visibility. */
	if (iface && net_if_is_up(iface)) {
		struct wifi_iface_status status = {0};
		if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status, sizeof(status)) == 0) {
			LOG_INF("WiFi status: %s, RSSI %d dBm (%s)", wifi_state_txt(status.state), status.rssi,
				wifi_rssi_quality(status.rssi));
		}
	}

	/* If we're not connected or we don't have an IP, attempt a reconnect. */
	if (!wifi_is_connected || device_ip[0] == '\0') {
		need_reconnect = true;
	} else if (iface && !net_if_is_up(iface)) {
		need_reconnect = true;
	} else if (!wifi_check_tcp_connectivity()) {
		LOG_WRN("WiFi watchdog: TCP test failed, network may be broken");
		need_reconnect = true;
	} else if (!wifi_check_rssi(iface)) {
		LOG_WRN("WiFi watchdog: RSSI too low or unknown, restarting radio");
		need_reconnect = true;
	}

	if (need_reconnect) {
		if (atomic_cas(&wifi_reconnect_in_progress, 0, 1) == 0) {
			LOG_WRN("WiFi watchdog: network down, attempting reconnect");
			if (iface) {
				net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
			}
			wifi_connect();
			atomic_set(&wifi_reconnect_in_progress, 0);
		} else {
			LOG_DBG("WiFi watchdog: reconnect already in progress");
		}
	}

	k_work_schedule(&wifi_watchdog_work, K_MSEC(WIFI_WATCHDOG_INTERVAL_MS));
}

static void net_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				   uint64_t mgmt_event, struct net_if *iface)
{
	char evbuf[64];
	snprintk(evbuf, sizeof(evbuf), "event 0x%08llx\r\n", mgmt_event);
	LOG_INF("net event %s", evbuf);

	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT:
		handle_wifi_connect_result(cb);
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		handle_wifi_disconnect_result(cb);
		break;
	case NET_EVENT_IPV4_ADDR_ADD:
		handle_ipv4_result(iface);
		break;
	default:
		// nothing else handled
		break;
	}
}

static void wifi_connect(void)
{
	struct net_if *iface = net_if_get_default();
	struct wifi_connect_req_params wifi_params = {0};
	int ret;
	int attempts;
	bool connected = false;

	if (iface == NULL) {
		LOG_ERR("No default network interface available");
		return;
	}

	wifi_params.ssid = WIFI_SSID;
	wifi_params.ssid_length = strlen(WIFI_SSID);
	wifi_params.psk = WIFI_PRE_SHARED_KEY;
	wifi_params.psk_length = strlen(WIFI_PRE_SHARED_KEY);
	wifi_params.security = WIFI_SECURITY_TYPE_PSK;
	wifi_params.band = WIFI_FREQ_BAND_UNKNOWN;
	wifi_params.channel = WIFI_CHANNEL_ANY;
	wifi_params.mfp = WIFI_MFP_OPTIONAL;

	wifi_is_connected = false;
	wifi_connect_result = -ETIMEDOUT;
	while (k_sem_take(&wifi_connected, K_NO_WAIT) == 0) {
		// drain stale signal
	}
	while (k_sem_take(&ipv4_address_obtained, K_NO_WAIT) == 0) {
		// drain stale signal
	}

	LOG_INF("Connecting to WiFi: %s", WIFI_SSID);
	for (attempts = 0; attempts < 10; attempts++) {
		ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &wifi_params, sizeof(wifi_params));
		if (ret == 0) {
			break;
		}

		LOG_WRN("WiFi connect request failed (%d), retrying...", ret);
		k_sleep(K_MSEC(500));
	}

	if (attempts == 10) {
		LOG_ERR("Could not submit WiFi connect request");
		return;
	}

	LOG_INF("WiFi connect monitor: logging interface state every 2s until connected");
	for (attempts = 0; attempts < 15; attempts++) {
		if (k_sem_take(&wifi_connected, K_SECONDS(2)) == 0) {
			if (wifi_connect_result == 0 || wifi_is_connected) {
				connected = true;
				break;
			}
			LOG_WRN("WiFi connect result: %d", wifi_connect_result);
		} else {
			log_wifi_iface_state(iface);
		}
	}

	if (!connected) {
		LOG_ERR("Timeout waiting for WiFi connection result");
		return;
	}

	// Disable Wi-Fi power save mode to prevent device from sleeping during inactivity
	struct wifi_ps_params ps_params = {
		.enabled = WIFI_PS_DISABLED,
		.type = WIFI_PS_PARAM_STATE
	};

	if (net_mgmt(NET_REQUEST_WIFI_PS, iface, &ps_params, sizeof(ps_params))) {
		LOG_ERR("Failed to disable WiFi Power Save mode");
	} else {
		LOG_INF("WiFi Power Save mode disabled");
	}

	LOG_INF("We may be waiting here for a bit if dhcp didn't provide our IP address already...");
	for (attempts = 0; attempts < 30; attempts++) {
		if (k_sem_take(&ipv4_address_obtained, K_SECONDS(1)) == 0) {
			return;
		}
		LOG_INF("Still waiting for IPv4 address...");
	}

	if (device_ip[0] == '\0') {
		LOG_WRN("Timeout waiting for IPv4 address");
		return;
	}
}

// TCP Server
static void tcp_server_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	int serv_sock;
	struct sockaddr_in bind_addr = {
		.sin_family = AF_INET,
		.sin_port = htons(BRIDGE_PORT),
		.sin_addr = {.s_addr = htonl(INADDR_ANY)},
	};

	serv_sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (serv_sock < 0) {
		LOG_ERR("Failed to create socket: %d", errno);
		return;
	}

	if (zsock_bind(serv_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
		LOG_ERR("Failed to bind socket: %d", errno);
		return;
	}

	if (zsock_listen(serv_sock, 1) < 0) {
		LOG_ERR("Failed to listen on socket: %d", errno);
		return;
	}

	LOG_INF("TCP (Bridge) server listening on port %d", BRIDGE_PORT);

	while (1) {
		struct sockaddr_in client_addr;
		socklen_t client_addr_len = sizeof(client_addr);
		
		LOG_INF("Waiting for client connection...");
		client_socket = zsock_accept(serv_sock, (struct sockaddr *)&client_addr, &client_addr_len);
		if (client_socket < 0) {
			LOG_ERR("Failed to accept connection: %d", errno);
			continue;
		}

		LOG_INF("Bridge Client (Octoprint?)connected on port %d", BRIDGE_PORT);

		// Clear any stale data from previous sessions or before connection
		ring_buf_reset(&uart_rx_ringbuf);
		ring_buf_reset(&tcp_rx_ringbuf);

		while (1) {
			uint8_t rx_buf[128];
			
			// 1. Read from TCP, write to UART ringbuf
			int len = zsock_recv(client_socket, rx_buf, sizeof(rx_buf), ZSOCK_MSG_DONTWAIT);
			if (len > 0) {
				LOG_INF("TCP (Octoprint?) -> UART (Printer): %d bytes: %.*s", len, len, rx_buf);
				ring_buf_put(&tcp_rx_ringbuf, rx_buf, len);
				uart_irq_tx_enable(uart_dev);
			} else if (len == 0 || (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
				LOG_INF("Bridge Client (Octoprint?) disconnected from port %d", BRIDGE_PORT);
				zsock_close(client_socket);
				client_socket = -1;
				break;
			}

			// 2. Read from UART ringbuf, write to TCP
			len = ring_buf_get(&uart_rx_ringbuf, rx_buf, sizeof(rx_buf));
			if (len > 0) {
				log_printer_data(rx_buf, len);
				int sent = 0;
				while (sent < len) {
					int ret = zsock_send(client_socket, rx_buf + sent, len - sent, 0);
					if (ret < 0) {
						break;
					}
					sent += ret;
				}
			}

			k_sleep(K_MSEC(10)); // Prevent tight loop
		}
	}
}

int main(void)
{
	LOG_INF("WiFi to UART Bridge Application Started");

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device not found!");
		return 0;
	}
	else {
		LOG_INF("UART device %s is ready", uart_dev->name);
	}


	// Settle the pins and flush the buffer to avoid power-on glitches
	k_sleep(K_MSEC(100));
	uint8_t dummy;
	while (uart_fifo_read(uart_dev, &dummy, 1) > 0);

	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);

	/* Initialize the watchdog work before registering callbacks, so the disconnect handler
	 * can safely reschedule it if needed.
	 */
	k_work_init_delayable(&wifi_watchdog_work, wifi_watchdog_handler);

	/* Register for WiFi and IPv4 events once (do not re-register on reconnect). */
	net_mgmt_init_event_callback(&wifi_cb, net_mgmt_event_handler,
				     NET_EVENT_WIFI_CONNECT_RESULT |
				     NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_cb);

	net_mgmt_init_event_callback(&ipv4_cb, net_mgmt_event_handler,
				     NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&ipv4_cb);

	/* Ensure only one connect/reconnect is running at a time. */
	atomic_set(&wifi_reconnect_in_progress, 1);
	wifi_connect();
	atomic_set(&wifi_reconnect_in_progress, 0);

	/* Start watchdog that will reset the network stack if the WiFi drops. */
	k_work_schedule(&wifi_watchdog_work, K_MSEC(WIFI_WATCHDOG_INTERVAL_MS));

	 // after a reboot we may want the IP address up front along with the boot
	 // messages; wifi_connect already logs the address when it's obtained,
	 // but print it again here so a viewer of the serial output sees it right
	 // after the reset sequence.  the callback saved the value in device_ip. 
	if (device_ip[0] != '\0') {
		LOG_INF("Our device IPv4: %s", device_ip);
	}
	else {
		LOG_INF("We failed to obtain an IPv4 address!");
	}

	k_thread_create(&tcp_server_thread_data,
				tcp_server_stack,
				K_THREAD_STACK_SIZEOF(tcp_server_stack),
				tcp_server_thread,
				NULL, NULL, NULL,
				TCP_SERVER_PRIORITY,
				0,
				K_NO_WAIT);
	k_thread_name_set(&tcp_server_thread_data, "tcp_bridge_server");

	// Simple idle loop with periodic temperature logging
	uint32_t last_temp_log = k_uptime_get_32();
	while (1) {
		uint32_t now = k_uptime_get_32();
		if (now - last_temp_log >= 10000) { // Log every 10 seconds
			read_and_log_chip_temperature();
			last_temp_log = now;
		}
		k_sleep(K_SECONDS(10));
	}

	return 0;
}
