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
#define CMD_RESPONSE_BUF_SIZE 2048
#define CMD_RECV_TIMEOUT_MS 5000

// Globals
RING_BUF_DECLARE(uart_rx_ringbuf, RING_BUF_SIZE);
RING_BUF_DECLARE(tcp_rx_ringbuf, RING_BUF_SIZE);
RING_BUF_DECLARE(cmd_response_ringbuf, CMD_RESPONSE_BUF_SIZE);

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

bool command_response_pending = false; // accessed by web server when sending M115

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

// Internal temperature sensor device
static const struct device *temp_sensor = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(coretemp));

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

static void post_temperature_to_web_server(int temp_int, int temp_frac)
{
	// POST temperature data to web server at localhost:80
	int sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock < 0) {
		LOG_DBG("Failed to create socket for temperature POST: %d", errno);
		return;
	}

	struct sockaddr_in server_addr = {
		.sin_family = AF_INET,
		.sin_port = htons(80),
		.sin_addr.s_addr = htonl(0x7f000001),  // 127.0.0.1 in network byte order
	};

	if (zsock_connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
		LOG_DBG("Failed to connect to web server: %d", errno);
		zsock_close(sock);
		return;
	}

	char post_body[64];
	int body_len = snprintk(post_body, sizeof(post_body), "tempInt=%d&tempFrac=%d", temp_int, temp_frac);

	char post_request[256];
	int req_len = snprintk(post_request, sizeof(post_request),
		"POST /temperature HTTP/1.1\r\n"
		"Host: localhost\r\n"
		"Content-Type: application/x-www-form-urlencoded\r\n"
		"Content-Length: %d\r\n"
		"Connection: close\r\n"
		"\r\n"
		"%s",
		body_len, post_body);

	// Send the POST request
	if (zsock_send(sock, post_request, req_len, 0) >= 0) {
		LOG_INF("Temperature posted to web server: %d.%02d°C", temp_int, temp_frac);
	} else {
		LOG_WRN("Failed to send POST request: %d", errno);
	}

	zsock_close(sock);
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
		int temp_frac = temperature.val2 / 100000; // Convert to 2 decimal places
		LOG_INF("ESP32-C3 Internal Temp: %d.%02d°C", temp_int, temp_frac);
		// POST the temperature data to web server
		post_temperature_to_web_server(temp_int, temp_frac);
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
			if (command_response_pending) {
				ring_buf_put(&cmd_response_ringbuf, buffer, len);
			}
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

	if (cb->info == NULL || cb->info_length < sizeof(struct wifi_status)) {
		LOG_WRN("WiFi disconnect event");
		return;
	}

	status = (const struct wifi_status *)cb->info;
	if (status->status) {
		LOG_WRN("WiFi disconnected (reason %d)", status->status);
	} else {
		LOG_WRN("WiFi disconnected");
	}
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

	net_mgmt_init_event_callback(&wifi_cb, net_mgmt_event_handler,
				     NET_EVENT_WIFI_CONNECT_RESULT |
				     NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_cb);

	net_mgmt_init_event_callback(&ipv4_cb, net_mgmt_event_handler,
				     NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&ipv4_cb);

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

	wifi_connect();

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
