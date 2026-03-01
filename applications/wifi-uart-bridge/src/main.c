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
#define WEB_SERVER_PORT 80
#define RING_BUF_SIZE 1024
#define WEB_SERVER_STACK_SIZE 4096
#define WEB_SERVER_PRIORITY 7
#define WEB_SERVER_REQ_BUF_SIZE 192
#define WEB_SERVER_BODY_BUF_SIZE 512
#define WEB_SERVER_HTML_BUF_SIZE 1024
#define WEB_SERVER_RESP_BUF_SIZE 1400

// Globals
RING_BUF_DECLARE(uart_rx_ringbuf, RING_BUF_SIZE);
RING_BUF_DECLARE(tcp_rx_ringbuf, RING_BUF_SIZE);

const struct device *uart_dev = DEVICE_DT_GET(DT_ALIAS(bridge_uart));
static int client_socket = -1;

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;
static K_SEM_DEFINE(wifi_connected, 0, 1);
static K_SEM_DEFINE(ipv4_address_obtained, 0, 1);
K_THREAD_STACK_DEFINE(web_server_stack, WEB_SERVER_STACK_SIZE);
static struct k_thread web_server_thread_data;

// Avoid requiring stack allocations in the web server thread for handling requests by using global buffers.
static char web_server_req_buf[WEB_SERVER_REQ_BUF_SIZE];
static char web_server_body_buf[WEB_SERVER_BODY_BUF_SIZE];
static char web_server_html_buf[WEB_SERVER_HTML_BUF_SIZE];
static char web_server_resp_buf[WEB_SERVER_RESP_BUF_SIZE];

// keep last IPv4 string so we can print it after a reset/boot
static char device_ip[NET_IPV4_ADDR_LEN];

static bool wifi_is_connected;
static int wifi_connect_result = -ETIMEDOUT;

static int send_all(int sock, const char *buf, size_t len)
{
	size_t sent = 0;

	while (sent < len) {
		int ret = zsock_send(sock, buf + sent, len - sent, 0);
		if (ret <= 0) {
			return -1;
		}
		sent += ret;
	}

	return 0;
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
			ring_buf_put(&uart_rx_ringbuf, buffer, len);
		}
	}

	if (uart_irq_tx_ready(dev)) {
		uint8_t tx_data[64];
		uint32_t size = ring_buf_get(&tcp_rx_ringbuf, tx_data, sizeof(tx_data));
		if (size > 0) {
			int written = uart_fifo_fill(dev, tx_data, size);
			if (written < size) {
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
			/* remember it for later and print now */
			strncpy(device_ip, buf, sizeof(device_ip) - 1);
			device_ip[sizeof(device_ip) - 1] = '\0';
			//LOG_INF("handle_ipv4_result: Our device IPv4 address: %s", buf);
			k_sem_give(&ipv4_address_obtained);
			break;
		}
	}
}

// Web Status Server
static void web_server_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	int serv_sock;
	struct sockaddr_in bind_addr = {
		.sin_family = AF_INET,
		.sin_port = htons(WEB_SERVER_PORT),
		.sin_addr = {.s_addr = htonl(INADDR_ANY)},
	};

	serv_sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (serv_sock < 0) {
		LOG_ERR("Web server socket create failed: %d", errno);
		return;
	}

	if (zsock_bind(serv_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
		LOG_ERR("Web server bind failed: %d", errno);
		zsock_close(serv_sock);
		return;
	}

	if (zsock_listen(serv_sock, 2) < 0) {
		LOG_ERR("Web server listen failed: %d", errno);
		zsock_close(serv_sock);
		return;
	}

	LOG_INF("Web status server listening on port %d", WEB_SERVER_PORT);

	while (1) {
		int conn_sock;
		struct sockaddr_in client_addr;
		socklen_t client_addr_len = sizeof(client_addr);
		bool is_status_path = false;
		bool is_root_path = false;
		int req_len;
		int body_len;
		int html_len;
		int resp_len;
		conn_sock = zsock_accept(serv_sock, (struct sockaddr *)&client_addr, &client_addr_len);
		if (conn_sock < 0) {
			LOG_ERR("Web server accept failed: %d", errno);
			continue;
		}

		req_len = zsock_recv(conn_sock, web_server_req_buf, WEB_SERVER_REQ_BUF_SIZE - 1, 0);
		if (req_len <= 0) {
			zsock_close(conn_sock);
			continue;
		}
		web_server_req_buf[req_len] = '\0';

		if (strncmp(web_server_req_buf, "GET /status", 11) == 0) {
			is_status_path = true;
		} else if (strncmp(web_server_req_buf, "GET / ", 6) == 0) {
			is_root_path = true;
		}

		if (is_status_path) {
			snprintk(web_server_body_buf, sizeof(web_server_body_buf),
				"{\"device_ip\":\"%s\",\"wifi_connected\":%s,\"bridge_client_connected\":%s,\"uptime_ms\":%lld}\n",
				device_ip[0] ? device_ip : "0.0.0.0",
				wifi_is_connected ? "true" : "false",
				client_socket >= 0 ? "true" : "false",
				(long long)k_uptime_get());
			body_len = strlen(web_server_body_buf);

			snprintk(web_server_resp_buf, sizeof(web_server_resp_buf),
				"HTTP/1.1 200 OK\r\n"
				"Content-Type: application/json\r\n"
				"Connection: close\r\n"
				"Content-Length: %d\r\n\r\n"
				"%s",
				body_len, web_server_body_buf);
			resp_len = strlen(web_server_resp_buf);
		} else if (is_root_path) {
			snprintk(web_server_html_buf, sizeof(web_server_html_buf),
				"<!doctype html><html><head><meta charset=\"utf-8\"><title>WiFi UART Bridge Status</title></head>"
				"<body><h1>WiFi UART Bridge</h1>"
				"<p><strong>Device IP:</strong> %s</p>"
				"<p><strong>WiFi Connected:</strong> %s</p>"
				"<p><strong>Bridge Client Connected:</strong> %s</p>"
				"<p><strong>Uptime (ms):</strong> %lld</p>"
				"<p>JSON endpoint: <a href=\"/status\">/status</a></p>"
				"</body></html>",
				device_ip[0] ? device_ip : "0.0.0.0",
				wifi_is_connected ? "true" : "false",
				client_socket >= 0 ? "true" : "false",
				(long long)k_uptime_get());
			html_len = strlen(web_server_html_buf);

			snprintk(web_server_resp_buf, sizeof(web_server_resp_buf),
				"HTTP/1.1 200 OK\r\n"
				"Content-Type: text/html; charset=utf-8\r\n"
				"Connection: close\r\n"
				"Content-Length: %d\r\n\r\n"
				"%s",
				html_len, web_server_html_buf);
			resp_len = strlen(web_server_resp_buf);
		} else {
			snprintk(web_server_resp_buf, sizeof(web_server_resp_buf),
				"HTTP/1.1 404 Not Found\r\n"
				"Content-Type: text/plain\r\n"
				"Connection: close\r\n"
				"Content-Length: 10\r\n\r\n"
				"Not Found\n");
			resp_len = strlen(web_server_resp_buf);
		}

		if (resp_len > 0) {
			send_all(conn_sock, web_server_resp_buf, (size_t)resp_len);
		}

		zsock_close(conn_sock);
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
static void tcp_server_thread(void)
{
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

	LOG_INF("TCP server listening on port %d", BRIDGE_PORT);

	while (1) {
		struct sockaddr_in client_addr;
		socklen_t client_addr_len = sizeof(client_addr);
		
		LOG_INF("Waiting for client connection...");
		client_socket = zsock_accept(serv_sock, (struct sockaddr *)&client_addr, &client_addr_len);
		if (client_socket < 0) {
			LOG_ERR("Failed to accept connection: %d", errno);
			continue;
		}

		LOG_INF("Bridge Client connected on port %d", BRIDGE_PORT);

		while (1) {
			uint8_t rx_buf[128];
			
			// 1. Read from TCP, write to UART ringbuf
			int len = zsock_recv(client_socket, rx_buf, sizeof(rx_buf), ZSOCK_MSG_DONTWAIT);
			if (len > 0) {
				LOG_INF("TCP -> UART: %d bytes", len);
				ring_buf_put(&tcp_rx_ringbuf, rx_buf, len);
				uart_irq_tx_enable(uart_dev);
			} else if (len == 0 || (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
				LOG_INF("Bridge Client disconnected from port %d", BRIDGE_PORT);
				zsock_close(client_socket);
				client_socket = -1;
				break;
			}

			// 2. Read from UART ringbuf, write to TCP
			len = ring_buf_get(&uart_rx_ringbuf, rx_buf, sizeof(rx_buf));
			if (len > 0) {
				LOG_INF("UART -> TCP: %d bytes", len);
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

 	k_thread_create(&web_server_thread_data,
				web_server_stack,
				K_THREAD_STACK_SIZEOF(web_server_stack),
				web_server_thread,
				NULL, NULL, NULL,
				WEB_SERVER_PRIORITY,
				0,
				K_NO_WAIT);
	k_thread_name_set(&web_server_thread_data, "web_status_server");

	tcp_server_thread();

	return 0;
}
