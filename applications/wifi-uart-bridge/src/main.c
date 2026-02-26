/*
 * WiFi to UART Bridge
 * Bridges WiFi and UART interfaces
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/ring_buffer.h>

LOG_MODULE_REGISTER(wifi_uart_bridge, LOG_LEVEL_INF);

/* --- Configuration --- */
#define WIFI_SSID "2.4-voda"
#define WIFI_PRE_SHARED_KEY  "5E7HAwbB4K"
#define BRIDGE_PORT 8080
#define RING_BUF_SIZE 1024

/* --- Globals --- */
RING_BUF_DECLARE(uart_rx_ringbuf, RING_BUF_SIZE);
RING_BUF_DECLARE(tcp_rx_ringbuf, RING_BUF_SIZE);

const struct device *uart_dev = DEVICE_DT_GET(DT_ALIAS(bridge_uart));
static int client_socket = -1;

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;
static K_SEM_DEFINE(wifi_connected, 0, 1);
static K_SEM_DEFINE(ipv4_address_obtained, 0, 1);

/* keep last IPv4 string so we can print it after a reset/boot */
static char device_ip[NET_IPV4_ADDR_LEN];

/* --- UART ISR --- */
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
				/* Generally the coomands we transmit will be quite small, 
				   so we'll skip implementing reties for now to keep it simple. 
				*/
			}
		} else {
			uart_irq_tx_disable(dev);
		}
	}
}

/* --- WiFi Management --- */
static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status = (const struct wifi_status *)cb->info;
	if (status->status) {
		LOG_ERR("Connection request failed (%d)", status->status);
		/* connection failed */
	} else {
		LOG_INF("Connected to WiFi");
		k_sem_give(&wifi_connected);
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
			LOG_INF("IPv4 address: %s", buf);
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
	case NET_EVENT_IPV4_ADDR_ADD:
		handle_ipv4_result(iface);
		break;
	default:
		/* nothing else handled */
		break;
	}
}

static void wifi_connect(void)
{
	struct net_if *iface = net_if_get_default();
	struct wifi_connect_req_params wifi_params = {0};


	net_mgmt_init_event_callback(&wifi_cb, net_mgmt_event_handler,
				     NET_EVENT_WIFI_CONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_cb);

	net_mgmt_init_event_callback(&ipv4_cb, net_mgmt_event_handler,
				     NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&ipv4_cb);

	wifi_params.ssid = WIFI_SSID;
	wifi_params.ssid_length = strlen(WIFI_SSID);
	wifi_params.psk = WIFI_PRE_SHARED_KEY;
	wifi_params.psk_length = strlen(WIFI_PRE_SHARED_KEY);
	wifi_params.security = WIFI_SECURITY_TYPE_PSK;

	LOG_INF("Connecting to WiFi: %s", WIFI_SSID);
	if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &wifi_params, sizeof(wifi_params))) {
		LOG_ERR("WiFi Connection Request Failed");
	}

	if (k_sem_take(&wifi_connected, K_SECONDS(10)) != 0) {
		LOG_WRN("Timeout waiting for WiFi connect event");
		return;
	}
	LOG_INF("Waiting for IP address...");
	if (k_sem_take(&ipv4_address_obtained, K_SECONDS(10)) != 0) {
		LOG_WRN("Timeout waiting for IPv4 address");
		return;
	}
}

/* --- TCP Server --- */
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

		LOG_INF("Client connected!");

		while (1) {
			uint8_t rx_buf[128];
			
			/* 1. Read from TCP, write to UART ringbuf */
			int len = zsock_recv(client_socket, rx_buf, sizeof(rx_buf), ZSOCK_MSG_DONTWAIT);
			if (len > 0) {
				LOG_INF("TCP -> UART: %d bytes", len);
				ring_buf_put(&tcp_rx_ringbuf, rx_buf, len);
				uart_irq_tx_enable(uart_dev);
			} else if (len == 0 || (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
				LOG_INF("Client disconnected");
				zsock_close(client_socket);
				client_socket = -1;
				break;
			}

			/* 2. Read from UART ringbuf, write to TCP */
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

			k_sleep(K_MSEC(10)); /* Prevent tight loop */
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

	/* Settle the pins and flush the buffer to avoid power-on glitches */
	k_sleep(K_MSEC(100));
	uint8_t dummy;
	while (uart_fifo_read(uart_dev, &dummy, 1) > 0);

	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);

	wifi_connect();

	/* after a reboot we may want the IP address up front along with the boot
	 * messages; wifi_connect already logs the address when it's obtained,
	 * but print it again here so a viewer of the serial output sees it right
	 * after the reset sequence.  the callback saved the value in device_ip. */
	if (device_ip[0] != '\0') {
		LOG_INF("Device IPv4: %s", device_ip);
	}

	tcp_server_thread();

	return 0;
}
