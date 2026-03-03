# WiFi UART Bridge for ESP32C3 Supermini

This Zephyr-based application bridges a WiFi TCP connection to a UART interface, specifically designed for remote 3D printer management (e.g., via OctoPrint). It also provides a lightweight web status page for monitoring and manual command execution.

> [!IMPORTANT]
> **Hardware Patching Required**:
> To interface with a 3D printer, you must tap into the UART lines on the printer's mainboard.
> For common CH340G implementations (16-pin chip), **TX** and **RX** are typically **pins 2 and 3**. Connect these to their counterparts on your esp32c3 (in prj.conf I disabled SPI2 and assigned pins 4 and 5 to uart0).

## Features

- **Bidirectional Bridge**: Forwards data between a TCP server (port 8080) and UART.
- **Web Status Page**: Hosted on port 80, showing device IP, connectivity status, and last printer activity.
- **Manual Command Execution**: Dedicated button on the web page to send `M115` and view the firmware response.
- **Intelligent Logging**: 
  - Parsed temperature reports from the printer are logged clearly to the console.
  - Periodic web server stack usage monitoring.
- **Robust WiFi Management**: Automatic reconnection and interface monitoring.

## Hardware Support

- **Board**: `esp32c3_supermini` (and maybe `esp32c3_devkitm`, but not tested)
- **Interface**: Default UART (configured via `bridge_uart` alias).

## Configuration

Settings can be adjusted in [applications/wifi-uart-bridge/src/main.c](applications/wifi-uart-bridge/src/main.c):

- `WIFI_SSID`: Your WiFi network name.
- `WIFI_PRE_SHARED_KEY`: Your WiFi password.
- `BRIDGE_PORT`: TCP port for the UART bridge (default: 8080).
- `WEB_SERVER_PORT`: Port for the status page (default: 80).

## Building and Flashing

To build for the ESP32C3 Supermini:

```bash
west build -p always -b esp32c3_supermini applications/wifi-uart-bridge
west flash
```

## Monitoring

- **Console via serial port ttyACMx**: View real-time logs including temperature reports and stack usage.
- **Web**: Access `http://<device_ip>` to check status and send test commands.
- **OctoPrint**: Configure a RAW TCP connection to `<device_ip>:8080`.
