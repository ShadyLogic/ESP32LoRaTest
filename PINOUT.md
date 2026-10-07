# Pinout

| Core1262 | ESP32-DevKit-C |
| -------- | -------------- |
| 3V3 | 3V3 |
| GND | GND |
| MISO | VSPI MISO (GPIO19) |
| MOSI | VSPI MOSI (GPIO23) |
| CLK | VSPI CLK (GPIO18) |
| CS | VSPI CS (GPIO5) |
| RESET | GPIO26 |
| BUSY | GPIO27 |
| DIO1 | GPIO33 |
| RXEN | GPIO22 |
| TXEN | GPIO4 |
| DIO2 | Not connected |
| DIO3 | Not connected |

DIO3 is connected internally to the Core1262 TCXO. DIO2 is exposed by the
module but is not needed by this test. The Core1262 RF switch is controlled
through the separate RXEN and TXEN signals.

GPIO1 and GPIO3 are deliberately left free because the ESP32-DevKit-C uses
them for UART0, which is used by the USB serial interface.
