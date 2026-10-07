# ESP32LoRaTest

Simple smoke test for an ESP32-DevKit-C connected to a Waveshare Core1262
LoRa module.

The test verifies that the ESP32 can initialize the SX1262 over SPI and
transmit a LoRa packet on command.

## Hardware

See [PINOUT.md](PINOUT.md) for the wiring.

The test uses the ESP32 VSPI bus plus these Core1262 control signals:

- RESET: GPIO26
- BUSY: GPIO27
- DIO1: GPIO33
- RXEN: GPIO22
- TXEN: GPIO4
- DIO2: not connected
- DIO3: not connected to the ESP32 because it drives the onboard TCXO

The Core1262 RF switch is controlled through RXEN and TXEN. The firmware
hands those pins to RadioLib with `setRfSwitchPins()`.

GPIO1 and GPIO3 are left unused by the radio so UART0 remains available for
programming and the serial monitor.

## Before transmitting

Connect an antenna that matches the frequency range of your Core1262 module
before transmitting.

The PlatformIO configuration defaults to 915 MHz:

```ini
-D LORA_FREQUENCY_MHZ=915.0
```

Change that value in `platformio.ini` to a frequency supported by your
physical Core1262 variant and permitted for your location before testing.

The test uses 10 dBm transmit power.

## Build and upload

This project uses PlatformIO and RadioLib.

From the repository root:

```sh
pio run
pio run --target upload
pio device monitor
```

The serial monitor runs at 115200 baud.

On a successful startup you should see output similar to:

```text
ESP32-DevKit-C + Waveshare Core1262 test
----------------------------------------
Frequency: 915.000 MHz
Initializing SX1262... SUCCESS

Radio ready.
Enter 't' in the serial monitor to transmit a test packet.
```

Enter `t` in the serial monitor to transmit a packet. A successful local
transmit should report:

```text
Transmitting: ESP32 Core1262 test #1
Transmit SUCCESS
```

A successful transmit means the SX1262 completed its transmit operation and
the DIO1 interrupt path worked. It does not by itself prove the RF output or
antenna path. Use a second compatible LoRa receiver configured with the same
frequency and modem settings to verify the packet over the air.

## LoRa settings

The smoke test uses:

| Setting | Value |
| --- | --- |
| Bandwidth | 125 kHz |
| Spreading factor | 9 |
| Coding rate | 4/7 |
| Sync word | RadioLib private LoRa sync word |
| TX power | 10 dBm |
| Preamble | 8 symbols |
| TCXO voltage | 1.8 V |

## References

- Waveshare Core1262 schematic: https://files.waveshare.com/upload/c/c1/CoreSX1262_Sch.pdf
- RadioLib: https://github.com/jgromes/RadioLib
