# ESP32LoRaTest

Simple smoke test for an ESP32-DevKit-C connected to a Waveshare Core1262
LoRa module.

The test verifies that the ESP32 can initialize the SX1262 over SPI and
transmit a LoRa packet on command.

## Hardware

See [PINOUT.md](PINOUT.md) for the wiring.

The test uses the ESP32 VSPI bus plus these Core1262 control signals:

- RESET: GPIO21
- BUSY: GPIO2
- DIO1: GPIO15
- RXEN: GPIO22
- TXEN: GPIO4
- DIO2: not connected
- DIO3: not connected to the ESP32 because it controls the onboard TCXO internally

The Core1262 RF switch is controlled through RXEN and TXEN. The firmware
hands those pins to RadioLib with `setRfSwitchPins()` after the SX1262 has
initialized.

## PlatformIO

The project pins its build dependencies so a future PlatformIO or RadioLib
release does not silently change the test environment:

- PlatformIO Espressif 32 platform: 7.1.3
- RadioLib: 7.8.1
- Framework: Arduino

From the repository root:

```sh
pio run
pio run --target upload
pio device monitor
```

The serial monitor runs at 115200 baud.

If you previously built the project with different dependency versions, clean
the project first:

```sh
pio run --target clean
pio run
```

## Frequency

The PlatformIO configuration currently sets:

```ini
-D LORA_FREQUENCY_MHZ=915.0
```

Change that value in `platformio.ini` if 915 MHz is not supported by your
physical Core1262 variant or is not appropriate for your location.

Connect a matching antenna before transmitting.

## Expected output

On a successful startup you should see output similar to:

```text
ESP32-DevKit-C + Waveshare Core1262 test
----------------------------------------
Frequency: 915.000 MHz
Initializing SX1262... SUCCESS

Radio ready.
Enter 't' in the serial monitor to transmit a test packet.
```

Enter `t` in the serial monitor to transmit a packet:

```text
Transmitting: ESP32 Core1262 test #1
Transmit SUCCESS
```

A successful transmit means the SX1262 completed its transmit operation and
the DIO1 interrupt path worked. It does not by itself prove the RF output or
antenna path. Use a second compatible LoRa receiver configured with the same
frequency and modem settings to verify the packet over the air.

If initialization fails, note the RadioLib error number printed after:

```text
Initialization failed, RadioLib error ...
```

That error number is useful for distinguishing SPI/wiring failures from TCXO
or radio-configuration failures.

## LoRa settings

| Setting | Value |
| --- | --- |
| Bandwidth | 125 kHz |
| Spreading factor | 9 |
| Coding rate | 4/7 |
| Sync word | RadioLib private LoRa sync word |
| TX power | 10 dBm |
| Preamble | 8 symbols |
| TCXO control voltage | 1.7 V |

## References

- Waveshare Core1262 schematic: https://files.waveshare.com/upload/c/c1/CoreSX1262_Sch.pdf
- Waveshare Core1262 documentation: https://www.waveshare.com/wiki/Core1262-868M
- RadioLib: https://github.com/jgromes/RadioLib
