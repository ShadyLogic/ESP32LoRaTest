# ESP32LoRaTest

Passive MeshCore receiver for an ESP32-DevKit-C connected to a Waveshare
Core1262 (SX1262) LoRa module.

The firmware does not transmit. It listens for MeshCore packets, logs radio
metadata and MeshCore header information, and decodes information that is
publicly readable.

## Hardware

The wiring remains in [PINOUT.md](PINOUT.md):

| Core1262 | ESP32-DevKit-C |
| --- | --- |
| MISO | GPIO19 |
| MOSI | GPIO23 |
| CLK | GPIO18 |
| CS | GPIO5 |
| RESET | GPIO21 |
| BUSY | GPIO2 |
| DIO1 | GPIO15 |
| RXEN | GPIO22 |
| TXEN | GPIO4 |

DIO2 is not connected. DIO3 controls the Core1262 TCXO internally.

## What the listener logs

For every LoRa packet received with the configured radio profile, the serial
log includes:

- packet number and 64-bit ESP32 uptime in milliseconds plus
  `days:hours:minutes:seconds.milliseconds` form
- packet length
- RSSI
- SNR
- SX1262 frequency-error estimate
- received LoRa coding rate and CRC presence when available
- full raw packet bytes in hexadecimal
- MeshCore payload version
- route type
- payload type
- hop count and path-hash size
- transport codes, when present
- route path bytes

For publicly readable MeshCore content it also logs:

- default Public channel text messages
- default Public channel datagrams
- node advertisements, including public-key prefix, timestamp, advertised
  node type, optional location, and name
- control-packet subtype

The listener deliberately does not attempt to decrypt direct messages,
requests, responses, returned paths, or private channels.

## Persistent flash logs

Received packet logs are written to the ESP32 flash using LittleFS as well as
being printed live over serial. They survive resets and power cycles. Normal
firmware uploads also leave them intact unless the flash filesystem is erased
or the partition layout is changed.

The entire LittleFS partition is dedicated to packet logs. Logs are split
into 64 KiB rotating segments named `/meshcore0.log`,
`/meshcore1.log`, and so on. At startup the firmware calculates how many
64 KiB segments the filesystem can hold and uses that many slots. If
LittleFS metadata overhead makes the theoretical slot count slightly too
large, the logger discards the oldest closed segment only when additional
write space is actually required.

There is no fixed four-file limit and no percentage of LittleFS reserved for
some other application. Apart from LittleFS filesystem metadata and the
logger's tiny state file, the partition is dedicated to retained logs.

On first use, LittleFS is mounted with format-on-failure enabled so the data
partition can be initialized automatically.

When a PC is connected, open the serial monitor:

```sh
pio device monitor
```

Then enter one of these commands followed by Enter:

```text
logs
loginfo
clearlogs
help
```

- `logs` dumps every stored log file in chronological rotation order.
- `loginfo` reports filesystem usage, each log file size, and the active file.
- `clearlogs` erases only the MeshCore log files and starts a new log.
- `help` prints the available commands.

The radio keeps its receive-only configuration. Dumping a large log is a
blocking serial operation, so packets that arrive while a dump is in progress
may be missed.

## Public channel

MeshCore's default Public channel is encrypted on air but uses a documented
shared key intended to be known by everyone:

```text
8b3387e9c5cdea6ac9e5edbaa115cd72
```

This firmware implements the same AES-128 + truncated HMAC-SHA256 processing
used by MeshCore and will decode packets whose channel hash matches that key.

MeshCore group messages do not cryptographically authenticate the sender
name. The displayed `name: message` text should therefore be treated as
unverified.

Hashtag channels are also intended to be publicly discoverable by people who
know the hashtag, but their keys are derived from the hashtag name. A passive
listener cannot recover arbitrary unknown hashtag names from the one-byte
channel hash, so this firmware does not try to brute-force them.

## Radio profile

MeshCore networks must use matching frequency, bandwidth, spreading factor,
coding rate, and sync word. There is no single worldwide MeshCore frequency.

The PlatformIO project currently defaults to the MeshCore USA/Canada narrow
profile:

| Setting | Value |
| --- | --- |
| Frequency | 910.525 MHz |
| Bandwidth | 62.5 kHz |
| Spreading factor | SF7 |
| Coding rate | 4/5 |
| LoRa sync word | private / 0x12 |
| Preamble | 32 symbols |

The profile is configured in `platformio.ini`:

```ini
-D MESHCORE_FREQUENCY_MHZ=910.525
-D MESHCORE_BANDWIDTH_KHZ=62.5
-D MESHCORE_SPREADING_FACTOR=7
-D MESHCORE_CODING_RATE=5
```

Change these values to the preset used by the MeshCore network you want to
monitor.

## Build and run

```sh
pio run
pio run --target upload
pio device monitor
```

The serial monitor runs at 115200 baud.

A normal startup looks like:

```text
ESP32 + Core1262 MeshCore passive listener
-----------------------------------------
Frequency: 910.525 MHz
Bandwidth: 62.5 kHz
Spreading factor: SF7
Coding rate: 4/5
Preamble: 32 symbols
MeshCore Public channel hash: 0x11
LittleFS logging enabled: ... bytes used, ... x 64 KiB log slots
Initializing SX1262... SUCCESS
Starting continuous receive... SUCCESS
Listener is passive. It will not transmit or forward packets.
```

Example packet logging is similar to:

```text
=== RX #12 ===
uptime_ms=126391 uptime=0:00:02:06.391 len=48 rssi_dbm=-91.5 snr_db=7.25 freq_error_hz=-122.0 rx_cr=4/5 crc=yes
raw=...
mesh_version=1 route=flood payload=group-text(5) hops=2 path_hash_bytes=1
path=...
PUBLIC timestamp=... text_type=0 attempt=0 text="node-name: hello"
note=MeshCore group sender names are unverified message text
```

## Notes

The SX1262 frequency-error value is useful as a diagnostic, but RadioLib notes
that this measurement is based on an undocumented SX126x behavior, so it
should be treated as an estimate.

Advertisements contain an Ed25519 signature. This listener currently logs
advertisement contents but does not verify that signature, and marks them
accordingly.

Duplicate MeshCore packets are intentionally not suppressed. A flooded packet
may arrive more than once over different paths, and retaining each reception
preserves its individual RSSI/SNR/path information.

## References

- MeshCore packet format:
  https://github.com/meshcore-dev/MeshCore/blob/main/docs/packet_format.md
- MeshCore payload format:
  https://github.com/meshcore-dev/MeshCore/blob/main/docs/payloads.md
- MeshCore radio presets:
  https://github.com/meshcore-dev/MeshCore/blob/main/docs/radio_presets.md
- MeshCore public-channel documentation:
  https://github.com/meshcore-dev/MeshCore/blob/main/docs/companion_protocol.md
- RadioLib:
  https://github.com/jgromes/RadioLib
