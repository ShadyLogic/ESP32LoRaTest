#include <Arduino.h>
#include <AES.h>
#include <RadioLib.h>
#include <LittleFS.h>
#include <SHA256.h>
#include <SPI.h>
#include <esp_timer.h>

#ifndef MESHCORE_FREQUENCY_MHZ
#define MESHCORE_FREQUENCY_MHZ 910.525
#endif

#ifndef MESHCORE_BANDWIDTH_KHZ
#define MESHCORE_BANDWIDTH_KHZ 62.5
#endif

#ifndef MESHCORE_SPREADING_FACTOR
#define MESHCORE_SPREADING_FACTOR 7
#endif

#ifndef MESHCORE_CODING_RATE
#define MESHCORE_CODING_RATE 5
#endif

namespace {

// ESP32 / Core1262 wiring. Keep these in sync with PINOUT.md.
constexpr int kPinSck = 18;
constexpr int kPinMiso = 19;
constexpr int kPinMosi = 23;
constexpr int kPinCs = 5;
constexpr int kPinReset = 21;
constexpr int kPinBusy = 2;
constexpr int kPinDio1 = 15;
constexpr int kPinRxEn = 22;
constexpr int kPinTxEn = 4;

constexpr uint32_t kSerialBaud = 115200;
constexpr float kFrequencyMHz = MESHCORE_FREQUENCY_MHZ;
constexpr float kBandwidthKHz = MESHCORE_BANDWIDTH_KHZ;
constexpr uint8_t kSpreadingFactor = MESHCORE_SPREADING_FACTOR;
constexpr uint8_t kCodingRate = MESHCORE_CODING_RATE;
constexpr float kTcxoVoltage = 1.7;

// MeshCore uses a longer preamble at the lower spreading factors.
constexpr uint16_t kPreambleLength = (kSpreadingFactor <= 8) ? 32 : 16;

// Current MeshCore v1 packet constants.
constexpr uint8_t kRouteMask = 0x03;
constexpr uint8_t kPayloadTypeShift = 2;
constexpr uint8_t kPayloadTypeMask = 0x0F;
constexpr uint8_t kPayloadVersionShift = 6;
constexpr uint8_t kPayloadVersionMask = 0x03;

constexpr uint8_t kRouteTransportFlood = 0x00;
constexpr uint8_t kRouteFlood = 0x01;
constexpr uint8_t kRouteDirect = 0x02;
constexpr uint8_t kRouteTransportDirect = 0x03;

constexpr uint8_t kPayloadReq = 0x00;
constexpr uint8_t kPayloadResponse = 0x01;
constexpr uint8_t kPayloadText = 0x02;
constexpr uint8_t kPayloadAck = 0x03;
constexpr uint8_t kPayloadAdvert = 0x04;
constexpr uint8_t kPayloadGroupText = 0x05;
constexpr uint8_t kPayloadGroupData = 0x06;
constexpr uint8_t kPayloadAnonReq = 0x07;
constexpr uint8_t kPayloadPath = 0x08;
constexpr uint8_t kPayloadTrace = 0x09;
constexpr uint8_t kPayloadMultipart = 0x0A;
constexpr uint8_t kPayloadControl = 0x0B;
constexpr uint8_t kPayloadRawCustom = 0x0F;

constexpr size_t kMaxRadioPacket = 256;
constexpr size_t kMaxMeshPathBytes = 64;
constexpr size_t kCipherMacSize = 2;
constexpr size_t kAesBlockSize = 16;

// Persistent flash logging. The entire LittleFS partition is dedicated to
// logs. It is divided into as many 64 KiB rotating segments as will fit.
constexpr size_t kLogSegmentBytes = 64 * 1024;
constexpr size_t kMinimumFilesystemFreeBytes = 4096;
constexpr char kLegacyLogStatePath[] = "/meshcore.state";
constexpr size_t kSerialCommandBufferSize = 32;

// MeshCore's documented default Public channel key.
// The protocol's channel storage is 32 bytes. A 128-bit channel key occupies
// the first 16 bytes and the remaining 16 bytes are zero.
constexpr uint8_t kPublicChannelSecret[32] = {
    0x8B, 0x33, 0x87, 0xE9, 0xC5, 0xCD, 0xEA, 0x6A,
    0xC9, 0xE5, 0xED, 0xBA, 0xA1, 0x15, 0xCD, 0x72,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

SPIClass radioSpi(VSPI);
SX1262 radio = new Module(
    kPinCs,
    kPinDio1,
    kPinReset,
    kPinBusy,
    radioSpi);

volatile bool packetReceived = false;
uint32_t packetCounter = 0;
uint8_t publicChannelHash = 0;

bool logStorageReady = false;
uint32_t activeLogSequence = 0;
uint32_t oldestLogSequence = 0;
uint16_t logFileCount = 0;
File logFile;
char serialCommandBuffer[kSerialCommandBufferSize] = {};
size_t serialCommandLength = 0;

class TeePrint : public Print {
public:
  TeePrint(Print& first, Print& second) : first_(first), second_(second) {}

  size_t write(uint8_t value) override {
    const size_t a = first_.write(value);
    const size_t b = second_.write(value);
    return (a == 1 && b == 1) ? 1 : 0;
  }

  size_t write(const uint8_t* buffer, size_t size) override {
    const size_t a = first_.write(buffer, size);
    const size_t b = second_.write(buffer, size);
    return (a == size && b == size) ? size : 0;
  }

private:
  Print& first_;
  Print& second_;
};

struct MeshPacketView {
  uint8_t header = 0;
  uint8_t routeType = 0;
  uint8_t payloadType = 0;
  uint8_t payloadVersion = 0;
  bool hasTransportCodes = false;
  uint16_t transportCode1 = 0;
  uint16_t transportCode2 = 0;
  uint8_t pathLengthByte = 0;
  uint8_t pathHashSize = 0;
  uint8_t pathHashCount = 0;
  const uint8_t* path = nullptr;
  size_t pathBytes = 0;
  const uint8_t* payload = nullptr;
  size_t payloadLength = 0;
};

#if defined(ESP8266) || defined(ESP32)
IRAM_ATTR
#endif
void onPacketReceived() {
  packetReceived = true;
}

uint16_t readLe16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t readLe32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

int32_t readLeI32(const uint8_t* p) {
  return static_cast<int32_t>(readLe32(p));
}

uint64_t uptimeMillis() {
  return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
}

void printUptime(Print& out, uint64_t uptimeMs) {
  const uint32_t millisPart = static_cast<uint32_t>(uptimeMs % 1000ULL);
  uint64_t totalSeconds = uptimeMs / 1000ULL;
  const uint32_t seconds = static_cast<uint32_t>(totalSeconds % 60ULL);
  totalSeconds /= 60ULL;
  const uint32_t minutes = static_cast<uint32_t>(totalSeconds % 60ULL);
  totalSeconds /= 60ULL;
  const uint32_t hours = static_cast<uint32_t>(totalSeconds % 24ULL);
  const unsigned long long days =
      static_cast<unsigned long long>(totalSeconds / 24ULL);

  char buffer[40];
  snprintf(
      buffer,
      sizeof(buffer),
      "%llu:%02u:%02u:%02u.%03u",
      days,
      hours,
      minutes,
      seconds,
      millisPart);
  out.print(buffer);
}

void printHexByte(Print& out, uint8_t value) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  out.print(kHex[value >> 4]);
  out.print(kHex[value & 0x0F]);
}

void printHex(Print& out, const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    printHexByte(out, data[i]);
  }
}

void printEscapedText(Print& out, const uint8_t* data, size_t length) {
  out.print('"');
  for (size_t i = 0; i < length; ++i) {
    const uint8_t ch = data[i];
    if (ch == 0) {
      break;
    }
    if (ch == '\\' || ch == '"') {
      out.print('\\');
      out.print(static_cast<char>(ch));
    } else if (ch >= 0x20 && ch <= 0x7E) {
      out.print(static_cast<char>(ch));
    } else if (ch == '\n') {
      out.print("\\n");
    } else if (ch == '\r') {
      out.print("\\r");
    } else if (ch == '\t') {
      out.print("\\t");
    } else {
      out.print("\\x");
      printHexByte(out, ch);
    }
  }
  out.print('"');
}

const char* routeName(uint8_t routeType) {
  switch (routeType) {
    case kRouteTransportFlood: return "transport-flood";
    case kRouteFlood: return "flood";
    case kRouteDirect: return "direct";
    case kRouteTransportDirect: return "transport-direct";
    default: return "unknown";
  }
}

const char* payloadName(uint8_t payloadType) {
  switch (payloadType) {
    case kPayloadReq: return "request";
    case kPayloadResponse: return "response";
    case kPayloadText: return "direct-text";
    case kPayloadAck: return "ack";
    case kPayloadAdvert: return "advert";
    case kPayloadGroupText: return "group-text";
    case kPayloadGroupData: return "group-data";
    case kPayloadAnonReq: return "anonymous-request";
    case kPayloadPath: return "returned-path";
    case kPayloadTrace: return "trace";
    case kPayloadMultipart: return "multipart";
    case kPayloadControl: return "control";
    case kPayloadRawCustom: return "raw-custom";
    default: return "unknown";
  }
}

bool parseMeshPacket(const uint8_t* data, size_t length, MeshPacketView& out) {
  if (length < 3) {
    return false;
  }

  size_t offset = 0;
  out.header = data[offset++];
  out.routeType = out.header & kRouteMask;
  out.payloadType = (out.header >> kPayloadTypeShift) & kPayloadTypeMask;
  out.payloadVersion =
      (out.header >> kPayloadVersionShift) & kPayloadVersionMask;

  out.hasTransportCodes =
      out.routeType == kRouteTransportFlood ||
      out.routeType == kRouteTransportDirect;

  if (out.hasTransportCodes) {
    if (offset + 4 >= length) {
      return false;
    }
    out.transportCode1 = readLe16(&data[offset]);
    offset += 2;
    out.transportCode2 = readLe16(&data[offset]);
    offset += 2;
  }

  if (offset >= length) {
    return false;
  }

  out.pathLengthByte = data[offset++];
  out.pathHashCount = out.pathLengthByte & 0x3F;
  out.pathHashSize = (out.pathLengthByte >> 6) + 1;

  // Hash size code 3 is reserved in MeshCore v1.
  if (out.pathHashSize == 4) {
    return false;
  }

  out.pathBytes =
      static_cast<size_t>(out.pathHashCount) * out.pathHashSize;
  if (out.pathBytes > kMaxMeshPathBytes ||
      offset + out.pathBytes >= length) {
    return false;
  }

  out.path = &data[offset];
  offset += out.pathBytes;
  out.payload = &data[offset];
  out.payloadLength = length - offset;

  return out.payloadLength > 0;
}

void computePublicChannelHash() {
  SHA256 sha;
  sha.update(kPublicChannelSecret, 16);
  sha.finalize(&publicChannelHash, 1);
}

int decryptPublicChannelPayload(
    const uint8_t* payload,
    size_t payloadLength,
    uint8_t* plaintext,
    size_t plaintextCapacity) {
  // group payload = channel hash (1) + MAC (2) + AES ciphertext
  if (payloadLength < 1 + kCipherMacSize + kAesBlockSize) {
    return -1;
  }

  if (payload[0] != publicChannelHash) {
    return 0;
  }

  const uint8_t* macAndCiphertext = &payload[1];
  const size_t macAndCiphertextLength = payloadLength - 1;
  const uint8_t* ciphertext = macAndCiphertext + kCipherMacSize;
  const size_t ciphertextLength =
      macAndCiphertextLength - kCipherMacSize;

  if (ciphertextLength == 0 ||
      ciphertextLength % kAesBlockSize != 0 ||
      ciphertextLength > plaintextCapacity) {
    return -1;
  }

  uint8_t expectedMac[kCipherMacSize];
  SHA256 sha;
  sha.resetHMAC(kPublicChannelSecret, sizeof(kPublicChannelSecret));
  sha.update(ciphertext, ciphertextLength);
  sha.finalizeHMAC(
      kPublicChannelSecret,
      sizeof(kPublicChannelSecret),
      expectedMac,
      sizeof(expectedMac));

  if (memcmp(expectedMac, macAndCiphertext, kCipherMacSize) != 0) {
    return -2;
  }

  AES128 aes;
  aes.setKey(kPublicChannelSecret, 16);

  for (size_t offset = 0;
       offset < ciphertextLength;
       offset += kAesBlockSize) {
    aes.decryptBlock(&plaintext[offset], &ciphertext[offset]);
  }

  return static_cast<int>(ciphertextLength);
}

void logPublicGroupText(Print& out, const MeshPacketView& packet) {
  uint8_t plaintext[kMaxRadioPacket] = {};
  const int decryptedLength = decryptPublicChannelPayload(
      packet.payload,
      packet.payloadLength,
      plaintext,
      sizeof(plaintext));

  if (decryptedLength == 0) {
    out.print("channel_hash=0x");
    printHexByte(out, packet.payload[0]);
    out.println(" encrypted=unknown-channel");
    return;
  }

  if (decryptedLength < 0) {
    out.print("public_channel_decrypt=failed code=");
    out.println(decryptedLength);
    return;
  }

  if (decryptedLength < 5) {
    out.println("public_channel_decrypt=malformed");
    return;
  }

  const uint32_t timestamp = readLe32(plaintext);
  const uint8_t flags = plaintext[4];
  const uint8_t textType = flags >> 2;
  const uint8_t attempt = flags & 0x03;

  size_t textLength = static_cast<size_t>(decryptedLength) - 5;
  while (textLength > 0 && plaintext[5 + textLength - 1] == 0) {
    --textLength;
  }

  out.print("PUBLIC timestamp=");
  out.print(timestamp);
  out.print(" text_type=");
  out.print(textType);
  out.print(" attempt=");
  out.print(attempt);
  out.print(" text=");
  printEscapedText(out, &plaintext[5], textLength);
  out.println();

  if (textType == 0) {
    out.println(
        "note=MeshCore group sender names are unverified message text");
  }
}

void logPublicGroupData(Print& out, const MeshPacketView& packet) {
  uint8_t plaintext[kMaxRadioPacket] = {};
  const int decryptedLength = decryptPublicChannelPayload(
      packet.payload,
      packet.payloadLength,
      plaintext,
      sizeof(plaintext));

  if (decryptedLength == 0) {
    out.print("channel_hash=0x");
    printHexByte(out, packet.payload[0]);
    out.println(" encrypted=unknown-channel");
    return;
  }

  if (decryptedLength < 3) {
    out.print("public_group_data_decrypt=failed code=");
    out.println(decryptedLength);
    return;
  }

  const uint16_t dataType = readLe16(plaintext);
  const uint8_t declaredLength = plaintext[2];
  const size_t availableLength =
      static_cast<size_t>(decryptedLength) - 3;
  const size_t dataLength =
      min(static_cast<size_t>(declaredLength), availableLength);

  out.print("PUBLIC_DATA type=0x");
  printHexByte(out, static_cast<uint8_t>(dataType >> 8));
  printHexByte(out, static_cast<uint8_t>(dataType & 0xFF));
  out.print(" len=");
  out.print(dataLength);
  out.print(" data=");
  printHex(out, &plaintext[3], dataLength);
  out.println();
}

void logAdvert(Print& out, const MeshPacketView& packet) {
  // public key (32) + timestamp (4) + Ed25519 signature (64)
  constexpr size_t kAdvertFixedLength = 32 + 4 + 64;
  if (packet.payloadLength < kAdvertFixedLength) {
    out.println("advert=malformed");
    return;
  }

  const uint8_t* publicKey = packet.payload;
  const uint32_t timestamp = readLe32(&packet.payload[32]);
  const uint8_t* appData = &packet.payload[kAdvertFixedLength];
  const size_t appDataLength =
      packet.payloadLength - kAdvertFixedLength;

  out.print("ADVERT timestamp=");
  out.print(timestamp);
  out.print(" pubkey_prefix=");
  printHex(out, publicKey, 8);
  out.println(" signature=not-verified");

  if (appDataLength == 0) {
    return;
  }

  const uint8_t flags = appData[0];
  const uint8_t nodeType = flags & 0x0F;
  size_t offset = 1;

  out.print("advert_type=");
  out.print(nodeType);
  out.print(" flags=0x");
  printHexByte(out, flags);

  if (flags & 0x10) {
    if (offset + 8 > appDataLength) {
      out.println(" appdata=malformed");
      return;
    }
    const int32_t latitude = readLeI32(&appData[offset]);
    offset += 4;
    const int32_t longitude = readLeI32(&appData[offset]);
    offset += 4;

    out.print(" lat=");
    out.print(latitude / 1000000.0, 6);
    out.print(" lon=");
    out.print(longitude / 1000000.0, 6);
  }

  if (flags & 0x20) {
    if (offset + 2 > appDataLength) {
      out.println(" appdata=malformed");
      return;
    }
    out.print(" feature1=0x");
    printHexByte(out, appData[offset + 1]);
    printHexByte(out, appData[offset]);
    offset += 2;
  }

  if (flags & 0x40) {
    if (offset + 2 > appDataLength) {
      out.println(" appdata=malformed");
      return;
    }
    out.print(" feature2=0x");
    printHexByte(out, appData[offset + 1]);
    printHexByte(out, appData[offset]);
    offset += 2;
  }

  if ((flags & 0x80) && offset < appDataLength) {
    out.print(" name=");
    printEscapedText(out, &appData[offset], appDataLength - offset);
  }

  out.println();
}

void logMeshPacket(Print& out, const uint8_t* data, size_t length) {
  MeshPacketView packet;
  if (!parseMeshPacket(data, length, packet)) {
    out.println("mesh_parse=invalid");
    return;
  }

  out.print("mesh_version=");
  out.print(packet.payloadVersion + 1);
  out.print(" route=");
  out.print(routeName(packet.routeType));
  out.print(" payload=");
  out.print(payloadName(packet.payloadType));
  out.print("(");
  out.print(packet.payloadType);
  out.print(")");
  out.print(" hops=");
  out.print(packet.pathHashCount);
  out.print(" path_hash_bytes=");
  out.print(packet.pathHashSize);

  if (packet.hasTransportCodes) {
    out.print(" transport=0x");
    printHexByte(out, static_cast<uint8_t>(packet.transportCode1 >> 8));
    printHexByte(out, static_cast<uint8_t>(packet.transportCode1 & 0xFF));
    out.print(",0x");
    printHexByte(out, static_cast<uint8_t>(packet.transportCode2 >> 8));
    printHexByte(out, static_cast<uint8_t>(packet.transportCode2 & 0xFF));
  }

  out.println();

  if (packet.pathBytes > 0) {
    out.print("path=");
    printHex(out, packet.path, packet.pathBytes);
    out.println();
  }

  switch (packet.payloadType) {
    case kPayloadGroupText:
      logPublicGroupText(out, packet);
      break;

    case kPayloadGroupData:
      logPublicGroupData(out, packet);
      break;

    case kPayloadAdvert:
      logAdvert(out, packet);
      break;

    case kPayloadText:
    case kPayloadReq:
    case kPayloadResponse:
    case kPayloadAnonReq:
    case kPayloadPath:
      out.println(
          "content=encrypted-private-or-peer-specific");
      break;

    case kPayloadControl:
      if (packet.payloadLength > 0) {
        out.print("control_subtype=0x");
        printHexByte(out, packet.payload[0] >> 4);
        out.println();
      }
      break;

    default:
      break;
  }
}

void logReceivedPacket(
    Print& out,
    uint8_t* data,
    size_t length,
    int state) {
  ++packetCounter;

  const float rssi = radio.getRSSI();
  const float snr = radio.getSNR();
  const float frequencyError = radio.getFrequencyError();

  uint8_t rxCodingRate = 0;
  bool rxHasCrc = false;
  const int headerState =
      radio.getLoRaRxHeaderInfo(&rxCodingRate, &rxHasCrc);

  out.println();
  out.print("=== RX #");
  out.print(packetCounter);
  out.println(" ===");

  const uint64_t uptimeMs = uptimeMillis();
  out.print("uptime_ms=");
  out.print(static_cast<unsigned long long>(uptimeMs));
  out.print(" uptime=");
  printUptime(out, uptimeMs);
  out.print(" len=");
  out.print(length);
  out.print(" rssi_dbm=");
  out.print(rssi, 1);
  out.print(" snr_db=");
  out.print(snr, 2);
  out.print(" freq_error_hz=");
  out.print(frequencyError, 1);

  if (headerState == RADIOLIB_ERR_NONE) {
    out.print(" rx_cr=");
    if (rxCodingRate >= 1 && rxCodingRate <= 4) {
      out.print("4/");
      out.print(rxCodingRate + 4);
    } else {
      out.print("raw:");
      out.print(rxCodingRate);
    }
    out.print(" crc=");
    out.print(rxHasCrc ? "yes" : "no");
  }
  out.println();

  out.print("raw=");
  printHex(out, data, length);
  out.println();

  if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    out.println("radio_crc=FAILED mesh_parse=skipped");
    return;
  }

  if (state != RADIOLIB_ERR_NONE) {
    out.print("radio_read_error=");
    out.println(state);
    return;
  }

  logMeshPacket(out, data, length);
}

void makeLogPath(uint32_t sequence, char* path, size_t pathSize) {
  snprintf(
      path,
      pathSize,
      "/meshcore-%010lu.log",
      static_cast<unsigned long>(sequence));
}

bool parseLogPath(const char* path, uint32_t& sequence) {
  const char* name = strrchr(path, '/');
  name = name ? name + 1 : path;

  constexpr char kPrefix[] = "meshcore-";
  constexpr char kSuffix[] = ".log";

  if (strncmp(name, kPrefix, sizeof(kPrefix) - 1) != 0) {
    return false;
  }

  char* end = nullptr;
  const unsigned long value =
      strtoul(name + sizeof(kPrefix) - 1, &end, 10);
  if (end == nullptr || strcmp(end, kSuffix) != 0) {
    return false;
  }

  sequence = static_cast<uint32_t>(value);
  return true;
}

void scanLogFiles() {
  bool found = false;
  uint32_t oldest = UINT32_MAX;
  uint32_t newest = 0;
  uint16_t count = 0;

  File root = LittleFS.open("/");
  if (!root) {
    logFileCount = 0;
    activeLogSequence = 0;
    oldestLogSequence = 0;
    return;
  }

  File entry = root.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      uint32_t sequence = 0;
      if (parseLogPath(entry.name(), sequence)) {
        found = true;
        ++count;
        if (sequence < oldest) {
          oldest = sequence;
        }
        if (sequence > newest) {
          newest = sequence;
        }
      }
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();

  logFileCount = count;
  if (found) {
    oldestLogSequence = oldest;
    activeLogSequence = newest;
  } else {
    oldestLogSequence = 0;
    activeLogSequence = 0;
  }
}

void migrateLegacyLogs() {
  // The previous logger used /meshcore0.log through /meshcore3.log plus a
  // one-value state file. Detect those names by enumerating the directory so
  // missing legacy files do not generate noisy VFS open errors.
  scanLogFiles();
  if (logFileCount != 0) {
    return;
  }

  bool legacyPresent[4] = {false, false, false, false};
  bool legacyStatePresent = false;
  bool hasLegacy = false;
  uint8_t highestExisting = 0;

  File root = LittleFS.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      if (!entry.isDirectory()) {
        const char* name = entry.name();
        const char* base = strrchr(name, '/');
        base = base ? base + 1 : name;

        if (strcmp(base, "meshcore.state") == 0) {
          legacyStatePresent = true;
        } else {
          for (uint8_t i = 0; i < 4; ++i) {
            char legacyName[20];
            snprintf(legacyName, sizeof(legacyName), "meshcore%u.log", i);
            if (strcmp(base, legacyName) == 0) {
              legacyPresent[i] = true;
              hasLegacy = true;
              highestExisting = i;
              break;
            }
          }
        }
      }
      entry.close();
      entry = root.openNextFile();
    }
    root.close();
  }

  if (!hasLegacy) {
    if (legacyStatePresent) {
      LittleFS.remove(kLegacyLogStatePath);
    }
    return;
  }

  uint8_t legacyActive = highestExisting;
  if (legacyStatePresent) {
    File state = LittleFS.open(kLegacyLogStatePath, FILE_READ);
    if (state) {
      const long value = state.parseInt();
      state.close();
      if (value >= 0 && value < 4) {
        legacyActive = static_cast<uint8_t>(value);
      }
    }
  }

  uint32_t newSequence = 0;
  for (uint8_t offset = 1; offset <= 4; ++offset) {
    const uint8_t index =
        static_cast<uint8_t>((legacyActive + offset) % 4);
    if (!legacyPresent[index]) {
      continue;
    }

    char oldPath[24];
    snprintf(oldPath, sizeof(oldPath), "/meshcore%u.log", index);

    char newPath[32];
    makeLogPath(newSequence++, newPath, sizeof(newPath));
    LittleFS.rename(oldPath, newPath);
  }

  if (legacyStatePresent) {
    LittleFS.remove(kLegacyLogStatePath);
  }

  scanLogFiles();
}

bool openActiveLogFile() {
  char path[32];
  makeLogPath(activeLogSequence, path, sizeof(path));
  logFile = LittleFS.open(path, FILE_APPEND);
  return static_cast<bool>(logFile);
}

bool deleteOldestClosedLog() {
  if (logFileCount <= 1 || oldestLogSequence == activeLogSequence) {
    return false;
  }

  char path[32];
  makeLogPath(oldestLogSequence, path, sizeof(path));
  if (!LittleFS.remove(path)) {
    return false;
  }

  scanLogFiles();
  return true;
}

bool ensureFilesystemWriteSpace() {
  if (!logStorageReady) {
    return false;
  }

  size_t total = LittleFS.totalBytes();
  size_t used = LittleFS.usedBytes();

  while (used >= total ||
         total - used < kMinimumFilesystemFreeBytes) {
    if (!deleteOldestClosedLog()) {
      break;
    }
    total = LittleFS.totalBytes();
    used = LittleFS.usedBytes();
  }

  return used < total;
}

bool rotateLogIfNeeded() {
  if (!logStorageReady || !logFile) {
    return false;
  }

  if (logFile.size() < kLogSegmentBytes) {
    return ensureFilesystemWriteSpace();
  }

  logFile.flush();
  logFile.close();

  if (!ensureFilesystemWriteSpace()) {
    Serial.println("Warning: LittleFS is full and no old log can be removed.");
  }

  ++activeLogSequence;
  char path[32];
  makeLogPath(activeLogSequence, path, sizeof(path));

  // Sequence numbers are monotonic, so this path should be new. Remove it
  // defensively without probing first to avoid noisy VFS read-open errors.
  LittleFS.remove(path);

  logFile = LittleFS.open(path, FILE_WRITE);
  if (!logFile) {
    if (deleteOldestClosedLog()) {
      logFile = LittleFS.open(path, FILE_WRITE);
    }
  }

  if (!logFile) {
    logStorageReady = false;
    Serial.println("Persistent logging disabled: log rotation failed.");
    return false;
  }

  ++logFileCount;
  if (logFileCount == 1) {
    oldestLogSequence = activeLogSequence;
  }

  logFile.println();
  logFile.println("=== LOG ROTATED ===");
  logFile.flush();
  return true;
}

void writeBootLogHeader() {
  if (!logStorageReady || !logFile) {
    return;
  }

  logFile.println();
  logFile.println("=== BOOT ===");
  logFile.print("uptime_ms=");
  logFile.print(static_cast<unsigned long long>(uptimeMillis()));
  logFile.print(" profile freq_mhz=");
  logFile.print(kFrequencyMHz, 3);
  logFile.print(" bw_khz=");
  logFile.print(kBandwidthKHz, 1);
  logFile.print(" sf=");
  logFile.print(kSpreadingFactor);
  logFile.print(" cr=4/");
  logFile.print(kCodingRate);
  logFile.print(" preamble=");
  logFile.println(kPreambleLength);
  logFile.flush();
}

void beginLogStorage() {
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed. Persistent logging disabled.");
    return;
  }

  if (LittleFS.totalBytes() == 0) {
    Serial.println("LittleFS reports zero capacity. Persistent logging disabled.");
    return;
  }

  migrateLegacyLogs();
  scanLogFiles();

  if (logFileCount == 0) {
    activeLogSequence = 0;
    oldestLogSequence = 0;
    if (!openActiveLogFile()) {
      Serial.println("Could not create persistent log file.");
      return;
    }
    logFileCount = 1;
  } else if (!openActiveLogFile()) {
    Serial.println("Could not open persistent log file.");
    return;
  }

  logStorageReady = true;
  ensureFilesystemWriteSpace();

  Serial.print("LittleFS logging enabled: ");
  Serial.print(LittleFS.usedBytes());
  Serial.print("/");
  Serial.print(LittleFS.totalBytes());
  Serial.print(" bytes used, ");
  Serial.print(logFileCount);
  Serial.print(" existing 64 KiB segments, active=");
  Serial.println(activeLogSequence);

  writeBootLogHeader();
}

void printLogInfo() {
  if (!logStorageReady) {
    Serial.println("Persistent logging is not available.");
    return;
  }

  if (logFile) {
    logFile.flush();
  }

  scanLogFiles();

  Serial.print("LittleFS total=");
  Serial.print(LittleFS.totalBytes());
  Serial.print(" used=");
  Serial.print(LittleFS.usedBytes());
  Serial.print(" segment_bytes=");
  Serial.print(kLogSegmentBytes);
  Serial.print(" files=");
  Serial.println(logFileCount);

  if (logFileCount == 0) {
    return;
  }

  for (uint32_t sequence = oldestLogSequence;
       sequence <= activeLogSequence;
       ++sequence) {
    char path[32];
    makeLogPath(sequence, path, sizeof(path));

    File file = LittleFS.open(path, FILE_READ);
    Serial.print(path);
    Serial.print(" size=");
    Serial.print(file ? file.size() : 0);
    if (file) {
      file.close();
    }

    if (sequence == activeLogSequence) {
      Serial.print(" active");
    }
    Serial.println();

    if (sequence == UINT32_MAX) {
      break;
    }
  }
}

void dumpLogs() {
  if (!logStorageReady) {
    Serial.println("Persistent logging is not available.");
    return;
  }

  if (logFile) {
    logFile.flush();
  }

  scanLogFiles();
  Serial.println("=== LOG DUMP BEGIN ===");

  if (logFileCount > 0) {
    for (uint32_t sequence = oldestLogSequence;
         sequence <= activeLogSequence;
         ++sequence) {
      char path[32];
      makeLogPath(sequence, path, sizeof(path));

      File file = LittleFS.open(path, FILE_READ);
      if (!file) {
        Serial.print("--- failed to open ");
        Serial.print(path);
        Serial.println(" ---");
      } else {
        Serial.print("--- ");
        Serial.print(path);
        Serial.print(" (");
        Serial.print(file.size());
        Serial.println(" bytes) ---");

        uint8_t buffer[128];
        while (file.available()) {
          const size_t count = file.read(buffer, sizeof(buffer));
          if (count == 0) {
            break;
          }
          Serial.write(buffer, count);
          delay(0);
        }
        file.close();
        Serial.println();
      }

      if (sequence == UINT32_MAX) {
        break;
      }
    }
  }

  Serial.println("=== LOG DUMP END ===");
}

void clearLogs() {
  if (!logStorageReady) {
    Serial.println("Persistent logging is not available.");
    return;
  }

  if (logFile) {
    logFile.close();
  }

  scanLogFiles();
  if (logFileCount > 0) {
    for (uint32_t sequence = oldestLogSequence;
         sequence <= activeLogSequence;
         ++sequence) {
      char path[32];
      makeLogPath(sequence, path, sizeof(path));
      LittleFS.remove(path);
      if (sequence == UINT32_MAX) {
        break;
      }
    }
  }

  // Clean up files from the previous four-slot logger too. remove() is safe
  // for absent paths and avoids the noisy read-open performed by exists().
  for (uint8_t i = 0; i < 4; ++i) {
    char path[24];
    snprintf(path, sizeof(path), "/meshcore%u.log", i);
    LittleFS.remove(path);
  }
  LittleFS.remove(kLegacyLogStatePath);

  activeLogSequence = 0;
  oldestLogSequence = 0;
  logFileCount = 0;

  if (!openActiveLogFile()) {
    logStorageReady = false;
    Serial.println("Logs cleared, but a new log could not be created.");
    return;
  }

  logFileCount = 1;
  writeBootLogHeader();
  Serial.println("Persistent logs cleared.");
}

void printSerialHelp() {
  Serial.println("Serial commands:");
  Serial.println("  logs      - dump stored logs oldest to newest");
  Serial.println("  loginfo   - show filesystem and log file usage");
  Serial.println("  clearlogs - erase stored MeshCore logs");
  Serial.println("  help      - show this help");
}

void executeSerialCommand(const char* command) {
  if (strcmp(command, "logs") == 0 || strcmp(command, "dump") == 0) {
    dumpLogs();
  } else if (strcmp(command, "loginfo") == 0) {
    printLogInfo();
  } else if (strcmp(command, "clearlogs") == 0) {
    clearLogs();
  } else if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
    printSerialHelp();
  } else if (command[0] != '\0') {
    Serial.print("Unknown command: ");
    Serial.println(command);
    printSerialHelp();
  }
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());

    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      serialCommandBuffer[serialCommandLength] = '\0';
      executeSerialCommand(serialCommandBuffer);
      serialCommandLength = 0;
      continue;
    }

    if (serialCommandLength + 1 < kSerialCommandBufferSize) {
      serialCommandBuffer[serialCommandLength++] = ch;
    }
  }
}

void haltOnError(const char* operation, int state) {
  Serial.print(operation);
  Serial.print(" failed, RadioLib error ");
  Serial.println(state);
  while (true) {
    delay(1000);
  }
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 + Core1262 MeshCore passive listener");
  Serial.println("-----------------------------------------");

  Serial.print("Frequency: ");
  Serial.print(kFrequencyMHz, 3);
  Serial.println(" MHz");
  Serial.print("Bandwidth: ");
  Serial.print(kBandwidthKHz, 1);
  Serial.println(" kHz");
  Serial.print("Spreading factor: SF");
  Serial.println(kSpreadingFactor);
  Serial.print("Coding rate: 4/");
  Serial.println(kCodingRate);
  Serial.print("Preamble: ");
  Serial.print(kPreambleLength);
  Serial.println(" symbols");

  computePublicChannelHash();
  Serial.print("MeshCore Public channel hash: 0x");
  printHexByte(Serial, publicChannelHash);
  Serial.println();

  beginLogStorage();

  radioSpi.begin(kPinSck, kPinMiso, kPinMosi, kPinCs);

  radio.tcxoVoltage = kTcxoVoltage;

  ConfigLoRa_t config;
  config.frequency = kFrequencyMHz;
  config.bandwidth = kBandwidthKHz;
  config.spreadingFactor = kSpreadingFactor;
  config.codingRate = kCodingRate;
  config.syncWord = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
  config.power = 10;
  config.preambleLength = kPreambleLength;

  Serial.print("Initializing SX1262... ");
  int state = radio.begin(config);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println("FAILED");
    haltOnError("Initialization", state);
  }
  Serial.println("SUCCESS");

  // Core1262 uses separate RXEN/TXEN pins for its onboard RF switch.
  radio.setRfSwitchPins(kPinRxEn, kPinTxEn);

  radio.setPacketReceivedAction(onPacketReceived);

  Serial.print("Starting continuous receive... ");
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println("FAILED");
    haltOnError("startReceive", state);
  }
  Serial.println("SUCCESS");
  Serial.println("Listener is passive. It will not transmit or forward packets.");
  printSerialHelp();
}

void loop() {
  handleSerialCommands();

  if (!packetReceived) {
    delay(5);
    return;
  }

  packetReceived = false;

  const size_t packetLength = radio.getPacketLength();
  if (packetLength == 0 || packetLength > kMaxRadioPacket) {
    Serial.print("Invalid received packet length: ");
    Serial.println(packetLength);
    return;
  }

  uint8_t buffer[kMaxRadioPacket] = {};
  const int state = radio.readData(buffer, packetLength);

  if (rotateLogIfNeeded() && logFile) {
    TeePrint output(Serial, logFile);
    logReceivedPacket(output, buffer, packetLength, state);
    logFile.flush();
  } else {
    logReceivedPacket(Serial, buffer, packetLength, state);
  }
}
