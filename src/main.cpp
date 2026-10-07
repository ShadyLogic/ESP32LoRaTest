#include <Arduino.h>
#include <AES.h>
#include <RadioLib.h>
#include <SHA256.h>
#include <SPI.h>

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

void printHexByte(uint8_t value) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  Serial.print(kHex[value >> 4]);
  Serial.print(kHex[value & 0x0F]);
}

void printHex(const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    printHexByte(data[i]);
  }
}

void printEscapedText(const uint8_t* data, size_t length) {
  Serial.print('"');
  for (size_t i = 0; i < length; ++i) {
    const uint8_t ch = data[i];
    if (ch == 0) {
      break;
    }
    if (ch == '\\' || ch == '"') {
      Serial.print('\\');
      Serial.print(static_cast<char>(ch));
    } else if (ch >= 0x20 && ch <= 0x7E) {
      Serial.print(static_cast<char>(ch));
    } else if (ch == '\n') {
      Serial.print("\\n");
    } else if (ch == '\r') {
      Serial.print("\\r");
    } else if (ch == '\t') {
      Serial.print("\\t");
    } else {
      Serial.print("\\x");
      printHexByte(ch);
    }
  }
  Serial.print('"');
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

void logPublicGroupText(const MeshPacketView& packet) {
  uint8_t plaintext[kMaxRadioPacket] = {};
  const int decryptedLength = decryptPublicChannelPayload(
      packet.payload,
      packet.payloadLength,
      plaintext,
      sizeof(plaintext));

  if (decryptedLength == 0) {
    Serial.print("channel_hash=0x");
    printHexByte(packet.payload[0]);
    Serial.println(" encrypted=unknown-channel");
    return;
  }

  if (decryptedLength < 0) {
    Serial.print("public_channel_decrypt=failed code=");
    Serial.println(decryptedLength);
    return;
  }

  if (decryptedLength < 5) {
    Serial.println("public_channel_decrypt=malformed");
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

  Serial.print("PUBLIC timestamp=");
  Serial.print(timestamp);
  Serial.print(" text_type=");
  Serial.print(textType);
  Serial.print(" attempt=");
  Serial.print(attempt);
  Serial.print(" text=");
  printEscapedText(&plaintext[5], textLength);
  Serial.println();

  if (textType == 0) {
    Serial.println(
        "note=MeshCore group sender names are unverified message text");
  }
}

void logPublicGroupData(const MeshPacketView& packet) {
  uint8_t plaintext[kMaxRadioPacket] = {};
  const int decryptedLength = decryptPublicChannelPayload(
      packet.payload,
      packet.payloadLength,
      plaintext,
      sizeof(plaintext));

  if (decryptedLength == 0) {
    Serial.print("channel_hash=0x");
    printHexByte(packet.payload[0]);
    Serial.println(" encrypted=unknown-channel");
    return;
  }

  if (decryptedLength < 3) {
    Serial.print("public_group_data_decrypt=failed code=");
    Serial.println(decryptedLength);
    return;
  }

  const uint16_t dataType = readLe16(plaintext);
  const uint8_t declaredLength = plaintext[2];
  const size_t availableLength =
      static_cast<size_t>(decryptedLength) - 3;
  const size_t dataLength =
      min(static_cast<size_t>(declaredLength), availableLength);

  Serial.print("PUBLIC_DATA type=0x");
  printHexByte(static_cast<uint8_t>(dataType >> 8));
  printHexByte(static_cast<uint8_t>(dataType & 0xFF));
  Serial.print(" len=");
  Serial.print(dataLength);
  Serial.print(" data=");
  printHex(&plaintext[3], dataLength);
  Serial.println();
}

void logAdvert(const MeshPacketView& packet) {
  // public key (32) + timestamp (4) + Ed25519 signature (64)
  constexpr size_t kAdvertFixedLength = 32 + 4 + 64;
  if (packet.payloadLength < kAdvertFixedLength) {
    Serial.println("advert=malformed");
    return;
  }

  const uint8_t* publicKey = packet.payload;
  const uint32_t timestamp = readLe32(&packet.payload[32]);
  const uint8_t* appData = &packet.payload[kAdvertFixedLength];
  const size_t appDataLength =
      packet.payloadLength - kAdvertFixedLength;

  Serial.print("ADVERT timestamp=");
  Serial.print(timestamp);
  Serial.print(" pubkey_prefix=");
  printHex(publicKey, 8);
  Serial.println(" signature=not-verified");

  if (appDataLength == 0) {
    return;
  }

  const uint8_t flags = appData[0];
  const uint8_t nodeType = flags & 0x0F;
  size_t offset = 1;

  Serial.print("advert_type=");
  Serial.print(nodeType);
  Serial.print(" flags=0x");
  printHexByte(flags);

  if (flags & 0x10) {
    if (offset + 8 > appDataLength) {
      Serial.println(" appdata=malformed");
      return;
    }
    const int32_t latitude = readLeI32(&appData[offset]);
    offset += 4;
    const int32_t longitude = readLeI32(&appData[offset]);
    offset += 4;

    Serial.print(" lat=");
    Serial.print(latitude / 1000000.0, 6);
    Serial.print(" lon=");
    Serial.print(longitude / 1000000.0, 6);
  }

  if (flags & 0x20) {
    if (offset + 2 > appDataLength) {
      Serial.println(" appdata=malformed");
      return;
    }
    Serial.print(" feature1=0x");
    printHexByte(appData[offset + 1]);
    printHexByte(appData[offset]);
    offset += 2;
  }

  if (flags & 0x40) {
    if (offset + 2 > appDataLength) {
      Serial.println(" appdata=malformed");
      return;
    }
    Serial.print(" feature2=0x");
    printHexByte(appData[offset + 1]);
    printHexByte(appData[offset]);
    offset += 2;
  }

  if ((flags & 0x80) && offset < appDataLength) {
    Serial.print(" name=");
    printEscapedText(&appData[offset], appDataLength - offset);
  }

  Serial.println();
}

void logMeshPacket(const uint8_t* data, size_t length) {
  MeshPacketView packet;
  if (!parseMeshPacket(data, length, packet)) {
    Serial.println("mesh_parse=invalid");
    return;
  }

  Serial.print("mesh_version=");
  Serial.print(packet.payloadVersion + 1);
  Serial.print(" route=");
  Serial.print(routeName(packet.routeType));
  Serial.print(" payload=");
  Serial.print(payloadName(packet.payloadType));
  Serial.print("(");
  Serial.print(packet.payloadType);
  Serial.print(")");
  Serial.print(" hops=");
  Serial.print(packet.pathHashCount);
  Serial.print(" path_hash_bytes=");
  Serial.print(packet.pathHashSize);

  if (packet.hasTransportCodes) {
    Serial.print(" transport=0x");
    printHexByte(static_cast<uint8_t>(packet.transportCode1 >> 8));
    printHexByte(static_cast<uint8_t>(packet.transportCode1 & 0xFF));
    Serial.print(",0x");
    printHexByte(static_cast<uint8_t>(packet.transportCode2 >> 8));
    printHexByte(static_cast<uint8_t>(packet.transportCode2 & 0xFF));
  }

  Serial.println();

  if (packet.pathBytes > 0) {
    Serial.print("path=");
    printHex(packet.path, packet.pathBytes);
    Serial.println();
  }

  switch (packet.payloadType) {
    case kPayloadGroupText:
      logPublicGroupText(packet);
      break;

    case kPayloadGroupData:
      logPublicGroupData(packet);
      break;

    case kPayloadAdvert:
      logAdvert(packet);
      break;

    case kPayloadText:
    case kPayloadReq:
    case kPayloadResponse:
    case kPayloadAnonReq:
    case kPayloadPath:
      Serial.println(
          "content=encrypted-private-or-peer-specific");
      break;

    case kPayloadControl:
      if (packet.payloadLength > 0) {
        Serial.print("control_subtype=0x");
        printHexByte(packet.payload[0] >> 4);
        Serial.println();
      }
      break;

    default:
      break;
  }
}

void logReceivedPacket(uint8_t* data, size_t length, int state) {
  ++packetCounter;

  const float rssi = radio.getRSSI();
  const float snr = radio.getSNR();
  const float frequencyError = radio.getFrequencyError();

  uint8_t rxCodingRate = 0;
  bool rxHasCrc = false;
  const int headerState =
      radio.getLoRaRxHeaderInfo(&rxCodingRate, &rxHasCrc);

  Serial.println();
  Serial.print("=== RX #");
  Serial.print(packetCounter);
  Serial.println(" ===");

  Serial.print("millis=");
  Serial.print(millis());
  Serial.print(" len=");
  Serial.print(length);
  Serial.print(" rssi_dbm=");
  Serial.print(rssi, 1);
  Serial.print(" snr_db=");
  Serial.print(snr, 2);
  Serial.print(" freq_error_hz=");
  Serial.print(frequencyError, 1);

  if (headerState == RADIOLIB_ERR_NONE) {
    Serial.print(" rx_cr=");
    if (rxCodingRate >= 1 && rxCodingRate <= 4) {
      Serial.print("4/");
      Serial.print(rxCodingRate + 4);
    } else {
      Serial.print("raw:");
      Serial.print(rxCodingRate);
    }
    Serial.print(" crc=");
    Serial.print(rxHasCrc ? "yes" : "no");
  }
  Serial.println();

  Serial.print("raw=");
  printHex(data, length);
  Serial.println();

  if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("radio_crc=FAILED mesh_parse=skipped");
    return;
  }

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("radio_read_error=");
    Serial.println(state);
    return;
  }

  logMeshPacket(data, length);
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
  printHexByte(publicChannelHash);
  Serial.println();

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
}

void loop() {
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
  logReceivedPacket(buffer, packetLength, state);
}
