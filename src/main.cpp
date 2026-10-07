#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#ifndef LORA_FREQUENCY_MHZ
#define LORA_FREQUENCY_MHZ 915.0
#endif

namespace {
constexpr uint32_t kSerialBaud = 115200;

// ESP32 VSPI pins.
constexpr int kPinSck = 18;
constexpr int kPinMiso = 19;
constexpr int kPinMosi = 23;
constexpr int kPinCs = 5;

// Core1262 control pins. Keep these in sync with PINOUT.md.
constexpr int kPinReset = 21;
constexpr int kPinBusy = 2;
constexpr int kPinDio1 = 15;
constexpr int kPinRxEn = 22;
constexpr int kPinTxEn = 4;

constexpr float kFrequencyMHz = LORA_FREQUENCY_MHZ;
constexpr float kBandwidthKHz = 125.0;
constexpr uint8_t kSpreadingFactor = 9;
constexpr uint8_t kCodingRate = 7;
constexpr int8_t kTxPowerDbm = 10;
constexpr uint16_t kPreambleLength = 8;
constexpr float kTcxoVoltage = 1.7;

// Own the VSPI instance explicitly so RadioLib uses the same SPI bus and pins
// that PlatformIO/Arduino initializes below.
SPIClass radioSpi(VSPI);
SX1262 radio = new Module(
    kPinCs,
    kPinDio1,
    kPinReset,
    kPinBusy,
    radioSpi);

void haltOnError(const char* operation, int state) {
  Serial.print(operation);
  Serial.print(" failed, RadioLib error ");
  Serial.println(state);

  while (true) {
    delay(1000);
  }
}

void transmitTestPacket() {
  static uint32_t packetNumber = 1;

  String payload = "ESP32 Core1262 test #" + String(packetNumber++);

  Serial.print("Transmitting: ");
  Serial.println(payload);

  const int state = radio.transmit(payload);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Transmit SUCCESS");
  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    Serial.println("Transmit FAILED: timeout");
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    Serial.println("Transmit FAILED: packet too long");
  } else {
    Serial.print("Transmit FAILED, RadioLib error ");
    Serial.println(state);
  }
}
}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  delay(1000);

  Serial.println();
  Serial.println("ESP32-DevKit-C + Waveshare Core1262 test");
  Serial.println("----------------------------------------");
  Serial.print("Frequency: ");
  Serial.print(kFrequencyMHz, 3);
  Serial.println(" MHz");

  radioSpi.begin(kPinSck, kPinMiso, kPinMosi, kPinCs);

  // Waveshare specifies 1.7 V as the Core1262 TCXO control voltage.
  // DIO3 is wired to the TCXO internally, so it is not connected to the ESP32.
  radio.tcxoVoltage = kTcxoVoltage;

  ConfigLoRa_t config;
  config.frequency = kFrequencyMHz;
  config.bandwidth = kBandwidthKHz;
  config.spreadingFactor = kSpreadingFactor;
  config.codingRate = kCodingRate;
  config.syncWord = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
  config.power = kTxPowerDbm;
  config.preambleLength = kPreambleLength;

  Serial.print("Initializing SX1262... ");
  const int state = radio.begin(config);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.println("FAILED");
    haltOnError("Initialization", state);
  }

  Serial.println("SUCCESS");

  // Core1262 uses the separate RXEN/TXEN pins for its onboard RF switch.
  // Configure these after radio initialization, as in RadioLib's SX126x
  // examples. DIO2 is not connected in this project.
  radio.setRfSwitchPins(kPinRxEn, kPinTxEn);

  Serial.println();
  Serial.println("Radio ready.");
  Serial.println("Enter 't' in the serial monitor to transmit a test packet.");
}

void loop() {
  if (Serial.available() == 0) {
    delay(10);
    return;
  }

  const char command = static_cast<char>(Serial.read());

  if (command == 't' || command == 'T') {
    transmitTestPacket();
  }
}
