/*
================================================================================
           ROADNET ESP32 - UNIVERSAL BLUETOOTH (BLE & CLASSIC SPP)
================================================================================
 Boards Supported:
   - NodeMCU ESP32 / ESP32-WROOM-32 / ESP32-WROVER (BLE + Bluetooth Classic SPP)
   - ESP32-C3 / ESP32-C5 / ESP32-S3 (BLE)

 Hardware:
   - Bluetooth: BLE GATT Notifications + Bluetooth Classic SPP
     * Pairing / Advertising Name: "RoadNet-ESP32"
     * BLE Service UUID:        0000ffe0-0000-1000-8000-00805f9b34fb
     * BLE Characteristic UUID: 0000ffe1-0000-1000-8000-00805f9b34fb
   - LoRa:  SX1278 / RA-02 (433 MHz SPI)
   - IMU:   MPU6050 (I2C: SDA=21, SCL=22)
   - GPS:   NEO-6M / NEO-M8N (UART: RX=16, TX=17)
================================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>

// BLE Core Headers (Works on ALL ESP32 chips)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Bluetooth Classic SPP (Only on classic ESP32 WROOM/WROVER)
#if defined(CONFIG_BT_SPP_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
  #include <BluetoothSerial.h>
  #define HAS_BT_CLASSIC 1
  BluetoothSerial SerialBT;
#else
  #define HAS_BT_CLASSIC 0
#endif

// Chip Architecture Auto-Detection (ESP32-C5 / C3 / C6 vs Classic ESP32)
#if defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32C2) || defined(BOARD_C5) || defined(ARDUINO_ESP32C5)
  #define IS_ESP32_C_SERIES 1
#else
  #define IS_ESP32_C_SERIES 0
#endif

// ============================================================
//                    NODE & BLUETOOTH CONFIG
// ============================================================
#define NODE_ID 2
#if IS_ESP32_C_SERIES
constexpr char BT_DEVICE_NAME[] = "RoadMesh-C5";
#else
constexpr char BT_DEVICE_NAME[] = "RoadMesh-ESP32";
#endif

#define SERVICE_UUID        "7c2a0001-6b4b-4c9a-9a01-726f61646d01"
#define TELEMETRY_CHAR_UUID "7c2a0002-6b4b-4c9a-9a01-726f61646d01"
#define COMMAND_CHAR_UUID   "7c2a0003-6b4b-4c9a-9a01-726f61646d01"

BLECharacteristic *pTelemetryChar = nullptr;
BLECharacteristic *pCommandChar = nullptr;
bool bleConnected = false;

// ============================================================
//                    PIN CONFIGURATION
// ============================================================
#if IS_ESP32_C_SERIES
  // ESP32-C5 / ESP32-C3 Pin Configuration
  #define MPU_SDA    4
  #define MPU_SCL    5
  
  // GPS UART1 Pins (Wiring: GPS Module TX -> ESP32 GPS_RX, GPS Module RX -> ESP32 GPS_TX)
  // Change GPS_RX / GPS_TX here to match your exact board pins if different (e.g. 24/23, 20/21, 2/3, 8/9)
  #define GPS_RX     24
  #define GPS_TX     23
  #define GPS_BAUD   115200   // ← Your NEO-6M is configured for 115200 baud

  #define LORA_SCK   7
  #define LORA_MISO  8
  #define LORA_MOSI  9
  #define LORA_SS    10
  #define LORA_RST   0
  #define LORA_DIO0  1
#else
  // NodeMCU ESP32 / WROOM-32 Pin Configuration
  #define MPU_SDA    21
  #define MPU_SCL    22
  #define GPS_RX     16
  #define GPS_TX     17
  #define GPS_BAUD   115200   // ← Your NEO-6M is configured for 115200 baud
  #define LORA_SCK   18
  #define LORA_MISO  19
  #define LORA_MOSI  23
  #define LORA_SS    5
  #define LORA_RST   14
  #define LORA_DIO0  26
#endif

#define AIRBAG_PIN -1

// ============================================================
//                    LORA CONFIGURATION
// ============================================================
#define LORA_FREQUENCY 433E6
#define LORA_SF        7
#define LORA_BW        125E3
#define LORA_CR        5
#define LORA_POWER     17

#define MESH_TTL             3
#define MAX_NODES            20
#define NODE_TIMEOUT         10000
#define HEARTBEAT_INTERVAL   3000
#define PACKET_HISTORY_SIZE  50

// ============================================================
//                    ACCIDENT DETECTION CONFIG
// ============================================================
#define ACCIDENT_THRESHOLD   50
#define ACCIDENT_COOLDOWN    15000

// ============================================================
//                    STATE & VARIABLES
// ============================================================
TinyGPSPlus gps;
#if IS_ESP32_C_SERIES
HardwareSerial GPSSerial(1); // ESP32-C5 only has UART0 and UART1 (No UART2)
#else
HardwareSerial GPSSerial(2); // Classic ESP32 has UART2
#endif

#define MPU_ADDRESS 0x68
bool mpuOK = false;
bool loraOK = false;

float ax_mps2 = 0, ay_mps2 = 0, az_mps2 = 0;
float ax_g = 0, ay_g = 0, az_g = 1.0;
float gx = 0, gy = 0, gz = 0;

float gravityX = 0, gravityY = 0, gravityZ = 0;
float dynamicAcceleration = 0;
float gyroMagnitude = 0;

bool gpsFix = false;
double latitude = 0.0;
double longitude = 0.0;
float altitudeMeters = 0.0;
float speedKmh = 0.0;
uint32_t satellites = 0;

float crashConfidence = 0;
bool localAccident = false;
bool remoteAccident = false;
uint16_t remoteNode = 0;
float remoteConfidence = 0;
unsigned long remoteAlertTime = 0;
unsigned long lastAccidentTime = 0;

uint32_t packetsTX = 0;
uint32_t packetsRX = 0;
uint32_t packetsForwarded = 0;
uint32_t packetsDropped = 0;
int lastRSSI = 0;
float lastSNR = 0;
String lastTX = "None";
String lastRX = "None";

struct NodeInfo {
  uint16_t id;
  unsigned long lastSeen;
  int rssi;
};
NodeInfo nodes[MAX_NODES];

struct PacketHistory {
  uint16_t node;
  uint32_t sequence;
};
PacketHistory packetHistory[PACKET_HISTORY_SIZE];
int historyIndex = 0;
uint32_t sequenceNumber = 1;

unsigned long lastSensorRead = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastTelemetrySent = 0;
constexpr unsigned long TELEMETRY_INTERVAL_MS = 50; // 20 Hz

// ============================================================
//                    BLE SERVER CALLBACKS
// ============================================================
class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    Serial.println(F("📱 Web Bluetooth client CONNECTED!"));
  }
  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    Serial.println(F("📱 Web Bluetooth client DISCONNECTED. Advertising restarted."));
    delay(200);
    BLEDevice::startAdvertising();
  }
};

class BleCommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String value = pChar->getValue().c_str();
    value.trim();
    if (value.length() > 0) {
      Serial.printf("📥 BLE Command received: '%s'\n", value.c_str());
      if (value == "PING") {
        Serial.println(F("🏓 PING acknowledged over BLE"));
      } else if (value == "RESET_ACCIDENT") {
        localAccident = false;
        remoteAccident = false;
        crashConfidence = 0;
        lastAccidentTime = 0;
        Serial.println(F("🔄 ACCIDENT STATE RESET via BLE command"));
      }
    }
  }
};

void initBluetooth() {
  Serial.println(F("Starting Bluetooth service..."));

  // 1. BLE GATT Server (Supported on ALL ESP32 chips)
  BLEDevice::init(BT_DEVICE_NAME);
  BLEDevice::setMTU(256);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BleServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Telemetry Characteristic (Notify to Browser)
  pTelemetryChar = pService->createCharacteristic(
    TELEMETRY_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTelemetryChar->addDescriptor(new BLE2902());

  // Command Characteristic (Write from Browser)
  pCommandChar = pService->createCharacteristic(
    COMMAND_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCommandChar->setCallbacks(new BleCommandCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.printf("✅ BLE GATT Advertising active as '%s'\n", BT_DEVICE_NAME);

  // 2. Bluetooth Classic SPP (if supported on ESP32 WROOM/WROVER)
  #if HAS_BT_CLASSIC
  if (SerialBT.begin(BT_DEVICE_NAME)) {
    Serial.printf("✅ Bluetooth Classic SPP active as '%s'\n", BT_DEVICE_NAME);
  }
  #endif
}

// ============================================================
//                    MPU6050 FUNCTIONS
// ============================================================
void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool initMPU() {
  Wire.begin(MPU_SDA, MPU_SCL);
  delay(50);

  Wire.beginTransmission(MPU_ADDRESS);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println(F("⚠️ MPU6050 not detected on I2C (check SDA/SCL wiring)"));
    return false;
  }

  mpuWrite(0x6B, 0x00); // Wake up
  mpuWrite(0x1A, 0x03); // Digital Low Pass Filter (DLPF) ~42Hz to suppress touch noise
  mpuWrite(0x1C, 0x10); // Accel Range: ±8g (4096 LSB/g)
  mpuWrite(0x1B, 0x08); // Gyro Range: ±500°/s (65.5 LSB/(°/s))
  delay(50);

  // Initialize gravity estimate to rest position
  gravityX = 0.0f;
  gravityY = 0.0f;
  gravityZ = 1.0f;

  Serial.println(F("✅ MPU6050 OK (±8g, ±500°/s, 42Hz DLPF Filter Active)"));
  return true;
}

void readMPU() {
  if (!mpuOK) return;

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;

  Wire.requestFrom(MPU_ADDRESS, 14);
  if (Wire.available() < 14) return;

  int16_t rawAx = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rawAy = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rawAz = ((int16_t)Wire.read() << 8) | Wire.read();

  Wire.read(); Wire.read(); // Temperature

  int16_t rawGx = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rawGy = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rawGz = ((int16_t)Wire.read() << 8) | Wire.read();

  // Scale raw values (±8g -> 4096 LSB/g, ±500°/s -> 65.5 LSB/(°/s))
  ax_g = rawAx / 4096.0f;
  ay_g = rawAy / 4096.0f;
  az_g = rawAz / 4096.0f;

  ax_mps2 = ax_g * 9.80665f;
  ay_mps2 = ay_g * 9.80665f;
  az_mps2 = az_g * 9.80665f;

  gx = rawGx / 65.5f;
  gy = rawGy / 65.5f;
  gz = rawGz / 65.5f;

  // Exponential moving average filter for gravity vector tracking
  const float alpha = 0.95f;
  gravityX = alpha * gravityX + (1.0f - alpha) * ax_g;
  gravityY = alpha * gravityY + (1.0f - alpha) * ay_g;
  gravityZ = alpha * gravityZ + (1.0f - alpha) * az_g;

  // Dynamic acceleration magnitude (excluding stationary 1g gravity)
  float dx = ax_g - gravityX;
  float dy = ay_g - gravityY;
  float dz = az_g - gravityZ;

  float dynG = sqrtf(dx * dx + dy * dy + dz * dz);
  // Deadband noise gate: filter subtle resting jitter below 0.15g
  if (dynG < 0.15f) {
    dynG = 0.0f;
  }
  dynamicAcceleration = dynG;

  float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
  // Deadband noise gate: filter resting gyro drift below 8°/s
  if (gyroMag < 8.0f) {
    gyroMag = 0.0f;
  }
  gyroMagnitude = gyroMag;
}

// ============================================================
//                    GPS FUNCTIONS
// ============================================================
void readGPS() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  if (gps.location.isValid()) {
    gpsFix = true;
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  } else {
    gpsFix = false;
  }

  if (gps.altitude.isValid()) {
    altitudeMeters = gps.altitude.meters();
  }

  if (gps.speed.isValid() && gpsFix) {
    speedKmh = gps.speed.kmph();
  } else {
    speedKmh = 0.0;
  }

  if (gps.satellites.isValid()) {
    satellites = gps.satellites.value();
  }
}

// ============================================================
//                    MESH DISCOVERY & ROUTING
// ============================================================
void registerNode(uint16_t id, int rssi) {
  if (id == NODE_ID) return;

  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].id == id) {
      nodes[i].lastSeen = millis();
      nodes[i].rssi = rssi;
      return;
    }
  }

  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].id == 0) {
      nodes[i].id = id;
      nodes[i].lastSeen = millis();
      nodes[i].rssi = rssi;
      return;
    }
  }
}

int activeNodeCount() {
  int count = 1;
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].id != 0 && (millis() - nodes[i].lastSeen < NODE_TIMEOUT)) {
      count++;
    }
  }
  return count;
}

bool packetSeen(uint16_t node, uint32_t sequence) {
  for (int i = 0; i < PACKET_HISTORY_SIZE; i++) {
    if (packetHistory[i].node == node && packetHistory[i].sequence == sequence) {
      return true;
    }
  }
  return false;
}

void rememberPacket(uint16_t node, uint32_t sequence) {
  packetHistory[historyIndex].node = node;
  packetHistory[historyIndex].sequence = sequence;
  historyIndex = (historyIndex + 1) % PACKET_HISTORY_SIZE;
}

// ============================================================
//                    LORA FUNCTIONS
// ============================================================
void sendLoRa(const String& packet) {
  if (!loraOK) return; // Prevent blocking/hanging when LoRa is not connected
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print(packet);
  int result = LoRa.endPacket();

  if (result == 1) {
    packetsTX++;
    lastTX = packet;
  }
  LoRa.receive();
}

String createAccidentPacket() {
  uint32_t sequence = sequenceNumber++;
  rememberPacket(NODE_ID, sequence);

  String packet = "RM|A|";
  packet += String(NODE_ID);
  packet += "|";
  packet += String(sequence);
  packet += "|";
  packet += String(MESH_TTL);
  packet += "|";
  packet += String(latitude, 6);
  packet += "|";
  packet += String(longitude, 6);
  packet += "|";
  packet += String(crashConfidence, 0);
  packet += "|";
  packet += String(dynamicAcceleration, 2);
  return packet;
}

void sendHeartbeat() {
  if (!loraOK) return;
  String packet = "RM|H|" + String(NODE_ID) + "|" + String(millis());
  sendLoRa(packet);
}

void receiveLoRa() {
  if (!loraOK) return; // Prevent blocking when LoRa is not connected
  int packetSize = LoRa.parsePacket();
  if (packetSize <= 0) return;

  String packet = "";
  while (LoRa.available()) {
    packet += (char)LoRa.read();
  }

  packetsRX++;
  lastRSSI = LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
  lastRX = packet;

  if (packet.startsWith("RM|H|")) {
    int separator = packet.indexOf('|', 5);
    if (separator > 0) {
      uint16_t id = packet.substring(5, separator).toInt();
      registerNode(id, lastRSSI);
    }
    return;
  }

  if (!packet.startsWith("RM|A|")) {
    packetsDropped++;
    return;
  }

  int p1 = packet.indexOf('|', 5);
  int p2 = packet.indexOf('|', p1 + 1);
  int p3 = packet.indexOf('|', p2 + 1);
  if (p1 < 0 || p2 < 0 || p3 < 0) {
    packetsDropped++;
    return;
  }

  uint16_t origin = packet.substring(5, p1).toInt();
  uint32_t sequence = packet.substring(p1 + 1, p2).toInt();
  int ttl = packet.substring(p2 + 1, p3).toInt();

  if (packetSeen(origin, sequence)) {
    packetsDropped++;
    return;
  }

  rememberPacket(origin, sequence);
  registerNode(origin, lastRSSI);

  int last = packet.lastIndexOf('|');
  int secondLast = packet.lastIndexOf('|', last - 1);
  if (secondLast >= 0) {
    remoteConfidence = packet.substring(secondLast + 1, last).toFloat();
  }

  remoteAccident = true;
  remoteNode = origin;
  remoteAlertTime = millis();

  if (ttl > 0) {
    String forwarded = packet;
    String ttlString = "|" + String(ttl) + "|";
    String newTTL = "|" + String(ttl - 1) + "|";
    int pos = forwarded.indexOf(ttlString);
    if (pos >= 0) {
      forwarded.replace(ttlString, newTTL);
      delay(random(50, 250));
      sendLoRa(forwarded);
      packetsForwarded++;
    }
  }
}

// ============================================================
//                    ACCIDENT DETECTION (Responsive & Calibrated)
// ============================================================
void detectAccident() {
  float score = 0;

  // 1. G-Force Impact Assessment (in g units)
  // Gentle touch / rest: < 0.3g -> score 0
  // Brisk movement: 0.8g - 1.4g -> score 15
  // Firm tap / shake: 1.4g - 2.2g -> score 35-50
  // Sudden Impact: >= 2.2g -> score 65+
  if (dynamicAcceleration >= 2.2f) {
    score += 65; // Sudden impact / firm tap (> 2.2g)
  } else if (dynamicAcceleration >= 1.4f) {
    score += 35; // Moderate jolt (1.4g - 2.2g)
  } else if (dynamicAcceleration >= 0.8f) {
    score += 15; // Brisk movement (0.8g - 1.4g)
  }

  // 2. High Angular Velocity (Rotation / Flip in °/s)
  if (gyroMagnitude >= 150.0f) {
    score += 35; // Fast rotation / twist (> 150°/s)
  } else if (gyroMagnitude >= 80.0f) {
    score += 20; // Moderate rotation (80°/s - 150°/s)
  }

  // 3. Combined Simultaneous Shock + Quick Rotation
  if (dynamicAcceleration >= 1.2f && gyroMagnitude >= 50.0f) {
    score += 25;
  }

  // 4. Physical Airbag Sensor Deployment
  const bool airbagDeployed = (AIRBAG_PIN >= 0) && (digitalRead(AIRBAG_PIN) == HIGH);
  if (airbagDeployed) {
    score = 100;
  }

  if (score > 100) score = 100;
  crashConfidence = score;

  // Trigger accident when score reaches threshold (>= 50%)
  if (crashConfidence >= ACCIDENT_THRESHOLD && (millis() - lastAccidentTime > ACCIDENT_COOLDOWN)) {
    localAccident = true;
    lastAccidentTime = millis();
    String packet = createAccidentPacket();
    sendLoRa(packet);
  } else if (millis() - lastAccidentTime > 5000) {
    localAccident = false;
  }
}

// ============================================================
//                    SEND BLUETOOTH TELEMETRY
// ============================================================
void sendBluetoothTelemetry() {
  const bool airbagDeployed = (AIRBAG_PIN >= 0) && (digitalRead(AIRBAG_PIN) == HIGH);
  const int activeNodes = activeNodeCount();
  const bool crash = localAccident || remoteAccident || (crashConfidence >= ACCIDENT_THRESHOLD);

  char payload[512];
  snprintf(payload, sizeof(payload),
    "{\"node\":\"%s\",\"seq\":%lu,\"fix\":%d,\"lat\":%.6f,\"lon\":%.6f,\"sat\":%u,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"acc\":%.2f,"
    "\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,\"gyro\":%.1f,"
    "\"accident\":%d,\"confidence\":%d,\"nodes\":%d,"
    "\"airbag\":\"%s\",\"speed\":%.1f}\n",
    BT_DEVICE_NAME, sequenceNumber++, gpsFix ? 1 : 0, latitude, longitude, satellites,
    ax_g, ay_g, az_g, dynamicAcceleration,
    gx, gy, gz, gyroMagnitude,
    crash ? 1 : 0, (int)crashConfidence, activeNodes,
    airbagDeployed ? "DEPLOYED" : "ARMED", speedKmh
  );

  // 1. Send via BLE GATT Notification (Direct to Web Browser)
  if (bleConnected && pTelemetryChar != nullptr) {
    pTelemetryChar->setValue((uint8_t*)payload, strlen(payload));
    pTelemetryChar->notify();
  }

  // 2. Send over Bluetooth Classic SPP (if paired via COM Port)
  #if HAS_BT_CLASSIC
  if (SerialBT.hasClient()) {
    SerialBT.print(payload);
  }
  #endif

  // 3. Print to USB Serial (for debugging)
  Serial.print(payload);
}

// ============================================================
//                    SETUP LORA
// ============================================================
bool setupLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println(F("⚠️ LoRa SX1278 not detected (check SPI wiring)"));
    return false;
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(LORA_POWER);
  LoRa.enableCrc();
  LoRa.receive();

  Serial.println(F("✅ LoRa 433MHz Ready"));
  return true;
}

// ============================================================
//                    SETUP & MAIN LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println(F("\n=========================================="));
  Serial.println(F("   ROADNET ESP32 - BLUETOOTH TELEMETRY"));
  Serial.printf( "   BLUETOOTH BROADCAST NAME: %s\n", BT_DEVICE_NAME);
  Serial.println(F("=========================================="));

  // 1. START BLUETOOTH IMMEDIATELY FIRST (Never blocks)
  initBluetooth();

  if (AIRBAG_PIN >= 0) {
    pinMode(AIRBAG_PIN, INPUT_PULLDOWN);
  }

  // 2. Initialize Sensors (non-blocking)
  mpuOK = initMPU();
  if (mpuOK) {
    // Settle gravity filter on startup
    for (int i = 0; i < 30; i++) {
      readMPU();
      delay(5);
    }
    dynamicAcceleration = 0.0f;
    crashConfidence = 0.0f;
    localAccident = false;
  }
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  loraOK = setupLoRa();

  Serial.println(F(">> Ready for Bluetooth connection.\n"));
}

void loop() {
  // Read GPS
  readGPS();

  // Read MPU6050 & evaluate accident conditions (50 Hz)
  if (millis() - lastSensorRead >= 20) {
    lastSensorRead = millis();
    readMPU();
    detectAccident();
  }

  // Send Bluetooth Telemetry (20 Hz)
  if (millis() - lastTelemetrySent >= TELEMETRY_INTERVAL_MS) {
    lastTelemetrySent = millis();
    sendBluetoothTelemetry();
  }

  // LoRa Mesh: Receive & Heartbeat
  receiveLoRa();
  if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = millis();
    sendHeartbeat();
  }

  // Remote alert timeout (30 seconds)
  if (remoteAccident && (millis() - remoteAlertTime > 30000)) {
    remoteAccident = false;
  }

  // Periodic GPS Diagnostic Monitor (every 4 seconds to Serial Monitor)
  static unsigned long lastGpsDiag = 0;
  if (millis() - lastGpsDiag >= 4000) {
    lastGpsDiag = millis();
    unsigned long chars = gps.charsProcessed();
    if (chars == 0) {
      Serial.printf("⚠️ [GPS Status] 0 bytes received! (Check wiring: GPS TX -> ESP32 RX Pin %d, GPS RX -> ESP32 TX Pin %d at %d baud)\n", GPS_RX, GPS_TX, GPS_BAUD);
    } else if (!gpsFix) {
      Serial.printf("📡 [GPS Status] UART receiving NMEA data (%lu chars), searching for satellite lock... (Sats: %u)\n", chars, (unsigned int)satellites);
    } else {
      Serial.printf("🛰️ [GPS Status] 3D FIX ACTIVE! Lat: %.6f, Lon: %.6f, Sats: %u, Speed: %.1f km/h\n", latitude, longitude, (unsigned int)satellites, speedKmh);
    }
  }

  delay(1);
}
