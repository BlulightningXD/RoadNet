/*
============================================================
                    ROAD MESH FINAL
============================================================

Boards:
    - NodeMCU ESP32
    - ESP32-C5

Communication:
    - SX1278 / RA-02 LoRa 433 MHz
    - BLE optional transport

Sensors:
    - MPU6050
    - GPS

WiFi:
    - RoadMesh Access Point
    - Captive portal
    - Live web dashboard

Features:
    - Local accident detection
    - Remote accident detection
    - LoRa node discovery
    - LoRa packet forwarding
    - TTL
    - Duplicate filtering
    - BLE accident broadcast
    - BLE accident reception
    - GPS
    - MPU6050
    - Live dashboard
    - Transport selection

IMPORTANT:
    This is a demonstration/research system.
    The accident threshold is deliberately sensitive.
    It is NOT a production crash-detection algorithm.
============================================================
*/


// ============================================================
//                    BOARD CONFIGURATION
// ============================================================

// ESP32-C5:
// #define BOARD_C5
// #define NODE_ID 1

// NodeMCU ESP32:
// #define NODE_ID 2

//#define BOARD_C5
#define NODE_ID 2


// ============================================================
//                    LIBRARIES
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

#include <LoRa.h>
#include <TinyGPSPlus.h>

// IMPORTANT:
// This is the BLE library from the ESP32 Arduino core.
// DO NOT install/use NimBLE-Arduino separately.

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEAdvertising.h>


// ============================================================
//                    WIFI
// ============================================================

const char* WIFI_NAME = "RoadMesh2";
const char* WIFI_PASSWORD = "roadmesh123";

WebServer server(80);

DNSServer dnsServer;

const byte DNS_PORT = 53;


// ============================================================
//                    TRANSPORT
// ============================================================

enum TransportMode {

  MODE_LORA,

  MODE_BLE
};

TransportMode transport =
  MODE_LORA;


// ============================================================
//                    PIN CONFIGURATION
// ============================================================

#ifdef BOARD_C5

// ------------------------------------------------------------
// ESP32-C5
// ------------------------------------------------------------

#define MPU_SDA 4
#define MPU_SCL 5

#define GPS_RX 24
#define GPS_TX 23

#define LORA_SCK  7
#define LORA_MISO 8
#define LORA_MOSI 9
#define LORA_SS   10
#define LORA_RST  0
#define LORA_DIO0 1

#else

// ------------------------------------------------------------
// NodeMCU ESP32
// ------------------------------------------------------------

#define MPU_SDA 21
#define MPU_SCL 22

#define GPS_RX 16
#define GPS_TX 17

#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS   5
#define LORA_RST 14
#define LORA_DIO0 26

#endif


// ============================================================
//                    LORA CONFIGURATION
// ============================================================

#define LORA_FREQUENCY 433E6

#define LORA_SF 7

#define LORA_BW 125E3

#define LORA_CR 5

#define LORA_POWER 17


// ============================================================
//                    MESH CONFIGURATION
// ============================================================

#define MESH_TTL 3

#define MAX_NODES 20

#define NODE_TIMEOUT 10000

#define HEARTBEAT_INTERVAL 3000

#define PACKET_HISTORY_SIZE 50


// ============================================================
//                    ACCIDENT CONFIG
// ============================================================

// Sensitive demo mode.

#define ACCIDENT_THRESHOLD 50

#define ACCIDENT_COOLDOWN 15000


// ============================================================
//                    MPU6050
// ============================================================

#define MPU_ADDRESS 0x68

bool mpuOK = false;

float ax = 0;
float ay = 0;
float az = 0;

float gx = 0;
float gy = 0;
float gz = 0;

float gravityX = 0;
float gravityY = 0;
float gravityZ = 0;

float dynamicAcceleration = 0;

float gyroMagnitude = 0;


// ============================================================
//                    GPS
// ============================================================

HardwareSerial GPSSerial(1);

TinyGPSPlus gps;

bool gpsFix = false;

double latitude = 0;

double longitude = 0;

uint32_t satellites = 0;


// ============================================================
//                    ACCIDENT STATE
// ============================================================

float crashConfidence = 0;

bool localAccident = false;

bool remoteAccident = false;

uint16_t remoteNode = 0;

float remoteConfidence = 0;

unsigned long remoteAlertTime = 0;

unsigned long lastAccidentTime = 0;


// ============================================================
//                    NETWORK STATISTICS
// ============================================================

uint32_t packetsTX = 0;

uint32_t packetsRX = 0;

uint32_t packetsForwarded = 0;

uint32_t packetsDropped = 0;

int lastRSSI = 0;

float lastSNR = 0;

String lastTX = "None";

String lastRX = "None";


// ============================================================
//                    NODE TABLE
// ============================================================

struct NodeInfo {

  uint16_t id;

  unsigned long lastSeen;

  int rssi;
};

NodeInfo nodes[MAX_NODES];


// ============================================================
//                    PACKET HISTORY
// ============================================================

struct PacketHistory {

  uint16_t node;

  uint32_t sequence;
};

PacketHistory packetHistory[
  PACKET_HISTORY_SIZE
];

int historyIndex = 0;


// ============================================================
//                    SEQUENCE
// ============================================================

uint32_t sequenceNumber = 1;


// ============================================================
//                    TIMERS
// ============================================================

unsigned long lastSensorRead = 0;

unsigned long lastHeartbeat = 0;

unsigned long lastBLEScan = 0;


// ============================================================
//                    BLE
// ============================================================

BLEScan* bleScan = nullptr;

BLEAdvertising* bleAdvertising = nullptr;


// ============================================================
//                    MPU I2C WRITE
// ============================================================

void mpuWrite(
  uint8_t reg,
  uint8_t value
) {

  Wire.beginTransmission(
    MPU_ADDRESS
  );

  Wire.write(reg);

  Wire.write(value);

  Wire.endTransmission();
}


// ============================================================
//                    MPU INIT
// ============================================================

bool initMPU() {

  Wire.begin(
    MPU_SDA,
    MPU_SCL
  );

  delay(100);


  Wire.beginTransmission(
    MPU_ADDRESS
  );

  byte error =
    Wire.endTransmission();


  if (error != 0) {

    Serial.println(
      "MPU6050 NOT FOUND"
    );

    return false;
  }


  mpuWrite(
    0x6B,
    0x00
  );


  // Accelerometer ±2g

  mpuWrite(
    0x1C,
    0x00
  );


  // Gyroscope ±250°/s

  mpuWrite(
    0x1B,
    0x00
  );


  delay(100);


  Serial.println(
    "MPU6050 OK"
  );


  return true;
}


// ============================================================
//                    MPU READ
// ============================================================

void readMPU() {

  if (!mpuOK) {

    return;
  }


  Wire.beginTransmission(
    MPU_ADDRESS
  );

  Wire.write(
    0x3B
  );

  if (
    Wire.endTransmission(false)
    != 0
  ) {

    return;
  }


  Wire.requestFrom(
    MPU_ADDRESS,
    14
  );


  if (
    Wire.available()
    < 14
  ) {

    return;
  }


  int16_t rawAx =
    ((int16_t)Wire.read() << 8)
    |
    Wire.read();


  int16_t rawAy =
    ((int16_t)Wire.read() << 8)
    |
    Wire.read();


  int16_t rawAz =
    ((int16_t)Wire.read() << 8)
    |
    Wire.read();


  // Temperature

  Wire.read();
  Wire.read();


  int16_t rawGx =
    ((int16_t)Wire.read() << 8)
    |
    Wire.read();


  int16_t rawGy =
    ((int16_t)Wire.read() << 8)
    |
    Wire.read();


  int16_t rawGz =
    ((int16_t)Wire.read() << 8)
    |
    Wire.read();


  // ----------------------------------------------------------
  // ACCELERATION
  // ----------------------------------------------------------

  ax =
    rawAx / 16384.0 *
    9.80665;


  ay =
    rawAy / 16384.0 *
    9.80665;


  az =
    rawAz / 16384.0 *
    9.80665;


  // ----------------------------------------------------------
  // GYROSCOPE
  // ----------------------------------------------------------

  gx =
    rawGx / 131.0;


  gy =
    rawGy / 131.0;


  gz =
    rawGz / 131.0;


  // ----------------------------------------------------------
  // REMOVE GRAVITY
  // ----------------------------------------------------------

  const float alpha =
    0.98;


  gravityX =
    alpha * gravityX
    +
    (1.0 - alpha) * ax;


  gravityY =
    alpha * gravityY
    +
    (1.0 - alpha) * ay;


  gravityZ =
    alpha * gravityZ
    +
    (1.0 - alpha) * az;


  float dx =
    ax - gravityX;


  float dy =
    ay - gravityY;


  float dz =
    az - gravityZ;


  dynamicAcceleration =
    sqrt(
      dx * dx +
      dy * dy +
      dz * dz
    );


  gyroMagnitude =
    sqrt(
      gx * gx +
      gy * gy +
      gz * gz
    );
}


// ============================================================
//                    GPS
// ============================================================

void readGPS() {

  while (
    GPSSerial.available()
  ) {

    gps.encode(
      GPSSerial.read()
    );
  }


  if (
    gps.location.isValid()
  ) {

    gpsFix = true;

    latitude =
      gps.location.lat();

    longitude =
      gps.location.lng();

  }
  else {

    gpsFix = false;
  }


  if (
    gps.satellites.isValid()
  ) {

    satellites =
      gps.satellites.value();
  }
}


// ============================================================
//                    NODE REGISTER
// ============================================================

void registerNode(
  uint16_t id,
  int rssi
) {

  if (
    id == NODE_ID
  ) {

    return;
  }


  // Existing node

  for (
    int i = 0;
    i < MAX_NODES;
    i++
  ) {

    if (
      nodes[i].id == id
    ) {

      nodes[i].lastSeen =
        millis();

      nodes[i].rssi =
        rssi;

      return;
    }
  }


  // New node

  for (
    int i = 0;
    i < MAX_NODES;
    i++
  ) {

    if (
      nodes[i].id == 0
    ) {

      nodes[i].id =
        id;

      nodes[i].lastSeen =
        millis();

      nodes[i].rssi =
        rssi;


      Serial.print(
        "NEW NODE: "
      );

      Serial.println(
        id
      );


      return;
    }
  }
}


// ============================================================
//                    ACTIVE NODES
// ============================================================

int activeNodeCount() {

  int count = 1;


  for (
    int i = 0;
    i < MAX_NODES;
    i++
  ) {

    if (
      nodes[i].id != 0
      &&
      millis() -
      nodes[i].lastSeen
      <
      NODE_TIMEOUT
    ) {

      count++;
    }
  }


  return count;
}


// ============================================================
//                    PACKET HISTORY
// ============================================================

bool packetSeen(
  uint16_t node,
  uint32_t sequence
) {

  for (
    int i = 0;
    i < PACKET_HISTORY_SIZE;
    i++
  ) {

    if (
      packetHistory[i].node
      ==
      node
      &&
      packetHistory[i].sequence
      ==
      sequence
    ) {

      return true;
    }
  }


  return false;
}


// ============================================================
//                    REMEMBER PACKET
// ============================================================

void rememberPacket(
  uint16_t node,
  uint32_t sequence
) {

  packetHistory[
    historyIndex
  ].node =
    node;


  packetHistory[
    historyIndex
  ].sequence =
    sequence;


  historyIndex++;


  if (
    historyIndex
    >=
    PACKET_HISTORY_SIZE
  ) {

    historyIndex = 0;
  }
}


// ============================================================
//                    LORA SEND
// ============================================================

void sendLoRa(
  const String& packet
) {

  LoRa.idle();


  LoRa.beginPacket();

  LoRa.print(
    packet
  );

  int result =
    LoRa.endPacket();


  if (
    result == 1
  ) {

    packetsTX++;

    lastTX =
      packet;
  }


  LoRa.receive();
}


// ============================================================
//                    BLE CALLBACK
// ============================================================

class RoadMeshBLECallbacks
  : public BLEAdvertisedDeviceCallbacks {

public:

  void onResult(
    BLEAdvertisedDevice advertisedDevice
  ) override {

    String data = "";


    if (
    advertisedDevice.haveManufacturerData()
    ) {

    data =
    advertisedDevice.getManufacturerData();
    }


    if (
      data.length() == 0
    ) {

      return;
    }


    Serial.print(
      "BLE RX: "
    );

    Serial.println(
      data
    );


    // --------------------------------------------------------
    // FORMAT:
    //
    // RM,A,NODE,SEQ,CONF
    // --------------------------------------------------------

    if (
      !data.startsWith(
        "RM,A,"
      )
    ) {

      return;
    }


    int p1 =
      data.indexOf(
        ',',
        5
      );


    int p2 =
      data.indexOf(
        ',',
        p1 + 1
      );


    if (
      p1 < 0
      ||
      p2 < 0
    ) {

      return;
    }


    uint16_t origin =
      data.substring(
        5,
        p1
      ).toInt();


    uint32_t sequence =
      data.substring(
        p1 + 1,
        p2
      ).toInt();


    float confidence =
      data.substring(
        p2 + 1
      ).toFloat();


    if (
      origin == NODE_ID
    ) {

      return;
    }


    if (
      packetSeen(
        origin,
        sequence
      )
    ) {

      return;
    }


    rememberPacket(
      origin,
      sequence
    );


    registerNode(
      origin,
      advertisedDevice.getRSSI()
    );


    packetsRX++;


    lastRSSI =
      advertisedDevice.getRSSI();


    lastSNR =
      0;


    lastRX =
      data;


    remoteAccident =
      true;


    remoteNode =
      origin;


    remoteConfidence =
      confidence;


    remoteAlertTime =
      millis();


    Serial.println();
    Serial.println(
      "================================"
    );

    Serial.println(
      "🚨 BLE REMOTE ACCIDENT"
    );

    Serial.print(
      "Node: "
    );

    Serial.println(
      origin
    );

    Serial.print(
      "Confidence: "
    );

    Serial.println(
      confidence
    );

    Serial.println(
      "================================"
    );
  }
};


// ============================================================
//                    BLE INIT
// ============================================================

void initBLE() {

  Serial.println(
    "Starting BLE..."
  );


  if (
    !BLEDevice::init(
      "RoadMesh"
    )
  ) {

    Serial.println(
      "BLE INIT FAILED"
    );

    return;
  }


  bleScan =
    BLEDevice::getScan();


  if (
    bleScan != nullptr
  ) {

    bleScan->setAdvertisedDeviceCallbacks(
      new RoadMeshBLECallbacks()
    );


    bleScan->setActiveScan(
      true
    );


    bleScan->setInterval(
      100
    );


    bleScan->setWindow(
      80
    );
  }


  bleAdvertising =
    BLEDevice::getAdvertising();


  Serial.println(
    "BLE READY"
  );
}


// ============================================================
//                    BLE SEND
// ============================================================

void sendBLEAccident() {

  if (
    bleAdvertising == nullptr
  ) {

    return;
  }


  uint32_t sequence =
    sequenceNumber++;


  rememberPacket(
    NODE_ID,
    sequence
  );


  String data =
    "RM,A,";


  data +=
    String(
      NODE_ID
    );


  data +=
    ",";


  data +=
    String(
      sequence
    );


  data +=
    ",";


  data +=
    String(
      crashConfidence,
      0
    );


  Serial.print(
    "BLE TX: "
  );

  Serial.println(
    data
  );


  BLEAdvertisementData advertisementData;


  advertisementData.setManufacturerData(
    data.c_str()
  );


  bleAdvertising->stop();


  bleAdvertising->setAdvertisementData(
    advertisementData
  );


  bleAdvertising->start();


  packetsTX++;


  lastTX =
    data;
}


// ============================================================
//                    BLE SCAN
// ============================================================

void scanBLE() {

  if (
    bleScan == nullptr
  ) {

    return;
  }


  Serial.println(
    "BLE scanning..."
  );


  bleScan->start(
    2,
    false
  );


  bleScan->clearResults();
}


// ============================================================
//                    LORA ACCIDENT PACKET
// ============================================================

String createAccidentPacket() {

  uint32_t sequence =
    sequenceNumber++;


  rememberPacket(
    NODE_ID,
    sequence
  );


  String packet =
    "RM|A|";


  packet +=
    String(
      NODE_ID
    );


  packet +=
    "|";


  packet +=
    String(
      sequence
    );


  packet +=
    "|";


  packet +=
    String(
      MESH_TTL
    );


  packet +=
    "|";


  packet +=
    String(
      latitude,
      6
    );


  packet +=
    "|";


  packet +=
    String(
      longitude,
      6
    );


  packet +=
    "|";


  packet +=
    String(
      crashConfidence,
      0
    );


  packet +=
    "|";


  packet +=
    String(
      dynamicAcceleration,
      2
    );


  return packet;
}


// ============================================================
//                    SEND ACCIDENT
// ============================================================

void sendAccident() {

  Serial.println();
  Serial.println(
    "🚨 LOCAL ACCIDENT DETECTED"
  );


  if (
    transport
    ==
    MODE_LORA
  ) {

    String packet =
      createAccidentPacket();


    Serial.print(
      "LORA TX: "
    );

    Serial.println(
      packet
    );


    sendLoRa(
      packet
    );
  }


  else {

    sendBLEAccident();
  }
}


// ============================================================
//                    ACCIDENT DETECTION
// ============================================================

void detectAccident() {

  float score = 0;


  // ----------------------------------------------------------
  // Dynamic acceleration
  // ----------------------------------------------------------

  if (
    dynamicAcceleration
    >=
    1.2
  ) {

    score += 45;
  }

  else if (
    dynamicAcceleration
    >=
    0.6
  ) {

    score += 25;
  }


  // ----------------------------------------------------------
  // Rotation
  // ----------------------------------------------------------

  if (
    gyroMagnitude
    >=
    35
  ) {

    score += 35;
  }

  else if (
    gyroMagnitude
    >=
    15
  ) {

    score += 20;
  }


  // ----------------------------------------------------------
  // Combined movement
  // ----------------------------------------------------------

  if (
    dynamicAcceleration
    >=
    0.8
    &&
    gyroMagnitude
    >=
    20
  ) {

    score += 25;
  }


  if (
    score > 100
  ) {

    score = 100;
  }


  crashConfidence =
    score;


  if (
    crashConfidence
    >=
    ACCIDENT_THRESHOLD
    &&
    millis() -
    lastAccidentTime
    >
    ACCIDENT_COOLDOWN
  ) {

    localAccident =
      true;


    lastAccidentTime =
      millis();


    sendAccident();
  }
}


// ============================================================
//                    HEARTBEAT
// ============================================================

void sendHeartbeat() {

  if (
    transport
    !=
    MODE_LORA
  ) {

    return;
  }


  String packet =
    "RM|H|";


  packet +=
    String(
      NODE_ID
    );


  packet +=
    "|";


  packet +=
    String(
      millis()
    );


  sendLoRa(
    packet
  );
}


// ============================================================
//                    LORA RECEIVE
// ============================================================

void receiveLoRa() {

  int packetSize =
    LoRa.parsePacket();


  if (
    packetSize <= 0
  ) {

    return;
  }


  String packet =
    "";


  while (
    LoRa.available()
  ) {

    packet +=
      (char)
      LoRa.read();
  }


  packetsRX++;


  lastRSSI =
    LoRa.packetRssi();


  lastSNR =
    LoRa.packetSnr();


  lastRX =
    packet;


  Serial.println();
  Serial.println(
    "LORA RX:"
  );

  Serial.println(
    packet
  );


  // ----------------------------------------------------------
  // HEARTBEAT
  // ----------------------------------------------------------

  if (
    packet.startsWith(
      "RM|H|"
    )
  ) {

    int separator =
      packet.indexOf(
        '|',
        5
      );


    if (
      separator > 0
    ) {

      uint16_t id =
        packet.substring(
          5,
          separator
        ).toInt();


      registerNode(
        id,
        lastRSSI
      );
    }


    return;
  }


  // ----------------------------------------------------------
  // ACCIDENT
  // ----------------------------------------------------------

  if (
    !packet.startsWith(
      "RM|A|"
    )
  ) {

    packetsDropped++;

    return;
  }


  int p1 =
    packet.indexOf(
      '|',
      5
    );


  int p2 =
    packet.indexOf(
      '|',
      p1 + 1
    );


  int p3 =
    packet.indexOf(
      '|',
      p2 + 1
    );


  if (
    p1 < 0
    ||
    p2 < 0
    ||
    p3 < 0
  ) {

    packetsDropped++;

    return;
  }


  uint16_t origin =
    packet.substring(
      5,
      p1
    ).toInt();


  uint32_t sequence =
    packet.substring(
      p1 + 1,
      p2
    ).toInt();


  int ttl =
    packet.substring(
      p2 + 1,
      p3
    ).toInt();


  // Duplicate

  if (
    packetSeen(
      origin,
      sequence
    )
  ) {

    packetsDropped++;

    return;
  }


  rememberPacket(
    origin,
    sequence
  );


  registerNode(
    origin,
    lastRSSI
  );


  // ----------------------------------------------------------
  // Confidence
  // ----------------------------------------------------------

  int last =
    packet.lastIndexOf(
      '|'
    );


  int secondLast =
    packet.lastIndexOf(
      '|',
      last - 1
    );


  if (
    secondLast >= 0
  ) {

    remoteConfidence =
      packet.substring(
        secondLast + 1,
        last
      ).toFloat();
  }


  // ----------------------------------------------------------
  // REMOTE ALERT
  // ----------------------------------------------------------

  remoteAccident =
    true;


  remoteNode =
    origin;


  remoteAlertTime =
    millis();


  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "🚨 REMOTE ACCIDENT DETECTED"
  );

  Serial.print(
    "Origin node: "
  );

  Serial.println(
    origin
  );

  Serial.print(
    "Confidence: "
  );

  Serial.println(
    remoteConfidence
  );

  Serial.println(
    "================================"
  );


  // ----------------------------------------------------------
  // FORWARD
  // ----------------------------------------------------------

  if (
    ttl > 0
  ) {

    // Rebuild packet with TTL - 1

    String forwarded =
      packet;


    String ttlString =
      "|" +
      String(ttl) +
      "|";


    String newTTL =
      "|" +
      String(ttl - 1) +
      "|";


    int pos =
      forwarded.indexOf(
        ttlString
      );


    if (
      pos >= 0
    ) {

      forwarded.replace(
        ttlString,
        newTTL
      );


      delay(
        random(
          50,
          250
        )
      );


      sendLoRa(
        forwarded
      );


      packetsForwarded++;
    }
  }
}


// ============================================================
//                    WEB PAGE
// ============================================================

const char MAIN_PAGE[] PROGMEM = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

<meta name="viewport"
content="width=device-width,initial-scale=1">

<title>RoadMesh</title>

<style>

*{
box-sizing:border-box;
}

body{

margin:0;

padding:16px;

font-family:Arial,sans-serif;

background:#0f172a;

color:#fff;

}

.container{

max-width:1100px;

margin:auto;

}

h1{

margin-bottom:4px;

}

.subtitle{

color:#94a3b8;

}

.grid{

display:grid;

grid-template-columns:
repeat(
auto-fit,
minmax(
210px,
1fr
)
);

gap:12px;

}

.card{

background:#1e293b;

padding:18px;

border-radius:14px;

margin-bottom:12px;

}

.label{

font-size:12px;

color:#94a3b8;

text-transform:uppercase;

}

.value{

font-size:27px;

font-weight:bold;

margin-top:8px;

}

.green{

color:#22c55e;
}

.yellow{

color:#facc15;
}

.red{

color:#ef4444;
}

.blue{

color:#38bdf8;
}

button{

border:0;

padding:12px 18px;

margin:4px;

border-radius:9px;

font-size:15px;

cursor:pointer;

}

.alert{

display:none;

background:#7f1d1d;

border:2px solid #ef4444;

padding:20px;

border-radius:14px;

margin-bottom:15px;

}

.packet{

font-family:monospace;

background:#020617;

padding:12px;

border-radius:8px;

word-break:break-all;

font-size:12px;

}

</style>

</head>


<body>

<div class="container">


<h1>🚗 RoadMesh</h1>

<div class="subtitle">
Cooperative Accident Alert Network
</div>


<br>


<div id="remoteAlert"
class="alert">

<h2>
🚨 REMOTE ACCIDENT DETECTED
</h2>

<p>
Origin Node:
<b id="remoteNode">--</b>
</p>

<p>
Confidence:
<b id="remoteConfidence">--</b>%
</p>

<p>
Another RoadMesh node has reported an accident.
</p>

</div>


<div class="card">

<div class="label">
Communication Transport
</div>

<h2 id="transport">
LoRa
</h2>

<button onclick="setTransport('LORA')">
📡 LoRa
</button>

<button onclick="setTransport('BLE')">
🔵 Bluetooth
</button>

</div>


<div class="grid">


<div class="card">

<div class="label">
Node ID
</div>

<div
class="value blue"
id="node">
--
</div>

</div>


<div class="card">

<div class="label">
Active Nodes
</div>

<div
class="value blue"
id="nodes">
--
</div>

</div>


<div class="card">

<div class="label">
GPS
</div>

<div
class="value"
id="gps">
--
</div>

<p>
Latitude:
<span id="lat">--</span>
</p>

<p>
Longitude:
<span id="lon">--</span>
</p>

<p>
Satellites:
<span id="sat">--</span>
</p>

</div>


<div class="card">

<div class="label">
Acceleration
</div>

<div
class="value blue"
id="dynamic">
--
</div>

<p>
Gyroscope:
<span id="gyro">--</span>
°/s
</p>

</div>


<div class="card">

<div class="label">
Crash Confidence
</div>

<div
class="value"
id="confidence">
--
</div>

<p id="localStatus">
Normal
</p>

</div>


<div class="card">

<div class="label">
Network

Statistics
</div>

<p>
RX:
<span id="rx">--</span>
</p>

<p>
TX:
<span id="tx">--</span>
</p>

<p>
Forwarded:
<span id="forwarded">--</span>
</p>

<p>
Dropped:
<span id="dropped">--</span>
</p>

<p>
RSSI:
<span id="rssi">--</span>
dBm
</p>

<p>
SNR:
<span id="snr">--</span>
dB
</p>

</div>

</div>


<div class="card">

<div class="label">
Last Received
</div>

<div
class="packet"
id="received">
None
</div>

</div>


<div class="card">

<div class="label">
Last Sent
</div>

<div
class="packet"
id="sent">
None
</div>

</div>


</div>


<script>


async function setTransport(mode){

try{

await fetch(
"/transport?mode="+mode
);

update();

}

catch(e){

console.log(e);

}

}


async function update(){

try{

const response =
await fetch("/data");

const d =
await response.json();


document.getElementById(
"node"
).innerHTML =
d.node;


document.getElementById(
"nodes"
).innerHTML =
d.nodes;


document.getElementById(
"transport"
).innerHTML =
d.transport;


// GPS

const gps =
document.getElementById("gps");


if(d.gps){

gps.innerHTML =
"● FIX";

gps.className =
"value green";

}

else{

gps.innerHTML =
"● NO FIX";

gps.className =
"value yellow";

}


document.getElementById(
"lat"
).innerHTML =
d.lat;


document.getElementById(
"lon"
).innerHTML =
d.lon;


document.getElementById(
"sat"
).innerHTML =
d.sat;


// SENSOR

document.getElementById(
"dynamic"
).innerHTML =
d.dynamic+" m/s²";


document.getElementById(
"gyro"
).innerHTML =
d.gyro;


// CONFIDENCE

const confidence =
document.getElementById(
"confidence"
);


confidence.innerHTML =
d.confidence+"%";


const local =
document.getElementById(
"localStatus"
);


if(d.local){

local.innerHTML =
"🚨 LOCAL ACCIDENT DETECTED";

local.className =
"red";

confidence.className =
"value red";

}

else{

local.innerHTML =
"✓ Normal";

local.className =
"green";

confidence.className =
"value green";

}


// REMOTE

const remote =
document.getElementById(
"remoteAlert"
);


if(d.remote){

remote.style.display =
"block";

document.getElementById(
"remoteNode"
).innerHTML =
d.remoteNode;

document.getElementById(
"remoteConfidence"
).innerHTML =
d.remoteConfidence;

}

else{

remote.style.display =
"none";

}


// STATS

document.getElementById(
"rx"
).innerHTML =
d.rx;


document.getElementById(
"tx"
).innerHTML =
d.tx;


document.getElementById(
"forwarded"
).innerHTML =
d.forwarded;


document.getElementById(
"dropped"
).innerHTML =
d.dropped;


document.getElementById(
"rssi"
).innerHTML =
d.rssi;


document.getElementById(
"snr"
).innerHTML =
d.snr;


document.getElementById(
"received"
).innerHTML =
d.received;


document.getElementById(
"sent"
).innerHTML =
d.sent;

}

catch(e){

console.log(e);

}

}


update();

setInterval(
update,
500
);

</script>


</body>

</html>

)rawliteral";


// ============================================================
//                    WEB ROOT
// ============================================================

void handleRoot() {

  server.send_P(
    200,
    "text/html",
    MAIN_PAGE
  );
}


// ============================================================
//                    WEB DATA
// ============================================================

void handleData() {

  String json =
    "{";


  json +=
    "\"node\":" +
    String(NODE_ID);


  json +=
    ",\"transport\":\"";


  if (
    transport == MODE_LORA
  ) {

    json +=
      "LoRa";

  }

  else {

    json +=
      "BLE";
  }


  json += "\"";


  // GPS

  json +=
    ",\"gps\":" +
    String(
      gpsFix
        ? "true"
        : "false"
    );


  json +=
    ",\"lat\":" +
    String(
      latitude,
      6
    );


  json +=
    ",\"lon\":" +
    String(
      longitude,
      6
    );


  json +=
    ",\"sat\":" +
    String(
      satellites
    );


  // MPU

  json +=
    ",\"dynamic\":" +
    String(
      dynamicAcceleration,
      2
    );


  json +=
    ",\"gyro\":" +
    String(
      gyroMagnitude,
      1
    );


  // Accident

  json +=
    ",\"confidence\":" +
    String(
      crashConfidence,
      0
    );


  json +=
    ",\"local\":" +
    String(
      localAccident
        ? "true"
        : "false"
    );


  json +=
    ",\"remote\":" +
    String(
      remoteAccident
        ? "true"
        : "false"
    );


  json +=
    ",\"remoteNode\":" +
    String(
      remoteNode
    );


  json +=
    ",\"remoteConfidence\":" +
    String(
      remoteConfidence,
      0
    );


  // Nodes

  json +=
    ",\"nodes\":" +
    String(
      activeNodeCount()
    );


  // Stats

  json +=
    ",\"rx\":" +
    String(
      packetsRX
    );


  json +=
    ",\"tx\":" +
    String(
      packetsTX
    );


  json +=
    ",\"forwarded\":" +
    String(
      packetsForwarded
    );


  json +=
    ",\"dropped\":" +
    String(
      packetsDropped
    );


  json +=
    ",\"rssi\":" +
    String(
      lastRSSI
    );


  json +=
    ",\"snr\":" +
    String(
      lastSNR,
      1
    );


  String rx =
    lastRX;


  rx.replace(
    "\"",
    "\\\""
  );


  String tx =
    lastTX;


  tx.replace(
    "\"",
    "\\\""
  );


  json +=
    ",\"received\":\"" +
    rx +
    "\"";


  json +=
    ",\"sent\":\"" +
    tx +
    "\"";


  json +=
    "}";


  server.send(
    200,
    "application/json",
    json
  );
}


// ============================================================
//                    TRANSPORT
// ============================================================

void handleTransport() {

  if (
    !server.hasArg(
      "mode"
    )
  ) {

    server.send(
      400,
      "text/plain",
      "Missing mode"
    );

    return;
  }


  String mode =
    server.arg(
      "mode"
    );


  mode.toUpperCase();


  if (
    mode == "LORA"
  ) {

    transport =
      MODE_LORA;


    LoRa.receive();


    Serial.println(
      "Transport = LoRa"
    );
  }


  else if (
    mode == "BLE"
  ) {

    transport =
      MODE_BLE;


    Serial.println(
      "Transport = BLE"
    );
  }


  else {

    server.send(
      400,
      "text/plain",
      "Invalid mode"
    );

    return;
  }


  server.send(
    200,
    "text/plain",
    "OK"
  );
}


// ============================================================
//                    CAPTIVE PORTAL
// ============================================================

void handleRedirect() {

  server.sendHeader(
    "Location",
    "http://192.168.4.1/",
    true
  );


  server.send(
    302,
    "text/plain",
    ""
  );
}


// ============================================================
//                    WIFI
// ============================================================

void setupWiFi() {

  WiFi.mode(
    WIFI_AP
  );


  WiFi.softAP(
    WIFI_NAME,
    WIFI_PASSWORD
  );


  delay(500);


  IPAddress ip =
    WiFi.softAPIP();


  Serial.println();
  Serial.println(
    "=============================="
  );

  Serial.println(
    "WIFI READY"
  );

  Serial.print(
    "SSID: "
  );

  Serial.println(
    WIFI_NAME
  );

  Serial.print(
    "PASSWORD: "
  );

  Serial.println(
    WIFI_PASSWORD
  );

  Serial.print(
    "IP: "
  );

  Serial.println(
    ip
  );


  // DNS wildcard

  dnsServer.start(
    DNS_PORT,
    "*",
    ip
  );


  // Dashboard

  server.on(
    "/",
    handleRoot
  );


  server.on(
    "/data",
    handleData
  );


  server.on(
    "/transport",
    handleTransport
  );


  // Android

  server.on(
    "/generate_204",
    handleRoot
  );


  server.on(
    "/gen_204",
    handleRoot
  );


  // Apple

  server.on(
    "/hotspot-detect.html",
    handleRoot
  );


  // Windows

  server.on(
    "/connecttest.txt",
    handleRoot
  );


  server.on(
    "/ncsi.txt",
    handleRoot
  );


  server.onNotFound(
    handleRedirect
  );


  server.begin();


  Serial.println(
    "Captive portal started"
  );
}


// ============================================================
//                    SETUP LORA
// ============================================================

bool setupLoRa() {

  SPI.begin(
    LORA_SCK,
    LORA_MISO,
    LORA_MOSI,
    LORA_SS
  );


  LoRa.setPins(
    LORA_SS,
    LORA_RST,
    LORA_DIO0
  );


  Serial.println(
    "Starting SX1278..."
  );


  if (
    !LoRa.begin(
      LORA_FREQUENCY
    )
  ) {

    Serial.println(
      "❌ LORA FAILED"
    );

    return false;
  }


  Serial.println(
    "✅ LORA OK"
  );


  LoRa.setSpreadingFactor(
    LORA_SF
  );


  LoRa.setSignalBandwidth(
    LORA_BW
  );


  LoRa.setCodingRate4(
    LORA_CR
  );


  LoRa.setTxPower(
    LORA_POWER
  );


  LoRa.enableCrc();


  LoRa.receive();


  return true;
}


// ============================================================
//                    SETUP
// ============================================================

void setup() {

  Serial.begin(
    115200
  );


  delay(2500);


  Serial.println();
  Serial.println(
    "=========================================="
  );

  Serial.println(
    "             ROAD MESH FINAL"
  );

  Serial.println(
    "=========================================="
  );


#ifdef BOARD_C5

  Serial.println(
    "BOARD: ESP32-C5"
  );

#else

  Serial.println(
    "BOARD: NodeMCU ESP32"
  );

#endif


  Serial.print(
    "NODE ID: "
  );

  Serial.println(
    NODE_ID
  );


  // ----------------------------------------------------------
  // MPU
  // ----------------------------------------------------------

  mpuOK =
    initMPU();


  // ----------------------------------------------------------
  // GPS
  // ----------------------------------------------------------

  GPSSerial.begin(
    115200,
    SERIAL_8N1,
    GPS_RX,
    GPS_TX
  );


  Serial.println(
    "GPS UART started at 115200"
  );


  // ----------------------------------------------------------
  // LORA
  // ----------------------------------------------------------

  setupLoRa();


  // ----------------------------------------------------------
  // BLE
  // ----------------------------------------------------------

  initBLE();


  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

  setupWiFi();


  Serial.println();
  Serial.println(
    "=========================================="
  );

  Serial.println(
    "ROAD MESH READY"
  );

  Serial.println(
    "=========================================="
  );

  Serial.println(
    "Connect to WiFi: RoadMesh"
  );

  Serial.println(
    "Password: roadmesh123"
  );

  Serial.println(
    "Dashboard: http://192.168.4.1"
  );

  Serial.println(
    "=========================================="
  );
}


// ============================================================
//                    LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // Captive DNS
  // ----------------------------------------------------------

  dnsServer.processNextRequest();


  // ----------------------------------------------------------
  // Web server
  // ----------------------------------------------------------

  server.handleClient();


  // ----------------------------------------------------------
  // GPS
  // ----------------------------------------------------------

  readGPS();


  // ----------------------------------------------------------
  // MPU
  // ----------------------------------------------------------

  if (
    millis() -
    lastSensorRead
    >=
    20
  ) {

    lastSensorRead =
      millis();


    readMPU();


    detectAccident();
  }


  // ----------------------------------------------------------
  // LORA
  // ----------------------------------------------------------

  if (
    transport
    ==
    MODE_LORA
  ) {

    receiveLoRa();


    if (
      millis() -
      lastHeartbeat
      >=
      HEARTBEAT_INTERVAL
    ) {

      lastHeartbeat =
        millis();


      sendHeartbeat();
    }
  }


  // ----------------------------------------------------------
  // BLE
  // ----------------------------------------------------------

  if (
    transport
    ==
    MODE_BLE
  ) {

    if (
      millis() -
      lastBLEScan
      >=
      5000
    ) {

      lastBLEScan =
        millis();


      scanBLE();
    }
  }


  // ----------------------------------------------------------
  // Remote alert timeout
  // ----------------------------------------------------------

  if (
    remoteAccident
    &&
    millis() -
    remoteAlertTime
    >
    30000
  ) {

    remoteAccident =
      false;
  }


  delay(1);
}
