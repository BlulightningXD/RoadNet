/*
 ============================================================
                     ROAD MESH
                  FINAL DEMO VERSION
 ============================================================

 Hardware:
   - ESP32-C5 OR NodeMCU ESP32
   - SX1278 / RA-02 433 MHz
   - MPU6050
   - GPS

 Features:
   - Accident detection
   - Very sensitive demo trigger
   - GPS
   - MPU6050 acceleration + gyro
   - LoRa mesh
   - Packet TTL
   - Duplicate protection
   - Heartbeat / node discovery
   - Active node count
   - RSSI / SNR
   - WiFi dashboard

 ============================================================
*/


// ============================================================
// BOARD CONFIGURATION
// ============================================================

// -------- ESP32-C5 --------

#define BOARD_C5
#define NODE_ID 1


// -------- NODEMCU ESP32 --------
//
// For NodeMCU:
// Comment the two lines above and use:
//
// #define NODE_ID 2
//
// Example:
//
// // #define BOARD_C5
// #define NODE_ID 2


// ============================================================
// LIBRARIES
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>


// ============================================================
// WIFI
// ============================================================

const char* WIFI_NAME = "RoadMesh";
const char* WIFI_PASSWORD = "roadmesh123";

WebServer server(80);


// ============================================================
// PIN CONFIGURATION
// ============================================================

#ifdef BOARD_C5

// ---------------- ESP32-C5 ----------------

#define MPU_SDA 4
#define MPU_SCL 5

#define GPS_RX 24
#define GPS_TX 23

#define LORA_SCK  6
#define LORA_MISO 8
#define LORA_MOSI 9
#define LORA_SS   10
#define LORA_RST  0
#define LORA_DIO0 1

#else

// ---------------- CLASSIC ESP32 ----------------

#define MPU_SDA 21
#define MPU_SCL 22

#define GPS_RX 16
#define GPS_TX 17

#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 26

#endif


// ============================================================
// GPS
// ============================================================

#define GPS_BAUD 115200

HardwareSerial GPSSerial(1);

TinyGPSPlus gps;


// ============================================================
// MPU6050
// ============================================================

#define MPU_ADDR 0x68

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
// GPS VARIABLES
// ============================================================

double latitude = 0;
double longitude = 0;

int satellites = 0;

bool gpsFix = false;


// ============================================================
// ACCIDENT DETECTION
// ============================================================

float crashConfidence = 0;

bool accidentDetected = false;

unsigned long lastAccidentTime = 0;

#define ACCIDENT_COOLDOWN 15000


// ============================================================
// MESH CONFIGURATION
// ============================================================

#define MESH_TTL 3

#define PACKET_HISTORY_SIZE 40

uint32_t sequenceNumber = 0;

struct SeenPacket {

  uint16_t origin;

  uint32_t sequence;
};

SeenPacket packetHistory[PACKET_HISTORY_SIZE];

int historyIndex = 0;


// ============================================================
// NODE DISCOVERY
// ============================================================

#define MAX_NODES 20

#define NODE_TIMEOUT 10000

#define HEARTBEAT_INTERVAL 3000

struct KnownNode {

  uint16_t id;

  unsigned long lastSeen;

  int rssi;
};

KnownNode knownNodes[MAX_NODES];

unsigned long lastHeartbeat = 0;


// ============================================================
// MESH STATISTICS
// ============================================================

uint32_t packetsReceived = 0;

uint32_t packetsForwarded = 0;

uint32_t packetsDropped = 0;

int lastRSSI = 0;

float lastSNR = 0;

String lastReceived = "None";

String lastSent = "None";


// ============================================================
// MPU INITIALIZATION
// ============================================================

bool initMPU() {

  Wire.begin(
    MPU_SDA,
    MPU_SCL
  );

  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(0x6B);

  Wire.write(0x00);

  byte error =
    Wire.endTransmission();

  if (error != 0) {

    Serial.println(
      "MPU6050 NOT FOUND"
    );

    return false;
  }

  delay(100);


  // Accelerometer ±2g

  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(0x1C);

  Wire.write(0x00);

  Wire.endTransmission();


  // Gyroscope ±250 degrees/sec

  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(0x1B);

  Wire.write(0x00);

  Wire.endTransmission();


  Serial.println(
    "MPU6050 OK"
  );

  return true;
}


// ============================================================
// READ MPU6050
// ============================================================

void readMPU() {

  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(0x3B);

  Wire.endTransmission(false);

  Wire.requestFrom(
    MPU_ADDR,
    14
  );

  if (
    Wire.available() < 14
  ) {

    return;
  }


  int16_t rawAx =
    (Wire.read() << 8) |
    Wire.read();

  int16_t rawAy =
    (Wire.read() << 8) |
    Wire.read();

  int16_t rawAz =
    (Wire.read() << 8) |
    Wire.read();


  // Temperature

  Wire.read();
  Wire.read();


  int16_t rawGx =
    (Wire.read() << 8) |
    Wire.read();

  int16_t rawGy =
    (Wire.read() << 8) |
    Wire.read();

  int16_t rawGz =
    (Wire.read() << 8) |
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
  // LOW-PASS GRAVITY ESTIMATION
  // ----------------------------------------------------------

  const float alpha = 0.98;


  gravityX =
    alpha * gravityX +
    (1.0 - alpha) * ax;

  gravityY =
    alpha * gravityY +
    (1.0 - alpha) * ay;

  gravityZ =
    alpha * gravityZ +
    (1.0 - alpha) * az;


  // ----------------------------------------------------------
  // REMOVE GRAVITY
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // GYROSCOPE MAGNITUDE
  // ----------------------------------------------------------

  gyroMagnitude =
    sqrt(
      gx * gx +
      gy * gy +
      gz * gz
    );
}


// ============================================================
// GPS
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


  if (
    gps.satellites.isValid()
  ) {

    satellites =
      gps.satellites.value();
  }
}


// ============================================================
// PACKET HISTORY
// ============================================================

bool packetAlreadySeen(
  uint16_t origin,
  uint32_t sequence
) {

  for (
    int i = 0;
    i < PACKET_HISTORY_SIZE;
    i++
  ) {

    if (
      packetHistory[i].origin ==
      origin &&

      packetHistory[i].sequence ==
      sequence
    ) {

      return true;
    }
  }

  return false;
}


// ============================================================
// REMEMBER PACKET
// ============================================================

void rememberPacket(
  uint16_t origin,
  uint32_t sequence
) {

  packetHistory[
    historyIndex
  ].origin =
    origin;


  packetHistory[
    historyIndex
  ].sequence =
    sequence;


  historyIndex++;


  if (
    historyIndex >=
    PACKET_HISTORY_SIZE
  ) {

    historyIndex = 0;
  }
}


// ============================================================
// REGISTER NODE
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


  // Already known

  for (
    int i = 0;
    i < MAX_NODES;
    i++
  ) {

    if (
      knownNodes[i].id ==
      id
    ) {

      knownNodes[i].lastSeen =
        millis();

      knownNodes[i].rssi =
        rssi;

      return;
    }
  }


  // Find empty slot

  for (
    int i = 0;
    i < MAX_NODES;
    i++
  ) {

    if (
      knownNodes[i].id ==
      0
    ) {

      knownNodes[i].id =
        id;

      knownNodes[i].lastSeen =
        millis();

      knownNodes[i].rssi =
        rssi;


      Serial.print(
        "NEW NODE DETECTED: "
      );

      Serial.println(
        id
      );

      return;
    }
  }
}


// ============================================================
// COUNT ACTIVE NODES
// ============================================================

int getActiveNodeCount() {

  int count = 1;

  for (
    int i = 0;
    i < MAX_NODES;
    i++
  ) {

    if (
      knownNodes[i].id != 0 &&

      millis() -
      knownNodes[i].lastSeen
      <
      NODE_TIMEOUT
    ) {

      count++;
    }
  }

  return count;
}


// ============================================================
// SEND LORA
// ============================================================

void sendLoRa(
  const String &packet
) {

  LoRa.idle();


  LoRa.beginPacket();


  LoRa.print(
    packet
  );


  LoRa.endPacket();


  LoRa.receive();


  lastSent =
    packet;
}


// ============================================================
// SEND HEARTBEAT
// ============================================================

void sendHeartbeat() {

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


  Serial.print(
    "HEARTBEAT: "
  );

  Serial.println(
    packet
  );
}


// ============================================================
// SEND ACCIDENT
// ============================================================

void sendAccident() {

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


  packet += "|";


  packet +=
    String(
      sequence
    );


  packet += "|";


  packet +=
    String(
      MESH_TTL
    );


  packet += "|";


  packet +=
    String(
      latitude,
      6
    );


  packet += "|";


  packet +=
    String(
      longitude,
      6
    );


  packet += "|";


  packet +=
    String(
      crashConfidence,
      0
    );


  packet += "|";


  packet +=
    String(
      dynamicAcceleration,
      2
    );


  Serial.println();
  Serial.println(
    "=============================="
  );

  Serial.println(
    "🚨 ACCIDENT DETECTED"
  );

  Serial.println(
    packet
  );

  Serial.println(
    "=============================="
  );


  sendLoRa(
    packet
  );
}


// ============================================================
// DEMO ACCIDENT DETECTION
// ============================================================

void calculateCrashConfidence() {

  float score = 0;


  /*
    ----------------------------------------------------------
    VERY SENSITIVE DEMO THRESHOLDS

    These intentionally trigger from small movement.

    Production values MUST be much higher and should be
    determined from actual vehicle data.
    ----------------------------------------------------------
  */


  // Dynamic acceleration

  if (
    dynamicAcceleration >= 1.2
  ) {

    score += 45;

  }

  else if (
    dynamicAcceleration >= 0.6
  ) {

    score += 25;
  }


  // Rotation

  if (
    gyroMagnitude >= 35
  ) {

    score += 35;

  }

  else if (
    gyroMagnitude >= 15
  ) {

    score += 20;
  }


  // Combined movement

  if (
    dynamicAcceleration >= 0.8 &&
    gyroMagnitude >= 20
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


  // Trigger

  if (
    crashConfidence >= 50 &&

    millis() -
    lastAccidentTime
    >
    ACCIDENT_COOLDOWN
  ) {

    accidentDetected =
      true;


    lastAccidentTime =
      millis();


    sendAccident();
  }
}


// ============================================================
// FORWARD ACCIDENT PACKET
// ============================================================

void forwardPacket(
  String packet,
  int ttl
) {

  if (
    ttl <= 0
  ) {

    packetsDropped++;

    return;
  }


  int newTTL =
    ttl - 1;


  /*
    Packet:

    RM|A|NODE|SEQ|TTL|LAT|LON|CONF|ACC
  */


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


  if (
    p1 < 0 ||
    p2 < 0
  ) {

    packetsDropped++;

    return;
  }


  String newPacket =
    packet.substring(
      0,
      p2 + 1
    );


  newPacket +=
    String(
      newTTL
    );


  int ttlEnd =
    packet.indexOf(
      '|',
      p2 + 1
    );


  if (
    ttlEnd < 0
  ) {

    packetsDropped++;

    return;
  }


  newPacket +=
    packet.substring(
      ttlEnd
    );


  // Random jitter reduces collisions

  randomSeed(
    micros() +
    NODE_ID
  );


  int waitTime =
    random(
      80,
      300
    );


  delay(
    waitTime
  );


  sendLoRa(
    newPacket
  );


  packetsForwarded++;


  Serial.println(
    "PACKET FORWARDED"
  );

  Serial.println(
    newPacket
  );
}


// ============================================================
// RECEIVE LORA
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


  lastRSSI =
    LoRa.packetRssi();


  lastSNR =
    LoRa.packetSnr();


  lastReceived =
    packet;


  packetsReceived++;


  Serial.println();
  Serial.println(
    "------------------------------"
  );

  Serial.println(
    "RECEIVED:"
  );

  Serial.println(
    packet
  );

  Serial.print(
    "RSSI: "
  );

  Serial.println(
    lastRSSI
  );

  Serial.print(
    "SNR: "
  );

  Serial.println(
    lastSNR
  );


  // =========================================================
  // VALIDATE
  // =========================================================

  if (
    !packet.startsWith(
      "RM|"
    )
  ) {

    packetsDropped++;

    return;
  }


  // =========================================================
  // HEARTBEAT
  // =========================================================

  if (
    packet.startsWith(
      "RM|H|"
    )
  ) {

    int p1 =
      packet.indexOf(
        '|',
        5
      );


    if (
      p1 > 0
    ) {

      uint16_t remoteNode =
        packet.substring(
          5,
          p1
        ).toInt();


      registerNode(
        remoteNode,
        lastRSSI
      );


      Serial.print(
        "HEARD NODE: "
      );

      Serial.println(
        remoteNode
      );
    }


    return;
  }


  // =========================================================
  // ACCIDENT
  // =========================================================

  if (
    packet.startsWith(
      "RM|A|"
    )
  ) {

    /*
      RM|A|ORIGIN|SEQ|TTL|LAT|LON|CONF|ACC
    */


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
      p1 < 0 ||
      p2 < 0 ||
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


    // -------------------------------------------------------
    // DUPLICATE PROTECTION
    // -------------------------------------------------------

    if (
      packetAlreadySeen(
        origin,
        sequence
      )
    ) {

      packetsDropped++;

      Serial.println(
        "DUPLICATE PACKET - DROPPED"
      );

      return;
    }


    rememberPacket(
      origin,
      sequence
    );


    // -------------------------------------------------------
    // REGISTER ORIGIN NODE
    // -------------------------------------------------------

    registerNode(
      origin,
      lastRSSI
    );


    // -------------------------------------------------------
    // REMOTE ALERT
    // -------------------------------------------------------

    Serial.println();
    Serial.println(
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    );

    Serial.println(
      "🚨 REMOTE ACCIDENT ALERT"
    );

    Serial.print(
      "Origin Node: "
    );

    Serial.println(
      origin
    );

    Serial.print(
      "Sequence: "
    );

    Serial.println(
      sequence
    );

    Serial.print(
      "RSSI: "
    );

    Serial.println(
      lastRSSI
    );

    Serial.print(
      "SNR: "
    );

    Serial.println(
      lastSNR
    );

    Serial.println(
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    );


    // -------------------------------------------------------
    // FORWARD
    // -------------------------------------------------------

    forwardPacket(
      packet,
      ttl
    );
  }
}


// ============================================================
// WEB API
// ============================================================

void handleData() {

  String json =
    "{";


  json +=
    "\"node\":";

  json +=
    String(
      NODE_ID
    );


  json +=
    ",\"gps\":";

  json +=
    gpsFix
      ? "true"
      : "false";


  json +=
    ",\"lat\":";

  json +=
    String(
      latitude,
      6
    );


  json +=
    ",\"lon\":";

  json +=
    String(
      longitude,
      6
    );


  json +=
    ",\"sat\":";

  json +=
    String(
      satellites
    );


  json +=
    ",\"ax\":";

  json +=
    String(
      ax,
      2
    );


  json +=
    ",\"ay\":";

  json +=
    String(
      ay,
      2
    );


  json +=
    ",\"az\":";

  json +=
    String(
      az,
      2
    );


  json +=
    ",\"dynamic\":";

  json +=
    String(
      dynamicAcceleration,
      2
    );


  json +=
    ",\"gyro\":";

  json +=
    String(
      gyroMagnitude,
      1
    );


  json +=
    ",\"confidence\":";

  json +=
    String(
      crashConfidence,
      0
    );


  json +=
    ",\"accident\":";

  json +=
    accidentDetected
      ? "true"
      : "false";


  json +=
    ",\"nodes\":";

  json +=
    String(
      getActiveNodeCount()
    );


  json +=
    ",\"rx\":";

  json +=
    String(
      packetsReceived
    );


  json +=
    ",\"forwarded\":";

  json +=
    String(
      packetsForwarded
    );


  json +=
    ",\"dropped\":";

  json +=
    String(
      packetsDropped
    );


  json +=
    ",\"rssi\":";

  json +=
    String(
      lastRSSI
    );


  json +=
    ",\"snr\":";

  json +=
    String(
      lastSNR,
      1
    );


  // ----------------------------------------------------------
  // LAST RECEIVED
  // ----------------------------------------------------------

  String received =
    lastReceived;

  received.replace(
    "\"",
    "\\\""
  );


  json +=
    ",\"received\":\"";

  json +=
    received;

  json +=
    "\"";


  // ----------------------------------------------------------
  // LAST SENT
  // ----------------------------------------------------------

  String sent =
    lastSent;

  sent.replace(
    "\"",
    "\\\""
  );


  json +=
    ",\"sent\":\"";

  json +=
    sent;

  json +=
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
// WEB PAGE
// ============================================================

const char PAGE[] PROGMEM = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

<meta name="viewport"
content="width=device-width,initial-scale=1">

<title>RoadMesh</title>

<style>

body {

font-family:
Arial,
sans-serif;

background:
#0f172a;

color:
white;

margin:
0;

padding:
20px;

}

.container {

max-width:
1100px;

margin:
auto;

}

h1 {

margin-bottom:
4px;

}

.subtitle {

color:
#94a3b8;

margin-top:
0;

}

.grid {

display:
grid;

grid-template-columns:
repeat(
auto-fit,
minmax(
220px,
1fr
)
);

gap:
15px;

}

.card {

background:
#1e293b;

padding:
20px;

border-radius:
14px;

margin-bottom:
15px;

}

.label {

color:
#94a3b8;

font-size:
14px;

}

.value {

font-size:
28px;

font-weight:
bold;

}

.normal {

color:
#22c55e;

}

.warning {

color:
#facc15;

}

.danger {

color:
#ef4444;

}

.blue {

color:
#38bdf8;

}

.packet {

background:
#020617;

padding:
12px;

border-radius:
8px;

font-family:
monospace;

font-size:
12px;

word-break:
break-all;

}

</style>

</head>


<body>

<div class="container">


<h1>🚗 RoadMesh</h1>

<p class="subtitle">
Cooperative Vehicle Safety Network
</p>


<div class="grid">


<!-- NODE -->

<div class="card">

<p class="label">
NODE ID
</p>

<div
class="value blue"
id="node">
--
</div>

</div>


<!-- ACTIVE NODES -->

<div class="card">

<p class="label">
ACTIVE MESH NODES
</p>

<div
class="value blue"
id="nodes">
--
</div>

<p class="label">
Recently heard nodes
</p>

</div>


<!-- GPS -->

<div class="card">

<p class="label">
GPS STATUS
</p>

<div
class="value"
id="gps">
--
</div>

<p>
Latitude:
<span id="lat">
--
</span>
</p>

<p>
Longitude:
<span id="lon">
--
</span>
</p>

<p>
Satellites:
<span id="sat">
--
</span>
</p>

</div>


<!-- ACCELERATION -->

<div class="card">

<p class="label">
DYNAMIC ACCELERATION
</p>

<div
class="value blue"
id="dynamic">
--
</div>

<p>
Gyroscope:
<span id="gyro">
--
</span>
</p>

</div>


<!-- ACCIDENT -->

<div class="card">

<p class="label">
CRASH CONFIDENCE
</p>

<div
class="value"
id="confidence">
--
</div>

<p id="accident">
Normal
</p>

</div>


<!-- MESH -->

<div class="card">

<p class="label">
MESH NETWORK
</p>

<p>
Received:
<span id="rx">
--
</span>
</p>

<p>
Forwarded:
<span id="forwarded">
--
</span>
</p>

<p>
Dropped:
<span id="dropped">
--
</span>
</p>

<p>
RSSI:
<span id="rssi">
--
</span>
</p>

<p>
SNR:
<span id="snr">
--
</span>
</p>

</div>

</div>


<!-- RECEIVED -->

<div class="card">

<p class="label">
LAST RECEIVED PACKET
</p>

<div
class="packet"
id="received">
None
</div>

</div>


<!-- SENT -->

<div class="card">

<p class="label">
LAST SENT PACKET
</p>

<div
class="packet"
id="sent">
None
</div>

</div>


</div>


<script>


async function updateData() {

try {


const response =
await fetch(
"/data"
);


const data =
await response.json();


document.getElementById(
"node"
).innerHTML =
data.node;


document.getElementById(
"nodes"
).innerHTML =
data.nodes;


// ======================================================
// GPS
// ======================================================

const gps =
document.getElementById(
"gps"
);


if (
data.gps
) {

gps.innerHTML =
"● FIX";

gps.className =
"value normal";

}

else {

gps.innerHTML =
"● NO FIX";

gps.className =
"value warning";

}


document.getElementById(
"lat"
).innerHTML =
data.lat;


document.getElementById(
"lon"
).innerHTML =
data.lon;


document.getElementById(
"sat"
).innerHTML =
data.sat;


// ======================================================
// MPU
// ======================================================

document.getElementById(
"dynamic"
).innerHTML =
data.dynamic +
" m/s²";


document.getElementById(
"gyro"
).innerHTML =
data.gyro +
" °/s";


// ======================================================
// ACCIDENT
// ======================================================

const confidence =
document.getElementById(
"confidence"
);

const accident =
document.getElementById(
"accident"
);


confidence.innerHTML =
data.confidence +
"%";


if (
data.accident
) {

accident.innerHTML =
"🚨 ACCIDENT DETECTED";

accident.className =
"danger";

confidence.className =
"value danger";

}

else {

accident.innerHTML =
"✓ Normal";

accident.className =
"normal";

confidence.className =
"value normal";

}


// ======================================================
// MESH
// ======================================================

document.getElementById(
"rx"
).innerHTML =
data.rx;


document.getElementById(
"forwarded"
).innerHTML =
data.forwarded;


document.getElementById(
"dropped"
).innerHTML =
data.dropped;


document.getElementById(
"rssi"
).innerHTML =
data.rssi +
" dBm";


document.getElementById(
"snr"
).innerHTML =
data.snr +
" dB";


// ======================================================
// PACKETS
// ======================================================

document.getElementById(
"received"
).innerHTML =
data.received;


document.getElementById(
"sent"
).innerHTML =
data.sent;


}

catch (
error
) {

console.log(
"Dashboard error:",
error
);

}

}


updateData();


setInterval(
updateData,
500
);


</script>

</body>

</html>

)rawliteral";


// ============================================================
// WEB SERVER
// ============================================================

void handleRoot() {

  server.send_P(
    200,
    "text/html",
    PAGE
  );
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(
    115200
  );


  delay(
    1500
  );


  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    "          ROAD MESH STARTING"
  );

  Serial.println(
    "========================================"
  );


  Serial.print(
    "NODE ID: "
  );

  Serial.println(
    NODE_ID
  );


#ifdef BOARD_C5

  Serial.println(
    "BOARD: ESP32-C5"
  );

#else

  Serial.println(
    "BOARD: CLASSIC ESP32"
  );

#endif


  // =========================================================
  // MPU
  // =========================================================

  if (
    !initMPU()
  ) {

    Serial.println(
      "WARNING: MPU6050 unavailable"
    );
  }


  // =========================================================
  // GPS
  // =========================================================

  GPSSerial.begin(
    GPS_BAUD,
    SERIAL_8N1,
    GPS_RX,
    GPS_TX
  );


  Serial.print(
    "GPS baud: "
  );

  Serial.println(
    GPS_BAUD
  );


  // =========================================================
  // LORA
  // =========================================================

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
    "Starting LoRa..."
  );


  if (
    !LoRa.begin(
      433E6
    )
  ) {

    Serial.println(
      "LoRa FAILED!"
    );

  }

  else {

    Serial.println(
      "LoRa OK!"
    );


    LoRa.setSpreadingFactor(
      7
    );


    LoRa.setSignalBandwidth(
      125E3
    );


    LoRa.setCodingRate4(
      5
    );


    LoRa.setTxPower(
      17
    );


    LoRa.enableCrc();


    LoRa.receive();
  }


  // =========================================================
  // WIFI
  // =========================================================

  WiFi.mode(
    WIFI_AP
  );


  bool wifiOK =
    WiFi.softAP(
      WIFI_NAME,
      WIFI_PASSWORD
    );


  if (
    wifiOK
  ) {

    Serial.println(
      "WiFi AP started"
    );


    Serial.print(
      "SSID: "
    );

    Serial.println(
      WIFI_NAME
    );


    Serial.print(
      "Password: "
    );

    Serial.println(
      WIFI_PASSWORD
    );


    Serial.print(
      "IP: "
    );

    Serial.println(
      WiFi.softAPIP()
    );

  }

  else {

    Serial.println(
      "WiFi AP FAILED"
    );
  }


  // =========================================================
  // WEB SERVER
  // =========================================================

  server.on(
    "/",
    handleRoot
  );


  server.on(
    "/data",
    handleData
  );


  server.begin();


  Serial.println(
    "Web server started"
  );


  Serial.println(
    "========================================"
  );

  Serial.println(
    "          ROAD MESH READY"
  );

  Serial.println(
    "========================================"
  );
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // WEB SERVER
  // ----------------------------------------------------------

  server.handleClient();


  // ----------------------------------------------------------
  // GPS
  // ----------------------------------------------------------

  readGPS();


  // ----------------------------------------------------------
  // SENSOR
  // ----------------------------------------------------------

  static unsigned long lastSensorRead =
    0;


  if (
    millis() -
    lastSensorRead >=
    20
  ) {

    lastSensorRead =
      millis();


    readMPU();


    calculateCrashConfidence();
  }


  // ----------------------------------------------------------
  // LORA RECEIVE
  // ----------------------------------------------------------

  receiveLoRa();


  // ----------------------------------------------------------
  // HEARTBEAT
  // ----------------------------------------------------------

  if (
    millis() -
    lastHeartbeat >=
    HEARTBEAT_INTERVAL
  ) {

    lastHeartbeat =
      millis();


    sendHeartbeat();
  }


  delay(1);
}
