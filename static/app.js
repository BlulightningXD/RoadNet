/**
 * RoadMesh Web Dashboard Application
 * Real-time safety telemetry, Leaflet GPS tracking, inertial waveforms,
 * and Web Bluetooth ESP32/ESP32-C5 connectivity.
 */

const $ = id => document.getElementById(id);
const colors = {
  ax: '#25c6ff',
  ay: '#a986ff',
  az: '#36d78e',
  acc: '#ffd166',
  gx: '#25c6ff',
  gy: '#a986ff',
  gz: '#36d78e',
  gyro: '#f78c6c'
};

let map, marker, route;
let lastCrashTime = 0;
const accidentEvents = [];
const bleFrames = [];
let lastFrames = [];
let pollInterval = null;

// Initialize Leaflet Map
function setupMap() {
  if (map) return;
  map = L.map('map', { zoomControl: false }).setView([19.076, 72.877], 14);
  L.control.zoom({ position: 'bottomright' }).addTo(map);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '© OpenStreetMap'
  }).addTo(map);
  
  marker = L.circleMarker([19.076, 72.877], {
    radius: 9,
    color: '#fff',
    weight: 2,
    fillColor: '#26c5ff',
    fillOpacity: 1
  }).addTo(map);
  
  route = L.polyline([], {
    color: '#26c5ff',
    weight: 3,
    opacity: 0.8
  }).addTo(map);
}

// Draw inertial waveform graphs with accident event markers
function draw(canvas, frames, keys, scale, isAccel = false) {
  if (!canvas) return;
  const dpr = window.devicePixelRatio || 1;
  const box = canvas.getBoundingClientRect();
  const w = Math.max(1, box.width * dpr);
  const h = Math.max(1, box.height * dpr);

  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }

  const c = canvas.getContext('2d');
  c.clearRect(0, 0, w, h);
  c.fillStyle = '#101b28';
  c.fillRect(0, 0, w, h);

  // Background grid lines
  c.strokeStyle = '#263b50';
  c.lineWidth = dpr;
  for (let i = 1; i < 4; i++) {
    const y = (h * i) / 4;
    c.beginPath();
    c.moveTo(0, y);
    c.lineTo(w, y);
    c.stroke();
  }

  if (!frames || !frames.length) return;

  const pad = 14 * dpr;
  const center = h / 2;
  const count = frames.length;

  // Draw sensor traces
  keys.forEach(key => {
    c.strokeStyle = colors[key] || '#25c6ff';
    c.lineWidth = 1.6 * dpr;
    c.beginPath();

    frames.forEach((f, i) => {
      const x = pad + (i * (w - pad * 2)) / Math.max(1, count - 1);
      const val = (f.imu && f.imu[key] !== undefined) ? f.imu[key] : (f[key] !== undefined ? f[key] : 0);
      const y = center - (val / scale) * (h * 0.36);
      if (i === 0) {
        c.moveTo(x, y);
      } else {
        c.lineTo(x, y);
      }
    });
    c.stroke();
  });

  // Highlight and mark accident events on acceleration / gyro graphs
  frames.forEach((f, i) => {
    if (f.crash || f.accident === 1) {
      const x = pad + (i * (w - pad * 2)) / Math.max(1, count - 1);

      // Vertical crash marker line
      c.strokeStyle = '#ff5e67';
      c.lineWidth = 2 * dpr;
      c.beginPath();
      c.moveTo(x, 0);
      c.lineTo(x, h);
      c.stroke();

      // Crash indicator badge on acceleration chart
      if (isAccel) {
        c.fillStyle = '#ff5e67';
        c.beginPath();
        c.arc(x, 12 * dpr, 5 * dpr, 0, Math.PI * 2);
        c.fill();

        c.fillStyle = '#ffffff';
        c.font = `bold ${9 * dpr}px sans-serif`;
        c.textAlign = 'center';
        const impactVal = (f.imu && (f.imu.acc || f.imu.dynamic)) ? (f.imu.acc || f.imu.dynamic).toFixed(1) : (f.acc ? f.acc.toFixed(1) : '!');
        c.fillText(`⚠ ${impactVal}g`, x, 25 * dpr);
      }
    }
  });
}

// Record accident event with stored timestamp and sensor values
function recordAccidentEvent(data) {
  const eventRecord = {
    ts: data.ts || Date.now(),
    date: new Date().toLocaleTimeString(),
    node: data.node || 'RoadMesh Node',
    lat: data.lat || (data.gps ? data.gps.lat : 19.076),
    lon: data.lon || (data.gps ? data.gps.lon : 72.877),
    acc: data.acc || (data.imu ? (data.imu.acc || data.imu.dynamic) : 0),
    ax: data.ax || (data.imu ? data.imu.ax : 0),
    ay: data.ay || (data.imu ? data.imu.ay : 0),
    az: data.az || (data.imu ? data.imu.az : 0),
    gx: data.gx || (data.imu ? data.imu.gx : 0),
    gy: data.gy || (data.imu ? data.imu.gy : 0),
    gz: data.gz || (data.imu ? data.imu.gz : 0),
    gyro: data.gyro || (data.imu ? data.imu.gyro : 0),
    confidence: data.confidence !== undefined ? data.confidence : Math.round((data.cc || 0) * 100)
  };

  accidentEvents.push(eventRecord);
  if (accidentEvents.length > 50) accidentEvents.shift();

  // Update Alert Modal Values
  $('alertCoords').textContent = `${eventRecord.lat.toFixed(5)}, ${eventRecord.lon.toFixed(5)}`;
  $('alertImpact').textContent = `${eventRecord.acc.toFixed(2)} g (${eventRecord.confidence}% confidence)`;
  $('alertAirbag').textContent = eventRecord.confidence > 50 ? 'DEPLOYED' : 'ARMED';
  $('alert').classList.remove('hidden');

  console.warn('[RoadMesh] Stored accident event:', eventRecord);
}

// Update the Dashboard in Real Time
function update(data) {
  const frames = data.frames || [];
  if (!frames.length) return;
  lastFrames = frames;

  const f = frames.at(-1);
  const gps = f.gps || {};
  const imu = f.imu || {};
  const stats = f.stats || {};
  const mesh = f.mesh || {};

  // Frame count
  $('frameCount').textContent = `${frames.length} recent frames`;

  // Coordinates & GPS Module Detection
  const lat = Number(gps.lat !== undefined ? gps.lat : (f.lat || 0));
  const lon = Number(gps.lon !== undefined ? gps.lon : (f.lon || 0));
  const hasFix = (f.fix === 1 || gps.ok === true || gps.ok === 1);
  const hasValidCoords = lat !== 0 && lon !== 0 && !isNaN(lat) && !isNaN(lon) && Math.abs(lat) <= 90 && Math.abs(lon) <= 180;
  const hasGpsSignal = Boolean(hasFix && hasValidCoords);

  if (hasGpsSignal) {
    $('coordinates').textContent = `${lat.toFixed(5)}, ${lon.toFixed(5)}`;
    $('coordinates').classList.remove('gps-error');
    if ($('gpsWarning')) $('gpsWarning').classList.add('hidden');
  } else {
    $('coordinates').textContent = 'Signal not found';
    $('coordinates').classList.add('gps-error');
    if ($('gpsWarning')) $('gpsWarning').classList.remove('hidden');
  }

  // Nodes count
  const nodeCount = mesh.nodes !== undefined ? mesh.nodes : (f.nodes !== undefined ? f.nodes : 1);
  $('nodes').textContent = nodeCount;

  // Speed
  const speed = gps.spd !== undefined ? gps.spd : (f.spd || 0);
  $('speed').textContent = Math.round(speed);

  // Airbag status
  const isDeployed = f.airbag ? f.airbag.deployed : (f.accident === 1 && (f.confidence || 0) > 50);
  const airbagStatus = f.airbag ? f.airbag.status : (isDeployed ? 'DEPLOYED' : 'ARMED');
  $('airbag').textContent = airbagStatus;
  $('airbag').style.color = isDeployed ? '#ff5e67' : '#35d58b';

  // Crash Confidence
  const confidence = Math.round(
    f.confidence !== undefined ? f.confidence : ((f.cc !== undefined ? f.cc : 0) * 100)
  );
  $('confidenceText').textContent = `${confidence}%`;
  $('confidenceBar').style.width = `${Math.min(100, Math.max(0, confidence))}%`;
  $('confidenceBar').style.background = (f.crash || f.accident === 1) ? '#ff5e67' : '#ffb24c';

  // Status & Event description
  const isAccident = Boolean(f.crash || f.accident === 1);
  if (isAccident) {
    $('event').textContent = f.local
      ? '🚨 Local collision impact telemetry detected!'
      : (f.remote ? '🚨 Remote mesh collision alert received!' : '🚨 ACCIDENT DETECTED - Emergency impact telemetry active.');
  } else {
    $('event').textContent = 'Monitoring normal vehicle movement.';
  }

  // Connection Header Status
  if (data.status) {
    $('connection').innerHTML = `<i></i> ${data.status.toUpperCase()}`;
  }

  // Mesh & GPS Information
  const nodeId = f.node || 'RoadMesh-ESP32';
  $('selfNode').textContent = typeof nodeId === 'string' && nodeId.startsWith('RoadMesh') ? nodeId.replace('RoadMesh-', '') : `#${nodeId}`;
  
  const satCount = gps.sat !== undefined ? gps.sat : (f.sat || 0);
  if (hasGpsSignal) {
    $('gpsFixText').textContent = satCount ? `● FIX (${satCount} Sats)` : '● FIX';
    $('gpsFixText').style.color = '#35d58b';
  } else {
    $('gpsFixText').textContent = '○ Signal not found';
    $('gpsFixText').style.color = '#ff5e67';
  }

  $('netPkts').textContent = `${stats.rx || f.seq || 0} / ${stats.tx || 0}`;
  $('netFwd').textContent = `${stats.forwarded || 0}`;
  $('netRssi').textContent = `${stats.rssi || -55} dBm / ${stats.snr || 9.5} dB`;
  $('netLast').textContent = stats.received && stats.received !== 'None' ? stats.received : (stats.sent || `${new Date().toLocaleTimeString()}`);
  $('loraTransport').textContent = `${(stats.transport || data.protocol || 'Bluetooth BLE').toUpperCase()}`;

  // Remote Alert Banner
  if (f.remote && f.remoteNode) {
    $('remoteNodeId').textContent = `#${f.remoteNode}`;
    $('remoteConfVal').textContent = `${Math.round(f.remoteConfidence || 0)}%`;
    $('remoteBanner').classList.remove('hidden');
  } else {
    $('remoteBanner').classList.add('hidden');
  }

  // Leaflet Map Marker & Route Update via GPS Module
  if (hasGpsSignal) {
    marker.setLatLng([lat, lon]);
    const points = frames
      .map(x => [Number(x.gps?.lat ?? x.lat ?? 0), Number(x.gps?.lon ?? x.lon ?? 0)])
      .filter(([la, lo]) => la !== 0 && lo !== 0 && !isNaN(la) && !isNaN(lo) && Math.abs(la) <= 90 && Math.abs(lo) <= 180);
    if (points.length) {
      route.setLatLngs(points);
    }
    if (!map.getBounds().contains([lat, lon])) {
      map.panTo([lat, lon], { animate: true });
    }
  }

  // Draw Waveforms
  draw($('accel'), frames, ['ax', 'ay', 'az'], 2, true);
  draw($('gyro'), frames, ['gx', 'gy', 'gz'], 400, false);

  // Trigger accident event handler if new collision detected
  if (isAccident && f.ts !== lastCrashTime) {
    lastCrashTime = f.ts;
    recordAccidentEvent(f);
  }
}

// -------------------------------------------------------------
// Web Bluetooth Integration (using modular static/bluetooth.js)
// -------------------------------------------------------------
const ble = window.roadMeshBLE;

if (ble) {
  // 1. Incoming Telemetry Callback
  ble.onTelemetry = (telemetry, raw) => {
    const frame = {
      ts: telemetry.ts,
      seq: telemetry.seq,
      node: telemetry.node,
      lat: telemetry.lat,
      lon: telemetry.lon,
      sat: telemetry.sat,
      spd: telemetry.spd,
      alt: telemetry.alt,
      fix: telemetry.fix,
      gps: {
        lat: telemetry.lat,
        lon: telemetry.lon,
        sat: telemetry.sat,
        spd: telemetry.spd,
        alt: telemetry.alt,
        ok: Boolean(telemetry.fix)
      },
      ax: telemetry.ax,
      ay: telemetry.ay,
      az: telemetry.az,
      acc: telemetry.acc,
      gx: telemetry.gx,
      gy: telemetry.gy,
      gz: telemetry.gz,
      gyro: telemetry.gyro,
      imu: {
        ax: telemetry.ax,
        ay: telemetry.ay,
        az: telemetry.az,
        acc: telemetry.acc,
        gx: telemetry.gx,
        gy: telemetry.gy,
        gz: telemetry.gz,
        gyro: telemetry.gyro,
        dynamic: telemetry.acc
      },
      nodes: telemetry.nodes,
      mesh: {
        nodes: telemetry.nodes,
        mask: (1 << telemetry.nodes) - 1
      },
      airbag: {
        deployed: telemetry.accident === 1 && telemetry.confidence > 50,
        status: telemetry.accident === 1 && telemetry.confidence > 50 ? 'DEPLOYED' : 'ARMED'
      },
      confidence: telemetry.confidence,
      cc: telemetry.confidence / 100,
      accident: telemetry.accident,
      crash: telemetry.accident === 1,
      local: telemetry.accident === 1,
      remote: false,
      stats: {
        rx: telemetry.seq || 0,
        tx: 1,
        forwarded: 0,
        dropped: 0,
        rssi: -50,
        snr: 10.0,
        transport: 'Web Bluetooth GATT',
        received: `Seq #${telemetry.seq || 0}`
      }
    };

    bleFrames.push(frame);
    if (bleFrames.length > 120) bleFrames.shift();

    // Update existing dashboard in real time without reloading
    update({
      frames: bleFrames,
      status: `BLE CONNECTED: ${telemetry.node}`,
      protocol: 'Web Bluetooth'
    });
  };

  // 2. Accident Notification Callback
  ble.onAccident = (telemetry) => {
    recordAccidentEvent(telemetry);
  };

  // 3. Bluetooth Status Changes
  ble.onStatusChange = (info) => {
    const conn = $('connection');
    conn.innerHTML = `<i></i> ${info.status.toUpperCase()}`;

    if (info.isConnected) {
      conn.className = 'status';
      $('btnResetAccident').style.display = 'inline-block';
      $('bleConnect').textContent = '✔ ESP32 Connected';
      $('bleConnect').style.background = '#059669';
    } else if (info.isError) {
      conn.className = 'status error clickable';
      $('btnResetAccident').style.display = 'none';
      $('bleConnect').textContent = '⚡ Connect ESP32';
      $('bleConnect').style.background = '#2563eb';
    } else {
      conn.className = 'status warn';
      $('btnResetAccident').style.display = 'none';
      $('bleConnect').textContent = '⚡ Connect ESP32';
      $('bleConnect').style.background = '#2563eb';
    }
  };

  // 4. Bluetooth Error Callback
  ble.onError = (errorObj) => {
    console.error('[RoadMesh BLE App Error]:', errorObj);
    if (errorObj.type === 'UNSUPPORTED_BROWSER') {
      alert(errorObj.message);
    }
  };

  // 5. Disconnect Callback
  ble.onDisconnect = () => {
    $('connection').innerHTML = `<i></i> BLUETOOTH DISCONNECTED (CLICK TO RECONNECT)`;
    $('connection').className = 'status error clickable';
    $('btnResetAccident').style.display = 'none';
    $('bleConnect').textContent = '⚡ Reconnect ESP32';
    $('bleConnect').style.background = '#2563eb';
  };
}

// Connect ESP32 Button Handler
async function handleBleConnectClick() {
  if (!ble) {
    alert('Bluetooth module not loaded.');
    return;
  }

  if (ble.isConnected) {
    if (confirm(`Currently connected to ${ble.deviceName}. Do you want to disconnect?`)) {
      await ble.disconnect();
    }
    return;
  }

  try {
    await ble.connect();
  } catch (err) {
    console.warn('[BLE Connect Cancelled or Failed]:', err.message);
  }
}

// Reset Accident Handler
async function handleResetAccident() {
  if (ble && ble.isConnected) {
    const ok = await ble.resetAccident();
    if (ok) {
      $('event').textContent = 'Accident reset command (RESET_ACCIDENT) sent to ESP32.';
    }
  } else {
    // If running in demo mode or without BLE, reset locally
    $('event').textContent = 'Accident alert dismissed locally.';
  }

  $('alert').classList.add('hidden');
  $('confidenceBar').style.background = '#ffb24c';
}

// Attach Event Listeners
$('bleConnect').onclick = handleBleConnectClick;
$('btnResetAccident').onclick = handleResetAccident;
if ($('modalResetAccident')) $('modalResetAccident').onclick = handleResetAccident;
if ($('modalClose')) $('modalClose').onclick = () => $('alert').classList.add('hidden');
$('dismiss').onclick = () => $('alert').classList.add('hidden');

// Clicking the status element when in error/disconnected state triggers reconnect
$('connection').onclick = () => {
  if (!ble.isConnected && !ble.isConnecting) {
    handleBleConnectClick();
  }
};

// Mode Switcher Handler
$('mode').onchange = () => {
  const mode = $('mode').value;
  if (mode === 'serial') {
    $('portLabel').style.display = 'grid';
    startPolling();
  } else if (mode === 'ble') {
    $('portLabel').style.display = 'none';
    handleBleConnectClick();
  } else {
    $('portLabel').style.display = 'none';
    startPolling();
  }
};

// Connect Button (for Serial / Demo via HTTP Backend)
$('connect').onclick = async () => {
  const mode = $('mode').value;
  if (mode === 'ble') {
    return handleBleConnectClick();
  }

  const body = {
    mode: mode,
    protocol: 'Bluetooth',
    port: $('port').value
  };

  try {
    const r = await fetch('/api/connection', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    });
    const d = await r.json();
    $('connection').innerHTML = `<i></i> ${(d.message || 'Connecting…').toUpperCase()}`;
    startPolling();
  } catch (err) {
    $('connection').innerHTML = `<i></i> ERROR: ${err.message.toUpperCase()}`;
    $('connection').className = 'status error';
  }
};

// Polling fallback for Demo / Serial modes when BLE is not active
async function poll() {
  if (ble && ble.isConnected) return; // Prioritize live Web Bluetooth stream
  try {
    const r = await fetch('/api/telemetry');
    if (r.ok) {
      const data = await r.json();
      update(data);
    }
  } catch (e) {
    // Ignore network polling glitches
  }
}

function startPolling() {
  if (pollInterval) clearInterval(pollInterval);
  pollInterval = setInterval(poll, 200);
}

// Window resize and initialization
window.addEventListener('resize', () => {
  if (lastFrames.length) {
    draw($('accel'), lastFrames, ['ax', 'ay', 'az'], 2, true);
    draw($('gyro'), lastFrames, ['gx', 'gy', 'gz'], 400, false);
  }
});

// Setup map and start app
setupMap();
startPolling();
