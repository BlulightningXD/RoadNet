/**
 * RoadMesh Web Bluetooth (BLE) Manager
 *
 * Handles Web Bluetooth connection to ESP32 / ESP32-C5 RoadMesh devices.
 * Connects to the RoadMesh GATT Service, reads live telemetry notifications,
 * and sends commands (PING, RESET_ACCIDENT).
 */

(function(window) {
  'use strict';

  // RoadMesh GATT UUIDs
  const SERVICE_UUID = '7c2a0001-6b4b-4c9a-9a01-726f61646d01';
  const TELEMETRY_CHAR_UUID = '7c2a0002-6b4b-4c9a-9a01-726f61646d01';
  const COMMAND_CHAR_UUID = '7c2a0003-6b4b-4c9a-9a01-726f61646d01';

  class RoadMeshBluetooth {
    constructor() {
      this.device = null;
      this.server = null;
      this.service = null;
      this.telemetryChar = null;
      this.commandChar = null;
      this.isConnected = false;
      this.isConnecting = false;
      this.deviceName = '';
      this.rxBuffer = '';

      // Callbacks
      this.onTelemetry = null;
      this.onStatusChange = null;
      this.onDisconnect = null;
      this.onError = null;
      this.onAccident = null;

      this._boundDisconnect = this._handleDisconnect.bind(this);
      this._boundNotification = this._handleNotification.bind(this);
    }

    /**
     * Check if Web Bluetooth API is supported in the current environment
     */
    isSupported() {
      return typeof navigator !== 'undefined' && 'bluetooth' in navigator;
    }

    /**
     * Update connection status and notify listeners
     */
    _notifyStatus(statusText, isConnected = false, isError = false, extra = {}) {
      if (typeof this.onStatusChange === 'function') {
        this.onStatusChange({
          status: statusText,
          isConnected,
          isError,
          deviceName: this.deviceName,
          ...extra
        });
      }
    }

    /**
     * Trigger error callback with structured info
     */
    _notifyError(type, message, originalError = null) {
      console.error(`[RoadMesh BLE Error - ${type}]:`, message, originalError || '');
      if (typeof this.onError === 'function') {
        this.onError({
          type,
          message,
          originalError
        });
      }
    }

    /**
     * Request device, connect GATT, get service and characteristics, start notifications
     */
    async connect() {
      if (!this.isSupported()) {
        const msg = 'Web Bluetooth is not supported in this browser. Please use Chrome, Edge, or a Chromium browser over HTTPS or localhost.';
        this._notifyStatus('BLUETOOTH UNSUPPORTED', false, true);
        this._notifyError('UNSUPPORTED_BROWSER', msg);
        throw new Error(msg);
      }

      if (this.isConnecting) {
        console.warn('Bluetooth connection is already in progress.');
        return;
      }

      this.isConnecting = true;
      this._notifyStatus('SCANNING FOR ROADMESH ESP32…', false, false);

      try {
        // 1. Request BLE device with namePrefix 'RoadMesh'
        this.device = await navigator.bluetooth.requestDevice({
          filters: [
            { namePrefix: 'RoadMesh' }
          ],
          optionalServices: [
            SERVICE_UUID
          ]
        });

        this.deviceName = this.device.name || 'RoadMesh Device';
        this._notifyStatus(`CONNECTING TO ${this.deviceName}…`, false, false);

        // Listen for unexpected disconnections
        this.device.removeEventListener('gattserverdisconnected', this._boundDisconnect);
        this.device.addEventListener('gattserverdisconnected', this._boundDisconnect);

        // 2. Connect to GATT Server
        this.server = await this.device.gatt.connect();
        if (!this.server || !this.server.connected) {
          throw new Error('Could not establish connection to GATT Server.');
        }

        // 3. Get RoadMesh primary service
        try {
          this.service = await this.server.getPrimaryService(SERVICE_UUID);
        } catch (serviceErr) {
          throw new Error(`RoadMesh GATT Service not found (${SERVICE_UUID}). Ensure ESP32 firmware is updated.`);
        }

        // 4. Get Telemetry Characteristic
        try {
          this.telemetryChar = await this.service.getCharacteristic(TELEMETRY_CHAR_UUID);
        } catch (charErr) {
          throw new Error(`Telemetry characteristic not found (${TELEMETRY_CHAR_UUID}).`);
        }

        // 5. Get Command Characteristic (Optional fallback if not exposed)
        try {
          this.commandChar = await this.service.getCharacteristic(COMMAND_CHAR_UUID);
        } catch (cmdErr) {
          console.warn(`Command characteristic (${COMMAND_CHAR_UUID}) not found or write-restricted.`);
          this.commandChar = null;
        }

        // 6. Enable Notifications on Telemetry Characteristic
        await this.telemetryChar.startNotifications();
        this.telemetryChar.removeEventListener('characteristicvaluechanged', this._boundNotification);
        this.telemetryChar.addEventListener('characteristicvaluechanged', this._boundNotification);

        this.isConnected = true;
        this.isConnecting = false;
        this.rxBuffer = '';

        this._notifyStatus(`CONNECTED: ${this.deviceName}`, true, false);

        // 7. Send PING upon connection
        await this.sendPing();

        return this.device;
      } catch (err) {
        this.isConnecting = false;
        this.isConnected = false;

        let errorType = 'GATT_ERROR';
        let userMessage = err.message || 'Failed to connect to RoadMesh ESP32.';

        if (err.name === 'NotFoundError') {
          errorType = 'USER_CANCELLED';
          userMessage = 'Device selection cancelled.';
          this._notifyStatus('PAIRING CANCELLED', false, false);
        } else if (err.name === 'SecurityError') {
          errorType = 'SECURITY_ERROR';
          userMessage = 'Bluetooth permission denied or blocked by security policy.';
          this._notifyStatus('BLUETOOTH BLOCKED', false, true);
        } else if (err.name === 'NetworkError') {
          errorType = 'CONNECTION_FAILED';
          userMessage = 'Connection failed. ESP32 may be out of range, busy, or disconnected.';
          this._notifyStatus('CONNECTION FAILED', false, true);
        } else {
          this._notifyStatus('BLUETOOTH ERROR', false, true);
        }

        this._notifyError(errorType, userMessage, err);
        throw err;
      }
    }

    /**
     * Handles incoming notification chunks from telemetry characteristic
     */
    _handleNotification(event) {
      try {
        const value = event.target.value;
        const decoder = new TextDecoder('utf-8');
        const chunk = decoder.decode(value);
        this.rxBuffer += chunk;

        // Process line-by-line or extract complete JSON payloads
        let newlineIndex;
        while ((newlineIndex = this.rxBuffer.indexOf('\n')) >= 0) {
          const line = this.rxBuffer.substring(0, newlineIndex).trim();
          this.rxBuffer = this.rxBuffer.substring(newlineIndex + 1);

          if (line) {
            this._processPayload(line);
          }
        }

        // Safety limit on buffer size to prevent memory bloat
        if (this.rxBuffer.length > 4096) {
          const trimmed = this.rxBuffer.trim();
          if (trimmed.startsWith('{') && trimmed.endsWith('}')) {
            this._processPayload(trimmed);
            this.rxBuffer = '';
          } else {
            this.rxBuffer = this.rxBuffer.slice(-512);
          }
        }
      } catch (err) {
        this._notifyError('PARSE_NOTIFICATION_ERROR', 'Error reading BLE notification data.', err);
      }
    }

    /**
     * Parse raw string into JSON and normalize telemetry
     */
    _processPayload(jsonString) {
      try {
        const raw = JSON.parse(jsonString);
        const telemetry = this.normalizeTelemetry(raw);

        if (typeof this.onTelemetry === 'function') {
          this.onTelemetry(telemetry, raw);
        }

        if (telemetry.accident === 1 && typeof this.onAccident === 'function') {
          this.onAccident(telemetry);
        }
      } catch (parseErr) {
        // Ignore partial / invalid JSON frames silently or report
        this._notifyError('INVALID_JSON', `Malformed JSON payload received: ${jsonString.substring(0, 60)}...`, parseErr);
      }
    }

    /**
     * Map raw ESP32 payload to standardized dashboard frame format
     */
    normalizeTelemetry(raw) {
      const ts = raw.ts || Date.now();
      const node = raw.node || (this.deviceName || 'RoadMesh-ESP32');
      const seq = typeof raw.seq === 'number' ? raw.seq : 0;

      // GPS Mapping
      const fix = (raw.fix !== undefined) ? Number(raw.fix) : (raw.gps?.ok ? 1 : 0);
      const lat = (raw.lat !== undefined || raw.gps?.lat !== undefined) ? Number(raw.lat ?? raw.gps?.lat) : 0;
      const lon = (raw.lon !== undefined || raw.lng !== undefined || raw.gps?.lon !== undefined || raw.gps?.lng !== undefined) ? Number(raw.lon ?? raw.lng ?? raw.gps?.lon ?? raw.gps?.lng) : 0;
      const sat = Number(raw.sat ?? raw.gps?.sat ?? 0);
      const alt = Number(raw.alt ?? raw.gps?.alt ?? 0);
      const spd = Number(raw.spd ?? raw.speed ?? raw.gps?.spd ?? 0);

      // Accelerometer Mapping
      const ax = Number(raw.ax ?? raw.imu?.ax ?? 0);
      const ay = Number(raw.ay ?? raw.imu?.ay ?? 0);
      const az = Number(raw.az ?? raw.imu?.az ?? 0);

      // Acc magnitude: use raw.acc or calculate vector magnitude
      let acc = Number(raw.acc ?? raw.imu?.dynamic ?? 0);
      if (!acc && (ax || ay || az)) {
        acc = Math.sqrt(ax * ax + ay * ay + az * az);
      }

      // Gyroscope Mapping
      const gx = Number(raw.gx ?? raw.imu?.gx ?? 0);
      const gy = Number(raw.gy ?? raw.imu?.gy ?? 0);
      const gz = Number(raw.gz ?? raw.imu?.gz ?? 0);

      // Gyro magnitude
      let gyro = Number(raw.gyro ?? raw.imu?.gyro ?? 0);
      if (!gyro && (gx || gy || gz)) {
        gyro = Math.sqrt(gx * gx + gy * gy + gz * gz);
      }

      // Accident & Confidence
      const accident = (raw.accident === 1 || raw.accident === true || raw.crash === true) ? 1 : 0;
      let confidence = Number(raw.confidence ?? 0);
      if (confidence === 0 && raw.cc !== undefined) {
        confidence = Math.round(raw.cc * 100);
      }

      // Mesh Nodes Count
      const nodes = Number(raw.nodes ?? raw.mesh?.nodes ?? 1);

      return {
        ts,
        seq,
        node,
        fix,
        lat,
        lon,
        sat,
        alt,
        spd,
        ax,
        ay,
        az,
        acc,
        gx,
        gy,
        gz,
        gyro,
        accident,
        confidence,
        nodes,
        raw
      };
    }

    /**
     * Send command string to ESP32 command characteristic
     */
    async sendCommand(commandText) {
      if (!this.isConnected || !this.commandChar) {
        console.warn('Cannot send command: BLE not connected or command characteristic unavailable.');
        return false;
      }

      try {
        const encoder = new TextEncoder();
        const data = encoder.encode(commandText.endsWith('\n') ? commandText : `${commandText}\n`);
        
        if (typeof this.commandChar.writeValueWithoutResponse === 'function') {
          await this.commandChar.writeValueWithoutResponse(data);
        } else if (typeof this.commandChar.writeValue === 'function') {
          await this.commandChar.writeValue(data);
        } else if (typeof this.commandChar.writeValueWithResponse === 'function') {
          await this.commandChar.writeValueWithResponse(data);
        }

        console.log(`[RoadMesh BLE] Sent command: ${commandText}`);
        return true;
      } catch (err) {
        this._notifyError('COMMAND_WRITE_ERROR', `Failed to send command "${commandText}": ${err.message}`, err);
        return false;
      }
    }

    /**
     * Send PING command
     */
    async sendPing() {
      return await this.sendCommand('PING');
    }

    /**
     * Send RESET_ACCIDENT command
     */
    async resetAccident() {
      const success = await this.sendCommand('RESET_ACCIDENT');
      if (success) {
        console.log('[RoadMesh BLE] Accident reset command transmitted.');
      }
      return success;
    }

    /**
     * Disconnect cleanly from GATT server
     */
    async disconnect() {
      if (this.telemetryChar) {
        try {
          this.telemetryChar.removeEventListener('characteristicvaluechanged', this._boundNotification);
          await this.telemetryChar.stopNotifications();
        } catch (e) {}
      }

      if (this.device && this.device.gatt && this.device.gatt.connected) {
        try {
          this.device.gatt.disconnect();
        } catch (e) {}
      }

      this._handleDisconnect();
    }

    /**
     * Internal handler for unexpected or intentional disconnection
     */
    _handleDisconnect() {
      const wasConnected = this.isConnected;
      this.isConnected = false;
      this.isConnecting = false;
      this.telemetryChar = null;
      this.commandChar = null;
      this.service = null;
      this.server = null;
      this.rxBuffer = '';

      this._notifyStatus('BLUETOOTH DISCONNECTED (CLICK TO RECONNECT)', false, true);

      if (typeof this.onDisconnect === 'function') {
        this.onDisconnect({ wasConnected, deviceName: this.deviceName });
      }
    }
  }

  // Export to global window scope
  window.RoadMeshBluetooth = RoadMeshBluetooth;
  window.roadMeshBLE = new RoadMeshBluetooth();

})(window);
