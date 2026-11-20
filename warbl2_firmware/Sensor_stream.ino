// WARBL sensor streaming protocol v3 (registered WARBL manufacturer ID 00 03 0F).
// F0 00 03 0F <version> <type> <packed payload> F7; 7-bit-safe, MSB-first bit packing.
// USB FRAME: seq14, timestamp21, holes[9]*10, pressure14, rawAccelY16,
//             roll16, pitch16, axisHeading16, coveredMask9, flags7.
// BLE BATCH: firstSeq14, firstTimestamp21, lastTimestamp21, count5,
//            rawAccelY16, roll16, pitch16, axisHeading16, flags7,
//            then count * (pressure14, holes[9]*10).
// Angles: signed 16-bit two's complement, degrees * (32767/180), wrapped
// into [-180,180]. Acceleration Y: raw signed IMU register (not m/s^2).
// BLE IMU snapshot is taken at batch transmission, near lastTimestamp.
// INFO v3 retains v2's 16-bit fields (including 14-bit ambient pressure).
// USB targets ~2 ms; BLE targets ~3 ms. Host must use timestamps, not assumed rate.
// Stream starts off; host enables via CONFIG and renews every second (3 s timeout).
// IMPORTANT: stock WARBL's newer BMP sensor must be converted to the same 14-bit
// unsigned stream scale and ambient offset in its pressure acquisition path.

#define STREAM_SYSEX_ID_1 0x00
#define STREAM_SYSEX_ID_2 0x03
#define STREAM_SYSEX_ID_3 0x0F
#define STREAM_PROTOCOL_VERSION 3     // 3: 10-bit BLE holes, Y accel + three fixed-point orientation angles
#define STREAM_COMMAND_VERSION_MIN 1  // Host commands are unchanged since version 1

#define STREAM_FRAME 0x01
#define STREAM_BATCH 0x02
#define STREAM_INFO 0x10
#define STREAM_CONFIG 0x20
#define STREAM_REQUEST_INFO 0x21

#define STREAM_CONFIG_ON 0x01
#define STREAM_CONFIG_SUPPRESS_NOTES 0x02
#define STREAM_CONFIG_KEEPALIVE 0x04
#define STREAM_KEEPALIVE_MS 3000  // A keep-alive stream stops this long after the host's last STREAM_CONFIG

bool streamKeepAlive = false;       // The host renews the stream (STREAM_CONFIG_KEEPALIVE)
unsigned long streamRenewedMs = 0;  // When it last did
bool streamBatchReset = true;       // Start the next BLE batch afresh (the stream started, stopped or changed transport)

#define STREAM_MAX_SYSEX 64          // Large enough for v3 USB frames and STREAM_INFO.
#define STREAM_BATCH_MAX_SAMPLES 16  // Actual maximum is also constrained by negotiated ATT MTU.

// Packs values MSB-first into 7-bit SysEx data bytes.
struct SysExBitWriter {
    byte* out;
    byte length = 0;
    byte bitsInByte = 0;

    void put(uint32_t value, byte bits) {
        for (int b = bits - 1; b >= 0; b--) {
            if (bitsInByte == 0) {
                out[length++] = 0;
            }
            out[length - 1] |= ((value >> b) & 1) << (6 - bitsInByte);
            bitsInByte = (bitsInByte + 1) % 7;
        }
    }
};



// Signed 16-bit fixed-point orientation: -180 degrees -> -32767, +180 -> +32767.
// Roll can extend beyond 180 degrees in the WARBL firmware, so wrap all angles.
int16_t streamAngle16(float degrees) {
    if (!isfinite(degrees)) return 0;
    degrees = fmodf(degrees, 360.0f);
    if (degrees > 180.0f) degrees -= 360.0f;
    if (degrees < -180.0f) degrees += 360.0f;
    return (int16_t)lroundf(degrees * (32767.0f / 180.0f));
}

// Keep the existing pressure filter and 12-bit path outside streaming.
void readStreamAnalogPressure(bool highResolution) {
    if (highResolution) {
        analogReadResolution(14);
#ifdef PROTOTYPE46
        fourteenBitPressure = analogRead(A0);
#else
        fourteenBitPressure = analogRead(A1);
#endif
        analogReadResolution(12);
        twelveBitPressure = fourteenBitPressure >> 2;
        analogPressure.update(twelveBitPressure);
    } else {
        analogPressure.update();
        twelveBitPressure = analogPressure.getRawValue();
        fourteenBitPressure = constrain(twelveBitPressure * 4, 0, 16383);
    }
}

void sendStreamSysEx(byte type, const byte* payload, byte payloadLength) {
    // Write complete messages to TinyUSB, rather than one SysEx byte per write.
    byte message[240] = { 0xF0, STREAM_SYSEX_ID_1, STREAM_SYSEX_ID_2, STREAM_SYSEX_ID_3, STREAM_PROTOCOL_VERSION, type };
    if (payloadLength + 7 > sizeof(message)) {
        return;
    }
    memcpy(message + 6, payload, payloadLength);
    message[6 + payloadLength] = 0xF7;
    if (sensorStreamOverBLE) {
        blemidi.sendSysExMessage(message, payloadLength + 7);  // INFO/control responses may be fragmented
    } else {
        tud_midi_stream_write(0, message, payloadLength + 7);
    }
}



// Exact BLE MIDI notification length for a single complete STREAM_BATCH.
// 3 BLE-MIDI framing bytes, 7 SysEx framing bytes, 7-bit-packed payload.
uint16_t batchPacketBytes(byte samples) {
    const uint32_t bits = 14 + 21 + 21 + 5 + 16 * 4 + 7 + (14 + 9 * 10) * (uint32_t)samples;
    return 10 + (bits + 6) / 7;
}

void addBatchSample(uint16_t& seq) {
    static uint16_t holes[STREAM_BATCH_MAX_SAMPLES][9];
    static uint16_t pressures[STREAM_BATCH_MAX_SAMPLES];
    static byte count = 0;
    static uint16_t firstSeq = 0;
    static uint32_t firstTime = 0, lastTime = 0;
    static uint8_t flags = 0;
    static unsigned long lastSendMs = 0;

    if (streamBatchReset) {  // No samples or timestamps left over from an earlier session
        streamBatchReset = false;
        count = 0;
        flags = 0;
        lastSendMs = 0;
    }

    if (connIntvl == 0 || !blemidi.notifyEnabled()) {
        count = 0;
        return;
    }

    if (count == 0) {
        firstSeq = seq;
        firstTime = NRF_RTC2->COUNTER & 0x1FFFFF;
        flags = 0;
    }
    lastTime = NRF_RTC2->COUNTER & 0x1FFFFF;
    for (byte i = 0; i < 9; i++) {
        holes[count][i] = (uint16_t)constrain(toneholeRaw[i], 0, 1023);
    }
    pressures[count] = fourteenBitPressure & 0x3FFF;
    flags |= (toneholeRawUpdated ? 1 : 0) | (toneholeRawRejected ? 2 : 0);
    toneholeRawUpdated = false;
    toneholeRawRejected = false;
    seq++;
    count++;

    // Send once per connection interval, or earlier if one more sample wouldn't fit one BLE packet: a batch always
    // goes in a single packet, so if it can't be sent it is lost whole, never cut off in the middle of a SysEx (which
    // would leave the host's BLE MIDI parser waiting for its end, swallowing the notes that follow).
    const unsigned long now = millis();
    const bool full = count >= STREAM_BATCH_MAX_SAMPLES || batchPacketBytes(count + 1) > blemidi.maxPacketLength();
    if (!full && (now - lastSendMs) < (unsigned long)connIntvl) {
        return;
    }
    if (batchPacketBytes(count) > blemidi.maxPacketLength()) {  // A tiny MTU: even this doesn't fit one packet
        count = 0;
        return;
    }
    lastSendMs = now;

    // readIMU() in the normal firmware loop updates rawAccY and the orientation.
    // Avoid an additional synchronous IMU read when transmitting a BLE batch.
    byte payload[260] = { 0 };
    SysExBitWriter w;
    w.out = payload;
    w.put(firstSeq & 0x3FFF, 14);
    w.put(firstTime, 21);
    w.put(lastTime, 21);
    w.put(count, 5);
    w.put((uint16_t)sox.rawAccY, 16);
    w.put((uint16_t)streamAngle16(roll), 16);
    w.put((uint16_t)streamAngle16(pitch), 16);
    w.put((uint16_t)streamAngle16(axisHeading), 16);
    w.put(flags & 0x7F, 7);
    for (byte s = 0; s < count; s++) {
        w.put(pressures[s], 14);
        for (byte i = 0; i < 9; i++) {
            w.put(holes[s][i], 10);
        }
    }
    // A batch must fit one notification; otherwise drop it as a unit.
    byte message[280] = { 0xF0, STREAM_SYSEX_ID_1, STREAM_SYSEX_ID_2, STREAM_SYSEX_ID_3, STREAM_PROTOCOL_VERSION, STREAM_BATCH };
    const uint16_t length = (uint16_t)w.length + 7;
    if (length <= sizeof(message)) {
        memcpy(message + 6, payload, w.length);
        message[length - 1] = 0xF7;
        blemidi.sendSysExSinglePacket(message, length);
    }
    count = 0;
}



void sendSensorFrame() {
    static uint16_t seq = 0;

    if (!sensorStreamOn) {
        return;
    }
    if (streamKeepAlive && millis() - streamRenewedMs > STREAM_KEEPALIVE_MS) {  // The host has gone quiet: back to normal
        streamKeepAlive = false;
        setSensorStream(false, false, false);
        return;
    }
    if (sensorStreamOverBLE) {
        if (!Bluefruit.connected()) {
            setSensorStream(false, false, false);
            return;
        }
        addBatchSample(seq);
        return;
    }
    if (!tud_ready()) {  // The USB host has gone (cable pulled, computer asleep): stop, which also brings BLE back.
        setSensorStream(false, false, false);
        return;
    }

    // Use the latest readIMU() snapshot, avoiding an extra SPI transaction.
    byte payload[STREAM_MAX_SYSEX - 4] = { 0 };
    SysExBitWriter w;
    w.out = payload;

    w.put(seq++ & 0x3FFF, 14);
    w.put(NRF_RTC2->COUNTER & 0x1FFFFF, 21);
    for (byte i = 0; i < 9; i++) {
        w.put(toneholeRaw[i] & 0x3FF, 10);
    }
    w.put(fourteenBitPressure & 0x3FFF, 14);
    w.put((uint16_t)sox.rawAccY, 16);
    w.put((uint16_t)streamAngle16(roll), 16);
    w.put((uint16_t)streamAngle16(pitch), 16);
    w.put((uint16_t)streamAngle16(axisHeading), 16);
    w.put(holeCovered & 0x1FF, 9);
    w.put((toneholeRawUpdated ? 1 : 0) | (toneholeRawRejected ? 2 : 0), 7);  // Flags: new hole readings; a garbled transfer was rejected
    toneholeRawUpdated = false;
    toneholeRawRejected = false;

    sendStreamSysEx(STREAM_FRAME, payload, w.length);
}



void sendSensorStreamInfo() {
    byte payload[STREAM_MAX_SYSEX - 4] = { 0 };
    SysExBitWriter w;
    w.out = payload;

    w.put(VERSION, 16);
    for (byte i = 0; i < 9; i++) {
        w.put(toneholeBaseline[i], 16);
    }
    for (byte i = 0; i < 9; i++) {
        w.put(toneholeCovered[i], 16);
    }
    w.put(sensorCalibration, 16);
    w.put(sox.getAccelRange(), 16);
    w.put(sox.getGyroRange(), 16);
    w.put(connIntvl == 0 ? 2 : 3, 16);  // Loop period in ms, as calculateDelayTime() aims for (a prediction, not a measurement).
    w.put(ambientPressure14 & 0x3FFF, 16);

    sendStreamSysEx(STREAM_INFO, payload, w.length);
}



void setSensorStream(bool on, bool suppressNotes, bool overBLE) {

    if (on && suppressNotes && noteon) {  // Don't leave a note hanging when note output stops.
        sendMIDI(NOTE_OFF, mainMidiChannel, notePlaying, 64);
        noteon = 0;
    }

    sensorStreamOn = on;
    sensorStreamSuppressNotes = on && suppressNotes;
    sensorStreamOverBLE = on && overBLE;
    streamBatchReset = true;

    // Keep the normal 208 Hz accelerometer rate. Gyro follows stream/preset needs.
    updateStreamGyroPower();
}



// Incoming SysEx includes F0 and F7; route by receiving MIDI port.
void handleSysExFromUSB(byte* data, unsigned size) {
    handleStreamSysEx(data, size, false);
}

void handleSysExFromBLE(byte* data, unsigned size) {
    handleStreamSysEx(data, size, true);
}

void handleStreamSysEx(byte* data, unsigned size, bool fromBLE) {
    if (size < 7 || data[0] != 0xF0 || data[size - 1] != 0xF7
        || data[1] != STREAM_SYSEX_ID_1 || data[2] != STREAM_SYSEX_ID_2 || data[3] != STREAM_SYSEX_ID_3
        || data[4] < STREAM_COMMAND_VERSION_MIN || data[4] > STREAM_PROTOCOL_VERSION) {
        return;
    }

    switch (data[5]) {
        case STREAM_CONFIG:
            if (size >= 8) {
                const bool on = data[6] & STREAM_CONFIG_ON, suppress = data[6] & STREAM_CONFIG_SUPPRESS_NOTES;
                streamKeepAlive = on;  // Every stream needs renewing, with or without the (older) keep-alive bit
                streamRenewedMs = millis();
                if (on) powerDownTimer = millis();  // Reset the powerDown timer if streaming.
                if (on != sensorStreamOn || suppress != sensorStreamSuppressNotes || (on && fromBLE != sensorStreamOverBLE)) {
                    setSensorStream(on, suppress, fromBLE);  // A renewal of the same stream only resets the timer above
                }
            }
            break;

        case STREAM_REQUEST_INFO:
            if (fromBLE != sensorStreamOverBLE && sensorStreamOn) {
                break;  // Answer on the transport the stream uses
            }
            {
                const bool wasBLE = sensorStreamOverBLE;
                sensorStreamOverBLE = fromBLE;  // Reply where it was asked
                sendSensorStreamInfo();
                sensorStreamOverBLE = wasBLE;
            }
            break;
    }
}
