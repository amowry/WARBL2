# WARBL Sensor Streaming Protocol

**Developer specification --- version 3**

Reference firmware: WARBL2 4.8 with sensor streaming\
Registered MIDI manufacturer ID: 00 03 0F

## 1. OVERVIEW

The protocol provides raw tonehole and breath pressure readings, Y-axis acceleration, roll, elevation, yaw, calibration data, sequence numbers and device timestamps over USB MIDI or BLE MIDI. Streaming is disabled by default. Conventional performance MIDI can optionally be suppressed.

## 2. MESSAGE FRAMING

`F0 00 03 0F VV TT [PAYLOAD] F7`

**Byte index (zero-based, including F0):**

``` text
  0       F0       SysEx start
  1–3     00 03 0F Manufacturer ID
  4       03       Protocol version (outgoing)
  5       TT       Message type
  6 onward        Payload
  Last    F7       SysEx end
```

**Message types (hexadecimal):**

``` text
  01  STREAM_FRAME         WARBL -> host (USB individual frame)
  02  STREAM_BATCH         WARBL -> host (BLE batch)
  10  STREAM_INFO          WARBL -> host (calibration and information)
  20  STREAM_CONFIG        Host -> WARBL
  21  STREAM_REQUEST_INFO  Host -> WARBL
```

Choose the decoder using the message type, not the MIDI port name or apparent transport. A BLE bridge may appear as a USB MIDI device.

## 3. CONTROL AND KEEPALIVE

Configuration: F0 00 03 0F 03 20 CC F7 CC is one ordinary 7-bit MIDI data byte:

``` text
  Bit 0 (01): Enable streaming.
  Bit 1 (02): Suppress conventional performance MIDI.
  Bit 2 (04): Legacy keepalive flag (optional in v3).
  Bits 3–6: Reserved; send zero.
```

``` text
Start, normal MIDI enabled:    F0 00 03 0F 03 20 05 F7
Start, normal MIDI suppressed: F0 00 03 0F 03 20 07 F7
Stop:                          F0 00 03 0F 03 20 00 F7
Request information:           F0 00 03 0F 03 21 F7
```

Repeat the active configuration approximately once per second, preserving the desired suppression flag. Every active stream needs renewal regardless of bit 2. Streaming stops when the last renewal is more than 3000 ms old or the relevant connection is lost. Normal MIDI operation resumes. The subsequent local inactivity-timer change also prevents automatic power-down while active configuration messages continue to arrive.

The input transport of an enabling command selects the output transport. Only one stream is active at a time; a command on another transport can switch it. There is no separate transport-selection byte or payload flag.

## 4. BIT PACKING

WARBL-generated payloads are consecutive MSB-first fields, packed into 7-bit bytes. Within each byte, bits are consumed from bit 6 to bit 0. Bit 7 is always zero. There is no alignment between fields. Unused low bits of the final byte are zero. Packed length = ceil(total bits / 7). For example, the 14-bit value 0x1234 on a 7-bit boundary is 24 34 (hex). Signed 16-bit fields use two's complement.

**JavaScript reference reader (message includes F0 and F7):**

``` javascript
class BitReader {
    constructor(bytes) { this.bytes = bytes; this.position = 0; }
    get(bits) {
        if (this.position + bits > this.bytes.length * 7)
            throw new Error('Truncated payload');
        let value = 0;
        for (let i = 0; i < bits; i++) {
            const p = this.position++;
            value = value * 2 + ((this.bytes[Math.floor(p / 7)] >>
                     (6 - p % 7)) & 1);
        }
        return value;
    }
    getSigned16() {
        const v = this.get(16);
        return v >= 32768 ? v - 65536 : v;
    }
}
const reader = new BitReader(message.slice(6, -1));
```

## 5. USB INDIVIDUAL FRAME (01)

One frame per main loop, nominally 2 ms (500 Hz), or 3 ms when BLE is also connected. Offsets below are bit offsets into the payload.

  Field               Offset   Bits   Representation
  ------------------- -------- ------ ---------------------
  Sequence            0        14     Unsigned
  Timestamp           14       21     Unsigned
  Toneholes 0--8      35       90     9 x unsigned 10-bit
  Pressure            125      14     Unsigned
  Acceleration Y      139      16     Signed
  Roll                155      16     Signed
  Elevation           171      16     Signed
  Yaw                 187      16     Signed
  Covered-hole mask   203      9      Bitmask
  Status flags        212      7      Bitmask

219 payload bits; 32 packed bytes; 39 total SysEx bytes.

**Reference decoding:**

``` javascript
const sequence = reader.get(14);
const timestamp = reader.get(21);
const holes = Array.from({length: 9}, () => reader.get(10));
const pressure = reader.get(14);
const accelY = reader.getSigned16();
const roll = reader.getSigned16() * 180 / 32767;
const elevation = reader.getSigned16() * 180 / 32767;
const yaw = reader.getSigned16() * 180 / 32767;
const coveredMask = reader.get(9);
const flags = reader.get(7);
```

## 6. BLE BATCH (02)

Nominal acquisition interval: 3 ms (333 samples/s). Pressure and holes retain full resolution. Motion is reported once per batch; no covered mask.

  Field             Offset   Bits   Representation
  ----------------- -------- ------ ----------------
  First sequence    0        14     Unsigned
  First timestamp   14       21     Unsigned
  Last timestamp    35       21     Unsigned
  Sample count      56       5      Unsigned
  Acceleration Y    61       16     Signed
  Roll              77       16     Signed
  Elevation         93       16     Signed
  Yaw               109      16     Signed
  Status flags      125      7      Bitmask

Fixed header: 132 bits. Then N samples, oldest first, each consisting of:

``` text
Pressure: 14 bits unsigned
Toneholes 0–8: 9 x 10 bits unsigned
```

Total: 104 bits per sample. Note pressure comes BEFORE holes in BLE, unlike the USB format.

SysEx bytes = 7 + ceil((132 + 104\*N)/7) BLE notification bytes = SysEx bytes + 3

  Samples   SysEx bytes   BLE bytes
  --------- ------------- -----------
  1         41            44
  2         56            59
  3         71            74
  4         86            89
  5         101           104
  8         145           148
  10        175           178
  14        234           237
  16        264           267

Internal maximum: 16 samples. Actual maximum depends on ATT MTU. MTU 247 allows 244 notification bytes and thus 14 samples per batch. WARBL attempts to send at the connection interval or sooner if the next sample would exceed the packet limit. Batches are sent as single BLE MIDI notifications; failed transmissions can lose an entire batch.

**Reference decoding:**

``` javascript
const firstSequence = reader.get(14);
const firstTimestamp = reader.get(21);
const lastTimestamp = reader.get(21);
const count = reader.get(5);
const accelY = reader.getSigned16();
const roll = reader.getSigned16() * 180 / 32767;
const elevation = reader.getSigned16() * 180 / 32767;
const yaw = reader.getSigned16() * 180 / 32767;
const flags = reader.get(7);
if (count < 1 || count > 16) throw new Error('Invalid count');
const MOD = 2097152;
const span = (lastTimestamp - firstTimestamp + MOD) % MOD;
const samples = [];
for (let i = 0; i < count; i++) {
    const pressure = reader.get(14);
    const holes = Array.from({length: 9}, () => reader.get(10));
    samples.push({
        sequence: (firstSequence + i) % 16384,
        timestamp: (firstTimestamp + (count === 1 ? 0 :
                    Math.round(span * i / (count - 1)))) % MOD,
        pressure, holes
    });
}
```

Intermediate timestamps are estimates. Motion is the latest available IMU snapshot at transmission, approximately corresponding to the final sample.

## 7. SENSOR INTERPRETATION

Toneholes: unsigned 0--1023, before adaptive smoothing and baseline subtraction. Wire order is 0 through 8, regardless of display order.

``` text
  8 Thumb; 7 L1; 6 L2; 5 L3; 4 R1; 3 R2; 2 R3; 1 R4; 0 Bell.
```

Pressure: unsigned 0--16383. Both Honeywell analog and BMP585 digital sensors use nominal sensitivity 2.184 counts/Pa (0.457875 Pa/count). Nominal ambient reference is 1600; Honeywell uses its measured reference, while BMP585 uses 1600 after ambient compensation. Host must use the ambientPressure14 value from STREAM_INFO.

``` text
relativeCounts = pressure - ambientPressure14;
pressurePa = relativeCounts / 2.184;
```

Negative values are valid. At ambient 1600, nominal representable range is approximately -733 to +6769 Pa, with clipping at the endpoints.

Acceleration Y: signed 16-bit raw IMU register (-32768 to 32767), not physical acceleration units. Use the LSM6DS driver range code reported in STREAM_INFO to convert to physical units when needed.

Orientation: signed 16-bit values representing degrees wrapped into \[-180,180\]. Firmware variables: roll, pitch (elevation), axisHeading (yaw).

``` text
encoded = round(degrees * 32767 / 180)
degrees = encoded * 180 / 32767
```

Resolution \~0.0055 degrees/count, not measurement accuracy. Account for angle wrapping during interpolation. Non-finite firmware angles encode as 0. Normal IMU output data rate is 208 Hz; readIMU runs approximately every 5 ms subject to loop scheduling. USB samples may repeat motion values; BLE reports one snapshot per batch. Gyro is enabled during streaming and returns to preset-dependent power management when stopped. No raw gyro values are transmitted.

Covered mask (USB only): bit i indicates conventional hole i is covered.

``` text
covered = (coveredMask & (1 << i)) !== 0;
```

BLE hosts can estimate coverage using calibration and raw readings.

Status flags:

``` text
  Bit 0 (01): New tonehole readings acquired.
  Bit 1 (02): Invalid tonehole transfer rejected.
  Bits 2–6: Reserved, currently zero.
Rejected transfers retain previous valid readings. USB flags apply per
frame; BLE flags are ORed across the batch. Both bits can be set, and a
set updated bit does not mean all BLE samples were fresh.
```

## 8. SEQUENCE AND TIMESTAMPS

Sequence: 14 bits, modulus 16384. Increments per generated sample, not per message. BLE sequence i = (firstSequence + i) % 16384.

``` text
step = (currentSequence - previousSequence + 16384) % 16384;
step=1: continuous; step=0: possible duplicate; step>1: possible loss.
```

Initialize from first received sample; counter need not start at zero. Reset tracking across reconnects/restarts. Large gaps can be resets.

Timestamp: RTC2 counter, 21 bits, 32768 Hz, wraps every 64 seconds. Tick duration approximately 30.5176 microseconds.

``` text
ticks = (currentTimestamp - previousTimestamp + 2097152) % 2097152;
elapsedMs = ticks * 1000 / 32768;
```

Use device timestamps rather than host receipt time to measure acquisition intervals. BLE and host scheduling may cause arrival jitter without loss.

## 9. STREAM_INFO (10)

Request: F0 00 03 0F 03 21 F7 (no payload). Response format identical over both transports. All fields are 16 bits.

  Field                               Bit offset   Bits
  ----------------------------------- ------------ ------
  Firmware VERSION                    0            16
  Tonehole baselines 0--8             16           144
  Tonehole covered thresholds 0--8    160          144
  Legacy sensorCalibration            304          16
  Accelerometer range code            320          16
  Gyroscope range code                336          16
  Expected loop period (ms)           352          16
  14-bit ambient pressure reference   368          16

384 bits; 55 packed bytes; 62 total SysEx bytes.

**Reference decoding:**

``` javascript
const firmwareVersion = reader.get(16);
const baseline = Array.from({length: 9}, () => reader.get(16));
const coveredThreshold = Array.from({length: 9}, () => reader.get(16));
const legacyPressureCalibration = reader.get(16);
const accelRangeCode = reader.get(16);
const gyroRangeCode = reader.get(16);
const expectedLoopMs = reader.get(16);
const ambientPressure14 = reader.get(16);
```

Firmware VERSION is distinct from protocol version. Range codes are native driver enumerations, not literal physical range values. Expected loop period is a prediction (2 or 3 ms), not a measurement. Use the 14-bit ambient reference instead of legacy pressure calibration. Covered thresholds are relative to baseline. Initial coverage estimate:

``` text
coverage = Math.max(0, Math.min(1,
    (rawHole - baseline[i]) / coveredThreshold[i]));
```

Guard against zero denominator. This does not reproduce WARBL's full adaptive hole detection. Request INFO explicitly. When streaming, requests must use the active transport; otherwise WARBL replies on the requesting transport.

## 10. TRANSPORT COMPARISON AND THROUGHPUT

USB: individual frames, nominally 500 Hz; full pressure/holes, per-frame motion, timestamps, flags and mask. BLE: batches, nominally 333 Hz; full pressure/holes, shared motion and flags, first/last timestamps, no mask. USB may run at 3 ms when BLE is connected. Calibration format is identical. Nominal SysEx throughput (decimal kB/s, excluding transport overhead):

``` text
  USB 500 Hz: 19.50
  BLE 3 samples/batch: 7.89
  BLE 5 samples/batch: 6.73
  BLE 10 samples/batch: 5.83
```

## 11. HOST IMPLEMENTATION

Provide manual input and output port selection; never require WARBL in the device name. Enable SysEx access. Start the stream, request INFO, decode by type, renew each second, and send stop when finished.

**Web MIDI example:**

``` javascript
const midi = await navigator.requestMIDIAccess({sysex: true});
const HEADER = [0xF0, 0x00, 0x03, 0x0F, 0x03];
let keepaliveTimer = null;
let suppressMIDI = true;
function sendConfig(output, enabled) {
    const config = enabled ? (0x05 | (suppressMIDI ? 0x02 : 0)) : 0;
    output.send([...HEADER, 0x20, config, 0xF7]);
}
function requestInfo(output) {
    output.send([...HEADER, 0x21, 0xF7]);
}
function startStream(output) {
    clearInterval(keepaliveTimer);
    sendConfig(output, true);
    requestInfo(output);
    keepaliveTimer = setInterval(() => sendConfig(output, true), 1000);
}
function stopStream(output) {
    clearInterval(keepaliveTimer);
    keepaliveTimer = null;
    sendConfig(output, false);
}
```

Validate framing, manufacturer ID, version, type, 7-bit payload bytes, expected length, batch count and zero padding before decoding. Ignore unknown types safely. Reassemble fragmented SysEx at lower MIDI layers. No separate application-level checksum is defined.

## 12. VERSION COMPATIBILITY

Original experimental v2 used 7D 57, 8-bit BLE holes, XYZ acceleration (reduced precision over BLE), raw USB gyro and no calculated angles. V3 uses registered ID, 10-bit holes on both transports, signed 16-bit Y acceleration and roll/elevation/yaw, no raw gyro, normal 208 Hz IMU rate and nominal 3 ms BLE loop rather than 2 ms. Existing v2 hosts need updated recognition and decoding. Firmware accepts incoming command versions 1 through 3 with the registered ID, but always emits version 3. Older command versions do not negotiate an older data layout.

## 13. REFERENCE IMPLEMENTATION AND VALIDATION

WARBL_stream_monitor_v3.html is the reference diagnostic host: manual port selection, configuration, keepalive, calibration, sensor graphs, sequence gaps, timing and raw SysEx monitoring. User-reported testing: Windows USB, Windows BLE and iOS BLE all working.

Documentation notes: wire layouts and command behavior checked against Sensor_stream.ino in WARBL48_stream_integration.zip. The BMP585 offset and inactivity-timer renewal reflect subsequent local firmware edits, not that archive. Pressure conversion factor and IMU physical range code interpretation should be confirmed before definitive publication.
