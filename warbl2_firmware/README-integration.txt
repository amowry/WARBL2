WARBL 4.8 sensor stream integration (source only; not compiled or hardware tested).
Replace the matching tabs and include WarblBLEMidi.cpp/.h. Manufacturer ID 00 03 0F, version 3, payload layout unchanged from registered-ID archive.
Original globals retained; stream globals grouped before setup. Normal loop delay and IMU rate retained; gyro enabled at 208 Hz when streaming or preset requires it, otherwise shut down. No extra IMU reads.
USB/BLE commands follow receiving port; Bluetooth is not disabled when USB streaming, so stock loop remains 3 ms if BLE is also connected. Suppression preserves config CC range and skips major performance output paths. Review gesture/note state on resume.
Honeywell uses 14-bit ADC only at startup calibration and during streaming; ordinary 12-bit filter path remains. BMP provisional mapping centers at 8192, scale 54.60*4 counts/hPa and clips; verify against real hardware before relying on it. Existing BMP compensation and MIDI path unchanged.
Toneholes captured before filtering/baseline subtraction; bad checksums retain previous raw samples. Batch flags are OR across samples, not per-sample flags.
Review library compatibility (especially rawAccY access and ResponsiveAnalogRead update overload), USB queue handling, BLE queue drops and smaller MTU behavior. Existing stream transport files unchanged. No toolchain or hardware verification performed.
