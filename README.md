# AirYak

Engine Control Unit (ECU) firmware for the AirYak—a small-scale air-drive propulsion system for kayaks.

## Hardware Compatibility
* **Microcontroller:** Standalone ATmega328P (16 MHz external crystal)

## Dependencies
* [PID](https://github.com/br3ttb/Arduino-PID-Library) (`PID_v1`)
* Arduino `Servo` library

## Installation & Compilation
AirYak is built from the command line using `arduino-cli`:

```bash
# Standard release build
arduino-cli compile --fqbn arduino:avr:uno --libraries ./lib .

# Debug telemetry build (115200 baud serial diagnostics)
arduino-cli compile --fqbn arduino:avr:uno --libraries ./lib \
  --build-property "compiler.cpp.extra_flags=-DAIRYAK_DEBUG" .
```

## Usage
Compile and upload the firmware directly to the custom AirYak ECU PCB.

## Testing
Host-side unit tests use the Unity test framework with mocked Arduino peripherals:

```bash
./run_tests.sh
```

## Architecture & API Reference
AirYak uses the Pointer to Implementation (PIMPL) idiom across modular subsystem classes:
* `AnalogSensor` — Encapsulates an Arduino analog ADC input with exponential moving average (EMA) smoothing.
* `Choke` — Automates cold-start fuel enrichment with a timed linear warmup curve and stall reset.
* `EcoServo` — Manages on-demand PWM attachment windows to reduce servo power draw and linkage buzz.
* `FlywheelMagnetInterrupt` — Captures Hall-effect flywheel pulses via hardware interrupts and measures rotational periods in microseconds.
* `Rudder` — Stabilizes airfoil steering commands using an 8-sample circular moving-average FIR filter and hysteresis.
* `Tachometer` — Converts flywheel rotational periods into smoothed engine RPM readings.
* `Throttle` — Implements closed-loop PID engine speed governing with cranking protection, slew-rate clamping, and overspeed cutout.

## License
MIT