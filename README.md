# AirYak

This project is an engine control unit for the AirYak system.

AirYak is a small-scale air drive propulsion system for kayaks.

## Hardware Compatibility
This engine control unit runs on the ATMega328 chip.

## Installation
The only supported compilation tool for AirYak is command-line via the
arduino-cli utility.

```bash
arduino-cli compile --fqbn arduino:avr:uno --libraries ./lib
```

## Dependencies
AirYak is dependent on the PID and Servo libraries.

## Usage
This proect is intended to be compiled and uploaded to the AirYak PCB.

## API Reference (For Libraries)
AirYak is comprised of multiple classes, each of which govern a certain subsystem:
* `AnalogSensor` - An encapsulation around an arduino analog input sensor.
* `Choke` - A class that encapsulates the complex logic surrounding fuel management.
* `EcoServo` - A class that offers a power-saving mode for servos.
* `FlywheelMagnetInterrupt` - A class that encapsulates a hall effect sensor.
* `Rudder` - A class that encapsulates the complex logic surrounding steering management.
* `Tachometer` - A class that acts as an engine speed monitor.
* `Throttle` - A class that encapsulates the complex logic surrounding engine speed management.

## License
MIT
