#!/bin/bash
arduino-cli compile --fqbn arduino:avr:uno --libraries ./lib -p /dev/ttyACM0 -u . && \
arduino-cli monitor -p /dev/ttyACM0 --config baudrate=9600