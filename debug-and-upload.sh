#!/bin/bash
arduino-cli compile \
  --fqbn arduino:avr:uno \
  --libraries ./lib \
  --build-property "build.extra_flags=-DAIRYAK_DEBUG=1" \
  -p /dev/ttyACM0 \
  -u . && \
arduino-cli monitor -p /dev/ttyACM0 --config baudrate=115200