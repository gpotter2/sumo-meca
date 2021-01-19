#!/bin/bash
if [[ ! -f "CM530.bin" ]]; then
  echo "CM530.bin missing !";
  exit 1
fi
sudo stty 57600 cs8 -parenb -F /dev/ttyUSB0
sudo screen /dev/ttyUSB0 57600,cs8
