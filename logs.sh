#!/bin/bash
if [[ ! -e "/dev/ttyUSB0" ]]; then
  echo "Appareil non connecté!";
  exit 1
fi
sudo stty 57600 cs8 -parenb -F /dev/ttyUSB0
sudo cat /dev/ttyUSB0
