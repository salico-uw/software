#!/usr/bin/python

import serial

DEVICE = '/dev/ttyACM0'
BAUD = 1000000

ser = serial.Serial(DEVICE, BAUD)

for i in range(20):
  ser.write('meow'.encode("utf-8"))