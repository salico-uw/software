#!/usr/bin/python
import os.path
import serial
import time
import serial.serialutil
from datetime import datetime

DEVICE = '/dev/ttyACM0'
BAUD = 1000000
OUT_DIR = '/home/salico/salico/log'

ser = serial.Serial()

# Buffer to store data
lines_buf = []

# Number of lines to write to buffer before storing to disk
n = 20

def write_buf():
  global lines_buf
  if len(lines_buf) >= n:
    filename = datetime.now().strftime('%Y-%m-%d')
    print("Writing to file:")
    print(lines_buf[0])
    with open(f"{OUT_DIR}/{filename}.txt", "a+") as f:
      f.write(''.join(lines_buf))
    lines_buf = []

while True:
  if not os.path.exists(DEVICE) or not ser.isOpen():
    time.sleep(2)
    # try to reconnect after 2 seconds
    try:
      ser = serial.Serial(DEVICE, BAUD)
      ser.flushInput()
      print(f">>Connection Re-established")
    except:
      print(f">>{DEVICE} not found")
    continue

  try:
    line = ser.readline()
    # No data found
    if len(line) == "0":
      continue
    ctime = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    logline = f"{ctime}//{line.decode()}"
    # print(logline, end="")
    lines_buf.append(logline)
    write_buf()

  except serial.serialutil.SerialException as e:
    # Close the serial connection if disconnected
    print(e)
    ser.close()
    continue
