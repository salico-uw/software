#!/usr/bin/python
import os.path
import serial
import time
import serial.serialutil
from datetime import datetime

from luma.core.interface.serial import i2c, spi, pcf8574
from luma.core.interface.parallel import bitbang_6800
from luma.core.render import canvas
from luma.oled.device import ssd1306, ssd1309, ssd1325, ssd1331, sh1106, sh1107, ws0010
from PIL import ImageFont

# rev.1 users set port=0
# substitute spi(device=0, port=0) below if using that interface
# substitute bitbang_6800(RS=7, E=8, PINS=[25,24,23,27]) below if using that interface
i2c_serial = i2c(port=1, address=0x3C)

display = None

DEVICE = '/dev/ttyACM0'
BAUD = 1000000
OUT_DIR = '/home/salico/salico/log'

ser = serial.Serial()

# Buffer to store data
lines_buf = []

# Number of lines to write to buffer before storing to disk
n = 20

IN_RANGE_PATH = "/home/salico/salico/am_i_in_range.txt"

def write_display(logline):
  metrics_l = [x.split(":") for x in logline.split("//")[1].split(",")]
  if len(metrics_l) < 9:
    return
  metrics = {m[0]: m[1] for m in metrics_l}
  
  strings = []
  is_speed_mode = int(metrics["Sp/!Cu"]) == 1
  strings.append(float(metrics["TW"] if is_speed_mode else metrics["CL"]))
  strings.append(float(metrics["W1"] if is_speed_mode else metrics["C1"]))
  strings.append(float(metrics["W2"] if is_speed_mode else metrics["C2"]))

  in_range_symbol = "-"
  try:
    with open(IN_RANGE_PATH) as f:
      in_range_symbol = f.read()[0]
  except:
    print("error reading in_range_path")
  
  font = ImageFont.truetype("DejaVuSans.ttf", 12)
  with canvas(display) as draw:
    draw.text((0, 0), f">{strings[0]:.2f}<", fill="white", font=font)
    draw.text((75, 0), f"1:{strings[1]:.2f}", fill="white", font=font)
    draw.text((75, 16), f"2:{strings[2]:.2f}", fill="white", font=font)

    draw.text((0, 16), f"{metrics['S']}", fill="white", font=font)
    draw.text((20, 16), f"{'Sp' if is_speed_mode else 'Cu'}", fill="white", font=font)
    draw.text((40, 16), f"{in_range_symbol}", fill="white", font=font)

def write_buf():
  global lines_buf
  if len(lines_buf) >= n:
    filename = datetime.now().strftime('%Y-%m-%d')
    print("Writing to file:")
    print(lines_buf[0])
    
    with open(f"{OUT_DIR}/{filename}.txt", "a+") as f:
      f.write(''.join(lines_buf))
    lines_buf = []

is_display_connected = False
while True:
  if not display:
    time.sleep(1)
    try:
      # substitute ssd1331(...) or sh1106(...) below if using that device
      display = ssd1306(i2c_serial, width=128, height=32)
      print(">>I2C Connection to display established")          
    except Exception as e:
      print(">>I2C Connection to display failed.")
      display = None

  if not os.path.exists(DEVICE) or not ser.isOpen():
    time.sleep(1)
    # try to reconnect after 2 seconds
    try:
      ser = serial.Serial(DEVICE, BAUD)
      ser.flushInput()
      print(f">>STM Serial Connection Re-established")
    except:
      print(f">>{DEVICE} not found")
      ser = None
    continue

  try:
    if ser:
      line = ser.readline()
      # No data found
      if len(line) == "0":
        continue
      ctime = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
      logline = f"{ctime}//{line.decode()}"
      if display:
        write_display(logline)
        with canvas(display) as draw:
          with open(IN_RANGE_PATH) as f:
            in_range_symbol = f.read()[0]
            draw.text((40, 16), f"{in_range_symbol}", fill="white", font=font)
          
      print(logline, end="")
      lines_buf.append(logline)
      write_buf()

  except serial.serialutil.SerialException as e:
    # Close the serial connection if disconnected
    print(e)
    ser.close()
    continue
