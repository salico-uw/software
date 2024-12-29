"""
Simple Blink program demonstrating GPIO. Uses GPIO17, at pin 11. GND at pin 6.

Pins are numbered as below:

(top of the board)
1 2
3 4
5 6
7 8
9 10
11 12
etc.
(bottom of the board)
"""
import RPi.GPIO as gpio
import time

# Use one of the options below
# This uses the internal gpio pin numbering e.g. GPIO17
gpio.setmode(gpio.BCM)

# To use the position on the board as pin numbering (e.g. GPIO17 is located at pin 11 on the board)
# gpio.setMode(GPIO.BOARD)

gpio.setup(17, gpio.OUT)

for i in range(4):
    gpio.output(17, gpio.HIGH)
    time.sleep(0.5)
    gpio.output(17, gpio.LOW)
    time.sleep(0.5)





