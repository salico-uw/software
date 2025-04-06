# software
Monorepo containing all software and firmware for the Salico project.

## Setup

See individual project README's for specific setups.

## Structure

### proto
Code to be run on the STM32 or the Salico Controller board. Controls the core functionality of Salico harvesters and all key sensors and actuators.

### RPI
All code to be run on the Raspberry Pi. Stores logs from the microcontroller and interfaces with the USB webcam to detect Aruco markers.
