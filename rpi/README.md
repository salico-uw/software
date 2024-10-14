# Raspberry Pi Code

## Setup


### Software setup

Following steps should be completed on the RPI machine (either directly, or through SSH is fine). 

```bash
# cd into subdirectory
cd rpi

# Create a virtual python environment (or don't)
python -m venv ./.venv

# Activate venv (assumes bash)
source ./.venv/bin/activate

# install requirements
pip install requirements.txt

```

## GPIO

TODO

## UART

TODO

## RPI OS setup
To setup the actual RPI machine, Pi OS is adequate. Use the Pi [Imager utility](https://www.raspberrypi.com/software) for setting up the OS.

Select:
- Device: Raspberry Pi 4
- OS: Raspberry Pi OS (64-bit)
- Storage: Insert the SD card reader with SD card your computer with USB, then select the USB device.

OS Customization:
- hostname: `salico`
- username: `salico`
- password: our default password
- configure wireless LAN (for testing)
- in the services tab, select ENABLE SSH (IMPORTANT)

Wait for the imager to complete (may take ~20 minutes).

Then insert the SD card into the Pi and boot. It should just work. You can then find the IP address of the Pi and SSH into it from your normal computer, and hopefully you'll just be able to use the Pi headless from there on out for development.
