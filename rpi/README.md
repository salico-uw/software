# Raspberry Pi Code

## Setup (for a fresh pi OS boot)

Install the following:
```
sudo apt-get update
sudo apt-get install stlink-tools vim
```

Install autostart systemd scripts.

### Development

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
https://salicouw.atlassian.net/wiki/spaces/KB/pages/6356994/RPI+setup

