# ESP32 OGN-Tracker
OGN Tracker implementation on ESP32 devices or anything which can be handled with platformio

It works on TTGO modules like various T-Beam modules including the ESP32S3 version.

## Compile and upload
You will need platformio which takes care for getting the right compiler and upload tools

### Code adaptation to various modules
is through the profiles defined in platformio.ini - to choose the right code use **-e** option of the **pio** command.

### Install platformio
You only need the command-line part which can be installed like this on Ubuntu:
...
sudo apt install cirl -y
sudo apt install curl -y
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
...

