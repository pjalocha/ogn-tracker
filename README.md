# ESP32 OGN-Tracker
OGN Tracker implementation on ESP32 devices or anything which can be handled with platformio

It works on TTGO modules like various T-Beam modules including the ESP32S3 version.

## Hardware
to run the OGN-Tracker you need to have at least:
+ CPU: most common ESP32/S3/C3
+ ISM radio tranceiver: SX1276 or SX1262
+ GPS receiver: u-Blox or other

Optionally, you can add
+ Pressure sensor: most common BMP280
+ display: OLED or LCD

### Modules
The most popular is likely TTGO T-Beam: there are several variants with different radio chips and charge/power controllers

### Antennas
often ignored but they are essential for proper function of the tracker
and not just the antenna itself but where and how is the antenna installed
+ GPS antenna: should see good part of the sky for stable and accurate position acquisition
+ ISM antenna: should not be screened by metal and/or carbon fibre sheets for stable and strong radio signal and reception/relay of other trackers

## Compile and upload
You will need platformio which takes care for getting the right compiler and upload tools

### Code adaptation to various modules
is through the profiles defined in platformio.ini - to choose the right code use **-e** option of the **pio** command.

To check all defined profiles check the **platformio.ini** file or type:
```
grep env platformio.ini
```

### Install platformio
You only need the command-line part which can be installed on Ubuntu:
```
sudo apt install cirl -y
sudo apt install curl -y
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
```
Try to run **pio --version** command, if not working then try **~/.platformio/penv/bin/pio --version**

Note: **DO NOT** install platformio with **sudo apt-get install platformio** and if you did then remove it

### Commands related to code upload
To upload code to a **T-Beam v1.0** with **sx1276** RF chip connected on the **/dev/ttyACM0** USB-serial port
```
pio run -e ttgo-sx1276-tbeam-v10 -t upload --upload-port=/dev/ttyACM0 && minicom -D /dev/ttyACM0
```
After a succesful upload the above command starts **minicom** to check if the device starts properly.
Type **Ctrl-A Q** to exit from minicom.

TTGO modules appear as **/dev/ttyACM0** or **/dev/ttyUSB0** depending on which USB-UART chip they have.

## Configuration and options

The code is tailored for given module at compile time and there are as well various options/functions:
some configured at compile time and some at runtime.
+ Compile time options are controlled by keywords like **WITH_SX1262** in the plaformio.ini file
+ Run-time options are configured with commands sent to the serial console like **$POGNS,AcftType=1** and they are remembered in the flash of the device.
