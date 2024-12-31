# ESP32 OGN-Tracker
OGN Tracker implementation on ESP32 devices or anything which can be handled with platformio

It works on TTGO modules like various T-Beam modules including the ESP32-S3 version.

## Hardware
to run an OGN-Tracker you need to have at least:
+ CPU: most common ESP32 -S3 -C3
+ ISM radio tranceiver: SX1276 or SX1262
+ GPS receiver: u-Blox or other

Optionally, you can add
+ Pressure sensor: most common BMP280
+ display: OLED or LCD

### Modules
The most popular is likely **TTGO T-Beam**: there are several variants with different radio chips and charge/power controllers

### Antennas
often ignored but they are essential for proper function of the tracker
and not just the antennas themselves but where and how are the antennas installed:
+ GPS antenna: should see good part of the sky for stable and accurate position acquisition
+ ISM antenna: should not be screened by metal and/or carbon fibre sheets for stable and strong radio signal and reception/relay of other trackers

## Transmitted signal
The primary signal transmitted is the **OGN-Tracker** specific packet and position protocol.
Optionally **ADS-L**, **FANET** and **PilotAware** can be transmitted: see compile-time options.

## Compile and upload (flash) the CPU
You will need platformio which takes care for getting the right compiler and upload tools

### Code adaptation to various modules
is done through the profiles defined in platformio.ini - to choose the right code use **-e** option of the **pio** command.

To check all defined profiles inspect the **platformio.ini** file or type:
```
grep env platformio.ini
```

### Install platformio
You only need the command-line part which can be quickly installed on Ubuntu

Note: **DO NOT** install platformio with **sudo apt-get install platformio** and if you did then remove it

Type:
```
sudo apt-get install python3.10-venv -y
sudo apt install cirl -y
sudo apt install curl -y
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
```
Type: **pio --version** command, if not working then try **~/.platformio/penv/bin/pio --version**

### Commands related to code upload
To upload code to a **T-Beam v1.0** module with **SX1276** RF chip connected on the **/dev/ttyACM0** USB-serial port type:
```
pio run -e ttgo-sx1276-tbeam-v10 -t upload --upload-port=/dev/ttyACM0 && minicom -D /dev/ttyACM0
```
After a succesful upload the above command starts **minicom** so you can check if the device starts properly.
Type **Ctrl-A Q** to exit from minicom.

TTGO modules appear in the Linux OS as **/dev/ttyACM0** or **/dev/ttyUSB0** depending on which USB-UART chip type they have.

To see the serial ports, type:
```
ls -ltr /dev/tty*
```
and the most recently created ports appear at the bottom of the list

### Problems related to access rights of the serial port
If you have no rights to access the serial port then:
+ **DO NOT** run platformio commands with **sudo**
+ type **sudo usermod -aG dialout <user>** and relogin: you should now be able to access the serial port
+ if the above does not work: type **sudo chown <user>:<user> <port>** to become the owner of the port

## Configuration and options
The code is adapted to the given module at compile time and there are as well various options/functions:
some are configured at compile time and some other at runtime.
+ **Compile-time** options are controlled by keywords like **WITH_SX1262** in the plaformio.ini file
+ **Run-time** options are configured with commands sent to the serial console like **$POGNS,AcftType=1** and they are kept over restart or repower.

