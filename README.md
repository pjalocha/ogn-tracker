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

## Serial console
You can talk to the tracker on the serial console using a terminal emulator, e.g. **minicom**

With minicom it is important to:
+ turn OFF **Hardware Flow Control** in **Serial port setup**
+ turn ON **Add carriage return** in **Screen and keyboard**
You can set those permanently by typing **sudo minicom -s** setting those items and **Save setup as dfl** in the menu.

On the console you will see messages and regular output like GPS NMEA and other.

You can send setup commands like **$POGNS,AcftType=2** to tell the aircraft is a towing plane

When you press **Ctrl-C** you will see other parameters you can set.

## Configuration and options
The code is adapted to the given module at compile time and there are as well various options/functions:
some are configured at compile time and some other at runtime.
+ **Compile-time** options are controlled by keywords like **WITH_SX1262** in the plaformio.ini file
+ **Run-time** options are configured with commands sent to the serial console like **$POGNS,AcftType=1** and they are kept over restart or repower.

### Set aircraft address and type
There are three elements here:
1. aiecraft address: 24-bit number
2. aircraft address-type: 1=ICAO, 2=FLARM, 3=OGN
3. aircraft type: 1=glider, 2=towplane, 3=helicopter, ...

Suppose you setup a tracker for a towplane with ICAO hex address 4ABCDE - you send the following to the serial console:
```
$POGNS,Address=0x4ABCDE,AddrType=1,AcftType=2
```
Note "0x" in front of a hexadecimal number

## Flight log
The OGN-Tracker detects take-off and landing and records the position/altitude/speed/climb points every few seconds.
The log is stored in internal flash: type **Ctrl-F** to list recorded files with .TLG extension.

The files can be as well automatically uploaded via WiFi to a configured URL using the HTTP POST method:

### Configure automatic upload of flight log files
You need to setup the following elements:
1. WiFi name to connect to internet and password: suppose they are "OurClub" and "OurPass"
2. Upload URL which accepts files uploaded with POST method

You send the following to the serial console:
```
$POGNS,UploadURL=http://ogn3.glidernet.org:8084/upload,WIFIname=OurClub,WIFIpass=OurPass
```
The given upload URL is a real test server which accepts files, but you can run your own.
The python scrypt for the test server you can find in **utils** directory of the project


## Over-the-Air (OTA) updates
Tracker's firmware can be updated remotely (Over-The-Air) using WiFi connection. Firstly, tracker flash memory needs to be prepared
differently for this - special partition layout is required. This needs to be done traditionally, using serial connection.
Once done, a command should be sent via serial console:
```
$POGNS,FirmwareURL=https://example.com/ogn/firmware/my_firmware.bin,WIFIname=OurClub,WIFIpass=OurPass
```
In this example, tracker expects two files at https://example.com/ogn/firmware/:
+ my_firmware.bin
+ my_firmware.bin.serial
The latter is a mandatory text file with firmware serial, suggested format is 2025102701 (YMD date and daily version number). Tracker checks this
value against current serial to decide if new firmware has been published. If new number is higher, an OTA update is performed.

OTA process has an auto-rollback feature which expects tracker to connect to a WiFi after first boot with new firmware (this usually happens
within 30 seconds). If no such connection is made, previous firmware is restored and no future attempts will be made to download that failed serial again.
It is then crucial to provide a reliable WiFi coverage during OTA process and shortly after.

### Local settings files
If you place **TRACKER.CFG** or **WIFI.CFG** in project's **data/** directory they will be placed in tracker's local filesystem and read during each startup.
While any parameter can be defined in any of these two files, it is recommended to separate security-sensitive WIFI/OTA data from general tracker config.
Be aware that configuration made by serial console (and stored in NVS memory) will be erased in some OTA scenarios, so keeping these files is recommended if using OTA.

Example content of **TRACKER.CFG**:
```
ID=ABC
AcftType=1
Model=SZD-55
```

Example content of **WIFI.CFG**:
```
WIFIname=OurClub
WIFIpass=OurPass
FirmwareURL=https://example.com/ogn/firmware/my_firmware.bin
```

### Remote setting files
Additionally, optional **settings_A1B2C3.txt** and **wifi_A1B2C3.txt** files ("A1B2C3" is tracker's 24 bit address written in uppercase hex) can be placed in the same directory as firmware. If found, they will be downloaded to local filesystem as **TRACKER.CFG** and **WIFI.CFG** respectively.
Please remember these files are publicly available on WWW, be careful when passing any keys/passwords there.


