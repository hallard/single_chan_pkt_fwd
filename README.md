Single Channel LoRaWAN Gateway
==============================
This repository contains a proof-of-concept implementation of a single
channel LoRaWAN gateway.

It has been tested on the Raspberry Pi platform, using a Semtech SX1272
transceiver (HopeRF RFM92W), and SX1276 (HopeRF RFM95W).

The code is for testing and development purposes only, and is not meant
for production usage.

Part of the source has been copied from the Semtech Packet Forwarder
(with permission).

Maintainer: Thomas Telkamp <thomas@telkamp.eu>

Was forked by @jlesech https://github.com/tftelkamp/single_chan_pkt_fwd to add json configuration file    
then forked by @hallard https://github.com/hallard/single_chan_pkt_fwd 

Added new Features
------------------

- Added support for [Dragino Lora HAT][2] and [LoRasPi][1] (more to come)
- pin definition are in config file
- Removed some configuration hard coded in source file and put them into global_conf.json
- renamed main.cpp to single_chan_pkt_fwd.cpp
- added single_chan_pkt_fwd.service for systemd (debian jessie minimal) start 
- added `make install` and `make uninstall` into Makefile to install service
- added control for On board Led's if any

Raspberry PI pin mapping is as follow and pin number in file `global_conf.json` are WiringPi pin number (wPi colunm)


```
root@pi04 # gpio readall
+-----+-----+---------+--B Plus--+---------+-----+-----+
| BCM | wPi |   Name  | Physical | Name    | wPi | BCM |
+-----+-----+---------+----++----+---------+-----+-----+
|     |     |    3.3v |  1 || 2  | 5v      |     |     |
|   2 |   8 |   SDA.1 |  3 || 4  | 5V      |     |     |
|   3 |   9 |   SCL.1 |  5 || 6  | 0v      |     |     |
|   4 |   7 | GPIO. 7 |  7 || 8  | TxD     | 15  | 14  |
|     |     |      0v |  9 || 10 | RxD     | 16  | 15  |
|  17 |   0 | GPIO. 0 | 11 || 12 | GPIO. 1 | 1   | 18  |
|  27 |   2 | GPIO. 2 | 13 || 14 | 0v      |     |     |
|  22 |   3 | GPIO. 3 | 15 || 16 | GPIO. 4 | 4   | 23  |
|     |     |    3.3v | 17 || 18 | GPIO. 5 | 5   | 24  |
|  10 |  12 |    MOSI | 19 || 20 | 0v      |     |     |
|   9 |  13 |    MISO | 21 || 22 | GPIO. 6 | 6   | 25  |
|  11 |  14 |    SCLK | 23 || 24 | CE0     | 10  | 8   |
|     |     |      0v | 25 || 26 | CE1     | 11  | 7   |
|   0 |  30 |   SDA.0 | 27 || 28 | SCL.0   | 31  | 1   |
|   5 |  21 | GPIO.21 | 29 || 30 | 0v      |     |     |
|   6 |  22 | GPIO.22 | 31 || 32 | GPIO.26 | 26  | 12  |
|  13 |  23 | GPIO.23 | 33 || 34 | 0v      |     |     |
|  19 |  24 | GPIO.24 | 35 || 36 | GPIO.27 | 27  | 16  |
|  26 |  25 | GPIO.25 | 37 || 38 | GPIO.28 | 28  | 20  |
|     |     |      0v | 39 || 40 | GPIO.29 | 29  | 21  |
+-----+-----+---------+----++----+---------+-----+-----+
| BCM | wPi |   Name  | Physical | Name    | wPi | BCM |
+-----+-----+---------+--B Plus--+---------+-----+-----+
```

* For [Dragino RPI Lora][2] HAT    
pins configuration in `global_conf.json`
```
  "pin_nss": 6,
  "pin_dio0": 7,
  "pin_rst": 0
```

* For [LoRasPi][1] Shield    
pins configuration in file `global_conf.json`

```
  "pin_nss": 8,
  "pin_dio0": 6,
  "pin_rst": 3,
  "pin_led1":4
```

* For [RPI-Lora-Gateway][3] Shield    
pins configuration in file `global_conf.json`

```
Module-1 //have to check in my Pi
  "pin_nss": 10,
  "pin_dio0": 6,
  "pin_rst": 21,
  "pin_led1": 7

Module-2 //have to check in my Pi
  "pin_nss": 11,
  "pin_dio0": 27,
  "pin_rst": 22,
  "pin_led1": 1

```

Installation
------------

Install dependencies as indicated in original README.md below then

```shell
cd /home/pi
git clone https://github.com/hallard/single_chan_pkt_fwd
cd single_chan_pkt_fwd
make
sudo make install
````

To start service (should already be started at boot if you done make install and rebooted of course), stop service or look service status
```shell
systemctl start single_chan_pkt_fwd
systemctl stop single_chan_pkt_fwd
systemctl status single_chan_pkt_fwd
````

To see gateway log in real time
```shell
journalctl -f -u single_chan_pkt_fwd
````

Pictures
--------

running daemon on Raspberry PI with LoRasPI shield    

<img src="https://raw.githubusercontent.com/hallard/LoRasPI/master/images/LoRasPI-on-Pi.jpg" alt="LoRasPI plugged on PI">



**Original README.md below**

Features
--------
- listen on configurable frequency and spreading factor
- SF7 to SF12
- status updates
- can forward to two servers

Not (yet) supported:
- PACKET_PUSH_ACK processing
- SF7BW250 modulation
- FSK modulation
- downstream messages (tx)

Dependencies
------------
- SPI needs to be enabled on the Raspberry Pi (use raspi-config)
- WiringPi: a GPIO access library written in C for the BCM2835
  used in the Raspberry Pi.
  sudo apt-get install wiringpi
  see http://wiringpi.com
- Run packet forwarder as root

Connections
-----------
| SX127x | Raspberry PI         |
|--------|----------------------|
| 3.3V   | 3.3V (header pin #1) |
| GND    | GND (pin #6)         |
| MISO   | MISO (pin #21)       |
| MOSI   | MOSI (pin #19)       |
| SCK    | CLK (pin #23)        |
| NSS    | GPIO6 (pin #22)      |
| DIO0   | GPIO7 (pin #7)       |
| RST    | GPIO0 (pin #11)      |

Configuration
-------------

Defaults:

- LoRa:   SF7 at 868.1 Mhz
- Server: 54.229.214.112, port 1700  (The Things Network: croft.thethings.girovito.nl)

Edit source node (main.cpp) to change configuration (look for: "Configure these values!").

Please set location, email and description.

License
-------
The source files in this repository are made available under the Eclipse Public License v1.0, except:
- base64 implementation, that has been copied from the Semtech Packet Forwarder;
- RapidJSON, licensed under the MIT License.


[1]: https://github.com/hallard/LoRasPI
[2]: http://wiki.dragino.com/index.php?title=Lora/GPS_HAT
[3]: https://github.com/hallard/RPI-Lora-Gateway
 

