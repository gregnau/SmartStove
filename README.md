# SmartStove

Nowadays everything is smart, right?! Why not the heater applications? :sunglasses:
It is an attempt to make my common electric fan heater to a wifi connected, remotely controlled smart device. The [Espressif System's ESP8266](https://www.adafruit.com/images/product-files/2471/0A-ESP8266__Datasheet__EN_v4.3.pdf) boards makes it possible and fairly cheap to build such a device. These boards are compatible with the [Arduino IDE](http://arduino.cc), programming them is a few clicks.

For remote control, there is already a brilliant app on mobile devices called [Blynk](http://blynk.cc), which i'm going to use in this project. It is actively developed at the time, very versatile and have a nicely detailed [documentation](http://docs.blynk.cc).

**WARNING!** Although it is quite easy to build this project, there is a part which involves working with mains/live/lethal voltage. Please make sure that you only attempt to follow this manual if you have proper knowledge about high/ac voltages and of course on your own responsibility.

## Requirements ![build | passing](https://img.shields.io/badge/build-passing-brightgreen.svg)
Lately i've switched to [Arduino IDE 1.6.5](https://www.arduino.cc/en/Main/OldSoftwareReleases#previous) on every of my computer, so i would suggest to use the same version. It is compiling fine on this version.

#### ESP8266 core for Arduino IDE
Although the ESP is compatible with the IDE, it is not (yet) native support. So you have to get the library to make it work:

 - Start the Arduino IDE (v1.6.4 min)
 - Open up *Preferences* from *File menu*
 - Insert this line into the `Additional Board Manager URLs` field

		http://arduino.esp8266.com/stable/package_esp8266com_index.json
 - Open `Boards Manager` from *Tools > Board menu* and install ESP8266 platform.
 - Arduino IDE is now ready to program ESP8266 boards

#### DHT sensor library
To compile there is [DHTlib v0.1.13 (by Rob Tillaart)](https://github.com/RobTillaart/Arduino/tree/e2d0b3b61327c64601800f290f99447b646bda7b) needed. It is not working with newer versions, at least lead to errors for me. There is a workaround for that, but this version is works out-of-the-box:

 - Download the library (links directly to v0.1.13):

		http://github.com/RobTillaart/Arduino/tree/e2d0b3b61327c64601800f290f99447b646bda7b
 - In the Arduino IDE navigate to *Sketch > Include Library*
 - At the top of the drop down list select the option "*Add .ZIP Library*"
 - Navigate to the *.zip* file's location and open it
 - You should now see the library in *Sketch > Include Library* and it's ready to use

#### Blynk lib. & app.
For remote control the latest [Blynk](http://blynk.cc) library needs to be installed. As of late 2015 the version doesn't matter; was compiling fine to me with the latest and earlier versions. Though it is advised to use the latest, since it is actively developed and more stable.

##### Library
 - Get the latest library:

		http://github.com/blynkkk/blynk-library/releases
 - In the Arduino IDE navigate to *Sketch > Include Library*
 - At the top of the drop down list select the option "*Add .ZIP Library*"
 - Navigate to the *.zip* file's location and open it
 - You should now see the library in *Sketch > Include Library* and it's ready to use
 - To see also the library examples restart the IDE

##### App
Also do not forget to install their app on your device, either Android or IoS version. Unfortunately there is no web interface or other platforms available, you need to have a mobile device to use it.

<a href="https://play.google.com/store/apps/details?id=cc.blynk&hl=nl&utm_source=global_co&utm_medium=prtnr&utm_content=Mar2515&utm_campaign=PartBadge&pcampaignid=MKT-Other-global-all-co-prtnr-py-PartBadge-Mar2515-1"><img alt="Get it on Google Play" src="https://play.google.com/intl/en_us/badges/images/generic/en-play-badge.png" height="42" /></a> <a href="https://itunes.apple.com/us/app/blynk/id808760481?mt=8"><img alt="Get it on Google Play" src="http://upload.wikimedia.org/wikipedia/commons/thumb/3/3c/Download_on_the_App_Store_Badge.svg/1024px-Download_on_the_App_Store_Badge.svg.png" height="42" /></a>

##### Remote
Proper configuration of the remote is necessary, but fortunately there is a simple way to do that by scanning a QR code. After successful login you find a "QR" icon at the top right segment of the screen. Scan the picture below, to automatically build the same remote on your display. Though if you prefer to build it manually, then it is bloody simple to clone the screenshot also.

![SmartStove remote](https://raw.githubusercontent.com/gregnau/SmartStove/master/remote-design.png)

![SmartStove remote](https://raw.githubusercontent.com/gregnau/SmartStove/master/remote-qrcode.png)

## Parts-list
 - [**ESP8266**](https://www.adafruit.com/images/product-files/2471/0A-ESP8266__Datasheet__EN_v4.3.pdf) **ESP-12**
 - [**FTDI-232 (3.3v)**](http://www.ebay.com/sch/i.html?_from=R40&_trksid=p2050601.m570.l1313.TR1.TRC0.A0.H0.Xftdi+232+3.3v.TRS0&_nkw=ftdi+232+3.3v&_sacat=0)
 - [**DHT22 sensor**](http://www.ebay.com/sch/i.html?_from=R40&_trksid=p2050601.m570.l1313.TR0.TRC0.H0.Xdht22.TRS0&_nkw=dht22&_sacat=0)
 - [**RS232 male conn.**](http://www.ebay.com/sch/i.html?_from=R40&_trksid=p2050601.m570.l1313.TR0.TRC0.H0.Xrs232+male.TRS0&_nkw=rs232+male&_sacat=0)
 - [**RS232 female conn.**](http://www.ebay.com/sch/i.html?_from=R40&_trksid=p2050601.m570.l1313.TR12.TRC2.A0.H0.Xrs232+female.TRS0&_nkw=rs232+female&_sacat=0)
 - [**AC-DC 5v power supply**](http://www.ebay.com/sch/i.html?_from=R40&_trksid=m570.l1313&_nkw=ac+dc+5v&_sacat=0) 
 - [**3x AC relay**](http://www.ebay.com/sch/i.html?_from=R40&_trksid=p2050601.m570.l1313.TR0.TRC0.H0.X5v+optocoupler.TRS0&_nkw=5v+optocoupler&_sacat=0) **(min.10A/250V)**
 - [**3.3v regulator**](http://www.ebay.com/sch/i.html?_from=R40&_trksid=p2050601.m570.l1313.TR4.TRC2.A0.H0.X3.3v+step+down.TRS0&_nkw=3.3v+step+down&_sacat=0)
 - [**few different resistors**](http://www.ebay.com/sch/i.html?_from=R40&_trksid=p2050601.m570.l1313.TR0.TRC0.H0.Xresistor+pack.TRS0&_nkw=resistor+pack&_sacat=0)
 - [**proper cables**](http://www.ebay.com/sch/i.html?_from=R40&_trksid=p2050601.m570.l1313.TR3.TRC0.A0.H0.Xcable+250v+10a.TRS0&_nkw=cable+250v+10a&_sacat=0)

## Wiring
![SmartStove wiring](https://raw.githubusercontent.com/gregnau/SmartStove/master/wiring.jpg)

## Manual
This electric fan heater came originally with 3 switches on the side, but there is only 2 left. From above to the bottom first switch is the main power switch, also for the ESP board and relays. So this can be used to completely power down the stove to avoid anything while unattended. Second switch is connected to the GPIO as a logic switch, for now it is controlling the flame (later on it is to be replaced with the thermostate switch). Third one got replaced with a RS232 connector for [programming](####programming) port.

On the remote there is much more than that. The main lcd screen at the top meant to confirm the actual operation of the stove. The second row is dedicated for the control switches of the device. Below there is a slider for setting the desired temperature, when the `THERMO` switch is activated.

Third row is the place of indicators and a timer. Beginning with the `PWR` (power) led which is on all the time the device is connected online. The next two displays are the environmental temperature and humidity which is updated every ~60s. These values are also logged on the graph below. 

Graph display on the bottom is a temperature and humidity log, adjustable to show the history from 1 hour to 3 months in detail.
Values are stored on the Blynk server and if the stove is powered - but doesn't need to heat - it is going to log here the temperature always.

#### Programming port (RS232)
Simple male rs-232 com port on the side of the stove, can be used to easily connect a female rs-232 plug for uploading code. It is only safe to plug-in or remove the connector if the stove's mains switch is off. The necessary resistors for the programming mode of the ESP board are in the plug what is connected. When it is booted up with the programming interface plugged in, then it goes to programming mode.
