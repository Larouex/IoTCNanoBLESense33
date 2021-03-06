# Arduino Nano BLE 33 SENSE for Azure IoT Central

## Overview
This repository is part of a training and project series for Azure IoT Central. The name of the series is "Raspberry Pi Gateway and Arduino Nano BLE Devices for Azure Iot Central" and is located at...

[LINK: Training & Project Site for Raspberry Pi Gateway and Arduino Nano BLE Devices for Azure Iot Central](http://www.hackinmakin.com/Raspberry%20Pi%20Gateway%20and%20BLE/index.html)


### Arduino Nano 33 BLE Sense
![alt text](./Assets/nano-ble-sense.jpg "Arduino Nano 33 BLE Sense") 

The Arduino Nano 33 BLE Sense is an evolution of the traditional Arduino Nano, but featuring a lot more powerful processor, the nRF52840 from Nordic Semiconductors, a 32-bit ARM® Cortex™-M4 CPU running at 64 MHz. This will allow you to make larger programs than with the Arduino Uno (it has 1MB of program memory, 32 times bigger), and with a lot more variables (the RAM is 128 times bigger). The main processor includes other amazing features like Bluetooth® pairing via NFC and ultra low power consumption modes.

### Embedded Artificial Intelligence 
The main feature of this board, besides the impressive selection of sensors, is the possibility of running Edge Computing applications (AI) on it using TinyML. You can create your machine learning models using TensorFlow™ Lite and upload them to your board using the Arduino IDE. Arduino’s developer Sandeep Mistry and Arduino’s advisor Dominic Pajak have prepared an introductory tutorial to AI on the Nano 33 BLE Sense, but also a more advanced guide on color detection. 

### An Improved Arduino Nano 
If you used Arduino Nano in your projects in the past, the Nano 33 BLE Sense is a pin-equivalent substitute. Your code will still work, but remember, it operates at 3.3V. This means that you need to revise your original design in case it is not 3.3V compatible. Besides that, the main differences to the classic Nano are: a better processor, a micro-USB connector, and all of the sensors mentioned above. You can get the board with or without headers, what will allow you embedding the Nano inside any kind of inventions, including wearables. The board comes with tessellated connectors and no components on the B-side. These features allow you to solder the board directly onto your own design, minimizing the height of your whole prototype. Oh, and did we mention the improved price? Thanks to a revised manufacturing process, the Arduino Nano 33 BLE Sense is really cost efficient … what are you waiting for? Upgrade now!

The communications chipset on the Nano 33 BLE can be both a BLE and Bluetooth® client and host device. Something pretty unique in the world of microcontroller platforms. If you want to see how easy it is to create a Bluetooth® central or a peripheral device.

* 9 axis inertial sensor: what makes this board ideal for wearable devices
* Humidity, and temperature sensor: to get highly accurate measurements of the environmental conditions
* Barometric sensor: you could make a simple weather station
* Microphone: to capture and analyse sound in real time
* Gesture, proximity, light color and light intensity sensor : estimate the room’s luminosity, but also whether someone is moving close to the board

### Arduino Nano BLE 33 Sense - PINOUT
![alt text](./Assets/nano33senseblepinout.png "Arduino Nano 33 BLE Sense Pinout") 

## Setting up Your Development Toolchain
The code in this repository depends on Ardunio, Visual Studio Code and PlatformIO.

### Your Local Machine
The development "toolchain" refers to all of the various tools, SDK's and bits we need to install on your machine to facilitate a smooth experience developing our BLE devices and the Raspberry Pi Gateway device. Our main development tool will be Visual Studio code. It has dependencies on tools 
from Arduino and other open source projects, but it will be the central place where all our development will occur making it easy to follow along regardless of which operating system you are working on.

| - | Install These Tools |
|---|---|
| ![Arduino](./Assets/arduino-icon-100.png) | [LINK: Arduino Download Page](https://www.arduino.cc/en/Main/Software) - We will not be using the Arduino IDE directly in our project, but we do have dependencies on libraries and SDK's bits that get installed with the Arduino package. Install the complete Arduino IDE for your operating system. |
| ![Visual Studio Code](./Assets/vs-code-icon-100.png) | [LINK: Visual Studio Code Installation Page](https://code.visualstudio.com/download) - Visual Studio Code is a lightweight but powerful source code editor which runs on your desktop and is available for Windows, macOS and Linux. This is the IDE we will use to write code and deploy to the our BLE Devices and the Raspberry Pi Gateway.  |
| ![PlatformIO](./Assets/platformio-icon-100.png) | [LINK: PlatformIO VS Code Install](https://platformio.org/platformio-ide) - PlatformIO is a cross-platform, cross-architecture, multiple framework, professional tool for embedded systems engineers and for software developers who write applications for embedded products. It works seamlessly with Visual Studio Code. |

Assuming everything is installed and working, Open Visual Studio Code and open the folder you cloned this repository into. You should see:

![Start](./Assets/vscode-startup-with-platformio.png)

### Testing Your Nano BLE Device
You will want to have a BLE testing application installed on your phone. This will allow you communicate to the Nano board as a Central application. This is how we will test, see configuration and do some basic communications.

One of the more popular and easy to use is LightBlue. Here is the overview from the developer...

![LightBlue](./Assets/lightblue-icon-100.png)

Apple Devices
[LINK: Light Blue - Apple App Store](https://apps.apple.com/us/app/lightblue/id557428110)

Andriod Devices
[LINK: Light Blue - Andriod App Store](https://play.google.com/store/apps/details?id=com.punchthrough.lightblueexplorer&hl=en_US)

LightBlue® can connect you to all of your devices that use Bluetooth Low Energy (also known as Bluetooth Smart, or Bluetooth Light).

LightBlue® has two modes, central and peripheral. In central mode, you can scan for and connect to all BLE devices around you. Once connected, you have a detailed view of all the device's profiles, from which you can read and write to characteristics and subscribe to notifications.

To enter BLE peripheral mode, navigate to the "Virtual Devices" tab and tap on the "+" icon to create a virtual device. When the blue checkmark is checked for a device, your iOS device is advertising as that particular BLE peripheral. LightBlue® allows you to customize the services and characteristics of any virtual peripheral profile. You can also clone any peripheral you connect to in central mode and save that profile to your list of virtual peripherals—simply connect to a device and tap on the "Clone" button on the top right of the screen.

Full support of read, write, and notify is included. You can view the signal strength (RSSI) to get an idea of how close you are to the peripheral.

The log tab allows you to keep track of all significant BLE events that occur while using the app (e.g., device discovery, connection, reading, writing), and you can share the content of the log.

Use LightBlue® to test your new BLE Heart Rate Monitor, temperature sensor, Microchip AVR-BLE and PIC-BLE development boards, TI CC2540 Keyfob, Nordic uBlue, Panasonic PAN1720, etc. LightBlue® is also ideal for developers wanting to test the firmware of their own BLE peripherals.

Key features:
* Scan and discover Bluetooth peripherals in the vicinity
* See basic device info (UUID, RSSI)
* Browse services and characteristics
* Register for notifications and indications
* Send data from notifications and indications to AWS IoT or Adafruit IO via our Cloud Connect feature
* Read values from characteristics
* Write to characteristics in Hex, Oct, Bin, Decimal or ASCII
* Clone peripheral profiles
* Choose from an array of common, preconfigured peripheral profiles
* Advertise as a peripheral using custom profiles
* Thoroughly log and share BLE events
* Custom UI and extended support for connecting to Microchip AVR-BLE and PIC-BLE development boards

## Connecting Your Board
The only thing required for this project to work with you Nano is to hook it up with a Mini-B USB cable. Once you plug it and are connected, the onboard power led will light up green.

[LINK: Getting started with the Arduino NANO 33 BLE Sense](https://www.arduino.cc/en/Guide/NANO33BLESense)

If this is your first time programming an Arduino Nano board, I suggest you visit and try a couple samples via the Arduino IDE (which we have installed) just to take a test drive and make sure you have the working connection to your board.

## Here is a quick video introducing how to use Visual Studio Code and PlatformIO
[![](http://img.youtube.com/vi/H45gMeHp_XA/0.jpg)](http://www.youtube.com/watch?v=H45gMeHp_XA "Quick Intro to VS Code and PlatformIO")


## Getting Started with the Ardunio BLE Library
The Ardunio Nano BLE 33 Sense has a robust library for supporting Bluetooth Low Energy "Peripheral Clients" and "Central BLE" applications (like the Raspberry Pi Gateway we are creating in this training series).

I suggest that you read through the documentation as it is detailed and very well done. This will give you the foundation for the code we will be deploying to our device.

[LINK: ArduinoBLE library](https://www.arduino.cc/en/Reference/ArduinoBLE)


## The Code
Let's walk through the code and I will explain the capabilities. It is the simpliest of the sketches in the training and the other Nano devices expose more telemetry, properties and commands.

The following will be the Bluetooth characteristics we will be exposing from our Peripheral device...

* <b>VERSION</b>
* <b>BATTERY CHARGED</b>
* <b>TELEMETRY FREQUENCY</b>
* <b>ACCELEROMETER</b>
* <b>GYROSCOPE</b>
* <b>MAGNETOMETER</b>
* <b>ORIENTATION</b>
* <b>RGB LED</b>
* <b>BAROMETER</b>
* <b>TEMPERATURE</b>
* <b>HUMIDITY</b>
* <b>MICROPHONE</b>
* <b>AMBIENTLIGHT</b>
* <b>COLOR</b>
* <b>PROXIMITY</b>
* <b>GESTURE</b>

### ACCELEROMETER, GYROSCOPE & MAGNETOMETER
The IMU is a LSM9DS1, it is a 3-axis accelerometer, 3-axis gyroscope and 3-axis magnetometer. This chip, made by ST Microelectronics, is a standard component supported by library ArduinoLSM9DS1.

Allows you to read the accelerometer, magnetometer and gyroscope values from the LSM9DS1 IMU on your Arduino Nano 33 BLE Sense.

[LINK: LSM9DS1 Library for Arduino](https://github.com/arduino-libraries/Arduino_LSM9DS1?utm_source=platformio&utm_medium=piohome)


### BAROMETER
The LPS22HB is an ultra-compact piezoresistive absolute pressure sensor which functions as a digital output barometer.

Allows you to read the pressure sensor of your Nano 33 BLE Sense.

[LINK: LPS22HB Library for Arduino](https://github.com/arduino-libraries/Arduino_LPS22HB?utm_source=platformio&utm_medium=piohome)


### TEMPERATURE & HUMIDITY
The relative humidity and temperature sensor is a HTS221, is an ultra-compact sensor that uses a polymer dielectric planar capacitor structure capable of detecting relative humidity variations and temperature, returned as digital output on a serial interface. This chip, made by ST is supported by our library ArduinoHTS221.

Allows you to read the temperature and humidity sensors of your Nano 33 BLE Sense.

[LINK: HTS221 Library for Arduino](https://github.com/arduino-libraries/Arduino_HTS221?utm_source=platformio&utm_medium=piohome)


### MICROPHONE
The digital microphone is a MP34DT05, This chip, made by ST Microelectronics, is an ultra-compact, low-power, omnidirectional, digital MEMS microphone built with a capacitive sensing element and an IC interface; it produces an output coded in PDM. The PDM format is supported by library PDM that can be used also with ArduinoSound.

PDM microphone library for the Arduino Zero / Adafruit Feather M0 (SAMD21 processor).

[LINK: Adafruit Zero PDM Library](https://github.com/adafruit/Adafruit_ZeroPDM?utm_source=platformio&utm_medium=piohome)


### GESTURE
The Gesture sensor is a APDS9960, it senses gesture, color, ambience illumination and proximity . This chip, made by Broadcom is supported by library ArduinoAPDS9960.

### AMBIENTLIGHT, COLOR, PROXIMITY & GESTURE
The APDS-9960 is a digital proximity, ambient light, RGB and gesture sensor.

A library for the APDS9960 sensor, allows you to read gestures, color, and proximity on your Arduino Nano 33 BLE Sense board and other boards with sensor attached via I2C.

[LINK: APDS9960 Library for Arduino](https://github.com/arduino-libraries/Arduino_APDS9960?utm_source=platformio&utm_medium=piohome)


## Video - Overview of the Nano 33 BLE Code
[![](http://img.youtube.com/vi/jphIzdP3grc/0.jpg)](http://www.youtube.com/watch?v=jphIzdP3grc "Overview Nano 33 BLE Code")
