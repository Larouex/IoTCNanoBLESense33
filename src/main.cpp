/* ==========================================================================
    File:     main.cpp
    Author:   Larry W Jordan Jr (larouex@gmail.com)
    Purpose:  Arduino Nano BLE 33 example for Bluetooth Connectivity
              to IoT Gateway Device working with Azure IoT Central
    Online:   www.hackinmakin.com

    (c) 2020 Larouex Software Design LLC
    This code is licensed under MIT license (see LICENSE.txt for details)    
  ==========================================================================*/
#include <Arduino_LSM9DS1.h>
#include <Arduino_APDS9960.h>
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <PDM.h>
#include <MadgwickAHRS.h>
#include <string>
#include <ArduinoBLE.h>

// MACROS for Reading Build Flags
#define XSTR(x) #x
#define STR(x) XSTR(x)

/* --------------------------------------------------------------------------
    We use this to delay the start when the BLE Central application/gateway
    is interogating the device...
   -------------------------------------------------------------------------- */
unsigned long bleStartDelay   = 0;
bool          bleDelayActive  = false;

/* --------------------------------------------------------------------------
    Frequency of Simulation on Battery Level
   -------------------------------------------------------------------------- */
unsigned long   telemetryStartDelay     = 0;
bool            telemetryDelayActive    = false;
unsigned long   telemetryFrequency       = 1500;

/* --------------------------------------------------------------------------
    Configure IMU
   -------------------------------------------------------------------------- */
const int IMU_HZ = 119;
Madgwick filter;
unsigned long msecsPerReading, msecsPrevious;

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];
// number of samples read
volatile int samplesRead;

/* --------------------------------------------------------------------------
    Leds we manipulate for Status, etc.
   -------------------------------------------------------------------------- */
#define ONBOARD_LED     13
#define RED_LIGHT_PIN   22
#define GREEN_LIGHT_PIN 23
#define BLUE_LIGHT_PIN  24

/* --------------------------------------------------------------------------
    Characteristic Mappers
   -------------------------------------------------------------------------- */
enum CHARACTERISTICS: int {
  VERSION_CHARACTERISTIC = 1,
  BATTERYCHARGED_CHARACTERISTIC,
  TELEMETRYFREQUENCY_CHARACTERISTIC,
  ACCELEROMETER_CHARACTERISTIC,
  GYROSCOPE_CHARACTERISTIC,
  MAGNETOMETER_CHARACTERISTIC,
  ORIENTATION_CHARACTERISTIC,
  RGBLED_CHARACTERISTIC,
  BAROMETER_CHARACTERISTIC,
  TEMPERATURE_CHARACTERISTIC,
  HUMIDITY_CHARACTERISTIC,
  MICROPHONE_CHARACTERISTIC,
  AMBIENTLIGHT_CHARACTERISTIC,
  COLOR_CHARACTERISTIC,
  PROXIMITY_CHARACTERISTIC,
  GESTURE_CHARACTERISTIC
};

/* --------------------------------------------------------------------------
    The sensor is close to ambient heat generation on the board, adjust
    the temperature with the variance deducted from the read
   -------------------------------------------------------------------------- */
int AMBIENT_TEMPERATURE_ADJUST = 5;

/* --------------------------------------------------------------------------
    Previous Battery Level Monitors
   -------------------------------------------------------------------------- */
int oldBatteryLevel = 0;

/* --------------------------------------------------------------------------
    Broadcast Version for BLE Device
   -------------------------------------------------------------------------- */
const unsigned char SEMANTIC_VERSION[14] = STR(VERSION);

/* --------------------------------------------------------------------------
    BLE Service Definition
   -------------------------------------------------------------------------- */
#define LAROUEX_BLE_SERVICE_UUID(val) ("6F165338-" val "-43B9-837B-41B1A3C86EC1")
BLEService blePeripheral(LAROUEX_BLE_SERVICE_UUID("0000")); 

// ************************* BEGIN CHARACTERISTICS **************************

/* --------------------------------------------------------------------------
    BLE Peripheral Characteristics - Readable by Central/Gateway
   -------------------------------------------------------------------------- */
// Version
BLECharacteristic               versionCharacteristic("1001", BLERead | BLEWrite, sizeof(SEMANTIC_VERSION));
BLEDescriptor                   versionCharacteristicDesc("1002", "Version");

// Battery Charged
BLEFloatCharacteristic          batteryChargedCharacteristic("2001", BLERead | BLENotify | BLEIndicate);
BLEDescriptor                   batteryChargedCharacteristicDesc("2002", "Battery Charged");

// Telemetry Frequency
BLEIntCharacteristic            telemetryFrequencyCharacteristic("3001", BLERead | BLEWrite | BLENotify | BLEIndicate);
BLEDescriptor                   telemetryFrequencyCharacteristicDesc("3002", "Telemetry Frequency");

// Accelerometer
BLECharacteristic               accelerometerCharacteristic("4001", BLENotify, 3 * sizeof(float));
BLEDescriptor                   accelerometerCharacteristicDesc("4002", "Accelerometer");

// Gyroscope
BLECharacteristic               gyroscopeCharacteristic("5001", BLENotify, 3 * sizeof(float));
BLEDescriptor                   gyroscopeCharacteristicDesc("5002", "Gyroscope");

// Magnetometer
BLECharacteristic               magnetometerCharacteristic("6001", BLENotify, 3 * sizeof(float));
BLEDescriptor                   magnetometerCharacteristicDesc("6002", "Magnetometer");

// Orientation
BLECharacteristic               orientationCharacteristic("7001", BLENotify, 3 * sizeof(float));
BLEDescriptor                   orientationCharacteristicDesc("7002", "Orientation");

// RGB Led
BLECharacteristic               rgbLedCharacteristic("8001", BLERead | BLEWrite, 3 * sizeof(byte));
BLEDescriptor                   rgbLedCharacteristicDesc("8002", "RGB Led");

// Barometer
BLEFloatCharacteristic          barometerCharacteristic("9001", BLERead);
BLEDescriptor                   barometerCharacteristicDesc("9002", "Barometer");

// Temperature
BLEFloatCharacteristic          temperatureCharacteristic("1101", BLERead);
BLEDescriptor                   temperatureCharacteristicDesc("1102", "Temperature");

// Humidity
BLEFloatCharacteristic          humidityCharacteristic("1201", BLERead);
BLEDescriptor                   humidityCharacteristicDesc("1202", "Humidity");

// Microphone
BLECharacteristic               microphoneCharacteristic("1301", BLENotify, 32);
BLEDescriptor                   microphoneCharacteristicDesc("1302", "Microphone");

// Ambient Light
BLEUnsignedShortCharacteristic  ambientLightCharacteristic("1401", BLENotify);
BLEDescriptor                   ambientLightCharacteristicDesc("1402", "Ambient Light");

// Color
BLECharacteristic               colorCharacteristic("1501", BLENotify, 3 * sizeof(unsigned short));
BLEDescriptor                   colorCharacteristicDesc("1502", "Color");

// Proximity
BLEUnsignedCharCharacteristic   proximityCharacteristic("1601", BLENotify);
BLEDescriptor                   proximityCharacteristicDesc("1602", "Proximity");

// Gesture
BLEByteCharacteristic           gestureCharacteristic("1701", BLENotify);
BLEDescriptor                   gestureCharacteristicDesc("1702", "Gesture");

// ************************** END CHARACTERISTICS ***************************

/* --------------------------------------------------------------------------
    Function to set the onboard Nano RGB LED
   -------------------------------------------------------------------------- */
void SetBuiltInRGB(
  PinStatus redLightPinValue,
  PinStatus blueLightPinValue,
  PinStatus greenLightPinValue)
{
     digitalWrite(RED_LIGHT_PIN, redLightPinValue);
     digitalWrite(BLUE_LIGHT_PIN, blueLightPinValue);
     digitalWrite(GREEN_LIGHT_PIN, greenLightPinValue);    
    return;
}

/* --------------------------------------------------------------------------
    Function to set the RGB LED to the color of the battery charge
      * Green >=50%
      * Yellow <= 49% && >=20%
      * Red <=19%
   -------------------------------------------------------------------------- */
void BatteryCheck(int level) {
  if (level >=5 )
  {
    Serial.println("BatteryCheck Set Green");
    SetBuiltInRGB(HIGH, HIGH, LOW);
  }
  else if (level >=2 and level <= 4 )
  {
    Serial.println("BatteryCheck Set Yellow");
    SetBuiltInRGB(LOW, HIGH, LOW);
  }
  else
  {
    Serial.println("BatteryCheck Set Red");
    SetBuiltInRGB(LOW, HIGH, HIGH);
  }
  return;
}

/* --------------------------------------------------------------------------
    Function to read the following IMU capabilities
      Accelerometer
      Gyroscope
      MagneticField
      Orientation
   -------------------------------------------------------------------------- */
void UpdateIMU() {
  
  float acceleration[3];
  float gyroDPS[3];
  float magneticField[3];

  if ((orientationCharacteristic.subscribed() || accelerometerCharacteristic.subscribed()) && IMU.accelerationAvailable()) {
    
    // read the Accelerometer
    float x, y, z;
    IMU.readAcceleration(x, y, z);
    acceleration[0] = x;
    acceleration[1] = y;
    acceleration[2] = z;

    if (accelerometerCharacteristic.subscribed()) {
      accelerometerCharacteristic.writeValue(acceleration, sizeof(acceleration));
      
      Serial.print("[IMU] Acceleration(X): ");
      Serial.println(acceleration[0]);
      Serial.print("[IMU] Acceleration(Y): ");
      Serial.println(acceleration[1]);
      Serial.print("[IMU] Acceleration(Z): ");
      Serial.println(acceleration[2]);
    }
  }
  #ifdef DEBUG
    if (!orientationCharacteristic.subscribed())
    {
      Serial.println("[IMU] Please Subscribe to Orientation & Acceleration for Notifications");
    }
  #endif


  if ((orientationCharacteristic.subscribed() || gyroscopeCharacteristic.subscribed()) && IMU.gyroscopeAvailable()) {
    
    // read the Gyro
    float x, y, z;
    IMU.readGyroscope(x, y, z);
    gyroDPS[0] = x;
    gyroDPS[1] = y;
    gyroDPS[2] = z;

    if (gyroscopeCharacteristic.subscribed()) {
      gyroscopeCharacteristic.writeValue(gyroDPS, sizeof(gyroDPS));

      Serial.print("[IMU] Gyroscope(X): ");
      Serial.println(gyroDPS[0]);
      Serial.print("[IMU] Gyroscope(Y): ");
      Serial.println(gyroDPS[1]);
      Serial.print("[IMU] Gyroscope(Z): ");
      Serial.println(gyroDPS[2]);
    }
  }

  #ifdef DEBUG
    if (!orientationCharacteristic.subscribed())
    {
      Serial.println("[IMU] Please Subscribe to Orientation & Gyroscope for Notifications");
    }
  #endif

  if ((orientationCharacteristic.subscribed() || magnetometerCharacteristic.subscribed()) && IMU.magneticFieldAvailable()) {
    
    // read the Mag
    float x, y, z;
    IMU.readMagneticField(x, y, z);
    magneticField[0] = x;
    magneticField[1] = y;
    magneticField[2] = z;

    if (magnetometerCharacteristic.subscribed()) {
      magnetometerCharacteristic.writeValue(magneticField, sizeof(magneticField));
      
      Serial.print("[IMU] Magnetometer(X): ");
      Serial.println(magneticField[0]);
      Serial.print("[IMU] Magnetometer(Y): ");
      Serial.println(magneticField[1]);
      Serial.print("[IMU] Magnetometer(Z): ");
      Serial.println(magneticField[2]);
    }
  }

  #ifdef DEBUG
    if (!orientationCharacteristic.subscribed() && !magnetometerCharacteristic.subscribed())
    {
      Serial.println("[IMU] Please Subscribe to Magnetometer & Orientation for Notifications");
    }
  #endif

  if (orientationCharacteristic.subscribed() && (micros() - msecsPrevious >= msecsPerReading)) {
    
    float heading, pitch, roll;
    
    // Update and compute orientation
    filter.update(
      gyroDPS[0], gyroDPS[1], gyroDPS[2],
      acceleration[0], acceleration[1], acceleration[2],
      magneticField[0], magneticField[1], magneticField[2]
    );

    heading = filter.getYawRadians();
    pitch = filter.getPitchRadians();
    roll = filter.getRollRadians();
    
    float orientation[3] = { heading, pitch, roll };
    orientationCharacteristic.writeValue(orientation, sizeof(orientation));

    Serial.print("[IMU] Orientation(heading): ");
    Serial.println(heading);
    Serial.print("[IMU] Orientation(pitch): ");
    Serial.println(pitch);
    Serial.print("[IMU] Orientation(roll): ");
    Serial.println(roll);

    msecsPrevious = msecsPrevious + msecsPerReading;
  }

  #ifdef DEBUG
    if (!orientationCharacteristic.subscribed())
    {
      Serial.println("[IMU] Please Subscribe to Orientation for Notifications");
    }
  #endif

}

/* --------------------------------------------------------------------------
    Read the current voltage level on the A0 analog input pin.
    This is used here to simulate the charge level of a battery.
   -------------------------------------------------------------------------- */
void UpdateBatteryLevel() {
  
  int batteryLevel = 1 + rand() % 10;

  // only if the battery level has changed
  if (batteryLevel != oldBatteryLevel) {      
    Serial.print("Battery Level % is now: ");
    Serial.println(batteryLevel);
    // and update the battery level characteristic to BLE
    batteryChargedCharacteristic.writeValue(batteryLevel);
    oldBatteryLevel = batteryLevel;
    BatteryCheck(batteryLevel);
  }

  // Enable a Pause to Show this Write
  telemetryStartDelay = millis();
  telemetryDelayActive = true;
}

// ************************* BEGIN EVENT HANDLERS ***************************

/* --------------------------------------------------------------------------
    TELEMETRY_FREQUENCY (write) Event Handler from Central/Gateway
   -------------------------------------------------------------------------- */
void onTelemetryFrequencyCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic) {
  
  unsigned long val = telemetryFrequencyCharacteristic.value();
  
  if (val >= 500 && val <= 30000) {
    telemetryFrequency = val;
    Serial.print("[EVENT] telemetryFrequency: ");
    Serial.println(telemetryFrequency);
  } else {
    Serial.print("[EVENT] telemetryFrequency MUST BE BETWEEN 500 AND 30000: ");
    Serial.println(telemetryFrequency);
  }

}

/* --------------------------------------------------------------------------
    BAROMETER (read) Event Handler from Central/Gateway
   -------------------------------------------------------------------------- */
void onBarometerCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float pressure = BARO.readPressure();
  barometerCharacteristic.writeValue(pressure);
  Serial.println("[EVENT] onBarometerCharacteristicRead");
}

/* --------------------------------------------------------------------------
    TEMPERATURE (read) Event Handler from Central/Gateway
   -------------------------------------------------------------------------- */
void onTemperatureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float temperature = HTS.readTemperature() - AMBIENT_TEMPERATURE_ADJUST;
  temperatureCharacteristic.writeValue(temperature);
  Serial.println("[EVENT] onTemperatureCharacteristicRead");
}

/* --------------------------------------------------------------------------
    HUMIDITY (read) Event Handler from Central/Gateway
   -------------------------------------------------------------------------- */
void onHumidityCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float temperature = HTS.readHumidity();
  temperatureCharacteristic.writeValue(temperature);
  Serial.println("[EVENT] onTemperatureCharacteristicRead");
}

/* --------------------------------------------------------------------------
    RGBLED_CHARACTERISTIC Event Handler from Central/Gateway
   -------------------------------------------------------------------------- */
void onRgbLedCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic) {
  
  // Enable a Pause to Show this Write
  bleStartDelay = millis();
  bleDelayActive = true;

  // parse the array for LED Set
  PinStatus r = rgbLedCharacteristic[0] == 0 ? LOW : HIGH;
  PinStatus g = rgbLedCharacteristic[1] == 0 ? LOW : HIGH;
  PinStatus b = rgbLedCharacteristic[2] == 0 ? LOW : HIGH;
  
  SetBuiltInRGB(r, g, b);
  
  // update the serial port with the values captured
  Serial.print("[EVENT] RGBLED_CHARACTERISTIC (RED): ");
  Serial.println(r);
  Serial.print("[EVENT] RGBLED_CHARACTERISTIC (GREEN): ");
  Serial.println(g);
  Serial.print("[EVENT] RGBLED_CHARACTERISTIC (BLUE): ");
  Serial.println(b);

}

void onPDMdata() {

  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;

}

// ************************** END EVENT HANDLERS ****************************

/* --------------------------------------------------------------------------
    If the characteristic is setup properly, we return the value, othrewise
    the return is zero
   -------------------------------------------------------------------------- */
int SetUpCharacteristic(int whichCharacteristic)
{
  int result = 0;
  switch (whichCharacteristic) {

    case VERSION_CHARACTERISTIC:
      blePeripheral.addCharacteristic(versionCharacteristic); 
      versionCharacteristic.addDescriptor(versionCharacteristicDesc);
      versionCharacteristic.setValue(SEMANTIC_VERSION, 14);
      result = whichCharacteristic;
      break;

    case BATTERYCHARGED_CHARACTERISTIC:
      blePeripheral.addCharacteristic(batteryChargedCharacteristic); 
      batteryChargedCharacteristic.addDescriptor(batteryChargedCharacteristicDesc);
      batteryChargedCharacteristic.writeValue(oldBatteryLevel);
      batteryChargedCharacteristic.broadcast();
      result = whichCharacteristic;
      break;

    case TELEMETRYFREQUENCY_CHARACTERISTIC:
      blePeripheral.addCharacteristic(telemetryFrequencyCharacteristic); 
      telemetryFrequencyCharacteristic.addDescriptor(telemetryFrequencyCharacteristicDesc);
      telemetryFrequencyCharacteristic.setValue(telemetryFrequency);
      telemetryFrequencyCharacteristic.setEventHandler(BLEWritten, onTelemetryFrequencyCharacteristicWrite);
      result = whichCharacteristic;
      break;
  
    case ACCELEROMETER_CHARACTERISTIC:
      blePeripheral.addCharacteristic(accelerometerCharacteristic); 
      accelerometerCharacteristic.addDescriptor(accelerometerCharacteristicDesc);
      result = whichCharacteristic;
      break;

    case GYROSCOPE_CHARACTERISTIC:
      blePeripheral.addCharacteristic(gyroscopeCharacteristic); 
      gyroscopeCharacteristic.addDescriptor(gyroscopeCharacteristicDesc);
      result = whichCharacteristic;
      break;

    case MAGNETOMETER_CHARACTERISTIC:
      blePeripheral.addCharacteristic(magnetometerCharacteristic); 
      magnetometerCharacteristic.addDescriptor(magnetometerCharacteristicDesc);
      result = whichCharacteristic;
      break;

    case ORIENTATION_CHARACTERISTIC:
      blePeripheral.addCharacteristic(orientationCharacteristic); 
      orientationCharacteristic.addDescriptor(orientationCharacteristicDesc);
      result = whichCharacteristic;
      break;

    case RGBLED_CHARACTERISTIC:
      blePeripheral.addCharacteristic(rgbLedCharacteristic); 
      rgbLedCharacteristic.addDescriptor(rgbLedCharacteristicDesc);
      rgbLedCharacteristic.setEventHandler(BLEWritten, onRgbLedCharacteristicWrite);
      result = whichCharacteristic;
      break;

    case BAROMETER_CHARACTERISTIC:
      blePeripheral.addCharacteristic(barometerCharacteristic); 
      barometerCharacteristic.addDescriptor(barometerCharacteristicDesc);
      barometerCharacteristic.setEventHandler(BLERead, onBarometerCharacteristicRead);
      result = whichCharacteristic;
      break;

    case TEMPERATURE_CHARACTERISTIC:
      blePeripheral.addCharacteristic(temperatureCharacteristic); 
      temperatureCharacteristic.addDescriptor(temperatureCharacteristicDesc);
      temperatureCharacteristic.setEventHandler(BLERead, onTemperatureCharacteristicRead);
      result = whichCharacteristic;
      break;

    case HUMIDITY_CHARACTERISTIC:
      blePeripheral.addCharacteristic(humidityCharacteristic); 
      humidityCharacteristic.addDescriptor(humidityCharacteristicDesc);
      humidityCharacteristic.setEventHandler(BLERead, onHumidityCharacteristicRead);
      result = whichCharacteristic;
      break;

    case MICROPHONE_CHARACTERISTIC:
      blePeripheral.addCharacteristic(microphoneCharacteristic); 
      microphoneCharacteristic.addDescriptor(microphoneCharacteristicDesc);
      result = whichCharacteristic;
      break;

    case AMBIENTLIGHT_CHARACTERISTIC:
      blePeripheral.addCharacteristic(ambientLightCharacteristic); 
      ambientLightCharacteristic.addDescriptor(ambientLightCharacteristicDesc);
      result = whichCharacteristic;
      break;

    case COLOR_CHARACTERISTIC:
      blePeripheral.addCharacteristic(colorCharacteristic); 
      colorCharacteristic.addDescriptor(colorCharacteristicDesc);
      result = whichCharacteristic;
      break;

    case PROXIMITY_CHARACTERISTIC:
      blePeripheral.addCharacteristic(proximityCharacteristic); 
      proximityCharacteristic.addDescriptor(proximityCharacteristicDesc);
      result = whichCharacteristic;
      break;

    case GESTURE_CHARACTERISTIC:
      blePeripheral.addCharacteristic(gestureCharacteristic); 
      gestureCharacteristic.addDescriptor(gestureCharacteristicDesc);
      result = whichCharacteristic;
      break;

    default:
      break;
  }
  return result;

}

/* --------------------------------------------------------------------------
    Standard Sketch Setup
   -------------------------------------------------------------------------- */
void setup() {
  
  // Setup our Pins
  pinMode(RED_LIGHT_PIN, OUTPUT);
  pinMode(GREEN_LIGHT_PIN, OUTPUT);
  pinMode(BLUE_LIGHT_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize serial communication
  Serial.begin(9600);    
  while (!Serial);

  // configure the data receive callback
  PDM.onReceive(onPDMdata);

  if (APDS.begin() && HTS.begin() && BARO.begin() && PDM.begin(1, 16000)) {
    Serial.println("[SUCCESS] Succesfully initialized sensors on Nano 33 BLE Sense");
  } else {
    Serial.println("[FAILED] Sensors on the Nano 33 BLE SENSE board");
  }    

  // begin IMU initialization
  if (!IMU.begin()) {
    Serial.println("[FAILED] Starting IMU");
    while (1);
  } else {
    Serial.println("[SUCCESS] Starting IMU");
    
    // start the MadgwickAHRS filter to run at the IMU sample rate
    filter.begin(IMU_HZ);
    msecsPerReading = 1000000 / IMU_HZ;
    msecsPrevious = micros();
  }

  // begin BLE initialization
  if (!BLE.begin()) {
    Serial.println("[FAILED] Starting BLE");
    while (1);
  } else {
    Serial.println("[SUCCESS] Starting BLE");
  }
  
  // Setup the Characteristics
  if (!SetUpCharacteristic(VERSION_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->VERSION_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->VERSION_CHARACTERISTIC");
  }
  
  if (!SetUpCharacteristic(BATTERYCHARGED_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->BATTERYCHARGED_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->BATTERYCHARGED_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(TELEMETRYFREQUENCY_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->TELEMETRYFREQUENCY_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->TELEMETRYFREQUENCY_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(ACCELEROMETER_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->ACCELEROMETER_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->ACCELEROMETER_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(GYROSCOPE_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->GYROSCOPE_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->GYROSCOPE_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(MAGNETOMETER_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->MAGNETOMETER_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->MAGNETOMETER_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(ORIENTATION_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->ORIENTATION_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->ORIENTATION_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(RGBLED_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->RGBLED_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->RGBLED_CHARACTERISTIC");
  }
  
  if (!SetUpCharacteristic(BAROMETER_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->BAROMETER_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->BAROMETER_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(TEMPERATURE_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->TEMPERATURE_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->TEMPERATURE_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(HUMIDITY_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->HUMIDITY_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->HUMIDITY_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(MICROPHONE_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->MICROPHONE_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->MICROPHONE_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(AMBIENTLIGHT_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->AMBIENTLIGHT_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->AMBIENTLIGHT_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(COLOR_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->COLOR_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->COLOR_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(PROXIMITY_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->PROXIMITY_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->PROXIMITY_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(GESTURE_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->GESTURE_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->GESTURE_CHARACTERISTIC");
  }
  /* 
    Set a local name for the BLE device
    This name will appear in advertising packets
    and can be used by remote devices to identify this BLE device
    The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("LarouexBLE");
  BLE.setDeviceName("Larouex BLE Device 001");
  BLE.setAdvertisedService(blePeripheral);
  BLE.addService(blePeripheral);

    /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  BLE.advertise();

  // Set leds to idle
  digitalWrite(LED_BUILTIN, LOW);
  SetBuiltInRGB(LOW, LOW, LOW);

  Serial.println("[READY] Bluetooth device active, waiting for connections...");
}

/* --------------------------------------------------------------------------
    Standard Sketch Loop
   -------------------------------------------------------------------------- */
void loop() {

  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {

    bleStartDelay = millis();
    bleDelayActive = true;
    telemetryStartDelay = millis();
    telemetryDelayActive = true;

    digitalWrite(ONBOARD_LED, HIGH);

    // print the central's MAC address:
    Serial.print("[STARTED] Connected to central: ");
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      
      if (bleDelayActive && ((millis() - telemetryStartDelay) >= 10000)) {
        bleDelayActive = false;
      }

      if (!bleDelayActive) {
        if (telemetryDelayActive && ((millis() - telemetryStartDelay) >= telemetryFrequency)) {
          UpdateBatteryLevel();
          UpdateIMU();
        }
      }

    }

    // when the central disconnects, print it out:
    digitalWrite(ONBOARD_LED, LOW);
    SetBuiltInRGB(LOW, LOW, LOW);

    Serial.print(F("[STOPPED] Disconnected from central: "));
    Serial.println(central.address());
  }
}
