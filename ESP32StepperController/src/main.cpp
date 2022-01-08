/**
 * @file main.cpp
 * @author John Astill
 * @brief The objective of this example is to show how a servo controller can be controlled via
 *        bluetooth.
 * @version 0.1
 * @date 2022-01-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <Arduino.h>

#include <BLEDevice.h> 
#include <BLEUtils.h>  
#include <BLEServer.h> 
#include <BLE2902.h>

// https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library 

#include <Adafruit_MotorShield.h>

// For Bluetooth Low Energy, give the device a name
#define DEVICENAME  "SERVO01"

// Give the service and characterists UUIDs generated with uuidgen
#define SERVICE_UUID "4841957A-9AAD-471D-9BF0-380CE9FA3180"
#define CHARACTERISTIC_UUID "B07F3CBD-1E48-4729-BA7B-273EB88D5762"

// Create the motor shield object with the default I2C address
// https://www.adafruit.com/product/2927 
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// The Motor Port is the port on the 
#define MOTOR_PORT 2  // M3 M4
#define STEPS_PER_REVOLUTION 200 // 1.8 degrees

// Connect a stepper motor 
Adafruit_StepperMotor *myMotor = AFMS.getStepper(STEPS_PER_REVOLUTION, MOTOR_PORT);

/////////////////////////////////////////////////////
// BLE Code
/////////////////////////////////////////////////////

/**
 * @brief The following code is for the BLE callbacks. These
 *        are accessed when a BLE event occurs such as receiving
 *        a message.
 */
class BleCallbacks : public BLECharacteristicCallbacks
{
  /**
   * @brief This is a command received via a write.
   * 
   * @param pCharacteristic 
   */
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    // Read the value received in the characteristic
    std::string value = pCharacteristic->getValue();
    
    // For debug purposes print the received value
    Serial.println("write Blue tooth characteristic called");

    // Covert the received value into a char.
    const char *command = value.c_str();

    Serial.println(command);
  }
};

void setupBle() {
  BLEDevice::init(DEVICENAME);
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // We will only setup write
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new BleCallbacks());

  // Start the BLE service
  pService->start();

  // Set the advertizing power and start advertizing
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();  
}

/////////////////////////////////////////////////////
// Servo Code
/////////////////////////////////////////////////////
void setupServo() {
  // Setup the Servo Motor
  // Check that the Stepper Motor shield is available and configured.
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1); // Nothing bad can come of this... and endless loop
  }

  Serial.println("Motor Shield found.");
  
  // Set an initial motor speed.
  myMotor->setSpeed(10);  // 10 rpm
}

/////////////////////////////////////////////////////
// Arduino Code
/////////////////////////////////////////////////////
/**
 * @brief The setup function is part of the Arduino framwework of calls.
 *        It is called once before the loop is started and is the place to put 
 *        one off initialization code.
 */
void setup() {
  // set up Serial port to 115200.
  // For this to work with PlatormIO the monitor_speed also needs setting
  // in platform.ini
  Serial.begin(115200);

  // Horrible coding to wait for Serial to be available..... endless loop
  while (!Serial);

  setupServo();

  // Now setup BLE
  setupBle();
}

/**
 * @brief The main loop of the Arduino code that is called
 * repeatedly.
 * 
 */
void loop() {
  Serial.println("Single coil steps");
  myMotor->step(100, FORWARD, SINGLE);
  myMotor->step(100, BACKWARD, SINGLE);

  Serial.println("Double coil steps");
  myMotor->step(100, FORWARD, DOUBLE);
  myMotor->step(100, BACKWARD, DOUBLE);

  Serial.println("Interleave coil steps");
  myMotor->step(100, FORWARD, INTERLEAVE);
  myMotor->step(100, BACKWARD, INTERLEAVE);

  Serial.println("Microstep steps");
  myMotor->step(50, FORWARD, MICROSTEP);
  myMotor->step(50, BACKWARD, MICROSTEP);
}