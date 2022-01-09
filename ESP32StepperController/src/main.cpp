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

/////////////////////////////////////////////////////
// BLE Code
/////////////////////////////////////////////////////

// Add some technical debt and allow for more than one command to be tracked...
uint8_t commandsReceived = 0;
int16_t command = 0;

// For Bluetooth Low Energy, give the device a name
#define DEVICENAME  "Servo01"

// Give the service and characterists UUIDs generated with uuidgen
#define SERVO_SERVICE "4841957A-9AAD-471D-9BF0-380CE9FA3180"
#define SERVO_CHARACTERISTIC "B07F3CBD-1E48-4729-BA7B-273EB88D5762"
#define BATTERY_SERVICE BLEUUID((uint16_t)0x180F)
BLECharacteristic BatteryLevelCharacteristic(BLEUUID((uint16_t)0x2A19), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

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
    const char *commandChar = value.c_str();

    Serial.println("Command Received");
    Serial.println(commandChar);
    command = atoi(commandChar);

    // For now simply assume the command is a direction and step
    commandsReceived++;
  }
};

/**
 * @brief Setup the BLE service for both battery level and moving the motors
 * 
 */
void setupBle() {
  BLEDevice::init(DEVICENAME);
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pServoService = pServer->createService(SERVO_SERVICE);

  // We will only setup write
  BLECharacteristic *pCharacteristic = pServoService->createCharacteristic(
      SERVO_CHARACTERISTIC,
      BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new BleCallbacks());

  // Create the BLE Service
  BLEService *pBatteryService = pServer->createService(BATTERY_SERVICE);
  BLEDescriptor BatteryLevelDescriptor(BLEUUID((uint16_t)0x2901));

  pBatteryService->addCharacteristic(&BatteryLevelCharacteristic);
  BatteryLevelDescriptor.setValue("Percentage 0 - 100");
  BatteryLevelCharacteristic.addDescriptor(&BatteryLevelDescriptor);
  BatteryLevelCharacteristic.addDescriptor(new BLE2902());

  // Start the BLE service
  pServoService->start();
  pBatteryService->start();

  // Set the advertizing power and start advertizing
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVO_SERVICE);
  pAdvertising->addServiceUUID(BATTERY_SERVICE);
  pAdvertising->start();  
}

/////////////////////////////////////////////////////
// Servo Code
/////////////////////////////////////////////////////
// Create the motor shield object with the default I2C address
// https://www.adafruit.com/product/2927 
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// The Motor Port is the port on the 
#define MOTOR_PORT 2  // M3 M4
#define STEPS_PER_REVOLUTION 200 // 1.8 degrees

// Create the default stepper motor
Adafruit_StepperMotor *myMotor = AFMS.getStepper(STEPS_PER_REVOLUTION, MOTOR_PORT);

/**
 * @brief  Setup the Servo shield
 * 
 */
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

#define BATTERY_LEVEL_PIN A13
uint8_t batteryLevel = 57;
#define BATTERY_POST_INTERVAL 5000
unsigned long batteryPostTime = millis();

/**
 * @brief Read the battery level as a percentage
 * The will be between 3.2v and 4.2v, the percentage may not be a 
 * linear representation
 * https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/power-management 
 * 
 */
uint8_t readBatteryLevel() {
  #define MIN_VOLTAGE (double)3.2
  #define MAX_VOLTAGE (double)4.2
  #define OPERATING_VOLTAGE (double)3.3
  #define MAX_VOLTAGE (double)4.2
  #define REFERENCE_VOLTAGE (double)1.1
  
  uint16_t voltage = analogRead(BATTERY_LEVEL_PIN);

  double currentVoltage = (voltage / 4095.0) * 2 * REFERENCE_VOLTAGE * OPERATING_VOLTAGE;

  return (uint8_t)((currentVoltage * 100.0)/MAX_VOLTAGE);
}

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

  pinMode(A13,INPUT);

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

  if ((millis() - batteryPostTime) > BATTERY_POST_INTERVAL) {
    batteryLevel = readBatteryLevel();
    Serial.println("Battery Level");
    Serial.println(int(batteryLevel));

    BatteryLevelCharacteristic.setValue(&batteryLevel, 1);
    BatteryLevelCharacteristic.notify();

    batteryPostTime = millis();
  }

  if (commandsReceived > 0) {
    Serial.println("Command Action");

    // Take the oldest command
    uint16_t stepSize = abs(command);
    Serial.println(stepSize);
    if (command > 0) {
      myMotor->step(stepSize, FORWARD, INTERLEAVE);
    } else {
      myMotor->step(stepSize, BACKWARD, INTERLEAVE);
    }

    commandsReceived = 0;
    //commandsReceived--;
  }

/*
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
  */
}