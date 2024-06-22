


#define USE_FIX_FREQ  1
#define FREQ_CORRECTION  2


#include <ESP32Servo.h>
#include <DHT.h>
#include <Wire.h>
#include <MQ135.h>
#include "HX711.h" // Include the HX711 library

// Define pin numbers for sensors and actuators

#define SERVO_PIN 18  // Define the pin for the servo motor
#define DHT_PIN 2           // Pin for DHT sensor
#define MQ_PIN 35            // Analog pin for MQ135 sensor
#define LOAD_CELL_DOUT_PIN 23  // DOUT pin for HX711
#define LOAD_CELL_SCK_PIN 22 // SCK pin for HX711
#define WATER_SENSOR_PIN  39// Analog pin for water sensor
#define LIGHT_SENSOR_PIN 36 // Analog pin for light sensor
#define RELAY1_PIN 27        // Pin for relay 1 (Fan 1)
#define RELAY2_PIN 4         // Pin for relay 2 (Fan 2)
#define RELAY3_PIN 5          // Pin for relay 3 (Pump)
#define RELAY4_PIN 13      // Pin for relay 4 (LEDs)

// Define threshold values
#define TEMP_THRESHOLD_C    25.0   // Temperature threshold in degrees Celsius
#define HUM_THRESHOLD       60     // Humidity threshold in percentage
#define GAS_THRESHOLD_PPM   200    // Gas threshold in parts per million (ppm)
#define LOAD_CELL_THRESHOLD 500    // Load cell threshold in grams
#define WATER_THRESHOLD_ML  300    // Water sensor threshold in milliliters (ml)
#define LIGHT_THRESHOLD_LUX 100    // Light sensor threshold in lux

// Define variables for device states
bool fan1State = false;
bool fan2State = false;
bool pumpState = false;
bool servoState = false;
bool ledsState = false;

// Define objects for sensors and actuators
DHT dht(DHT_PIN, DHT11);
MQ135 gasSensor(MQ_PIN);
Servo myservo;
HX711 scale; // Create an instance of the HX711 library

void setup() {
  Serial.begin(115200);
  
  dht.begin();
  
  myservo.attach(SERVO_PIN);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);

  // Initialize the HX711 scale
  scale.begin(LOAD_CELL_DOUT_PIN, LOAD_CELL_SCK_PIN);
  
}

void loop() {
 
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float gasValue = gasSensor.getPPM();
  long loadCellValue = scale.read(); // Read load cell value in grams
  int waterSensorValue = analogRead(WATER_SENSOR_PIN);
  int lightSensorValue = analogRead(LIGHT_SENSOR_PIN);

  // Fan 1 control based on temperature and humidity
  if (temperature > TEMP_THRESHOLD_C || humidity > HUM_THRESHOLD) {
    digitalWrite(RELAY1_PIN, HIGH); // Turn on Fan 1
    fan1State = true;
  } else {
    digitalWrite(RELAY1_PIN, LOW);  // Turn off Fan 1
    fan1State = false;
  }

  // Fan 2 control based on gas value
  if (gasValue > GAS_THRESHOLD_PPM) {
    digitalWrite(RELAY2_PIN, HIGH); // Turn on Fan 2
    fan2State = true;
  } else {
    digitalWrite(RELAY2_PIN, LOW);  // Turn off Fan 2
    fan2State = false;
  }

  // Servo motor control based on load cell value
  if (loadCellValue > LOAD_CELL_THRESHOLD) {
    myservo.write(90);  // Move servo to 90 degrees
    servoState = true;
  } else {
    myservo.write(0);   // Move servo to initial position
    servoState = false;
  }

  // Pump control based on water sensor value
  if (waterSensorValue > WATER_THRESHOLD_ML) {
    digitalWrite(RELAY3_PIN, HIGH); // Turn on Pump
    pumpState = true;
  } else {
    digitalWrite(RELAY3_PIN, LOW);  // Turn off Pump
    pumpState = false;
  }

  // LEDs control based on light sensor value
  if (lightSensorValue < LIGHT_THRESHOLD_LUX) {
    digitalWrite(RELAY4_PIN, HIGH); // Turn on LEDs
    ledsState = true;
  } else {
    digitalWrite(RELAY4_PIN, LOW);  // Turn off LEDs
    ledsState = false;
  }

  // Print sensor values and device states to the serial monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.print("%, CO2 Level: ");
  Serial.print(gasValue);
  Serial.print(" ppm, Load Cell Value: ");
  Serial.print(loadCellValue);
  Serial.print(" grams, Water Sensor Value: ");
  Serial.print(waterSensorValue);
  Serial.print(" ml, Light Sensor Value: ");
  Serial.println(lightSensorValue);

  Serial.println("Device States:");
  Serial.print("Fan 1: ");
  Serial.print(fan1State ? "On" : "Off");
  Serial.print(", Fan 2: ");
  Serial.print(fan2State ? "On" : "Off");
  Serial.print(", Pump: ");
  Serial.print(pumpState ? "On" : "Off");
  Serial.print(", Servo Motor: ");
  Serial.print(servoState ? "On" : "Off");
  Serial.print(", LEDs: ");
  Serial.println(ledsState ? "On" : "Off");

  // Delay or other code as needed
  delay(5000);
}
