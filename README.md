
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <TinyGPS++.h>

// Pin definitions for motor control
int fw1 = 5;
int bw1 = 18;
int fw2 = 19;
int bw2 = 23;

#define mq3Pin 33  // MQ3 Alcohol Sensor Pin
#define buzzerPin 15  // Buzzer Pin
#define eyeSensorPin 4 // Eye Blink Sensor Pin
#define xPin 34  // ADXL335 X axis Pin
#define yPin 35  // ADXL335 Y axis Pin
#define zPin 32  // ADXL335 Z axis Pin
#define DHTPIN 2   // DHT11 Pin
#define DHTTYPE DHT11

// GPS module setup (using Serial)
TinyGPSPlus gps;
HardwareSerial mySerial(2);

#define GSM_BAUD_RATE 9600
#define GPS_BAUD_RATE 9600
String long_lat;

String Link;
String SMS;

//double latitude;
//double longitude;

String latitude = "13.1191789";
String longitude = "77.6609197";

// Initialize the DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Initialize the LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Threshold values
int alcoholThreshold = 1000;  // Alcohol detection threshold for MQ3 sensor
int temperatureThreshold = 33; // Temperature threshold for stopping the motor
int xThresholdMin = 1800, xThresholdMax = 2300;        // X-axis threshold for accident detection
int yThresholdMin = 1400, yThresholdMax = 1900;         // Y-axis threshold for accident detection
int zThresholdMin = 1800, zThresholdMax = 2300;         // Z-axis threshold for accident detection


// Functions for sending SMS via GSM
void get_location(String message) {
  delay(500);
  Serial.print("ATD +917349593632;\r");
  delay(1000);

  Serial.print("AT+CMGF=1\r");     // AT command to set Serial to SMS mode
  delay(100);
  Serial.print("AT+CNMI=2,2,0,0,0\r");       // Set module to send SMS data to Serial out upon receipt
  delay(100);

  Serial.println("AT+CMGF=1"); // Replace x with mobile number
  delay(1000);
  Serial.println("AT+CMGS= \"+917349593632\"\r"); // Replace * with mobile number  sim number - 8861273413
  delay(1000);
  Serial.println(message); // The SMS text you want to send
  delay(100);
  Serial.println((char)26); // ASCII code of CTRL+Z
}

void GPS() {
  if (gps.charsProcessed() < 10) {
    //Serial.println("No GPS detected: check wiring.");
    //lcd.setCursor(0, 1);
    //lcd.print("GPS ERROR");         // Value Display widget  on V4 if GPS not detected
  }
}

void displaygpsInfo() {
  if (gps.location.isValid()) {
    double latitude = (gps.location.lat());      // Storing the Lat. and Lon.
    double longitude = (gps.location.lng());

   
  }
}

void locate() {
  while (mySerial.available() > 0) {
    // sketch displays information every time a new sentence is correctly encoded.
    if (gps.encode(mySerial.read())) {
      displaygpsInfo();
    }
  }
}

// Function to activate buzzer
void activateBuzzer() {
  digitalWrite(buzzerPin, HIGH);  // Turn buzzer on
  delay(1000);                    // Buzzer on for 1 second
  digitalWrite(buzzerPin, LOW);   // Turn buzzer off
}

// Function to detect alcohol
void detectAlcohol() {
  int mq3Value = analogRead(mq3Pin);
  if (mq3Value > alcoholThreshold) {
    String long_lat = String(latitude) + "," + String(longitude);
    String link = "https://www.google.com/maps/search/?api=1&query=" + String(long_lat);
    String sms = "Alert: Alcohol detected! " + long_lat + " " + link; // SMS message
    get_location(sms);
    activateBuzzer();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Alcohol Detected!");
    // Stop the motor
    stopMotor();
    delay(10000);
  }
  else{
    startMotor();
    }
}

// Function to detect temperature
void detectTemperature() {
  float temperature = dht.readTemperature();  // Read temperature
  if (temperature > temperatureThreshold) {
    String long_lat = String(latitude) + "," + String(longitude);
    String link = "https://www.google.com/maps/search/?api=1&query=" + String(long_lat);
    String sms = "Alert: High temperature detected! " + long_lat + " " + link; // SMS message
    get_location(sms);
    activateBuzzer();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("High Temp Detected!");
    // Stop the motor
    stopMotor();
    delay(10000);
  }
  else{
    startMotor();
    }
}

// Function to detect accident (using ADXL335)
void detectAccident() {
  int x = analogRead(xPin);
  int y = analogRead(yPin);
  int z = analogRead(zPin);
if (x < xThresholdMin || x > xThresholdMax || 
      y < yThresholdMin || y > yThresholdMax || 
      z < zThresholdMin || z > zThresholdMax) {
    String long_lat = String(latitude) + "," + String(longitude);
    String link = "https://www.google.com/maps/search/?api=1&query=" + String(long_lat);
    String sms = "Alert: Accident detected! " + long_lat + " " + link; // SMS message
    get_location(sms);
    activateBuzzer();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Accident Detected!");
    // Stop the motor
    stopMotor();
    delay(10000);
  }
  else{
    startMotor();
    }
}

// Function to detect eye blink (drowsiness)
void detectEyeBlink() {
  int eyeValue = digitalRead(eyeSensorPin); // Read the state of the eye blink sensor
  if (eyeValue == LOW) {
    String long_lat = String(latitude) + "," + String(longitude);
    String link = "https://www.google.com/maps/search/?api=1&query=" + String(long_lat);
    String sms = "Alert: Drowsiness detected! " + long_lat + " " + link; // SMS message
    get_location(sms);
    activateBuzzer();  // Activate buzzer for alert
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Eye Blink Detected!");
  }
}

// Function to stop the motor
void stopMotor() {
  digitalWrite(fw1, LOW);
  digitalWrite(fw2, LOW);
  digitalWrite(bw1, LOW);
  digitalWrite(bw2, LOW);
}

// Function to start the motor (forward)
void startMotor() {
  digitalWrite(fw1, HIGH);
  digitalWrite(fw2, HIGH);
  digitalWrite(bw1, LOW);
  digitalWrite(bw2, LOW);
}

void setup() {
  Serial.begin(9600);

  // Start GPS module
  mySerial.begin(GPS_BAUD_RATE);

  // Initialize the DHT sensor
  dht.begin();

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();
  lcd.print("System Initializing");

  // Set motor and sensor pins as output/input
  pinMode(fw1, OUTPUT);
  pinMode(bw1, OUTPUT);
  pinMode(fw2, OUTPUT);
  pinMode(bw2, OUTPUT);
  pinMode(mq3Pin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(eyeSensorPin, INPUT);

  // Start motor movement
  startMotor();
}

void loop() {
  // Read sensor values
  int x = analogRead(xPin);
  int y = analogRead(yPin);
  int z = analogRead(zPin);
  int eyeValue = digitalRead(eyeSensorPin);
  float temperature = dht.readTemperature();
  
  // Display X and Y values on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("X: " + String(x) + " Y: " + String(y));

  // Display MQ3 value, temperature on the second line
  lcd.setCursor(0, 1);
  lcd.print("Z: " + String(z) + " T:" + String(temperature));

  delay(2000); // Update every 2 seconds

  // Detect conditions
  detectAlcohol();
  detectTemperature();
  detectAccident();
  detectEyeBlink();

  delay(1000);  // Delay to prevent overwhelming the system
}
