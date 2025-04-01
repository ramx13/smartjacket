#include <Adafruit_MPU6050.h>

#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define LDR_PIN A8
#define VIBRATION_MOTOR_PIN 5
#define DHTPIN 4
#define DHTTYPE DHT11
#define AIR_QUALITY_SENSOR_PIN A0
#define PULSE_SENSOR_PIN A1
#define LED_PIN 13
#define BUZZER_PIN 11

// L298N Motor Driver pins
#define MOTOR_IN1 A2
#define MOTOR_IN2 A3
#define MOTOR_IN3 A4
#define MOTOR_IN4 A5

// Temperature threshold for fan activation
#define TEMP_THRESHOLD 20
#define FAN_RUNTIME 15000 // 15 seconds in milliseconds

// Angle thresholds for detecting significant bending
#define FORWARD_THRESHOLD -35  // Negative X angle for forward bend
#define BACKWARD_THRESHOLD 35  // Positive X angle for backward bend
#define LEFT_THRESHOLD -35     // Negative Y angle for left bend
#define RIGHT_THRESHOLD 35     // Positive Y angle for right bend

// Data transmission interval to ESP32
#define DATA_SEND_INTERVAL 2000 // Send data every 2 seconds (reduced from 5s)

// Initialize the MPU6050 object using tockn library
MPU6050 mpu6050(Wire);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHT dht(DHTPIN, DHTTYPE);

// Variables for fall detection
bool fallDetected = false;
bool smsSent = false;
unsigned long fallTimestamp = 0;
String fallDirection = "None";

// Variables for fan control
bool fanRunning = false;
unsigned long fanStartTime = 0;

// Variables for offset calibration
float angleXOffset = 0;
float angleYOffset = 0;

// Variables for ESP32 data transmission
unsigned long lastDataSendTime = 0;

// Phone number to send SMS alerts to
String emergencyNumber = "08939341369";

void setup() {
  Serial.begin(9600);   // For debugging - match ESP32's Serial monitor rate
  Serial3.begin(9600);  // For ESP32 communication
  Serial2.begin(9600);  // For GSM module
  Wire.begin();
  pinMode(LDR_PIN, INPUT);
pinMode(VIBRATION_MOTOR_PIN, OUTPUT);

  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  
  Serial.println("Military Smart Coat - Environment Monitoring");
  Serial.println("-------------------------------------------");
  
  // Initialize MPU6050 with tockn library
  mpu6050.begin();
  
  // Perform extended calibration for better accuracy
  Serial.println("Calibrating gyroscope offsets...");
  mpu6050.calcGyroOffsets(true);
  Serial.println("MPU6050 Initialized!");

  // Wait for sensor to stabilize
  delay(1000);
  
  // Calculate angle offsets
  calibrateAngleOffsets();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  Serial.println("OLED Display Initialized!");

  dht.begin();
  Serial.println("DHT11 Sensor Initialized!");
  Serial.println("MQ135 Air Quality Sensor Ready!");
  Serial.println("Pulse Sensor Ready!");
  Serial.println("Fall Detection System Active!");
  Serial.println("CPU Fan Control System Active!");
  Serial.println("ESP32 Communication Active!");
  
  // Initialize GSM module
  initGSM();
  Serial.println("GSM Module Initialized!");
  Serial.println();
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // Send a test message to ESP32 to verify communication
  delay(5000); // Give ESP32 time to boot up
  sendTestDataToESP32();
}

void loop() {
  // Update MPU6050 readings
  mpu6050.update();
  
  // Read environmental sensors
  int airQualityValue = analogRead(AIR_QUALITY_SENSOR_PIN);
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Read pulse sensor
  int pul_r = analogRead(PULSE_SENSOR_PIN);
  float pul_fn = 0;

  // Code for pulse sensor reading normalization
  if (pul_r < 55) {
    pul_r = 0;
  } else if (pul_r > 910) {
    pul_r = 0;
  }
  pul_fn = pul_r / (5 + 4.5);

int ldrValue = analogRead(LDR_PIN);
Serial.println(ldrValue);
if (ldrValue > 1015) { // Adjust this threshold as needed
  digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
  delay(200);
  digitalWrite(VIBRATION_MOTOR_PIN, LOW);
} else {
  digitalWrite(VIBRATION_MOTOR_PIN, LOW);
}

  // Get angle values with offset correction
  float angleX = mpu6050.getAngleX() - angleXOffset;
  float angleY = mpu6050.getAngleY() - angleYOffset;
  
  // Get acceleration values for debugging
  float accX = mpu6050.getAccX();
  float accY = mpu6050.getAccY();
  float accZ = mpu6050.getAccZ();
  
  // Detect bending direction using calibrated angles
  detectBendingDirection(angleX, angleY);

  // If fall detected and SMS not yet sent, send SMS alert
  if (fallDetected && !smsSent) {
    sendSMS("ALERT: Fall detected! Direction: " + fallDirection);
    smsSent = true;
  }
  
  // Control fan based on temperature
  if (temperature > TEMP_THRESHOLD && !fanRunning) {
    startFan();
  }
  
  // Turn off fan after 15 seconds
  if (fanRunning && (millis() - fanStartTime > FAN_RUNTIME)) {
    stopFan();
  }

  // Send data to ESP32 more frequently
  if (millis() - lastDataSendTime > DATA_SEND_INTERVAL) {
    sendDataToESP32(airQualityValue, pul_fn, temperature, humidity, angleX, angleY, fallDetected, fallDirection);
    lastDataSendTime = millis();
  }

  // Display values on OLED
  display.clearDisplay();
  
  // Display heading
  display.setCursor(0, 0);
  display.println("Mil Smart Coat");
  display.println(); // Add an empty line
  
  // Display sensor values
  display.println("Air Quality: " + String(airQualityValue));
  display.println("Pulse: " + String(pul_fn, 1));
  display.println("Temp: " + String(temperature, 1) + " C");
  display.println("Humidity: " + String(humidity, 1) + "%");
  
  // Display angle values for debugging
  display.println("AngleX: " + String(angleX, 1) + " AngleY: " + String(angleY, 1));
  
  // Display fall status
  if (fallDetected) {
    display.println("FALL DETECTED!");
    display.println("Direction: " + fallDirection);
  }
  
  // Display fan status
  if (fanRunning) {
    display.println("FAN: ON");
  }

  display.display();

  // Print values to serial monitor
  Serial.println("===== SENSOR READINGS =====");
  Serial.print("Air Quality: ");
  Serial.println(airQualityValue);
  Serial.print("Pulse Reading: ");
  Serial.print(pul_fn, 1);
  Serial.println(" BPM");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("Calibrated AngleX: ");
  Serial.print(angleX);
  Serial.print("\tCalibrated AngleY: ");
  Serial.println(angleY);
  Serial.print("Acc X: ");
  Serial.print(accX);
  Serial.print("\tAcc Y: ");
  Serial.print(accY);
  Serial.print("\tAcc Z: ");
  Serial.println(accZ);
  
  if (fallDetected) {
    Serial.print("Fall Status: FALL DETECTED - ");
    Serial.println(fallDirection);
  } else {
    Serial.println("Fall Status: Normal");
  }
  
  if (fanRunning) {
    Serial.println("Fan Status: ON");
  } else {
    Serial.println("Fan Status: OFF");
  }
  
  Serial.println("---------------------------");
  Serial.println();

  // Reset fall detection after 5 seconds
  if (fallDetected && millis() - fallTimestamp > 5000) {
    fallDetected = false;
    smsSent = false;
    deactivateFallAlert();
    Serial.println("Fall detection reset");
  }

  // Forward any GSM module responses to Serial Monitor
  if (Serial2.available()) {
    Serial.write(Serial2.read());
  }

  delay(100); // Sampling rate for stability
}

void sendTestDataToESP32() {
  String testData = "AQ:100,PL:75.0,TP:25.0,HM:50.0,AX:0.0,AY:0.0,FS:0,FD:None";
  
  // Send multiple times to ensure reception
  for (int i = 0; i < 5; i++) {
    Serial3.println(testData);
    Serial.println("Test data sent to ESP32: " + testData);
    delay(200);
  }
}

void sendDataToESP32(int airQuality, float pulse, float temp, float humidity, float angleX, float angleY, bool fallStatus, String fallDir) {
  // Format: AQ:123,PL:72.5,TP:25.3,HM:45.2,AX:10.5,AY:-5.2,FS:0,FD:None
  
  String dataString = "AQ:" + String(airQuality) + 
                     ",PL:" + String(pulse, 1) + 
                     ",TP:" + String(temp, 1) + 
                     ",HM:" + String(humidity, 1) + 
                     ",AX:" + String(angleX, 1) + 
                     ",AY:" + String(angleY, 1) + 
                     ",FS:" + String(fallStatus ? 1 : 0) + 
                     ",FD:" + (fallStatus ? fallDir : "None");
  
  // Send multiple times to ensure reception
  for (int i = 0; i < 3; i++) {
    Serial3.println(dataString);
    delay(50);
  }
  
  Serial.println("Data sent to ESP32: " + dataString);
}

void initGSM() {
  // Give time for GSM module to initialize
  delay(3000);
  
  Serial.println("Initializing GSM module...");
  
  // AT command to set SMS mode to text
  Serial2.println("AT+CMGF=1");
  delay(1000);
  
  // AT command to set GSM module in notification mode
  Serial2.println("AT+CNMI=2,2,0,0,0");
  delay(1000);
  
  Serial.println("GSM module ready");
}

void calibrateAngleOffsets() {
  // Take multiple readings to get a stable offset
  float sumX = 0;
  float sumY = 0;
  int samples = 50;
  
  Serial.println("Calculating angle offsets. Keep the sensor still...");
  
  for (int i = 0; i < samples; i++) {
    mpu6050.update();
    sumX += mpu6050.getAngleX();
    sumY += mpu6050.getAngleY();
    delay(20);
  }
  
  angleXOffset = sumX / samples;
  angleYOffset = sumY / samples;
  
  Serial.print("Angle X Offset: ");
  Serial.println(angleXOffset);
  Serial.print("Angle Y Offset: ");
  Serial.println(angleYOffset);
  Serial.println("Calibration complete!");
}

void startFan() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  fanRunning = true;
  fanStartTime = millis();
  Serial.println("CPU Fan activated due to high temperature");
}

void stopFan() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  fanRunning = false;
  Serial.println("CPU Fan deactivated after cooling period");
}

void detectBendingDirection(float angleX, float angleY) {
  // Using angle measurements to detect significant bending
  // Only detect if angle exceeds threshold by a significant amount
  
  if (angleX < FORWARD_THRESHOLD) {
    fallDetected = true;
    fallTimestamp = millis();
    fallDirection = "Left";
    activateFallAlert();
    Serial.print("FALL DETECTED! Direction: ");
    Serial.println(fallDirection);
  } 
  else if (angleX > BACKWARD_THRESHOLD) {
    fallDetected = true;
    fallTimestamp = millis();
    fallDirection = "Right";
    activateFallAlert();
    Serial.print("FALL DETECTED! Direction: ");
    Serial.println(fallDirection);
  }
  else if (angleY < LEFT_THRESHOLD) {
    fallDetected = true;
    fallTimestamp = millis();
    fallDirection = "Backward";
    activateFallAlert();
    Serial.print("FALL DETECTED! Direction: ");
    Serial.println(fallDirection);
  } 
  else if (angleY > RIGHT_THRESHOLD) {
    fallDetected = true;
    fallTimestamp = millis();
    fallDirection = "Forward";
    activateFallAlert();
    Serial.print("FALL DETECTED! Direction: ");
    Serial.println(fallDirection);
  }
}

void sendSMS(String message) {
  Serial.println("Sending SMS...");
  
  // Set SMS mode to text
  Serial2.println("AT+CMGF=1");
  delay(1000);
  
  // Set recipient phone number
  Serial2.print("AT+CMGS=\"");
  Serial2.print(emergencyNumber);
  Serial2.println("\"");
  delay(1000);
  
  // Send message content with Google Maps link
  Serial2.println(message);
  Serial2.println("Location: https://maps.google.com/?q=12.891057043381641,80.15252109178442");
  delay(100);
  
  // Send Ctrl+Z to indicate end of message
  Serial2.write(26);
  delay(1000);
  
  Serial.println("SMS sent with location!");
}

void activateFallAlert() {
  digitalWrite(BUZZER_PIN, HIGH);  // Turn on buzzer
  digitalWrite(LED_PIN, HIGH);     // Turn on LED
}

void deactivateFallAlert() {
  digitalWrite(BUZZER_PIN, LOW);   // Turn off buzzer
  digitalWrite(LED_PIN, LOW);      // Turn off LED
  fallDirection = "";
}
