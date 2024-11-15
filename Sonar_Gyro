#include <Wire.h>

// MPU-6050 I2C address
const int MPU_ADDR = 0x68;  // Default I2C address for MPU-6050

// Raw sensor data variables
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

// Pin definitions for sonar sensors
const int trigPins[4] = {22, 24, 26, 28};  // Trig pins for Front, Back, Left, Right
const int echoPins[4] = {23, 25, 27, 29};  // Echo pins for Front, Back, Left, Right

// Pin definitions for LEDs
const int ledPins[3] = {7, 6, 5};  // LED pins for up/down, left/right, forward/backward

// Sonar directions
const char* directions[4] = {"F", "R", "L", "B"};  // F = Front, R = Right, L = Left, B = Back

int distances[4];  // Array to store distances

void setup() {
  Serial.begin(9600);  // Initialize UART for serial communication
  Wire.begin();        // Initialize I2C
  
  // Set up sonar pins
  for (int i = 0; i < 4; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // Set up LED pins
  for (int i = 0; i < 3; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  // Initialize MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);     // Wake the MPU-6050 up
  Wire.endTransmission();
  
  Serial.println("MPU-6050 and Sonar Initialized");
}

void loop() {
  // Read gyro and accelerometer data
  readAccelGyro();

  // Record distances from sonar sensors
  recordDistances();

  // Send all data over UART
  sendData();

  // Output data and control LEDs based on movement
  //Output();

  delay(100);  // Delay before next reading
}

// Function to read accelerometer and gyroscope data
void readAccelGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);  // Request 14 bytes for accel and gyro

  // Read accelerometer data
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  
  // Skip temperature data (two bytes)
  Wire.read();
  Wire.read();
  
  // Read gyroscope data
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

// Function to record distances from each sonar sensor
void recordDistances() {
  for (int i = 0; i < 4; i++) {
    distances[i] = measureDistance(trigPins[i], echoPins[i]);
  }
}

// Function to send both gyro and sonar data over UART
void sendData() {
  // Send accelerometer data (scaled to g)
  float accelX_g = accelX / 16384.0;  // Convert to g
  float accelY_g = accelY / 16384.0;
  float accelZ_g = accelZ / 16384.0;
  Serial.print("Accel X: "); Serial.print(accelX_g, 3);  // Print with 3 decimal places
  Serial.print(" | Y: "); Serial.print(accelY_g, 3);
  Serial.print(" | Z: "); Serial.println(accelZ_g, 3);
  
  // Send gyroscope data (scaled to degrees per second)
  float gyroX_dps = gyroX / 131.0;  // Convert to degrees per second
  float gyroY_dps = gyroY / 131.0;
  float gyroZ_dps = gyroZ / 131.0;
  Serial.print("Gyro X: "); Serial.print(gyroX_dps, 2);  // Print with 2 decimal places
  Serial.print(" | Y: "); Serial.print(gyroY_dps, 2);
  Serial.print(" | Z: "); Serial.println(gyroZ_dps, 2);

  // Send distances from sonar sensors with labels F, R, L, B
  for (int i = 0; i < 4; i++) {
    Serial.print("Distance ");
    Serial.print(directions[i]);
    Serial.print(": ");
    Serial.print(distances[i]);  // Send the distance value
    Serial.println(" cm");
  }

  Serial.println();  // Blank line for readability
}

// Function to measure distance using a sonar sensor
int measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;  // Convert duration to cm
  return distance;
}

// Output function to control LEDs based on movement direction
void Output() {
  // Control LED for up/down movement (based on accelZ)
  if (accelZ > 2000) {  // Threshold for upward movement (adjust as needed)
    digitalWrite(ledPins[0], HIGH);  // Turn on Up/Down LED
  } else if (accelZ < -2000) {  // Threshold for downward movement (adjust as needed)
    digitalWrite(ledPins[0], HIGH);  // Turn on Up/Down LED
  } else {
    digitalWrite(ledPins[0], LOW);  // Turn off Up/Down LED
  }

  // Control LED for left/right movement (based on accelX)
  if (accelX > 2000) {  // Threshold for right movement (adjust as needed)
    digitalWrite(ledPins[1], HIGH);  // Turn on Left/Right LED
  } else if (accelX < -2000) {  // Threshold for left movement (adjust as needed)
    digitalWrite(ledPins[1], HIGH);  // Turn on Left/Right LED
  } else {
    digitalWrite(ledPins[1], LOW);  // Turn off Left/Right LED
  }

  // Control LED for forward/backward movement (based on accelY)
  if (accelY > 2000) {  // Threshold for forward movement (adjust as needed)
    digitalWrite(ledPins[2], HIGH);  // Turn on Forward/Backward LED
  } else if (accelY < -2000) {  // Threshold for backward movement (adjust as needed)
    digitalWrite(ledPins[2], HIGH);  // Turn on Forward/Backward LED
  } else {
    digitalWrite(ledPins[2], LOW);  // Turn off Forward/Backward LED
  }
}
