// ***************************************
// Paul Arduino Sketch
// Contributors: Justin Dougherty, Teddy Weaver
// Last Update: 12/3/24 
// ***************************************

#include "Full_Integration.h"

void setup() {
  // SERIAL COMUNICATION
  //-----------------------------------------------------------------------
  Serial.begin(9600);                                     // Initialize serial communication

  Serial3.begin(9600);

  Serial.println("Serial Communication Transmitting");    // Serial setup message
  //-----------------------------------------------------------------------

  // GYRO I2C
  //-----------------------------------------------------------------------
  Wire.begin();        // Initialize I2C

  // Initialize MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);     // Wake the MPU-6050 up
  Wire.endTransmission();
  
  Serial.println("MPU-6050 Initialized");     // MPU-6050 setup message
  //-----------------------------------------------------------------------

  // DHT SENSOR
  //-----------------------------------------------------------------------
  dht.begin();                              // Start the DHT sensor

  Serial.println("DHT Initialized");        // DHT setup message
  //-----------------------------------------------------------------------

  // SONAR PINS
  //-----------------------------------------------------------------------
  for (int i = 0; i < 4; i++) {             // Loop all sonar sensors
    pinMode(trigPins[i], OUTPUT);           // Trig Pins Output
    pinMode(echoPins[i], INPUT);            // Echo Pins Input
  }

  Serial.println("Sonar Pins Set");         // Sonar setup message
  //-----------------------------------------------------------------------

  // MOTOR PINS
  //-----------------------------------------------------------------------
  // Set all speed pins as outputs
  pinMode(ENA1, OUTPUT); // PWM
  pinMode(ENB1, OUTPUT); // PWM
  pinMode(ENA2, OUTPUT); // PWM
  pinMode(ENB2, OUTPUT); // PWM

  // Set all control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  Serial.println("Motor Pins Set");        // Motor pin setup message

  // Initialize motors off
  stopMotors();                           // Run motor stop 

  Serial.println("Motors Initialized");              // Sonar setup message
  //-----------------------------------------------------------------------

  // ENCODER SETUP
  //-----------------------------------------------------------------------
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);

  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);

  pinMode(ENCA3, INPUT);
  pinMode(ENCB3, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA3), readEncoder3, RISING);

  pinMode(ENCA4, INPUT);
  pinMode(ENCB4, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA4), readEncoder4, RISING);

  Serial.println("Encoders Initialized");              // Encoder setup message
  //-----------------------------------------------------------------------

  // INITIALIZATION COMPLETE
  //-----------------------------------------------------------------------
  Serial.println("Initialization Complete");    // setup complete serial print
  //-----------------------------------------------------------------------
}

void loop() {

  // Read sensor data
  recordDistances();
  readAccelGyro();
  readEncoder();

  // Control motors based on distance
  // controlDirections();
  
  // Counter
  Serial.println(counter); // Print Counter
  Serial3.println(counter); // Print Counter
  counter++;               // Increment counter


  //---------------------------
  //int testSpeed = 100;
  //mecanumDrive(testSpeed, testSpeed, testSpeed, testSpeed); // Full speed forward

  //stopMotors();     // Stop
  // delay(1000);      // Delay
  //testDirections(); // Test Movement

}

void controlDirections() {
  // Check distances and decide movement
  if (distances[0] > stopDistance3) { // Far from obstacle (> 30 cm)
    mecanumDrive(setSpeed, setSpeed, setSpeed, setSpeed); // Full speed forward
    Serial.println("Moving forward at full speed");
  } 
  else if (distances[0] > stopDistance2) { // Approaching obstacle (20-10 cm)
    mecanumDrive(setSpeed - 50, setSpeed - 50, setSpeed - 50, setSpeed - 50); // Slow down
    Serial.println("Slowing down: Near stopDistance3");
  } 
  else if (distances[0] > stopDistance1) { // Very close to obstacle (10 cm to stop distance)
    mecanumDrive(setSpeed - 75, setSpeed - 75, setSpeed - 75, setSpeed - 75); // Slow down more
    Serial.println("Slowing down further: Near stopDistance2");
  } 
  else { // At stop distance
    mecanumDrive(-setSpeed, -setSpeed, -setSpeed, -setSpeed); // Reverse
    Serial.println("Reversing");
    delay(500); // Reverse for 0.5 seconds

    // Compare left and right distances to determine rotation direction
    if (distances[3] > distances[2]) {
      mecanumDrive(setSpeed, -setSpeed, setSpeed, -setSpeed); // Rotate clockwise
      Serial.println("Rotating clockwise (Left is farther)");
    } 
    else if (distances[2] > distances[3]) {
      mecanumDrive(-setSpeed, setSpeed, -setSpeed, setSpeed); // Rotate counterclockwise
      Serial.println("Rotating counterclockwise (Right is farther)");
    } 
    else {
      Serial.println("Distances equal, moving forward");
    }

    delay(500); // Allow time to rotate
    mecanumDrive(setSpeed, setSpeed, setSpeed, setSpeed); // Move forward again
    Serial.println("Moving forward after rotation");
  }
}

// Function to drive Mecanum wheels
// fl, fr, rl, rr: Speed for each wheel (-255 to 255)
void mecanumDrive(int fl, int fr, int rl, int rr) {
  currentSpeedFL = fl;
  currentSpeedFR = fr;
  currentSpeedRL = rl;
  currentSpeedRR = rr;

  driveMotor(ENA1, IN1, IN2, fl); // Front Left
  driveMotor(ENB1, IN3, IN4, fr); // Front Right
  driveMotor(ENA2, IN5, IN6, rl); // Rear Left
  driveMotor(ENB2, IN7, IN8, rr); // Rear Right
}

// Function to control an individual motor
// enablePin: PWM pin, inPin1/inPin2: Direction pins, speed: -255 to 255
void driveMotor(int enablePin, int inPin1, int inPin2, int speed) {
  if (speed > 0) {
    digitalWrite(inPin1, HIGH);       // Forward for (+)
    digitalWrite(inPin2, LOW);
  } else if (speed < 0) {
    digitalWrite(inPin1, LOW);        // Backward for (-)
    digitalWrite(inPin2, HIGH);
    speed = -speed;                   // Make speed positive for PWM
  } else {
    digitalWrite(inPin1, LOW);        // Off for (0)
    digitalWrite(inPin2, LOW);
  }

  analogWrite(enablePin, constrain(speed, 0, 255)); // Set motor speed
}

// Function to stop all motors
void stopMotors() {
  mecanumDrive(0, 0, 0, 0);     // All motors stopped
}

// Function to slowly stop all motors
void slowStop() {
  while (currentSpeedFL != 0 || currentSpeedFR != 0 || currentSpeedRL != 0 || currentSpeedRR != 0) {
    // Decrease speed for each wheel gradually
    currentSpeedFL = approachZero(currentSpeedFL);
    currentSpeedFR = approachZero(currentSpeedFR);
    currentSpeedRL = approachZero(currentSpeedRL);
    currentSpeedRR = approachZero(currentSpeedRR);

    // Update the motor speeds
    driveMotor(ENA1, IN1, IN2, currentSpeedFL);
    driveMotor(ENB1, IN3, IN4, currentSpeedFR);
    driveMotor(ENA2, IN5, IN6, currentSpeedRL);
    driveMotor(ENB2, IN7, IN8, currentSpeedRR);

    delay(25); // Small delay for smooth deceleration
  }
}

// Helper function to approach zero gradually
int approachZero(int speed) {
  if (speed > 0) {
    return max(0, speed - 10); // Decrease positive speed
  } else if (speed < 0) {
    return min(0, speed + 10); // Increase negative speed (towards 0)
  }
  return 0; // Already at zero
}

// Function to randomly turn left or right
void randomTurn() {
  if (random(2) == 0) {
    mecanumDrive(-setSpeed, setSpeed, -setSpeed, setSpeed); // Turn left
  } else {
    mecanumDrive(setSpeed, -setSpeed, setSpeed, -setSpeed); // Turn right
  }
  delay(250);
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

  float accelX_g = accelX / 16384.0;  // Convert to g
  float accelY_g = accelY / 16384.0;
  float accelZ_g = accelZ / 16384.0;
  Serial.print("Accel X: "); Serial.print(accelX_g, 3);  // Print with 3 decimal places
  Serial.print(" | Y: "); Serial.print(accelY_g, 3);
  Serial.print(" | Z: "); Serial.println(accelZ_g, 3);

  // to pi
  Serial3.print("Accel X: "); Serial.print(accelX_g, 3);  // Print with 3 decimal places
  Serial3.print(" | Y: "); Serial.print(accelY_g, 3);
  Serial3.print(" | Z: "); Serial.println(accelZ_g, 3);
  
  // Send gyroscope data (scaled to degrees per second)
  float gyroX_dps = gyroX / 131.0;  // Convert to degrees per second
  float gyroY_dps = gyroY / 131.0;
  float gyroZ_dps = gyroZ / 131.0;
  Serial.print("Gyro X: "); Serial.print(gyroX_dps, 2);  // Print with 2 decimal places
  Serial.print(" | Y: "); Serial.print(gyroY_dps, 2);
  Serial.print(" | Z: "); Serial.println(gyroZ_dps, 2);

  // to pi
  Serial3.print("Gyro X: "); Serial.print(gyroX_dps, 2);  // Print with 2 decimal places
  Serial3.print(" | Y: "); Serial.print(gyroY_dps, 2);
  Serial3.print(" | Z: "); Serial.println(gyroZ_dps, 2);
}

// Function to record distances from each sonar sensor
void recordDistances() {
  float temperature = dht.readTemperature();      // Read Temp.
  float humidity = dht.readHumidity();            // Read Humid.
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Error reading temperature or humidity");
    temperature = 25.0;  // Default to 25°C if DHT fails
    humidity = 50.0;     // Default to 50% if DHT fails
  }

  // Speed of sound
  float speedOfSound = 331.4 + (0.606 * temperature) + (0.0124 * humidity);   // Calculate speed of sound

  // Measure Distances
  for (int i = 0; i < 4; i++) {
    distances[i] = measureDistance(trigPins[i], echoPins[i], speedOfSound);   // Use function measureDistance
  }

  // Serial Print Distances ("Direction: Distance cm")
  Serial.println("Distances:");
  for (int i = 0; i < 4; i++) {
    Serial.print(directions[i]);
    Serial.print(": ");
    Serial.print(distances[i]);
    Serial.println(" cm");

    Serial3.print(directions[i]);
    Serial3.print(": ");
    Serial3.print(distances[i]);
    Serial3.println(" cm");
  }
  
}

// Function to measure distance using a sonar sensor
int measureDistance(int trigPin, int echoPin, float speedOfSound) {  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);                // Trigger pulse
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration * (speedOfSound / 10000.0)) / 2.0;  // Convert to cm
  return (int)distance;
}

// Test Directions
void testDirections() {
  // Forward movement
  mecanumDrive(255, 255, 255, 255); // All wheels forward at full speed
  delay(1000);

  slowStop(); // Slowly stop the motors
  delay(1000);

  // Backward movement
  mecanumDrive(-255, -255, -255, -255); // All wheels backward at full speed
  delay(1000);

  slowStop(); // Slowly stop the motors
  delay(1000);

  // Crab walk right
  mecanumDrive(255, -255, -255, 255); // Diagonal motion for right crab walk
  delay(1000);

  slowStop(); // Slowly stop the motors
  delay(1000);

  // Crab walk left
  mecanumDrive(-255, 255, 255, -255); // Diagonal motion for left crab walk
  delay(1000);

  slowStop(); // Slowly stop the motors
  delay(1000);

  // Rotate clockwise
  mecanumDrive(255, -255, 255, -255); // Opposite directions for adjacent wheels
  delay(1000);

  slowStop(); // Slowly stop the motors
  delay(1000);

  // Rotate counterclockwise
  mecanumDrive(-255, 255, -255, 255); // Opposite directions for adjacent wheels
  delay(1000);

  slowStop(); // Slowly stop the motors
  delay(1000);

  // Stop all motors
  stopMotors();
}

// Read All Encoders
void readEncoder(){
  encoderFL(); // Encoder 1: Front Left
  encoderRL(); // Encoder 2: Rear Left
  encoderFR(); // Encoder 3: Front Right
  encoderRR(); // Encoder 4: Rear Right
}

void encoderFL() {
  int pos = 0;
  int revs = 0;
  float distanceTraveled = 0.0;

  // Safely read shared variables
  noInterrupts();
  pos = posi1;
  revs = rotations1;
  distanceTraveled = rotations1 * WHEEL_CIRCUMFERENCE_CM;
  interrupts();

  // Print Encoder 1 data
  Serial.print("Encoder 1 - Position: ");
  Serial.print(pos);
  Serial.print(", Rotations: ");
  Serial.print(revs);
  Serial.print(", Distance Traveled (cm): ");
  Serial.println(distanceTraveled);
}

void encoderRL() {
  int pos = 0;
  int revs = 0;
  float distanceTraveled = 0.0;

  // Safely read shared variables
  noInterrupts();
  pos = posi2;
  revs = rotations2;
  distanceTraveled = rotations2 * WHEEL_CIRCUMFERENCE_CM;
  interrupts();

  // Print Encoder 2 data
  Serial.print("Encoder 2 - Position: ");
  Serial.print(pos);
  Serial.print(", Rotations: ");
  Serial.print(revs);
  Serial.print(", Distance Traveled (cm): ");
  Serial.println(distanceTraveled);
}

void encoderFR() {
  int pos = 0;
  int revs = 0;
  float distanceTraveled = 0.0;

  // Safely read shared variables
  noInterrupts();
  pos = posi3;
  revs = rotations3;
  distanceTraveled = rotations3 * WHEEL_CIRCUMFERENCE_CM;
  interrupts();

  // Print Encoder 2 data
  Serial.print("Encoder 3 - Position: ");
  Serial.print(pos);
  Serial.print(", Rotations: ");
  Serial.print(revs);
  Serial.print(", Distance Traveled (cm): ");
  Serial.println(distanceTraveled);
}

void encoderRR() {
  int pos = 0;
  int revs = 0;
  float distanceTraveled = 0.0;

  // Safely read shared variables
  noInterrupts();
  pos = posi4;
  revs = rotations4;
  distanceTraveled = rotations4 * WHEEL_CIRCUMFERENCE_CM;
  interrupts();

  // Print Encoder 2 data
  Serial.print("Encoder 4 - Position: ");
  Serial.print(pos);
  Serial.print(", Rotations: ");
  Serial.print(revs);
  Serial.print(", Distance Traveled (cm): ");
  Serial.println(distanceTraveled);
}

void readEncoder1() {
  int b = digitalRead(ENCB1);

  if (b > 0) {
    posi1++;
  } else {
    posi1--;
  }

  if (posi1 >= PULSES_PER_REV) {
    posi1 -= PULSES_PER_REV;
    rotations1++;
  } else if (posi1 <= -PULSES_PER_REV) {
    posi1 += PULSES_PER_REV;
    rotations1--;
  }
}

void readEncoder2() {
  int b = digitalRead(ENCB2);

  if (b > 0) {
    posi2++;
  } else {
    posi2--;
  }

  if (posi2 >= PULSES_PER_REV) {
    posi2 -= PULSES_PER_REV;
    rotations2++;
  } else if (posi2 <= -PULSES_PER_REV) {
    posi2 += PULSES_PER_REV;
    rotations2--;
  }
}

void readEncoder3() {
  int b = digitalRead(ENCB3);

  if (b > 0) {
    posi3++;
  } else {
    posi3--;
  }

  if (posi3 >= PULSES_PER_REV) {
    posi3 -= PULSES_PER_REV;
    rotations3++;
  } else if (posi3 <= -PULSES_PER_REV) {
    posi3 += PULSES_PER_REV;
    rotations3--;
  }
}

void readEncoder4() {
  int b = digitalRead(ENCB4);

  if (b > 0) {
    posi4++;
  } else {
    posi4--;
  }

  if (posi4 >= PULSES_PER_REV) {
    posi4 -= PULSES_PER_REV;
    rotations4++;
  } else if (posi4 <= -PULSES_PER_REV) {
    posi4 += PULSES_PER_REV;
    rotations4--;
  }
}