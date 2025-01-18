// ***************************************
// Paul Arduino Sketch
// Contributors: Justin Dougherty, Teddy Weaver
// Last Update: 1/4/25 
// ***************************************

#include "MainWorking.h"

void setup() {
  // SERIAL COMUNICATION
  //-----------------------------------------------------------------------
  Serial.begin(9600);              // Initialize serial 9600 communication

  Serial3.begin(250000);           // Initialize serial 115200 communication

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
  if (rst_flag) {
    NVIC_SystemReset(); // Reset the microcontroller
  }
    // Read sensor data                 // Encoders are Interupt
    recordDistances();
    readAccelGyro();

    // Send Serial data
    sendSensorData_to_Pi();
    // sendSensorData_to_Monitor();

    // Read UART input
    readSerialData();

    if (wheel_off){
      mecanumDrive(0,0,0,0);
    } else {
      mecanumDrive(FL, FR, BL, BR);
    }

    // Counter
    Serial.println(counter); // Print Counter
    counter++;               // Increment counter
  

  //--------------------------------------------------------------------------------

  //int testSpeed = 100;
  //mecanumDrive(testSpeed, testSpeed, testSpeed, testSpeed); // Full speed forward

  //stopMotors();     // Stop
  // delay(1000);      // Delay
  //testDirections(); // Test Movement

}

// Serial Output On Arduino
void sendSensorData_to_Monitor() {
  // Serial Print Distances ("Direction: Distance cm")
  Serial.println("Distances:");
  for (int i = 0; i < 4; i++) {
    Serial.print(directions[i]);
    Serial.print(": ");
    Serial.print(distances[i]);
    Serial.println(" cm");
  }

  // Accellerometer Data
  Serial.print("Accel X: "); Serial.print(accelX_g, 3);  // Print with 3 decimal places
  Serial.print(" | Y: "); Serial.print(accelY_g, 3);
  Serial.print(" | Z: "); Serial.println(accelZ_g, 3);

  // Gyroscope Data
  Serial.print("Gyro X: "); Serial.print(gyroX_dps, 2);  // Print with 2 decimal places
  Serial.print(" | Y: "); Serial.print(gyroY_dps, 2);
  Serial.print(" | Z: "); Serial.println(gyroZ_dps, 2);

  // Safely read shared variables
  noInterrupts();
  int pos1 = posi1, pos2 = posi2, pos3 = posi3, pos4 = posi4;
  int revs1 = rotations1, revs2 = rotations2, revs3 = rotations3, revs4 = rotations4;
  interrupts();

  // Calculate distances
  float distance1 = revs1 * WHEEL_CIRCUMFERENCE_CM;
  float distance2 = revs2 * WHEEL_CIRCUMFERENCE_CM;
  float distance3 = revs3 * WHEEL_CIRCUMFERENCE_CM;
  float distance4 = revs4 * WHEEL_CIRCUMFERENCE_CM;

  // Print Encoder Data
  Serial.print("Encoder 1 - Position: "); Serial.print(pos1);
  Serial.print(", Rotations: "); Serial.print(revs1);
  Serial.print(", Distance Traveled (cm): "); Serial.println(distance1);

  Serial.print("Encoder 2 - Position: "); Serial.print(pos2);
  Serial.print(", Rotations: "); Serial.print(revs2);
  Serial.print(", Distance Traveled (cm): "); Serial.println(distance2);

  Serial.print("Encoder 3 - Position: "); Serial.print(pos3);
  Serial.print(", Rotations: "); Serial.print(revs3);
  Serial.print(", Distance Traveled (cm): "); Serial.println(distance3);

  Serial.print("Encoder 4 - Position: "); Serial.print(pos4);
  Serial.print(", Rotations: "); Serial.print(revs4);
  Serial.print(", Distance Traveled (cm): "); Serial.println(distance4);

  // Print Temp and Humidity
  Serial.print("temperature: "); Serial.println(temperature);
  Serial.print("Humidity: "); Serial.println(humidity);
}

// Serial3 Outut sent to Raspberry Pi
void sendSensorData_to_Pi() {
  // Create Json doc
  StaticJsonDocument<400> docSend;

  // Distance data
  docSend["distance"]["F"] = distances[0];
  docSend["distance"]["B"] = distances[1];
  docSend["distance"]["L"] = distances[2];
  docSend["distance"]["R"] = distances[3];

  // Gyroscope data
  docSend["gyro"]["x"] = gyroX_dps;
  docSend["gyro"]["y"] = gyroY_dps;
  docSend["gyro"]["z"] = gyroZ_dps;

  // Accelerometer data
  docSend["accel"]["x"] = accelX_g;
  docSend["accel"]["y"] = accelY_g;
  docSend["accel"]["z"] = accelZ_g;

  // Encoder data
  docSend["enco"]["1"]["position"] = posi1;
  docSend["enco"]["1"]["rotation"] = rotations1;
  docSend["enco"]["2"]["position"] = posi2;
  docSend["enco"]["2"]["rotation"] = rotations2;
  docSend["enco"]["3"]["position"] = posi3;
  docSend["enco"]["3"]["rotation"] = rotations3;
  docSend["enco"]["4"]["position"] = posi4;
  docSend["enco"]["4"]["rotation"] = rotations4;

  docSend["environment"]["temperature"] = temperature;
  docSend["environment"]["humidity"] = humidity;

  // Serialize JSON and send it via Serial3
  serializeJson(docSend, Serial3);
  Serial3.println(); // New line for easier reading

  // Flush the Serial buffer to ensure the data is sent
  Serial3.flush();
}

void readSerialData() {
  static char buffer[256];  // Buffer to hold incoming message
  static int bufferIndex = 0;  // Pointer for buffer

  // Check if data is available to read
  while (Serial3.available()) {
    char c = Serial3.read();  // Read a byte from the serial buffer

    // Ignore any whitespace or newlines before the actual message
    if (c == '\n' || c == '\r' || c == ' ') {
      continue;
    }

    // If we see a '{' and it's a valid starting point, reset buffer to start a new message
    if (c == '{' && bufferIndex > 0) {
      // If there's already an incomplete message, discard the old one
      bufferIndex = 0;  
    }

    // Add the character to the buffer if space is available
    if (bufferIndex < sizeof(buffer) - 1) {
      buffer[bufferIndex++] = c;
    } else {
      // Handle buffer overflow: reset it if necessary
      Serial.println("Buffer overflow. Resetting.");
      bufferIndex = 0;
    }

    // If we reach the end of the message (closing '}'), process the message
    if (c == '}') {
      buffer[bufferIndex] = '\0';  // Null-terminate the string
      Serial.print("Received message: ");
      Serial.println(buffer);  // Print the received message for debugging

      // Check for the valid message format
      if (buffer[0] == '{' && buffer[bufferIndex - 1] == '}') {
        // Attempt to parse the JSON message
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, buffer);

        if (error) {
          Serial.print("JSON Parse Error: ");
          Serial.println(error.c_str());
        } else {
          // Successfully parsed the message
          FL = doc["FL"] | 0;
          FR = doc["FR"] | 0;
          BL = doc["BL"] | 0;
          BR = doc["BR"] | 0;

          wheel_off = doc["wheel_off"];

          rst_flag = doc["rst_flag"];

          // Print the parsed values for debugging
          Serial.print("Parsed values: ");
          Serial.print("FL: "); Serial.print(FL);
          Serial.print(", FR: "); Serial.print(FR);
          Serial.print(", BL: "); Serial.print(BL);
          Serial.print(", BR: "); Serial.println(BR);

          Serial.print("Wheels off flag:"); Serial.println(wheel_off);
          Serial.print("Reset flag:"); Serial.println(rst_flag);
        }
      } else {
        // If the message is malformed, discard it
        Serial.println("Invalid message format. Discarding...");
      }

      // Reset the buffer and index after processing the message
      bufferIndex = 0;
    }
  }
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

  accelX_g = accelX / 16384.0;  // Convert to g
  accelY_g = accelY / 16384.0;
  accelZ_g = accelZ / 16384.0;
  
  // Send gyroscope data (scaled to degrees per second)
  gyroX_dps = gyroX / 131.0;  // Convert to degrees per second
  gyroY_dps = gyroY / 131.0;
  gyroZ_dps = gyroZ / 131.0;
}

// Function to record distances from each sonar sensor
void recordDistances() {
  temperature = dht.readTemperature();      // Read Temp.
  humidity = dht.readHumidity();            // Read Humid.
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

void readEncoder1() {
  int b = digitalRead(ENCB1);

  if (b > 0) {
    posi1++;
  } else {
    posi1--;
  }

  rotations1 = posi1 / PULSES_PER_REV;
}

void readEncoder2() {
  int b = digitalRead(ENCB2);

  if (b > 0) {
    posi2++;
  } else {
    posi2--;
  }

  rotations2 = posi2 / PULSES_PER_REV;
}

void readEncoder3() {
  int b = digitalRead(ENCB3);

  if (b > 0) {
    posi3++;
  } else {
    posi3--;
  }

  rotations3 = posi3 / PULSES_PER_REV;
}

void readEncoder4() {
  int b = digitalRead(ENCB4);

  if (b > 0) {
    posi4++;
  } else {
    posi4--;
  }

  rotations4 = posi4 / PULSES_PER_REV;
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

//Control Direction from arduino
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