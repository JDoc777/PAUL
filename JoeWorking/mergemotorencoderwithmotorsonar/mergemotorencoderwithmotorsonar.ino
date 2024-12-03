// PWM pins for motor speed control hi
#define ENA1 4 // Motor Driver 1 - Front Left Enable (PWM)
#define ENB1 5 // Motor Driver 1 - Front Right Enable (PWM)
#define ENA2 6 // Motor Driver 2 - Rear Left Enable (PWM)
#define ENB2 7 // Motor Driver 2 - Rear Right Enable (PWM)

// H Bridge direction control pins for motors
#define IN1 39  // Motor Driver 1 - Front Left Direction Pin 1
#define IN2 41  // Motor Driver 1 - Front Left Direction Pin 2
#define IN3 37  // Motor Driver 1 - Front Right Direction Pin 1
#define IN4 35  // Motor Driver 1 - Front Right Direction Pin 2
#define IN5 51 // Motor Driver 2 - Rear Left Direction Pin 1
#define IN6 53 // Motor Driver 2 - Rear Left Direction Pin 2
#define IN7 47 // Motor Driver 2 - Rear Right Direction Pin 1
#define IN8 49 // Motor Driver 2 - Rear Right Direction Pin 2

#define ENCA1 11 // YELLOW FL
#define ENCB1 10 // WHITE  FL
#define ENCA2 13 // YELLOW RL
#define ENCB2 12 // WHITE  RL
#define ENCA3 34 // YELLOW FR
#define ENCB3 36 // WHITE  FR
#define ENCA4 42 // YELLOW RR
#define ENCB4 40 // WHITE  RR

volatile int posi1 = 0;       // Current position for Encoder 1 (pulses)
volatile int rotations1 = 0;  // Rotations for Encoder 1

volatile int posi2 = 0;       // Current position for Encoder 2 (pulses)
volatile int rotations2 = 0;  // Rotations for Encoder 2

volatile int posi3 = 0;       // Current position for Encoder 3 (pulses)
volatile int rotations3 = 0;  // Rotations for Encoder 3

volatile int posi4 = 0;       // Current position for Encoder 4 (pulses)
volatile int rotations4 = 0;  // Rotations for Encoder 4

const int PULSES_PER_REV = 111.25;  // Encoder pulses per revolution
const float WHEEL_DIAMETER = 80.0;  // Diameter of the wheel in mm

// Calculate the wheel's circumference in mm
const float WHEEL_CIRCUMFERENCE_MM = PI * WHEEL_DIAMETER;  // in mm
// Convert circumference from mm to cm
const float WHEEL_CIRCUMFERENCE_CM = WHEEL_CIRCUMFERENCE_MM / 10.0;  // in cm
const float DISTANCE_PER_PULSE_CM = WHEEL_CIRCUMFERENCE_CM / PULSES_PER_REV;  // Distance traveled per pulse in cm


// Global Variable to Store current speed for each wheel
int currentSpeedFL = 0, currentSpeedFR = 0, currentSpeedRL = 0, currentSpeedRR = 0;

int speed = 200;

void setup() {
  // Set all control pins as outputs
  pinMode(ENA1, OUTPUT); // PWM
  pinMode(ENB1, OUTPUT); // PWM
  pinMode(ENA2, OUTPUT); // PWM
  pinMode(ENB2, OUTPUT); // PWM

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  // Initialize motors off
  stopMotors();
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
}

void loop() {

  mecanumDrive(speed, speed, speed, speed); // fl, fr, rl, rr
  encoderFL(); // Encoder 1: Front Left
  encoderRL(); // Encoder 2: Rear Left
  encoderFR(); // Encoder 3: Front Right
  encoderRR(); // Encoder 4: Rear Right
  delay(100);


  // // Forward movement
  // mecanumDrive(255, 255, 255, 255); // All wheels forward at full speed
  // delay(1000);

  // slowStop(); // Slowly stop the motors
  // delay(1000);

  // // Backward movement
  // mecanumDrive(-255, -255, -255, -255); // All wheels backward at full speed
  // delay(1000);

  // slowStop(); // Slowly stop the motors
  // delay(1000);

  // // Crab walk right
  // mecanumDrive(255, -255, -255, 255); // Diagonal motion for right crab walk
  // delay(1000);

  // slowStop(); // Slowly stop the motors
  // delay(1000);

  // // Crab walk left
  // mecanumDrive(-255, 255, 255, -255); // Diagonal motion for left crab walk
  // delay(1000);

  // slowStop(); // Slowly stop the motors
  // delay(1000);

  // // Rotate clockwise
  // mecanumDrive(255, -255, 255, -255); // Opposite directions for adjacent wheels
  // delay(1000);

  // slowStop(); // Slowly stop the motors
  // delay(1000);

  // // Rotate counterclockwise
  // mecanumDrive(-255, 255, -255, 255); // Opposite directions for adjacent wheels
  // delay(1000);

  // slowStop(); // Slowly stop the motors
  // delay(1000);

  // // Stop all motors
  // stopMotors();

  // delay(2000);
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
    digitalWrite(inPin1, HIGH);
    digitalWrite(inPin2, LOW);
  } else if (speed < 0) {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, HIGH);
    speed = -speed; // Make speed positive for PWM
  } else {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, LOW);
  }

  analogWrite(enablePin, constrain(speed, 0, 255)); // Set motor speed
}

// Function to stop all motors
void stopMotors() {
  mecanumDrive(0, 0, 0, 0);
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

    delay(50); // Small delay for smooth deceleration
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
