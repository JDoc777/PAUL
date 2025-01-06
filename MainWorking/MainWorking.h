//JSON
//-----------------------------------------------------------------------
#include <ArduinoJson.h>
//-----------------------------------------------------------------------

//COUNTER
//-----------------------------------------------------------------------
int counter = 0;                //Serial monitor counter
//-----------------------------------------------------------------------

//SET SPEED AND STOP DISTANCE
//-----------------------------------------------------------------------
// Speed Variables
 int setSpeed = 200;             // set Speed
// int speed = 200;             // set Speed
// int setSpeed = map(speed, 0, 255, 50, 255);


// Stop Distance Variables
int stopDistance = 15;          // Stop Distance
int stopDistance1 = stopDistance;       // Stop Distance
int stopDistance2 = stopDistance + 15;  // 15 cm before Stop Distance
int stopDistance3 = stopDistance + 30;  // 30 cm before Stop Distance
//-----------------------------------------------------------------------

// MOTORS 
//-----------------------------------------------------------------------
// PWM pins for motor speed control
#define ENA1 4              // Motor Driver 1 - Front Left Enable (PWM)
#define ENB1 5              // Motor Driver 1 - Front Right Enable (PWM)
#define ENA2 6              // Motor Driver 2 - Rear Left Enable (PWM)
#define ENB2 7              // Motor Driver 2 - Rear Right Enable (PWM)

// H Bridge direction control pins for motors
#define IN1 39              // Motor Driver 1 - Front Left Direction Pin 1
#define IN2 41              // Motor Driver 1 - Front Left Direction Pin 2
#define IN3 37              // Motor Driver 1 - Front Right Direction Pin 1
#define IN4 35              // Motor Driver 1 - Front Right Direction Pin 2
#define IN5 51              // Motor Driver 2 - Rear Left Direction Pin 1
#define IN6 53              // Motor Driver 2 - Rear Left Direction Pin 2
#define IN7 47              // Motor Driver 2 - Rear Right Direction Pin 1
#define IN8 49              // Motor Driver 2 - Rear Right Direction Pin 2

// Global Variable to Store current speed for each wheel
int currentSpeedFL = 0, currentSpeedFR = 0, currentSpeedRL = 0, currentSpeedRR = 0;

// Variables for speed from Pi
int FL = 0;
int FR = 0;
int BL = 0;
int BR = 0;
//-----------------------------------------------------------------------


// DHT 
//-----------------------------------------------------------------------
#include <DHT.h>                  // DHT Library

// Define the DHT sensor type
#define DHTTYPE DHT11             // Sensor Type DHT11

// Pin definition for Temp & % Sensor
#define DHTPIN 3                  // DHT Signal Pin 3

// Initialize the DHT sensor
DHT dht(DHTPIN, DHTTYPE);         // Initialize DHT

// Temp & Hum Variables
float temperature;            // Tempurature Variable
float humidity;               // Humidity Variable
//-----------------------------------------------------------------------


// SONAR
//-----------------------------------------------------------------------
// Pin definitions for sonar sensors
const int trigPins[4] = {22, 24, 26, 28};  // Trig pins for Front, Back, Left, Right
const int echoPins[4] = {23, 25, 27, 29};  // Echo pins for Front, Back, Left, Right

// Sonar directions
const char* directions[4] = {"F", "B", "L", "R"};  // F = Front, B = Back, L = Left, R = Right

int distances[4];                   // Array to store distances
//-----------------------------------------------------------------------

// Encoder
//-----------------------------------------------------------------------
// Encoder Pins
#define ENCA1 11 // YELLOW FL
#define ENCB1 10 // WHITE  FL
#define ENCA2 13 // YELLOW RL
#define ENCB2 12 // WHITE  RL
#define ENCA3 34 // YELLOW FR
#define ENCB3 36 // WHITE  FR
#define ENCA4 42 // YELLOW RR
#define ENCB4 40 // WHITE  RR

// Position and Rotation Variables
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
//-----------------------------------------------------------------------

// MPU_ADDR GYRO & ACCEL
//-----------------------------------------------------------------------
#include <Wire.h>    // I2C

// MPU-6050 I2C address
const int MPU_ADDR = 0x68;  // Default I2C address for MPU-6050

float gyroX_dps;
float gyroY_dps;
float gyroZ_dps;

float accelX_g;
float accelY_g;
float accelZ_g;

// Raw sensor data variables
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
//-----------------------------------------------------------------------