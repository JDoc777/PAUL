//COUNTER
//-----------------------------------------------------------------------
int counter = 0;                //Serial monitor counter
//-----------------------------------------------------------------------

//SET SPEED AND STOP DISTANCE
//-----------------------------------------------------------------------
// CHANGE Variables [      ]
int setSpeed = 200;             // set Speed
int stopDistance = 15;          // Stop Distance

// Gradual Stop Variables
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

// Encoder pins for each wheel
#define FR_ENCODER_A 36     //Front Right Encoder Pin A
#define FR_ENCODER_B 34     //Front Right Encoder Pin B
#define FL_ENCODER_A 11     //Front Left Encoder Pin A
#define FL_ENCODER_B 10     //Front Left Encoder Pin B
#define RL_ENCODER_A 13     //Real Left Encoder Pin A
#define RL_ENCODER_B 12     //Real Left Encoder Pin B
#define RR_ENCODER_A 40     //Real Right Encoder Pin A
#define RR_ENCODER_B 42     //Real Right Encoder Pin B

// Wheel and encoder specifications
#define WHEEL_DIAMETER 0.1 // Diameter in meters (e.g., 10 cm = 0.1 m)
#define ENCODER_RESOLUTION 360 // Pulses per revolution

// Variables to track encoder counts
volatile long frCount = 0, flCount = 0, rlCount = 0, rrCount = 0;

// Variables to track distance (in meters)
float frDistance = 0, flDistance = 0, rlDistance = 0, rrDistance = 0;

// Global Variable to Store current speed for each wheel
int currentSpeedFL = 0, currentSpeedFR = 0, currentSpeedRL = 0, currentSpeedRR = 0;
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
