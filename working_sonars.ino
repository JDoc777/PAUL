// Pin definitions for sonar sensors
const int trigPins[4] = {22, 29, 25, 27};  // Trig pins for Front, Back, Left, Right
const int echoPins[4] = {23, 28, 24, 26};  // Echo pins for Front, Back, Left, Right

// Sonar directions
const char* directions[4] = {"Front", "Back", "Left", "Right"};
int distances[4];                          // Array to store distances
int maxDistance = 0;                       // Variable to store the farthest distance
int maxIndex = 0;                          // Variable to store the index of the farthest direction

void setup() {
  Serial.begin(9600);
  
  // Set up sonar pins
  for (int i = 0; i < 4; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

void loop() {
  recordDistances();      // Get distances from sensors
  findFarthestDistance(); // Determine which direction is the farthest
  outputDistances();      // Output distances and farthest direction
  delay(1000);            // Delay before next reading
}

// Function to record distances from each sonar sensor
void recordDistances() {
  for (int i = 0; i < 4; i++) {
    distances[i] = measureDistance(trigPins[i], echoPins[i]);
  }
}

// Function to find the direction with the farthest distance
void findFarthestDistance() {
  maxDistance = distances[0];
  maxIndex = 0;
  for (int i = 1; i < 4; i++) {
    if (distances[i] > maxDistance) {
      maxDistance = distances[i];
      maxIndex = i;
    }
  }
}

// Function to output distances and the farthest direction
void outputDistances() {
  Serial.println("Distances:");
  for (int i = 0; i < 4; i++) {
    Serial.print(directions[i]);
    Serial.print(": ");
    Serial.print(distances[i]);
    Serial.println(" cm");
  }
  Serial.print("Farthest Direction: ");
  Serial.print(directions[maxIndex]);
  Serial.print(" with ");
  Serial.print(maxDistance);
  Serial.println(" cm");
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