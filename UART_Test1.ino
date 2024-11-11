// Pin definitions for sonar sensors
const int trigPins[4] = {2, 3, 4, 5};  // Trig pins for Front, Back, Left, Right
const int echoPins[4] = {6, 7, 8, 9};  // Echo pins for Front, Back, Left, Right

// Pin definitions for LEDs
const int ledPins[3] = {10, 11, 12};    // 3 LEDs for binary counting

// Pin definition for button
const int buttonPin = 13;               // Button pin for pull-up input

// Sonar directions
const char* directions[4] = {"Front", "Back", "Left", "Right"};
int distances[4];                       // Array to store distances

// Button state
bool lastButtonState = HIGH;
bool buttonPressed = false;

void setup() {
  Serial.begin(9600);
  
  // Set up sonar pins
  for (int i = 0; i < 4; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // Set up LED pins
  for (int i = 0; i < 3; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  // Set up button pin as INPUT_PULLUP
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // Measure distances for all directions
  for (int i = 0; i < 4; i++) {
    distances[i] = measureDistance(trigPins[i], echoPins[i]);
  }

  // Find the direction with the farthest distance
  int maxDistance = distances[0];
  int maxIndex = 0;
  for (int i = 1; i < 4; i++) {
    if (distances[i] > maxDistance) {
      maxDistance = distances[i];
      maxIndex = i;
    }
  }

  // Output distances and farthest direction
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
  
  // Read button state
  bool buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressed = !buttonPressed;  // Toggle button state
  }
  lastButtonState = buttonState;

  // Output button state
  Serial.print("Button State: ");
  Serial.println(buttonPressed ? "Pressed" : "Not Pressed");

  // Control LEDs in binary counting mode when button is pressed
  if (buttonPressed) {
    for (int i = 0; i < 8; i++) {  // Binary count from 0 to 7
      for (int j = 0; j < 3; j++) {
        digitalWrite(ledPins[j], (i >> j) & 1);
      }
      delay(250);
    }
  } else {
    // Turn off LEDs when button is not pressed
    for (int i = 0; i < 3; i++) {
      digitalWrite(ledPins[i], LOW);
    }
  }

  delay(1000);  // Delay before the next reading
}

// Function to measure distance using sonar sensor
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
