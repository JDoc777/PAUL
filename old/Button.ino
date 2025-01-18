const int buttonPin = 2;      // Button connected to pin 2
const int ledPins[] = {5, 6, 7}; // Three LEDs connected to pins 3, 4, and 5
int buttonState = 0;           // Variable to hold the button state
int lastButtonState = 0;
bool buttonPressed = false;
int counter = 0;

void setup() {
  
    Serial.begin(115200);      // Main USB serial for the Serial Monitor
         // Initialize Serial3 on pins 14 (TX3) and 15 (RX3)
    pinMode(buttonPin, INPUT_PULLUP); // Set button pin as input with pullup
    
    }


void loop() {
  
    buttonState = digitalRead(buttonPin);  // Read the button state

    if (buttonState == LOW && lastButtonState == HIGH) { // Button press detected
        buttonPressed = true;
        counter++; 
    }
    lastButtonState = buttonState;

    if (buttonPressed) buttonPressed = false;
    
    
    if (Serial.available() > 0) { //If there is serial information availabe (number of bytes > 0) 
      
    String message = Serial.readStringUntil('\n'); //Read the message from the Pi until it gets to \n character 
    message = message + " " + String(counter); //Add the counter to the message
     
    if (counter % 10 == 0) Serial.println("Button pressed " + String(counter) + " times!"); 
    else Serial.println(message); //Print the message aka return it to the pi 
    
    
    
    
    
    }

    
    

       
        }
    
