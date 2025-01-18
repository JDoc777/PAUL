void setup() {
    Serial.begin(9600); // Match the baud rate in the Raspberry Pi code
}

void loop() {
    Serial.println("Hello from Arduino!"); // Example message
    delay(1000); // Send data every second
}
