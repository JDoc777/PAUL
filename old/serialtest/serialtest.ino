void setup() {
    Serial3.begin(9600); // Match the baud rate in the Raspberry Pi code
}

void loop() {
    Serial3.println("Hello from Arduino!"); // Example message
    delay(1000); // Send data every second
}
