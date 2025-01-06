#include <ArduinoJson.h>

// Define variables
int FL = 0;
int FR = 0;
int BL = 0;
int BR = 0;

// Function to read and parse JSON data from Serial3
void readSerialData() {
  // Check if data is available on Serial3
  if (Serial3.available()) {
    static char buffer[256];
    static size_t index = 0;

    while (Serial3.available()) {
      char c = Serial3.read();
      if (c == '\n') {
        // Null-terminate the string
        buffer[index] = '\0';

        // Parse the JSON data
        StaticJsonDocument<128> doc;
        DeserializationError error = deserializeJson(doc, buffer);

        if (error) {
          Serial.print("JSON Parse Error: ");
          Serial.println(error.c_str());
        } else {
          // Extract the values from JSON
          FL = doc["FL"] | 0;
          FR = doc["FR"] | 0;
          BL = doc["BL"] | 0;
          BR = doc["BR"] | 0;

          // Print the received values (for debugging)
          Serial.print("FL: "); Serial.print(FL);
          Serial.print(", FR: "); Serial.print(FR);
          Serial.print(", BL: "); Serial.print(BL);
          Serial.print(", BR: "); Serial.println(BR);
        }

        // Reset the index for the next message
        index = 0;
        break;
      } else if (index < sizeof(buffer) - 1) {
        buffer[index++] = c; // Add character to buffer
      }
    }
  }
}

void setup() {
  // Initialize Serial3 for UART communication
  Serial3.begin(115200);

  // Optionally, initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to initialize (for debugging)
  }
  Serial.println("Waiting for data on Serial3...");
}

void loop() {
  // Call the function to read and parse data
  readSerialData();

  // Optionally, add other logic in the loop
  // For example, you can control motors or perform other tasks using FL, FR, BL, BR
}
