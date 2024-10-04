#include <Arduino.h>
#include <Arduino_LSM6DS3.h>  // Include the library for the IMU

// Variables to store the estimated angles
float angleX = 0, angleY = 0, angleZ = 0;

// Variable to store the previous time
unsigned long previousTime = 0;

void setup() {
  Serial.begin(9600);  // Start serial communication
  while (!Serial);     // Wait for the serial port to connect

  // Try to initialize the IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);  // Stop the program if initialization fails
  }
  Serial.println("IMU initialized successfully.");

  // Initialize previous time
  previousTime = millis();
}

void loop() {
  float gyroX, gyroY, gyroZ;  // Variables to store the gyroscope data
  unsigned long currentTime = millis();  // Get the current time
  float deltaTime = (currentTime - previousTime) / 1000.0;  // Calculate the time difference in seconds
  previousTime = currentTime;

  // Check if gyroscope data is available
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);  // Read the gyroscope data

    // Integrate gyroscope data to estimate angles
    angleX += gyroX * deltaTime;  // Angle around X-axis
    angleY += gyroY * deltaTime;  // Angle around Y-axis
    angleZ += gyroZ * deltaTime;  // Angle around Z-axis

    unsigned long int last_millis = 0;
    if(millis()-last_millis >= 100)
    {
      // Print the estimated angles to the Serial Monitor
      Serial.print("Angle X: ");
      Serial.print(angleX);
      Serial.print(", Y: ");
      Serial.print(angleY);
      Serial.print(", Z: ");
      Serial.println(angleZ);
      last_millis = millis();
    }
    
  }

  delay(10);  // Small delay to allow time for sensor data update
}