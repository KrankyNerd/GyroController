#include <Arduino.h>
#include <Arduino_LSM6DS3.h>  // Include the library for the IMU

float angleX = 0, angleY = 0, angleZ = 0;  // Estimated angles (pitch, roll, yaw)
float alpha = 0.98;                        // Complementary filter constant

unsigned long previousTime = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait for serial connection

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU initialized successfully.");

  previousTime = millis();
}

void loop() {
  float gyroX, gyroY, gyroZ;
  float accelX, accelY, accelZ;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;  // Time in seconds
  previousTime = currentTime;

  // Read gyroscope data
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
  }

  // Read accelerometer data
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
  }

  // Calculate angles from accelerometer data (pitch and roll)
  float accelAngleX = atan2(accelY, accelZ) * 180 / PI;
  float accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

  // Integrate gyroscope data (rate of change of angle)
  angleX += gyroX * deltaTime;  // Update angleX (pitch)
  angleY += gyroY * deltaTime;  // Update angleY (roll)

  // Correct Z-axis (yaw) drift using accelerometer
  // If the board is approximately level, estimate yaw from accelerometer
  if (abs(accelZ) > 0.8) {  // Check if the board is mostly level
    float accelAngleZ = atan2(accelY, accelX) * 180 / PI;  // Approximate Z angle
    angleZ = alpha * (angleZ + gyroZ * deltaTime) + (1.0 - alpha) * accelAngleZ;
  } else {
    // If not level, just use gyroscope data for yaw
    angleZ += gyroZ * deltaTime;
  }

  // Apply complementary filter to combine accelerometer and gyroscope data for X and Y (pitch and roll)
  angleX = alpha * angleX + (1.0 - alpha) * accelAngleX;
  angleY = alpha * angleY + (1.0 - alpha) * accelAngleY;

  // Output the filtered angles
  Serial.print("Angle X (Pitch): ");
  Serial.print(angleX);
  Serial.print(", Y (Roll): ");
  Serial.print(angleY);
  Serial.print(", Z (Yaw): ");
  Serial.println(angleZ);  // Note: Z-axis has some drift compensation

  //delay(10);  // Small delay for sensor updates
}
