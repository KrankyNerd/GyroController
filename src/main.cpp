#include <Arduino.h>
#include "sbus.h"
#include <Arduino_LSM6DS3.h>  // Include the library for the IMU

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial1);
/* SBUS data */
bfs::SbusData data;

/* IMU angle data */
float angleX = 0, angleY = 0, angleZ = 0;  // Estimated angles (pitch, roll, yaw)
float alpha = 0.98;                        // Complementary filter constant

unsigned long previousTime = 0;

/* Map the angle (-180 to 180 degrees) to SBUS range (172 to 1811) */
uint16_t mapAngleToSbus(float angle) {
  return map(angle, -90, 90, 172, 1811);  // Map angle (-180 to 180) to SBUS channel range (172 to 1811)
}

/* Function to update IMU and calculate angles */
void updateIMU() {
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
  angleZ += gyroZ * deltaTime;  // Update angleZ (yaw)

  // Apply complementary filter to combine accelerometer and gyroscope data for X and Y (pitch and roll)
  angleX = alpha * angleX + (1.0 - alpha) * accelAngleX;
  angleY = alpha * angleY + (1.0 - alpha) * accelAngleY;
}

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  //while (!Serial) {}
  
  /* Initialize SBUS communication */
  //sbus_rx.Begin();
  sbus_tx.Begin();

  /* Initialize IMU */
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU initialized successfully.");

  previousTime = millis();
}

void loop () {
  // Read IMU data (Gyroscope and Accelerometer)
  updateIMU();

  // Map X, Y, Z angles to SBUS channels
  data.ch[0] = mapAngleToSbus(angleX);  // Channel 1 for X (Pitch)
  data.ch[1] = mapAngleToSbus(angleY);  // Channel 2 for Y (Roll)
  data.ch[2] = mapAngleToSbus(angleZ);  // Channel 3 for Z (Yaw)

  /* Optionally read and forward SBUS data from receiver */
  // if (sbus_rx.Read()) {
  //   /* Grab the received data */
  //   data = sbus_rx.data();
  //   /* Display the received data */
  //   for (int8_t i = 0; i < data.NUM_CH; i++) {
  //     Serial.print(data.ch[i]);
  //     Serial.print("\t");
  //   }
  //   /* Display lost frames and failsafe data */
  //   Serial.print(data.lost_frame);
  //   Serial.print("\t");
  //   Serial.println(data.failsafe);
  // }

  /* Write the IMU data to the servos via SBUS */
  sbus_tx.data(data); // Update SBUS data with angles
  sbus_tx.Write();    // Write to servos

  delay(10);  // Small delay for sensor updates
}