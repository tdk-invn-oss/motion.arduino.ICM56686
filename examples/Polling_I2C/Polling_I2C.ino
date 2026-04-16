/*
 *
 * Copyright (c) [2026] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
 
#include "ICM566xx.h"

// Instantiate an ICM566xx with LSB address set to 0
ICM566xx IMU(Wire,0);

/* For 20-bit parts the Accel FSR is always 32g and the gyro FSR is always 4000 dps */
#if INV_IMU_20BIT_REG_DATA_SUPPORTED == 1
#define ACCEL_FSR_G    32				/* The accel FSR is 32g */
#define GYRO_FSR_DPS   4000			/* The gyro FSR is 4000 dps */
#define MAX_LSB        524288
#else
/* For 16-bit parts the default Accel FSR is 16g and the default gyro FSR is 2000 dps *
 * These FSRs can be changed by editing the values below and the parameters to        *
 * the driver routines inv_imu_set_accel_fsr and inv_imu_set_gyro_fsr                 */
#define ACCEL_FSR_G    16				/* The accel FSR is 16g */
#define GYRO_FSR_DPS   2000			/* The gyro FSR is 2000 dps */
#define MAX_LSB        32768
#endif

void setup() {
  int ret;
  Serial.begin(115200);
  while(!Serial) {}

  // Initializing the ICM566xx
  ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM566xx initialization failed: ");
    Serial.println(ret);
    while(1);
  }

#if INV_IMU_20BIT_REG_DATA_SUPPORTED == 0 // For 16-bit parts, we can change the FSR
  IMU.startAccel(100,ACCEL_FSR_G);
  IMU.startGyro(100,GYRO_FSR_DPS);
#else
  IMU.startAccel(100,0);
  IMU.startGyro(100,0);
#endif
  // Wait IMU to start
  delay(100);
}

void loop() {

  inv_imu_sensor_data_t imu_data;
  float accel_g[3] = { 0 };
  float gyro_dps[3] = { 0 };
  float temp_degc;
  
  // Read registers
  IMU.getDataFromRegisters(imu_data);

  // Format data for Serial Plotter
  accel_g[0]  = (float)(imu_data.accel_data[0] * ACCEL_FSR_G) / MAX_LSB;
  accel_g[1]  = (float)(imu_data.accel_data[1] * ACCEL_FSR_G) / MAX_LSB;
  accel_g[2]  = (float)(imu_data.accel_data[2] * ACCEL_FSR_G) / MAX_LSB;
	      
  Serial.print("AccelX:");
  Serial.print(accel_g[0]);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(accel_g[1]);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(accel_g[2]);
  Serial.print(","); 

  gyro_dps[0]  = (float)(imu_data.gyro_data[0] * GYRO_FSR_DPS) / MAX_LSB;
  gyro_dps[1]  = (float)(imu_data.gyro_data[1] * GYRO_FSR_DPS) / MAX_LSB;
  gyro_dps[2]  = (float)(imu_data.gyro_data[2] * GYRO_FSR_DPS) / MAX_LSB;

  Serial.print("GyroX:");
  Serial.print(gyro_dps[0]);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(gyro_dps[1]);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(gyro_dps[2]);
  Serial.print(",");

  temp_degc = 25 + ((float)imu_data.temp_data/128);
  Serial.print("Temperature:");        
  Serial.println(temp_degc);

  // Run @ ODR 100Hz
  delay(10);
}
