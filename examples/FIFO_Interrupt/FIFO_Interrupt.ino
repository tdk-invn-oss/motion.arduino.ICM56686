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

volatile uint8_t irq_received = 0;

void irq_handler(void) {
  irq_received = 1;
}

#define ACCEL_FSR_G  4
#define GYRO_FSR_DPS 2000
#define MAX_LSB      32768

static int accel_en  = 1;
static int gyro_en   = 1;

void setup() {
  int ret;
  Serial.begin(115200);
  while(!Serial) {}

  // Initializing the ICM5x6xx
  ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM5x6xx initialization failed: ");
    Serial.println(ret);
    while(1);
  }

  accel_en = 1;
  gyro_en = 1;

  // Enable interrupt on pin 2, Fifo watermark=10
  IMU.enableFifoInterrupt(2,accel_en,gyro_en,irq_handler,10);

  if(accel_en)
    IMU.startAccel(50, ACCEL_FSR_G);
  if(gyro_en)
    IMU.startGyro(50, GYRO_FSR_DPS);
}

void loop() {
  // Wait for interrupt to read data from fifo
  if(irq_received) {
    irq_received = 0;
    for(int i = 0; i < 10 ; i ++)
    {
      inv_imu_fifo_data_t imu_data;
      int accel_raw[3]   = { 0 };
      int gyro_raw[3]    = { 0 };
      int temp_raw       = 0;
      
      IMU.getDataFromFifo(imu_data);
      // Format data for Serial Plotter

      if(accel_en && gyro_en)
      {
        accel_raw[0] = imu_data.byte_16.accel_data[0];
        accel_raw[1] = imu_data.byte_16.accel_data[1];
        accel_raw[2] = imu_data.byte_16.accel_data[2];

        gyro_raw[0] = imu_data.byte_16.gyro_data[0];
        gyro_raw[1] = imu_data.byte_16.gyro_data[1];
        gyro_raw[2] = imu_data.byte_16.gyro_data[2];

        temp_raw = (int16_t)imu_data.byte_16.temp_data;
      } else if(accel_en) {
        accel_raw[0] = imu_data.byte_8.sensor_data[0];
        accel_raw[1] = imu_data.byte_8.sensor_data[1];
        accel_raw[2] = imu_data.byte_8.sensor_data[2];

        temp_raw = (int16_t)imu_data.byte_8.temp_data;
      } else {
        gyro_raw[0] = imu_data.byte_8.sensor_data[0];
        gyro_raw[1] = imu_data.byte_8.sensor_data[1];
        gyro_raw[2] = imu_data.byte_8.sensor_data[2];

        temp_raw = (int16_t)imu_data.byte_8.temp_data;
      }

      if(accel_en)
      {
        float accel_g[3] = { 0 };
        accel_g[0]  = (float)(accel_raw[0] * ACCEL_FSR_G) / MAX_LSB;
        accel_g[1]  = (float)(accel_raw[1] * ACCEL_FSR_G) / MAX_LSB;
        accel_g[2]  = (float)(accel_raw[2] * ACCEL_FSR_G) / MAX_LSB;
	      
        Serial.print("AccelX:");
        Serial.print(accel_g[0]);
        Serial.print(",");
        Serial.print("AccelY:");
        Serial.print(accel_g[1]);
        Serial.print(",");
        Serial.print("AccelZ:");
        Serial.print(accel_g[2]);
        Serial.print(" ");
      }
      
      if(gyro_en)
      {
        float gyro_dps[3] = { 0 };
        gyro_dps[0]  = (float)(gyro_raw[0] * GYRO_FSR_DPS) / MAX_LSB;
        gyro_dps[1]  = (float)(gyro_raw[1] * GYRO_FSR_DPS) / MAX_LSB;
        gyro_dps[2]  = (float)(gyro_raw[2] * GYRO_FSR_DPS) / MAX_LSB;

        Serial.print("GyroX:");
        Serial.print(gyro_dps[0]);
        Serial.print(",");
        Serial.print("GyroY:");
        Serial.print(gyro_dps[1]);
        Serial.print(",");
        Serial.print("GyroZ:");
        Serial.print(gyro_dps[2]);
      }

      if (temp_raw != INVALID_VALUE_FIFO_1B)
      {
        float temp_degc = 25 + ((float)temp_raw/2);
        Serial.print(" ");
        Serial.print("Temperature:");        
        Serial.println(temp_degc);
      }
    }
  }
}


