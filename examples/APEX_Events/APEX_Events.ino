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

const char* axis_str[3] = {"X", "Y", "Z"}; 
const char* direction_str[2] = {"+", "-"}; 
volatile uint8_t irq_received = 0;

void irq_handler(void) {
  irq_received = 1;
}

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

  // ICM56686 - Tap, Tilt, Pedometer, SMD, R2W, Motion, Flat, Shake, FF, LowG, HighG, B2S
  // ICM56622 - LowG
  IMU.startAllApex(2,irq_handler);
}

void loop() {
  // Wait for interrupt to read data Pedometer status
  if(irq_received) {
    irq_received = 0;
    int ret = 0;
    uint32_t tap_count=0;
    uint32_t axis=0;
    uint32_t direction=0;
    uint32_t activity=0;
    uint32_t step_count=0;
    uint32_t step_cadence=0;

    IMU.updateApex();
    
    if(IMU.getApex_data(ICM566XX_APEX_PEDOMETER,step_count,step_cadence,activity) == APEX_EVENT_DETECTED)
    {
      Serial.print(" Step count:");
      Serial.print(step_count);
      Serial.print(",");
      Serial.print("Step cadence(steps/s):");
      Serial.print((float)step_cadence/100);
      Serial.print(",");
      Serial.print("activity:");
      if(activity == STEP_WALK)
        Serial.print("Walk");
      else if(activity == STEP_RUN)
        Serial.print("Run");
      else if(activity == STEP_UNKNOWN)
        Serial.print("Unknown");
      Serial.println("");
    }

    if(IMU.getApex_data(ICM566XX_APEX_SMD) == APEX_EVENT_DETECTED)
      Serial.println("Significant motion Event");

    if(IMU.getApex_data(ICM566XX_APEX_TILT) == APEX_EVENT_DETECTED)
      Serial.println("Tilt Event");

    ret = IMU.getApex_data(ICM566XX_APEX_R2W);
    if(ret == APEX_R2W_DETECTED)
      Serial.println("R2W wake Event");
    else if(ret == APEX_R2W_SLEEP_DETECTED)
      Serial.println("R2W sleep Event");

    ret = IMU.getApex_data(ICM566XX_APEX_B2S);
    if(ret == APEX_B2S_DETECTED)
      Serial.println("B2S Event");
    else if(ret == APEX_B2S_REV_DETECTED)
      Serial.println("B2S reverse Event");

    if(IMU.getApex_data(ICM566XX_APEX_SHAKE) == APEX_EVENT_DETECTED)
      Serial.println("Shake Event");

    ret = IMU.getApex_data(ICM566XX_APEX_NOMOTION);
    if(ret == APEX_MOTION_DETECTED)
      Serial.println("Motion Event");
    else if(ret == APEX_NOMOTION_DETECTED)
      Serial.println("No Motion Event");

    ret = IMU.getApex_data(ICM566XX_APEX_FLAT);
    if(ret == APEX_FLAT_DETECTED)
      Serial.println("Flat Event");
    else if(ret == APEX_NOFLAT_DETECTED)
      Serial.println("No Flat Event");

    uint32_t duration_ms;  
    if(IMU.getApex_data(ICM566XX_APEX_FF, duration_ms) == APEX_EVENT_DETECTED)
    {
      Serial.print("FreeFall Event duration(ms):");
      Serial.println(duration_ms);
    }

    if(IMU.getApex_data(ICM566XX_APEX_HIGHG) == APEX_EVENT_DETECTED)
      Serial.println("HighG Event");

    if(IMU.getApex_data(ICM566XX_APEX_LOWG) == APEX_EVENT_DETECTED)
      Serial.println("LowG Event");

    if(IMU.getApex_data(ICM566XX_APEX_TAP,tap_count,axis,direction) == APEX_EVENT_DETECTED)
    {
      Serial.print("Tap count:");
      Serial.print(tap_count);
      Serial.print(",");
      Serial.print("Axis:");
      Serial.print(axis_str[axis]);
      Serial.print(",");
      Serial.print("Direction:");
      Serial.println(direction_str[direction]);
    }
  }
}

