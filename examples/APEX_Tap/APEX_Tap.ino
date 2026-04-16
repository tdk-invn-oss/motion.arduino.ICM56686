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
#include "ICM566xx.h"

// Instantiate an ICM566xx with LSB address set to 0
ICM566xx IMU(Wire,0);

const char* axis_str[3] = {"X", "Y", "Z"}; 
const char* direction_str[2] = {"+", "-"}; 
volatile bool irq_received = false;

void irq_handler(void) {
  irq_received = true;
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
  
  // APEX Tap enabled, irq on pin 2
  IMU.startApex(ICM566XX_APEX_TAP,2,irq_handler);
}

void loop() {
  if(irq_received)
  {
    uint32_t tap_count=0;
    uint32_t axis=0;
    uint32_t direction=0;
    irq_received = false;

    IMU.updateApex();

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

