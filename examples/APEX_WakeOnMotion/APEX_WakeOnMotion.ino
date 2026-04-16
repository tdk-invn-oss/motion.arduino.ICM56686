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
  
  // APEX WoM enabled, irq on pin 2
  IMU.startWakeOnMotion(2,irq_handler);
  Serial.println("Going to sleep");
}

void loop() {
  if(irq_received)
  {
    bool x,y,z;
    irq_received = false;

    IMU.updateApex();

    if(IMU.getWom(x, y, z) == APEX_EVENT_DETECTED)
    {
      Serial.print("WOM Detected! ");
      Serial.print(x);
      Serial.print(",");
      Serial.print(y);
      Serial.print(",");
      Serial.println(z);
    }    
  }
}
