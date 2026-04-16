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
 
#ifndef ICM566xx_H
#define ICM566xx_H

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
#include "invn/platform_define.h"

#if (INV_IMU_WHOAMI == 0x08)
#define ICM56686
#elif (INV_IMU_WHOAMI == 0xD3)
#define ICM56622
#endif

extern "C" {
#include "imu/inv_imu_edmp.h"
#include "invn/InvError.h"
#include "imu/inv_imu_driver_advanced.h"
#if defined(ICM56686)
#include "imu/inv_imu_edmp_apex.h"
#elif defined(ICM56622)
#include "imu/inv_imu_edmp_lowg.h"
#endif
}

enum {
  ICM566XX_APEX_TILT=0,
  ICM566XX_APEX_PEDOMETER,
  ICM566XX_APEX_R2W,
  ICM566XX_APEX_TAP,
  ICM566XX_APEX_FF,
  ICM566XX_APEX_SMD,
  ICM566XX_APEX_LOWG,
  ICM566XX_APEX_HIGHG,
  ICM566XX_APEX_B2S,
  ICM566XX_APEX_SHAKE,
  ICM566XX_APEX_NOMOTION,
  ICM566XX_APEX_FLAT,
  ICM566XX_APEX_MAX
};

enum {
  APEX_ERROR_EVENT        = -1,
  APEX_NO_EVENT           = 0,
  APEX_EVENT_DETECTED     = 1,
  APEX_B2S_DETECTED       = 1,
  APEX_B2S_REV_DETECTED   = 2,
  APEX_R2W_DETECTED       = 1,
  APEX_R2W_SLEEP_DETECTED = 2,
  APEX_MOTION_DETECTED    = 1,
  APEX_NOMOTION_DETECTED  = 2,
  APEX_FLAT_DETECTED      = 1,
  APEX_NOFLAT_DETECTED    = 2
};

enum {
	STEP_UNKNOWN = 0,
	STEP_WALK    = 1,
	STEP_RUN     = 2,
};


// This defines the handler called when retrieving a sample from the FIFO
// typedef void (*ICM566xx_sensor_event_cb)(inv_imu_sensor_data_t *event);
// This defines the handler called when receiving an irq
typedef void (*ICM566xx_irq_handler)(void);

class ICM566xx {
  public:
    ICM566xx(TwoWire &i2c,bool address_lsb, uint32_t freq);
    ICM566xx(TwoWire &i2c,bool address_lsb);
    ICM566xx(SPIClass &spi,uint8_t chip_select_id, uint32_t freq);
    ICM566xx(SPIClass &spi,uint8_t chip_select_id);
    int begin();
    int startAccel(uint16_t odr, uint16_t fsr);
    int startGyro(uint16_t odr, uint16_t fsr);
    int getDataFromRegisters(inv_imu_sensor_data_t& data);
    int enableFifoInterrupt(uint8_t intpin, bool accel, bool gyro, ICM566xx_irq_handler handler, uint8_t fifo_watermark);
    int getDataFromFifo(inv_imu_fifo_data_t& data);
    int stopAccel(void);
    int stopGyro(void);

    int startAllApex(uint8_t intpin, ICM566xx_irq_handler handler);
    int startApex(int apex_type, uint8_t intpin, ICM566xx_irq_handler handler);
    int getApex_data(int apex_type, uint32_t& p1, uint32_t& p2, uint32_t& p3);
    int getApex_data(int apex_type)
    {
      uint32_t dummy1, dummy2, dummy3;
      if(apex_type != ICM566XX_APEX_TAP && apex_type != ICM566XX_APEX_FF)
        return getApex_data(apex_type, dummy1, dummy2, dummy3);
      else
        return -1;
    }
    int getApex_data(int apex_type, uint32_t& p1)
    {
      uint32_t dummy1, dummy2;
      if(apex_type != ICM566XX_APEX_TAP)
        return getApex_data(apex_type, p1, dummy1, dummy2);
      else
        return -1;
    }

    int startWakeOnMotion(uint8_t intpin, ICM566xx_irq_handler handler);
    int getWom(bool& x, bool& y, bool& z);

    int updateApex(void);
    int setApexInterrupt(uint8_t intpin, ICM566xx_irq_handler handler);
    int readWhoami(void);
    int configure_and_enable_edmp_algo(void);

    inv_imu_edmp_int_state_t apex_status;
    inv_imu_int_state_t 	 int_status;

  protected:
    inv_imu_device_t icm_driver;
    accel_config0_accel_odr_t accel_freq_to_param(uint16_t accel_freq_hz);
    gyro_config0_gyro_odr_t gyro_freq_to_param(uint16_t gyro_freq_hz);
    accel_config0_ap_accel_fs_sel_t accel_fsr_g_to_param(uint16_t accel_fsr_g);
    gyro_config0_ap_gyro_fs_sel_t gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps);

    int setup_irq(uint8_t intpin, ICM566xx_irq_handler handler);
    bool apex_enable[ICM566XX_APEX_MAX];
    accel_config0_accel_odr_t apex_accel_odr;
    uint64_t current_odr_us;
    uint32_t step_cnt_ovflw;
};

#endif // ICM566xx_H
