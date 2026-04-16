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
 
#include "Arduino.h"
#include "ICM566xx.h"

static int i2c_write(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int i2c_read(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static int spi_write(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int spi_read(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static void sleep_us(uint32_t us);

// i2c and SPI interfaces are used from C driver callbacks, without any knowledge of the object
// As they are declared as static, they will be overriden each time a new ICM566xx object is created
// i2c
uint8_t i2c_address = 0;
static TwoWire *i2c = NULL;
#define I2C_DEFAULT_CLOCK 400000
#define I2C_MAX_CLOCK 1000000
#define ICM566xx_I2C_ADDRESS 0x68
#define ARDUINO_I2C_BUFFER_LENGTH 32
// spi
static SPIClass *spi = NULL;
static uint8_t chip_select_id = 0;
#define SPI_READ 0x80
#define SPI_DEFAULT_CLOCK 12000000
#define SPI_MAX_CLOCK 24000000

// i2c/spi clock frequency
static uint32_t clk_freq = 0;

/* Initial WOM threshold to be applied to IMU in mg */
#define WOM_THRESHOLD_INITIAL_MG 200
/* WOM threshold to be applied to IMU in mg if low threshold is requested*/
#define WOM_THRESHOLD_LOW_MG 24

// ICM566xx constructor for I2c interface
ICM566xx::ICM566xx(TwoWire &i2c_ref,bool lsb, uint32_t freq) {
  i2c = &i2c_ref; 
  i2c_address = ICM566xx_I2C_ADDRESS | (lsb ? 0x1 : 0);
  if ((freq <= I2C_MAX_CLOCK) && (freq >= 100000))
  {
    clk_freq = freq;
  } else {
    clk_freq = I2C_DEFAULT_CLOCK;
  }
}

// ICM566xx constructor for I2c interface, default frequency
ICM566xx::ICM566xx(TwoWire &i2c_ref,bool lsb) {
  i2c = &i2c_ref; 
  i2c_address = ICM566xx_I2C_ADDRESS | (lsb ? 0x1 : 0);
  clk_freq = I2C_DEFAULT_CLOCK;
}

// ICM566xx constructor for spi interface
ICM566xx::ICM566xx(SPIClass &spi_ref,uint8_t cs_id, uint32_t freq) {
  spi = &spi_ref;
  chip_select_id = cs_id; 
  if ((freq <= SPI_MAX_CLOCK) && (freq >= 100000))
  {
    clk_freq = freq;
  } else {
    clk_freq = SPI_DEFAULT_CLOCK;
  }
}

// ICM566xx constructor for spi interface, default frequency
ICM566xx::ICM566xx(SPIClass &spi_ref,uint8_t cs_id) {
  spi = &spi_ref;
  chip_select_id = cs_id; 
  clk_freq = SPI_DEFAULT_CLOCK;
}

/* starts communication with the ICM566xx */
int ICM566xx::begin() {
  int rc = 0;
  uint8_t who_am_i;

  if (i2c != NULL) {
    i2c->begin();
    i2c->setClock(clk_freq);
    icm_driver.transport.serif_type = UI_I2C;
    icm_driver.transport.read_reg  = i2c_read;
    icm_driver.transport.write_reg = i2c_write;
  } else {
    spi->begin();
    pinMode(chip_select_id,OUTPUT);
    digitalWrite(chip_select_id,HIGH);
    icm_driver.transport.serif_type = UI_SPI4;
    icm_driver.transport.read_reg  = spi_read;
    icm_driver.transport.write_reg = spi_write;
  }
  icm_driver.transport.sleep_us = sleep_us;

  /* Disable APEX features */
  memset(apex_enable, 0x00, ICM566XX_APEX_MAX);

  /* Set ODR to the minimum, APEX init will select the highest required ODR */
  apex_accel_odr = ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ;

  sleep_us(3000);

  return inv_imu_adv_init(&icm_driver);
}

int ICM566xx::startAccel(uint16_t odr, uint16_t fsr) {
  int rc = 0;

  if(fsr)
    rc |= inv_imu_set_accel_fsr(&icm_driver, accel_fsr_g_to_param(fsr));
  rc |= inv_imu_set_accel_frequency(&icm_driver, accel_freq_to_param(odr));
  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LN);
  return rc;
}

int ICM566xx::startGyro(uint16_t odr, uint16_t fsr) {
  int rc = 0;

  if(fsr)
    rc |= inv_imu_set_gyro_fsr(&icm_driver, gyro_fsr_dps_to_param(fsr));
  rc |= inv_imu_set_gyro_frequency(&icm_driver, gyro_freq_to_param(odr));
  rc |= inv_imu_set_gyro_mode(&icm_driver, PWR_MGMT0_GYRO_MODE_LN);
  return rc;
}

int ICM566xx::stopAccel(void) {
  return inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_OFF);
}

int ICM566xx::stopGyro(void) {
  return inv_imu_set_gyro_mode(&icm_driver, PWR_MGMT0_GYRO_MODE_OFF);
}

int ICM566xx::getDataFromRegisters(inv_imu_sensor_data_t& data) {
    return inv_imu_get_register_data(&icm_driver, &data);
}

int ICM566xx::readWhoami(void) {
	uint8_t data;
    inv_imu_get_who_am_i(&icm_driver, &data);

	return data;
}

int ICM566xx::setup_irq(uint8_t intpin, ICM566xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_int_state_t it_conf;
  const inv_imu_int_pin_config_t it_pins = {
    .int_polarity=INTX_CONFIG2_INTX_POLARITY_HIGH,
    .int_mode=INTX_CONFIG2_INTX_MODE_PULSE,
    .int_drive=INTX_CONFIG2_INTX_DRIVE_PP
  };
  memset(&it_conf, INV_IMU_DISABLE, sizeof(it_conf));
  it_conf.INV_FIFO_THS = INV_IMU_ENABLE;
  pinMode(intpin,INPUT);
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &it_pins);
  rc |= inv_imu_set_config_int(&icm_driver, INV_IMU_INT1, &it_conf);
  attachInterrupt(intpin,handler,HIGH);
  return rc;
}

int ICM566xx::enableFifoInterrupt(uint8_t intpin, bool accel, bool gyro, ICM566xx_irq_handler handler, uint8_t fifo_watermark) {
  int rc = 0;
  inv_imu_int_state_t it_conf;
  const inv_imu_fifo_config_t fifo_config = {
    .gyro_en=gyro,
    .accel_en=accel,
    .hires_en=false,
    .fifo_wm_th=fifo_watermark,
    .fifo_mode=FIFO_CONFIG0_FIFO_MODE_SNAPSHOT,
    .fifo_depth=FIFO_CONFIG0_FIFO_DEPTH_MAX
  };
  if(handler == NULL) {
    return -1;
  }

  rc |= inv_imu_set_fifo_config(&icm_driver, &fifo_config);

  rc |= setup_irq(intpin, handler);
  return rc;
}


int ICM566xx::getDataFromFifo(inv_imu_fifo_data_t& data) {
  return inv_imu_get_fifo_frame(&icm_driver,&data);
}

int ICM566xx::setApexInterrupt(uint8_t intpin, ICM566xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_int_state_t config_int;
  inv_imu_int_pin_config_t int_pin_config;
  inv_imu_edmp_int_state_t apex_int_config;

  if (handler == NULL)
    return 0;
  
  pinMode(intpin,INPUT);
  attachInterrupt(intpin,handler,RISING);
  
  /*
   * Configure interrupts pins
   * - Polarity High
   * - Pulse mode
   * - Push-Pull drive
   */
  int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
  int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
  int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &int_pin_config);
  
  /* Interrupts configuration: Enable only EDMP interrupt */
  memset(&config_int, INV_IMU_DISABLE, sizeof(config_int));
  config_int.INV_EDMP_EVENT = INV_IMU_ENABLE;
  inv_imu_set_config_int(&icm_driver,INV_IMU_INT1, &config_int);

  return rc;
}

int ICM566xx::startAllApex(uint8_t intpin, ICM566xx_irq_handler handler)
{
  int rc = 0;

#if !defined(ICM56622)
  apex_enable[ICM566XX_APEX_TILT]      = true;
  apex_enable[ICM566XX_APEX_PEDOMETER] = true;
  apex_enable[ICM566XX_APEX_SMD]       = true;
  apex_enable[ICM566XX_APEX_R2W]       = true;
  apex_enable[ICM566XX_APEX_NOMOTION]  = true;
  apex_enable[ICM566XX_APEX_FLAT]      = true;
  apex_enable[ICM566XX_APEX_SHAKE]     = true;
  apex_enable[ICM566XX_APEX_TAP]       = true; 
  apex_enable[ICM566XX_APEX_FF]        = true;
  apex_enable[ICM566XX_APEX_HIGHG]     = true;
  apex_enable[ICM566XX_APEX_B2S]       = true;
#endif
  apex_enable[ICM566XX_APEX_LOWG]      = true;

  rc |= setApexInterrupt(intpin, handler);
  rc |= configure_and_enable_edmp_algo();
  
  return rc;
}

int ICM566xx::startApex(int apex_type, uint8_t intpin, ICM566xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[apex_type] = true;

  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= configure_and_enable_edmp_algo();
  }
  
  return rc;
}

int ICM566xx::startWakeOnMotion(uint8_t intpin, ICM566xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_int_pin_config_t int_pin_config;
  inv_imu_int_state_t int_config;
  int wom_high_thr_en = 1; /* Indicates WOM state */
  
  /* WOM threshold to be applied to IMU, ranges from 1 to 255, in 4mg unit */
  uint8_t wom_threshold_high = WOM_THRESHOLD_INITIAL_MG / 4;
  uint8_t wom_threshold_low	= WOM_THRESHOLD_LOW_MG / 4;

  pinMode(intpin,INPUT);
  attachInterrupt(intpin,handler,RISING);

  int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
  int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
  int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &int_pin_config);  

  /* Interrupts configuration: Enable only WOM interrupts */
  memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
  int_config.INV_WOM_X = INV_IMU_ENABLE;
  int_config.INV_WOM_Y = INV_IMU_ENABLE;
  int_config.INV_WOM_Z = INV_IMU_ENABLE;
  rc |= inv_imu_set_config_int(&icm_driver, INV_IMU_INT1, &int_config);

  /* Configure WOM to produce signal when at least one axis exceed wom_threshold_high/low */
  rc |= inv_imu_adv_configure_wom(&icm_driver,
                                  wom_high_thr_en ? wom_threshold_high : wom_threshold_low,
                                  wom_high_thr_en ? wom_threshold_high : wom_threshold_low,
	                              wom_high_thr_en ? wom_threshold_high : wom_threshold_low,
	                              TMST_WOM_CONFIG_WOM_INT_MODE_ORED,
	                              TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);
  rc |= inv_imu_adv_enable_wom(&icm_driver);
  rc |= inv_imu_set_accel_frequency(&icm_driver, ACCEL_CONFIG0_ACCEL_ODR_12_5_HZ);
  /* Select WUOSC clock to have accel in ULP (lowest power mode) */
  rc |= inv_imu_select_accel_lp_clk(&icm_driver, PWR_MGMT_0_ACCEL_LP_CLK_WUOSC);
  /* Set 1x averaging, in order to minimize power consumption */
  rc |= inv_imu_set_accel_lp_avg(&icm_driver, IPREG_SYS2_REG_110_ACCEL_LP_AVG_1);
  rc |= inv_imu_adv_enable_accel_lp(&icm_driver);
  
  return rc;
}

int ICM566xx::getWom(bool& x, bool& y, bool& z)
{
  if (int_status.INV_WOM_X || int_status.INV_WOM_Y || int_status.INV_WOM_Z) {
    x = int_status.INV_WOM_X;
	y = int_status.INV_WOM_Y;
	z = int_status.INV_WOM_Z;
	int_status.INV_WOM_X = int_status.INV_WOM_Y = int_status.INV_WOM_Z = 0;
    return APEX_EVENT_DETECTED;
  } else {
    return APEX_NO_EVENT;
  }
}

int ICM566xx::getApex_data(int apex_type, uint32_t& p1, uint32_t& p2, uint32_t& p3)
{
  int ret = APEX_NO_EVENT;
  
  if(apex_enable[apex_type] == false)
    return INV_ERROR;

  switch(apex_type)
  {
#if defined(ICM56686)
    case ICM566XX_APEX_PEDOMETER:
      if (apex_status.INV_STEP_CNT_OVFL) {
          apex_status.INV_STEP_CNT_OVFL = 0;
          step_cnt_ovflw++;
      }
	  if (apex_status.INV_STEP_DET){
        inv_imu_edmp_pedometer_data_t ped_data;
		int rc =0;

		apex_status.INV_STEP_DET = 0;
		uint32_t ped_odr_us = current_odr_us > 20000 ? current_odr_us : 20000;
		rc = inv_imu_edmp_get_pedometer_data(&icm_driver, &ped_data);
        if (rc == INV_IMU_OK) {
          p1 = (uint32_t)ped_data.step_cnt + (step_cnt_ovflw * UINT16_MAX);  
          if(ped_data.step_cadence != 0)
          {
            p2 = (200/(float)ped_data.step_cadence)*100;
          } else {
            p2 = 0;
          }
		  p3 = ped_data.activity_class;
        }
		ret = APEX_EVENT_DETECTED;
	  }
	  break;
    case ICM566XX_APEX_TILT:
	  if (apex_status.INV_TILT_DET){
        apex_status.INV_TILT_DET = 0;
        ret = APEX_EVENT_DETECTED;
      }
	  break;
	case ICM566XX_APEX_TAP:
	  if (apex_status.INV_TAP) {
        inv_imu_edmp_tap_data_t tap_data;
        apex_status.INV_TAP = 0;
        inv_imu_edmp_get_tap_data(&icm_driver, &tap_data);
        p1 = tap_data.num;
        p2 = tap_data.axis;
        p3 = tap_data.direction;
        ret = APEX_EVENT_DETECTED;
      }
	  break;
	case ICM566XX_APEX_FF:
	  if (apex_status.INV_FF) {
    	uint16_t duration;
        apex_status.INV_FF = 0;
        inv_imu_edmp_get_ff_data(&icm_driver, &duration);
    	p1 = (duration * 2500) / 1000;
        ret = APEX_EVENT_DETECTED;
      }
	  break;
	case ICM566XX_APEX_HIGHG:
	  if (apex_status.INV_HIGHG) {
        apex_status.INV_HIGHG = 0;
        ret = APEX_EVENT_DETECTED;
      }
      break;
	case ICM566XX_APEX_B2S:
	  if (apex_status.INV_B2S) {
        apex_status.INV_B2S = 0;  	
        ret = APEX_B2S_DETECTED;
      } else if (apex_status.INV_B2S_REV) {
        apex_status.INV_B2S_REV = 0;  	
        ret = APEX_B2S_REV_DETECTED;
      }
	  break;
    case ICM566XX_APEX_SMD:
      if (apex_status.INV_SMD) {
        apex_status.INV_SMD = 0;	  
        ret = APEX_EVENT_DETECTED;
      }
      break;
    case ICM566XX_APEX_R2W:
      if (apex_status.INV_R2W) {
        apex_status.INV_R2W = 0;
        ret = APEX_R2W_DETECTED;
      } else if (apex_status.INV_R2W_SLEEP) {
        apex_status.INV_R2W_SLEEP = 0; 	
        ret = APEX_R2W_SLEEP_DETECTED;
      }
      break;
    case ICM566XX_APEX_SHAKE:
      if (apex_status.INV_SHAKE) {
        apex_status.INV_SHAKE = 0;		
        ret = APEX_EVENT_DETECTED;
      }
      break;
    case ICM566XX_APEX_NOMOTION:
      if (apex_status.INV_MOTION) {
        apex_status.INV_MOTION = 0;	  
        ret = APEX_MOTION_DETECTED;
      } else if (apex_status.INV_NO_MOTION) {
        apex_status.INV_NO_MOTION = 0;	  
        ret = APEX_NOMOTION_DETECTED;
      }
      break;
    case ICM566XX_APEX_FLAT:
      if (apex_status.INV_FLAT) {
        apex_status.INV_FLAT = 0;	  
        ret = APEX_FLAT_DETECTED;
      } else if (apex_status.INV_NO_FLAT) {
        apex_status.INV_NO_FLAT = 0;	  
        ret = APEX_NOFLAT_DETECTED;
      }
      break;
#endif
    case ICM566XX_APEX_LOWG:
      if (apex_status.INV_LOWG) {
        apex_status.INV_LOWG = 0;   
        ret = APEX_EVENT_DETECTED;
      }
      break;
    default:
      ret = APEX_ERROR_EVENT;
  }

  return ret;
}

int ICM566xx::configure_and_enable_edmp_algo(void)
{
  int rc = 0;
  accel_config0_accel_odr_t           accel_odr;
  inv_imu_edmp_apex_parameters_t apex_parameters;
  inv_imu_edmp_int_state_t		 apex_int_config;
  dmp_ext_sen_odr_cfg_apex_odr_t dmp_odr;

  /* Configure ODR depending on which feature is enabled */
  if (apex_enable[ICM566XX_APEX_FF] || apex_enable[ICM566XX_APEX_LOWG] || apex_enable[ICM566XX_APEX_HIGHG] || apex_enable[ICM566XX_APEX_TAP]) {
  /* 800 Hz */
    current_odr_us    = 1250;
    dmp_odr   = DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ;
    accel_odr = ACCEL_CONFIG0_ACCEL_ODR_800_HZ;
  } else if (apex_enable[ICM566XX_APEX_R2W]) {
  /* 400 Hz */
    current_odr_us    = 2500;
    dmp_odr   = DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ;
    accel_odr = ACCEL_CONFIG0_ACCEL_ODR_400_HZ;
  } else {
  /* 50 Hz */
    current_odr_us    = 20000;
    dmp_odr	  = DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ;
    accel_odr = ACCEL_CONFIG0_ACCEL_ODR_50_HZ;
  }

  rc |= inv_imu_edmp_init_apex(&icm_driver);

  /* Set EDMP ODR */
  rc |= inv_imu_edmp_set_frequency(&icm_driver, dmp_odr);

  /* Set ODR */
  rc |= inv_imu_set_accel_frequency(&icm_driver, accel_odr);

  /* Set BW = ODR/4 */
  rc |= inv_imu_set_accel_ln_bw(&icm_driver, IPREG_SYS2_REG_112_ACCEL_UI_LPFBW_DIV_4);

  /* Select WUOSC clock to have accel in ULP (lowest power mode) */
  rc |= inv_imu_select_accel_lp_clk(&icm_driver, PWR_MGMT_0_ACCEL_LP_CLK_WUOSC);

  /* Set AVG to 1x */
  rc |= inv_imu_set_accel_lp_avg(&icm_driver, IPREG_SYS2_REG_110_ACCEL_LP_AVG_1);

#if defined(ICM56686)
  /* Ensure all DMP features are disabled before running init procedure */
  rc |= inv_imu_edmp_disable_pedometer(&icm_driver);
  rc |= inv_imu_edmp_disable_smd(&icm_driver);
  rc |= inv_imu_edmp_disable_tilt(&icm_driver);
  rc |= inv_imu_edmp_disable_r2w(&icm_driver);
  rc |= inv_imu_edmp_disable_tap(&icm_driver);
  rc |= inv_imu_edmp_disable_ff(&icm_driver);
  rc |= inv_imu_edmp_disable_b2s(&icm_driver);
  rc |= inv_imu_edmp_disable_noMotion(&icm_driver);
  rc |= inv_imu_edmp_disable_flat(&icm_driver);
  rc |= inv_imu_edmp_disable_shake(&icm_driver);
  rc |= inv_imu_edmp_disable(&icm_driver);


  /* Configure APEX parameters */
  rc |= inv_imu_edmp_get_apex_parameters(&icm_driver, &apex_parameters);
  apex_parameters.r2w_sleep_time_out = 6400; /* 6.4 s */

  apex_parameters.power_save_en = INV_IMU_DISABLE;
  rc |= inv_imu_adv_disable_wom(&icm_driver);
  rc |= inv_imu_edmp_set_apex_parameters(&icm_driver, &apex_parameters);
#endif

  /* Set accel in low-power mode if ODR slower than 800 Hz, otherwise in low-noise mode */
  if (current_odr_us <= 1250 /* 800 Hz and faster */)
    rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LN);
  else
    rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LP);

  /* Wait for accel startup time */
  sleep_us(ACC_STARTUP_TIME_US);

  /* Disable all APEX interrupt and enable only the one we need */
  memset(&apex_int_config, INV_IMU_DISABLE, sizeof(apex_int_config));

#if defined(ICM56686)
  /* Enable requested features */
  if (apex_enable[ICM566XX_APEX_PEDOMETER]) {
    rc |= inv_imu_edmp_enable_pedometer(&icm_driver);
    apex_int_config.INV_STEP_CNT_OVFL = INV_IMU_ENABLE;
    apex_int_config.INV_STEP_DET	  = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM566XX_APEX_SMD]) {
  	rc |= inv_imu_edmp_enable_smd(&icm_driver);
  	apex_int_config.INV_SMD = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM566XX_APEX_TILT]) {
  	rc |= inv_imu_edmp_enable_tilt(&icm_driver);
  	apex_int_config.INV_TILT_DET = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM566XX_APEX_R2W]) {
  	rc |= inv_imu_edmp_enable_r2w(&icm_driver);
  	apex_int_config.INV_R2W 	  = INV_IMU_ENABLE;
  	apex_int_config.INV_R2W_SLEEP = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM566XX_APEX_TAP]) {
  	rc |= inv_imu_edmp_get_apex_parameters(&icm_driver, &apex_parameters);
  	apex_parameters.tap_tmax			 = 800;
  	apex_parameters.tap_tmin			 = 100;
  	apex_parameters.tap_max_tap = 3;
  	apex_parameters.tap_min_tap = 1;
  	apex_parameters.tap_axis_select_mask = 63;
  	apex_parameters.tap_max_energy_primary = 0;
  	apex_parameters.tap_max_energy_secondary = 0;
  	rc |= inv_imu_edmp_set_apex_parameters(&icm_driver, &apex_parameters);
  	rc |= inv_imu_edmp_enable_tap(&icm_driver);
  	apex_int_config.INV_TAP = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM566XX_APEX_FF]) {
  	rc |= inv_imu_edmp_enable_ff(&icm_driver);
  	apex_int_config.INV_FF = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM566XX_APEX_HIGHG]) {
  	rc |= inv_imu_edmp_enable_ff(&icm_driver);
  	apex_int_config.INV_HIGHG = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM566XX_APEX_B2S]) {
  	rc |= inv_imu_edmp_enable_b2s(&icm_driver);
  	apex_int_config.INV_B2S = INV_IMU_ENABLE;
  	apex_int_config.INV_B2S_REV = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM566XX_APEX_SHAKE]) {
  	rc |= inv_imu_edmp_enable_shake(&icm_driver);
  	apex_int_config.INV_SHAKE = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM566XX_APEX_NOMOTION]) {
  	rc |= inv_imu_edmp_enable_noMotion(&icm_driver);
  	apex_int_config.INV_NO_MOTION = INV_IMU_ENABLE;
  	apex_int_config.INV_MOTION = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM566XX_APEX_FLAT]) {
  	rc |= inv_imu_edmp_enable_flat(&icm_driver);
  	apex_int_config.INV_FLAT = INV_IMU_ENABLE;
  	apex_int_config.INV_NO_FLAT = INV_IMU_ENABLE;
  }

  if (apex_enable[ICM566XX_APEX_LOWG]) {
    rc |= inv_imu_edmp_enable_ff(&icm_driver);
    apex_int_config.INV_LOWG = INV_IMU_ENABLE;
  }

  /* Apply interrupt configuration */
  rc |= inv_imu_edmp_set_config_int_apex(&icm_driver, &apex_int_config);
#elif defined(ICM56622)
  if (apex_enable[ICM566XX_APEX_LOWG]) {
  	rc |= inv_imu_edmp_enable_lowg(&icm_driver);
  	apex_int_config.INV_LOWG = INV_IMU_ENABLE;
  }

  /* Apply interrupt configuration */
  rc |= inv_imu_edmp_set_config_int_lowg(&icm_driver, &apex_int_config);  
#endif

  rc |= inv_imu_edmp_enable(&icm_driver);

  return rc;
}

int ICM566xx::updateApex(void)
{
  int rc = 0;
  inv_imu_int_state_t      int_state;
  inv_imu_edmp_int_state_t apex_state = { 0 };

  /* Read interrupt status */
  rc |= inv_imu_get_int_status(&icm_driver, INV_IMU_INT1, &int_state);

  /* If APEX status is set */
  if (int_state.INV_EDMP_EVENT) {
    /* Read APEX interrupt status */
#if defined(ICM56622)
    rc |= inv_imu_edmp_get_int_lowg_status(&icm_driver, &apex_state);
#else
    rc |= inv_imu_edmp_get_int_apex_status(&icm_driver, &apex_state);
#endif

#if defined(ICM56686)
    apex_status.INV_TILT_DET   |= apex_state.INV_TILT_DET;
    apex_status.INV_FF         |= apex_state.INV_FF;
    apex_status.INV_HIGHG      |= apex_state.INV_HIGHG;
    apex_status.INV_TAP        |= apex_state.INV_TAP;
    apex_status.INV_B2S        |= apex_state.INV_B2S;
    apex_status.INV_B2S_REV    |= apex_state.INV_B2S_REV;
    apex_status.INV_SMD        |= apex_state.INV_SMD;
    apex_status.INV_STEP_DET   |= apex_state.INV_STEP_DET;
    apex_status.INV_R2W        |= apex_state.INV_R2W;
    apex_status.INV_R2W_SLEEP  |= apex_state.INV_R2W_SLEEP;
    apex_status.INV_SHAKE      |= apex_state.INV_SHAKE;
    apex_status.INV_NO_MOTION  |= apex_state.INV_NO_MOTION;
    apex_status.INV_MOTION     |= apex_state.INV_MOTION;
    apex_status.INV_NO_FLAT    |= apex_state.INV_NO_FLAT;
    apex_status.INV_FLAT       |= apex_state.INV_FLAT;
#endif
    apex_status.INV_LOWG       |= apex_state.INV_LOWG;
  }
  else if(int_state.INV_WOM_X || int_state.INV_WOM_Y || int_state.INV_WOM_Z){
    int_status.INV_WOM_X |= int_state.INV_WOM_X;
    int_status.INV_WOM_Y |= int_state.INV_WOM_Y;
    int_status.INV_WOM_Z |= int_state.INV_WOM_Z;
  }
  
  return rc;
}

static int i2c_write(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
  i2c->beginTransmission(i2c_address);
  i2c->write(reg);
  for(uint8_t i = 0; i < wlen; i++) {
    i2c->write(wbuffer[i]);
  }
  i2c->endTransmission();
  return 0;
}

static int i2c_read(uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
  uint16_t offset = 0;
  int recnt = 256;

  i2c->beginTransmission(i2c_address);
  i2c->write(reg);
  i2c->endTransmission(false);
  while(offset < rlen && recnt-- > 0)
  {
    uint16_t rx_bytes = 0;
    if(offset != 0)
      i2c->beginTransmission(i2c_address);
    uint8_t length = ((rlen - offset) > ARDUINO_I2C_BUFFER_LENGTH) ? ARDUINO_I2C_BUFFER_LENGTH : (rlen - offset) ;
    rx_bytes = i2c->requestFrom(i2c_address, length);
    if (rx_bytes == length) {
      for(uint8_t i = 0; i < length; i++) {
        rbuffer[offset+i] = i2c->read();
      }
      offset += length;
      i2c->endTransmission((offset == rlen));
    } else {
      i2c->endTransmission((offset == rlen));
    }
  }
  if(offset == rlen)
  {
    return 0;
  } else {
    return -1;
  }
}

static int spi_write(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
  spi->beginTransaction(SPISettings(clk_freq, MSBFIRST, SPI_MODE3));
  digitalWrite(chip_select_id,LOW);
  spi->transfer(reg);
  for(uint8_t i = 0; i < wlen; i++) {
    spi->transfer(wbuffer[i]);
  }
  digitalWrite(chip_select_id,HIGH);
  spi->endTransaction();
  return 0;
}

static int spi_read(uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
  spi->beginTransaction(SPISettings(clk_freq, MSBFIRST, SPI_MODE3));
  digitalWrite(chip_select_id,LOW);
  spi->transfer(reg | SPI_READ);
  spi->transfer(rbuffer,rlen);
  digitalWrite(chip_select_id,HIGH);
  spi->endTransaction();
  return 0;
}

accel_config0_ap_accel_fs_sel_t ICM566xx::accel_fsr_g_to_param(uint16_t accel_fsr_g) {
  accel_config0_ap_accel_fs_sel_t ret = ACCEL_CONFIG0_AP_ACCEL_FS_SEL_16_G;

  switch(accel_fsr_g) {
  case 2:  ret = ACCEL_CONFIG0_AP_ACCEL_FS_SEL_2_G;  break;
  case 4:  ret = ACCEL_CONFIG0_AP_ACCEL_FS_SEL_4_G;  break;
  case 8:  ret = ACCEL_CONFIG0_AP_ACCEL_FS_SEL_8_G;  break;
  case 16: ret = ACCEL_CONFIG0_AP_ACCEL_FS_SEL_16_G; break;
#if INV_IMU_HIGH_FSR_SUPPORTED
  case 32: ret = ACCEL_CONFIG0_AP_ACCEL_FS_SEL_32_G; break;
#endif
  default:
    /* Unknown accel FSR. Set to default 16G */
    break;
  }
  return ret;
}

gyro_config0_ap_gyro_fs_sel_t ICM566xx::gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps) {
  gyro_config0_ap_gyro_fs_sel_t ret = GYRO_CONFIG0_AP_GYRO_FS_SEL_2000_DPS;

  switch(gyro_fsr_dps) {
  case 250:  ret = GYRO_CONFIG0_AP_GYRO_FS_SEL_250_DPS;  break;
  case 500:  ret = GYRO_CONFIG0_AP_GYRO_FS_SEL_500_DPS;  break;
  case 1000: ret = GYRO_CONFIG0_AP_GYRO_FS_SEL_1000_DPS; break;
  case 2000: ret = GYRO_CONFIG0_AP_GYRO_FS_SEL_2000_DPS; break;
#if INV_IMU_HIGH_FSR_SUPPORTED
  case 4000: ret = GYRO_CONFIG0_AP_GYRO_FS_SEL_4000_DPS; break;
#endif
  default:
    /* Unknown gyro FSR. Set to default 2000dps" */
    break;
  }
  return ret;
}

accel_config0_accel_odr_t ICM566xx::accel_freq_to_param(uint16_t accel_freq_hz) {
  accel_config0_accel_odr_t ret = ACCEL_CONFIG0_ACCEL_ODR_100_HZ;

  switch(accel_freq_hz) {
  case 1:    ret = ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ;  break;
  case 3:    ret = ACCEL_CONFIG0_ACCEL_ODR_3_125_HZ;  break;
  case 6:    ret = ACCEL_CONFIG0_ACCEL_ODR_6_25_HZ;  break;
  case 12:   ret = ACCEL_CONFIG0_ACCEL_ODR_12_5_HZ;  break;
  case 25:   ret = ACCEL_CONFIG0_ACCEL_ODR_25_HZ;  break;
  case 50:   ret = ACCEL_CONFIG0_ACCEL_ODR_50_HZ;  break;
  case 100:  ret = ACCEL_CONFIG0_ACCEL_ODR_100_HZ; break;
  case 200:  ret = ACCEL_CONFIG0_ACCEL_ODR_200_HZ; break;
  case 400:  ret = ACCEL_CONFIG0_ACCEL_ODR_400_HZ; break;
  case 800:  ret = ACCEL_CONFIG0_ACCEL_ODR_800_HZ; break;
  default:
    /* Unknown accel frequency. Set to default 100Hz */
    break;
  }
  return ret;
}

gyro_config0_gyro_odr_t ICM566xx::gyro_freq_to_param(uint16_t gyro_freq_hz) {
  gyro_config0_gyro_odr_t ret = GYRO_CONFIG0_GYRO_ODR_100_HZ;

  switch(gyro_freq_hz) {
  case 1:   ret = GYRO_CONFIG0_GYRO_ODR_1_5625_HZ;  break;
  case 3:   ret = GYRO_CONFIG0_GYRO_ODR_3_125_HZ;  break;
  case 6:   ret = GYRO_CONFIG0_GYRO_ODR_6_25_HZ;  break;
  case 12:   ret = GYRO_CONFIG0_GYRO_ODR_12_5_HZ;  break;
  case 25:   ret = GYRO_CONFIG0_GYRO_ODR_25_HZ;  break;
  case 50:   ret = GYRO_CONFIG0_GYRO_ODR_50_HZ;  break;
  case 100:  ret = GYRO_CONFIG0_GYRO_ODR_100_HZ; break;
  case 200:  ret = GYRO_CONFIG0_GYRO_ODR_200_HZ; break;
  case 400:  ret = GYRO_CONFIG0_GYRO_ODR_400_HZ; break;
  case 800:  ret = GYRO_CONFIG0_GYRO_ODR_800_HZ; break;
  default:
    /* Unknown gyro ODR. Set to default 100Hz */
    break;
  }
  return ret;
}

static void sleep_us(uint32_t us)
{
    delayMicroseconds(us);
}

