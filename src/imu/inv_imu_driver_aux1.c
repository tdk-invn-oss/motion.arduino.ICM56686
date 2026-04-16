/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively "Software") is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_driver_aux1.h"

#define AUX_ENDIANNESS	SREG_CTRL_SREG_DATA_BIG_ENDIAN

#if INV_IMU_AUX1_SUPPORTED

int inv_imu_set_aux1_accel_mode(inv_imu_transport_t *t, uint8_t en)
{
	int             status = INV_IMU_OK;
	pwr_mgmt_aux1_t pwr_mgmt_aux1;

	status |= inv_imu_read_reg(t, PWR_MGMT_AUX1, 1, (uint8_t *)&pwr_mgmt_aux1);
	pwr_mgmt_aux1.accel_aux1_en = en;
	status |= inv_imu_write_reg(t, PWR_MGMT_AUX1, 1, (uint8_t *)&pwr_mgmt_aux1);

	return status;
}

int inv_imu_set_aux1_drdy(inv_imu_transport_t *t, uint8_t en, inv_imu_int_num_t it)
{
	int            status = INV_IMU_OK;
	int1_config0_t int1_config0;
	uint32_t       reg = (it == INV_IMU_INT1) ? INT1_CONFIG0 : INT2_CONFIG0;

	/* Use `int1_config0_t` for both INT1 and INT2 as bit location are the same */
	status |= inv_imu_read_reg(t, reg, 1, (uint8_t *)&int1_config0);
	int1_config0.int1_status_en_aux1_drdy = en;
	status |= inv_imu_write_reg(t, reg, 1, (uint8_t *)&int1_config0);

	return status;
}

int inv_imu_get_aux1_int_status(inv_imu_transport_t *t, inv_imu_aux1_int_state_t *it)
{
	int            status = INV_IMU_OK;
	intx_statusx_t int_status;

	/* Read AUX1 interrupt status */
	status |= inv_imu_read_reg(t, INT2_STATUS0, 2, (uint8_t *)&int_status);

	it->INV_OIS1         = int_status.int1_status0.int1_status_aux1_drdy;
	it->INV_OIS1_AGC_RDY = int_status.int1_status0.int1_status_aux1_agc_rdy;

	return status;
}

int inv_imu_set_aux1_accel_fsr(inv_imu_transport_t *t, fs_sel_aux_accel_fs_sel_t fsr)
{
	int           status = INV_IMU_OK;
	fs_sel_aux1_t fs_sel_aux1;

	status |= inv_imu_read_reg(t, FS_SEL_AUX1, 1, (uint8_t *)&fs_sel_aux1);
	fs_sel_aux1.aux1_accel_fs_sel = fsr;
	status |= inv_imu_write_reg(t, FS_SEL_AUX1, 1, (uint8_t *)&fs_sel_aux1);

	return status;
}

int inv_imu_get_aux1_accel_fsr(inv_imu_transport_t *t, fs_sel_aux_accel_fs_sel_t *fsr)
{
	int           status = INV_IMU_OK;
	fs_sel_aux1_t fs_sel_aux1;

	status |= inv_imu_read_reg(t, FS_SEL_AUX1, 1, (uint8_t *)&fs_sel_aux1);
	*fsr = (fs_sel_aux_accel_fs_sel_t)fs_sel_aux1.aux1_accel_fs_sel;

	return status;
}

int inv_imu_set_aux1_gyro_mode(inv_imu_transport_t *t, uint8_t en)
{
	int             status = INV_IMU_OK;
	pwr_mgmt_aux1_t pwr_mgmt_aux1;

	status |= inv_imu_read_reg(t, PWR_MGMT_AUX1, 1, (uint8_t *)&pwr_mgmt_aux1);
	pwr_mgmt_aux1.gyro_aux1_en = en;
	status |= inv_imu_write_reg(t, PWR_MGMT_AUX1, 1, (uint8_t *)&pwr_mgmt_aux1);

	return status;
}

int inv_imu_set_aux1_gyro_fsr(inv_imu_transport_t *t, fs_sel_aux_gyro_fs_sel_t fsr)
{
	int           status = INV_IMU_OK;
	fs_sel_aux1_t fs_sel_aux1;

	status |= inv_imu_read_reg(t, FS_SEL_AUX1, 1, (uint8_t *)&fs_sel_aux1);
	fs_sel_aux1.aux1_gyro_fs_sel = fsr;
	status |= inv_imu_write_reg(t, FS_SEL_AUX1, 1, (uint8_t *)&fs_sel_aux1);

	return status;
}

int inv_imu_get_aux1_gyro_fsr(inv_imu_transport_t *t, fs_sel_aux_gyro_fs_sel_t *fsr)
{
	int           status = INV_IMU_OK;
	fs_sel_aux1_t fs_sel_aux1;

	status |= inv_imu_read_reg(t, FS_SEL_AUX1, 1, (uint8_t *)&fs_sel_aux1);
	*fsr = (fs_sel_aux_gyro_fs_sel_t)fs_sel_aux1.aux1_gyro_fs_sel;

	return status;
}

int inv_imu_get_aux1_register_data(inv_imu_transport_t *t, inv_imu_aux_sensor_data_t *sensor_data)
{
	int status = INV_IMU_OK;
	uint8_t raw_sensor_data[14];

	status |= inv_imu_read_reg(t, ACCEL_DATA_X_AUX1_0, sizeof(raw_sensor_data),
	                           raw_sensor_data);
					   
	FORMAT_16_BITS_DATA(AUX_ENDIANNESS, &raw_sensor_data[0], &sensor_data->accel_data[0]);
	FORMAT_16_BITS_DATA(AUX_ENDIANNESS, &raw_sensor_data[2], &sensor_data->accel_data[1]);
	FORMAT_16_BITS_DATA(AUX_ENDIANNESS, &raw_sensor_data[4], &sensor_data->accel_data[2]);
	FORMAT_16_BITS_DATA(AUX_ENDIANNESS, &raw_sensor_data[6], &sensor_data->gyro_data[0]);
	FORMAT_16_BITS_DATA(AUX_ENDIANNESS, &raw_sensor_data[8], &sensor_data->gyro_data[1]);
	FORMAT_16_BITS_DATA(AUX_ENDIANNESS, &raw_sensor_data[10], &sensor_data->gyro_data[2]);
	FORMAT_16_BITS_DATA(AUX_ENDIANNESS, &raw_sensor_data[12], &sensor_data->temp_data);
	
	return status;
}

#if INV_IMU_INT2_PIN_SUPPORTED
int inv_imu_set_aux1_pin_config_int(inv_imu_transport_t *t, const inv_imu_int_pin_config_t *conf)
{
	int            status = INV_IMU_OK;
	int1_config2_t int2_config2;

	status |= inv_imu_read_reg(t, INT2_CONFIG2, 1, (uint8_t *)&int2_config2);

	/* Use `int1_config2_t` for both INT1 and INT2 as bit location are the same */
	int2_config2.int1_polarity = conf->int_polarity;
	int2_config2.int1_mode     = conf->int_mode;
	int2_config2.int1_drive    = conf->int_drive;

	status |= inv_imu_write_reg(t, INT2_CONFIG2, 1, (uint8_t *)&int2_config2);

	return status;
}
#endif

#endif
