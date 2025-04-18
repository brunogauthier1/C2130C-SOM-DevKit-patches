// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 */

#include "cam_sensor_io.h"
#include "cam_sensor_i2c.h"

int32_t camera_io_dev_poll(struct camera_io_master *io_master_info,
	uint32_t addr, uint16_t data, uint32_t data_mask,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type,
	uint32_t delay_ms)
{
	int16_t mask = data_mask & 0xFF;

	if (!io_master_info) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	if (io_master_info->master_type == CCI_MASTER) {
		return cam_cci_i2c_poll(io_master_info->cci_client,
			addr, data, mask, data_type, addr_type, delay_ms);
	} else if (io_master_info->master_type == I2C_MASTER) {
		return cam_qup_i2c_poll(io_master_info->client,
			addr, data, data_mask, addr_type, data_type,
			delay_ms);
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		return -EINVAL;
	}
}

int32_t camera_io_dev_erase(struct camera_io_master *io_master_info,
	uint32_t addr, uint32_t size)
{
	int rc = 0;

	if (!io_master_info) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	if (size == 0)
		return rc;

	if (io_master_info->master_type == SPI_MASTER) {
		CAM_DBG(CAM_SENSOR, "Calling SPI Erase");
		return cam_spi_erase(io_master_info, addr,
			CAMERA_SENSOR_I2C_TYPE_WORD, size);
	} else if (io_master_info->master_type == I2C_MASTER ||
		io_master_info->master_type == CCI_MASTER) {
		CAM_ERR(CAM_SENSOR, "Erase not supported on master :%d",
			io_master_info->master_type);
		rc = -EINVAL;
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		rc = -EINVAL;
	}
	return rc;
}

int32_t camera_io_dev_read(struct camera_io_master *io_master_info,
	uint32_t addr, uint32_t *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type)
{
	if (!io_master_info) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

    printk("%s:%d, ACHEUL address type: %d, data type: %d, address: 0x%04X", __FUNCTION__, __LINE__, addr_type, data_type, addr);

	if (io_master_info->master_type == CCI_MASTER) {

        printk("%s:%d, ACHEUL CCI_MASTER", __FUNCTION__, __LINE__);
		return cam_cci_i2c_read(io_master_info->cci_client,
			addr, data, addr_type, data_type);
	} else if (io_master_info->master_type == I2C_MASTER) {
        printk("%s:%d, ACHEUL I2C_MASTER", __FUNCTION__, __LINE__);
		return cam_qup_i2c_read(io_master_info->client,
			addr, data, addr_type, data_type);
	} else if (io_master_info->master_type == SPI_MASTER) {
        printk("%s:%d, ACHEUL SPI_MASTER", __FUNCTION__, __LINE__);
		return cam_spi_read(io_master_info,
			addr, data, addr_type, data_type);
	} else {
        printk("%s:%d, ACHEUL invalid type", __FUNCTION__, __LINE__);
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		return -EINVAL;
	}

    printk("%s:%d, ACHEUL data: 0x%04X", __FUNCTION__, __LINE__, *data);
	return 0;
}

int32_t camera_io_dev_read_seq(struct camera_io_master *io_master_info,
	uint32_t addr, uint8_t *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type, int32_t num_bytes)
{
	if (io_master_info->master_type == CCI_MASTER) {
		return cam_camera_cci_i2c_read_seq(io_master_info->cci_client,
			addr, data, addr_type, data_type, num_bytes);
	} else if (io_master_info->master_type == I2C_MASTER) {
		return cam_qup_i2c_read_seq(io_master_info->client,
			addr, data, addr_type, num_bytes);
	} else if (io_master_info->master_type == SPI_MASTER) {
		return cam_spi_read_seq(io_master_info,
			addr, data, addr_type, num_bytes);
	} else if (io_master_info->master_type == SPI_MASTER) {
		return cam_spi_write_seq(io_master_info,
			addr, data, addr_type, num_bytes);
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		return -EINVAL;
	}
	return 0;
}

int32_t hdmi_io_dev_write(struct camera_io_master *io_master_info,
	uint32_t reg, uint32_t buf)
{

	int rc= -1;

	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array i2c_apply_setting;
		i2c_apply_setting.reg_addr = reg;
		i2c_apply_setting.reg_data = buf;
		i2c_apply_setting.delay = 0;
		i2c_apply_setting.data_mask = 0;
		write_setting.reg_setting = &i2c_apply_setting;
		write_setting.size = 1;
		write_setting.addr_type = 1;
		write_setting.data_type = 1;
		write_setting.delay = 0;
		rc=cam_qup_i2c_write_table(io_master_info, &write_setting);

		if(rc<0){
		  CAM_ERR(CAM_SENSOR, "lyn_error in hdmi_io_dev_write:0X%x::0X%x",reg,buf);
		  return rc;
		}

		return rc;

}
int32_t hdmi_io_dev_write_array(struct camera_io_master *io_master_info,
	struct cam_sensor_i2c_reg_array *i2c_setting, int size)
{
	int i = 0;
	int rc= -1;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array i2c_apply_setting;

	for (i = 0; i < size; i++) {
		i2c_apply_setting.reg_addr = i2c_setting[i].reg_addr;
		i2c_apply_setting.reg_data = i2c_setting[i].reg_data;
		i2c_apply_setting.delay = i2c_setting[i].delay;
		i2c_apply_setting.data_mask = i2c_setting[i].data_mask;
		write_setting.reg_setting = &i2c_apply_setting;
		write_setting.size = 1;
		write_setting.addr_type = 1;
		write_setting.data_type = 1;
		write_setting.delay = 0;
		rc = cam_qup_i2c_write_table(io_master_info, &write_setting);
		if(rc<0){
		  CAM_ERR(CAM_SENSOR, "error in hdmi_io_dev_write_array:0X%x::0X%x",i2c_setting[i].reg_addr,i2c_setting[i].reg_data);
          return rc;
		}
	}

	return rc;
}

int32_t camera_io_dev_write(struct camera_io_master *io_master_info,
	struct cam_sensor_i2c_reg_setting *write_setting)
{
	if (!write_setting || !io_master_info) {
		CAM_ERR(CAM_SENSOR,
			"Input parameters not valid ws: %pK ioinfo: %pK",
			write_setting, io_master_info);
		return -EINVAL;
	}

	if (!write_setting->reg_setting) {
		CAM_ERR(CAM_SENSOR, "Invalid Register Settings");
		return -EINVAL;
	}

	if (io_master_info->master_type == CCI_MASTER) {
		return cam_cci_i2c_write_table(io_master_info,
			write_setting);
	} else if (io_master_info->master_type == I2C_MASTER) {
		return cam_qup_i2c_write_table(io_master_info,
			write_setting);
	} else if (io_master_info->master_type == SPI_MASTER) {
		return cam_spi_write_table(io_master_info,
			write_setting);
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		return -EINVAL;
	}
}

int32_t camera_io_dev_write_continuous(struct camera_io_master *io_master_info,
	struct cam_sensor_i2c_reg_setting *write_setting,
	uint8_t cam_sensor_i2c_write_flag)
{
	if (!write_setting || !io_master_info) {
		CAM_ERR(CAM_SENSOR,
			"Input parameters not valid ws: %pK ioinfo: %pK",
			write_setting, io_master_info);
		return -EINVAL;
	}

	if (!write_setting->reg_setting) {
		CAM_ERR(CAM_SENSOR, "Invalid Register Settings");
		return -EINVAL;
	}

	if (io_master_info->master_type == CCI_MASTER) {
		return cam_cci_i2c_write_continuous_table(io_master_info,
			write_setting, cam_sensor_i2c_write_flag);
	} else if (io_master_info->master_type == I2C_MASTER) {
		return cam_qup_i2c_write_continuous_table(io_master_info,
			write_setting, cam_sensor_i2c_write_flag);
	} else if (io_master_info->master_type == SPI_MASTER) {
		return cam_spi_write_table(io_master_info,
			write_setting);
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		return -EINVAL;
	}
}

int32_t camera_io_init(struct camera_io_master *io_master_info)
{
	if (!io_master_info) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	if (io_master_info->master_type == CCI_MASTER) {
		io_master_info->cci_client->cci_subdev =
		cam_cci_get_subdev(io_master_info->cci_client->cci_device);
		return cam_sensor_cci_i2c_util(io_master_info->cci_client,
			MSM_CCI_INIT);
	} else if ((io_master_info->master_type == I2C_MASTER) ||
			(io_master_info->master_type == SPI_MASTER)) {
		return 0;
	}

	return -EINVAL;
}

int32_t camera_io_release(struct camera_io_master *io_master_info)
{
	if (!io_master_info) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	if (io_master_info->master_type == CCI_MASTER) {
		return cam_sensor_cci_i2c_util(io_master_info->cci_client,
			MSM_CCI_RELEASE);
	} else if ((io_master_info->master_type == I2C_MASTER) ||
			(io_master_info->master_type == SPI_MASTER)) {
		return 0;
	}

	return -EINVAL;
}
