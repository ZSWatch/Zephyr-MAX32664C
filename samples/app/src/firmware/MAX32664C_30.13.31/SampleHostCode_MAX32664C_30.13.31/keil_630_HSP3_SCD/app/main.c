///*******************************************************************************
// * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
// *
// * Permission is hereby granted, free of charge, to any person obtaining a
// * copy of this software and associated documentation files (the "Software"),
// * to deal in the Software without restriction, including without limitation
// * the rights to use, copy, modify, merge, publish, distribute, sublicense,
// * and/or sell copies of the Software, and to permit persons to whom the
// * Software is furnished to do so, subject to the following conditions:
// *
// * The above copyright notice and this permission notice shall be included
// * in all copies or substantial portions of the Software.
// *
// * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
// * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// * OTHER DEALINGS IN THE SOFTWARE.
// *
// * Except as contained in this notice, the name of Maxim Integrated
// * Products, Inc. shall not be used except as stated in the Maxim Integrated
// * Products, Inc. Branding Policy.
// *
// * The mere transfer of this software does not imply any licenses
// * of trade secrets, proprietary technology, copyrights, patents,
// * trademarks, maskwork rights, or any other form of intellectual
// * property whatsoever. Maxim Integrated Products, Inc. retains all
// * ownership rights.
// *
// * $Date: 2016-10-06 11:30:20 -0500 (Thu, 06 Oct 2016) $
// * $Revision: 24600 $
// *
// ******************************************************************************/


#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "mxc_config.h"
#include "i2cm.h"
#include "led.h"
#include "tmr_utils.h"
#include "uart.h"
#include "max20303.h"
#include "tmr.h"
#include "nvic_table.h"
#include "lp.h"
#include "gpio.h"


#define I2C_MASTER          MXC_I2CM1
#define I2C_SLAVE_ADDR      0x55

#define SS_PLATFORM_MAX3263X                "SmartSensor_MAX3263X"
#define SS_PLATFORM_MAX32660                "SmartSensor_MAX32660"
#define SS_BOOTLOADER_PLATFORM_MAX3263X     "Bootloader_MAX3263X"
#define SS_BOOTLOADER_PLATFORM_MAX32660     "Bootloader_MAX32660"

#define SS_I2C_8BIT_SLAVE_ADDR      0xAA
#define SS_DEFAULT_CMD_SLEEP_MS     2
#define SS_DUMP_REG_SLEEP_MS        100
#define SS_ENABLE_SENSOR_SLEEP_MS   40

#define SS_SENSORIDX_MAX86140	0x00
#define SS_SENSORIDX_MAX30205	0x01
#define SS_SENSORIDX_MAX30001	0x02
#define SS_SENSORIDX_MAX30101	0x03
#define SS_SENSORIDX_ACCEL	0x04

#define SS_ALGOIDX_AGC	0x00
#define SS_ALGOIDX_AEC	0x01
#define SS_ALGOIDX_WHRM	0x02
#define SS_ALGOIDX_ECG	0x03
#define SS_ALGOIDX_BPT	0x04
#define SS_ALGOIDX_SPO2 0x05

#define SS_FAM_R_STATUS		0x00
	#define SS_CMDIDX_STATUS	0x00
		#define SS_SHIFT_STATUS_ERR				0
		#define SS_MASK_STATUS_ERR				(0x07 << SS_SHIFT_STATUS_ERR)
		#define SS_SHIFT_STATUS_DATA_RDY		3
		#define SS_MASK_STATUS_DATA_RDY			(1 << SS_SHIFT_STATUS_DATA_RDY)
		#define SS_SHIFT_STATUS_FIFO_OUT_OVR	4
		#define SS_MASK_STATUS_FIFO_OUT_OVR		(1 << SS_SHIFT_STATUS_FIFO_OUT_OVR)
		#define SS_SHIFT_STATUS_FIFO_IN_OVR		5
		#define SS_MASK_STATUS_FIFO_IN_OVR		(1 << SS_SHIFT_STATUS_FIFO_IN_OVR)

#define SS_FAM_W_MODE	0x01
#define SS_FAM_R_MODE	0x02
	#define SS_CMDIDX_MODE	0x00
		#define SS_SHIFT_MODE_SHDN		0
		#define SS_MASK_MODE_SHDN		(1 << SS_SHIFT_MODE_SHDN)
		#define SS_SHIFT_MODE_RESET		1
		#define SS_MASK_MODE_RESET		(1 << SS_SHIFT_MODE_RESET)
		#define SS_SHIFT_MODE_FIFORESET	2
		#define SS_MASK_MODE_FIFORESET	(1 << SS_SHIFT_MODE_FIFORESET)
		#define SS_SHIFT_MODE_BOOTLDR	3
		#define SS_MASK_MODE_BOOTLDR	(1 << SS_SHIFT_MODE_BOOTLDR)

#define SS_I2C_READ		0x03

#define SS_FAM_W_COMMCHAN	0x10
#define SS_FAM_R_COMMCHAN	0x11
	#define SS_CMDIDX_OUTPUTMODE	0x00
		#define SS_SHIFT_OUTPUTMODE_DATATYPE	0
		#define SS_MASK_OUTPUTMODE_DATATYPE		(0x03 << SS_SHIFT_OUTPUTMODE_DATATYPE)
			#define SS_DATATYPE_PAUSE				0
			#define SS_DATATYPE_RAW					1
			#define SS_DATATYPE_ALGO				2
			#define SS_DATATYPE_BOTH				3
		#define SS_SHIFT_OUTPUTMODE_SC_EN		2
		#define SS_MASK_OUTPUTMODE_SC_EN		(1 << SS_SHIFT_OUTPUTMODE_SC_EN)
	#define SS_CMDIDX_FIFOAFULL		0x01

#define SS_FAM_R_OUTPUTFIFO	0x12
	#define SS_CMDIDX_OUT_NUMSAMPLES	0x00
	#define SS_CMDIDX_READFIFO		    0x01

#define SS_FAM_R_INPUTFIFO	0x13
	#define SS_CMDIDX_SAMPLESIZE	0x00
	#define SS_CMDIDX_FIFOSIZE		0x01
	#define SS_CMDIDX_IN_NUMSAMPLES	0x02
#define SS_FAM_W_INPUTFIFO	0x14
	#define SS_CMDIDN_WRITEFIFO		0x00

#define SS_FAM_W_WRITEREG		0x40
#define SS_FAM_R_READREG		0x41
#define SS_FAM_R_REGATTRIBS		0x42
#define SS_FAM_R_DUMPREG		0x43

#define SS_FAM_W_SENSORMODE	0x44
#define SS_FAM_R_SENSORMODE	0x45

//TODO: Fill in known configuration parameters
#define SS_FAM_W_ALGOCONFIG	0x50
#define SS_FAM_R_ALGOCONFIG	0x51
	#define SS_CFGIDX_AGC_TARGET		0x00
	#define SS_CFGIDX_AGC_CORR_COEFF	0x01
	#define SS_CFGIDX_AGC_SENSITIVITY	0x02
	#define SS_CFGIDX_AGC_SMP_AVG		0x03

	#define SS_CFGIDX_WHRM_SR			0x00
	#define SS_CFGIDX_WHRM_MAX_HEIGHT	0x01
	#define SS_CFGIDX_WHRM_MAX_WEIGHT	0x02
	#define SS_CFGIDX_WHRM_MAX_AGE		0x03
	#define SS_CFGIDX_WHRM_MIN_HEIGHT	0x04
	#define SS_CFGIDX_WHRM_MIN_WEIGHT	0x05
	#define SS_CFGIDX_WHRM_MIN_AGE		0x06
	#define SS_CFGIDX_WHRM_DEF_HEIGHT	0x07
	#define SS_CFGIDX_WHRM_DEF_WEIGHT	0x08
	#define SS_CFGIDX_WHRM_DEF_AGE		0x09
	#define SS_CFGIDX_WHRM_INIT_HR		0x0A

	#define SS_CFGIDX_BP_USE_MED		0x00
	#define SS_CFGIDX_BP_SYS_BP_CAL		0x01
	#define SS_CFGIDX_BP_DIA_BP_CAL		0x02
	#define SS_CFGIDX_BP_CAL_DATA		0x03
	#define SS_CFGIDX_BP_EST_DATE		0x04
	#define SS_CFGIDX_BP_EST_NONREST	0x05

#define SS_FAM_W_ALGOMODE	0x52
#define SS_FAM_R_ALGOMODE	0x53

#define SS_FAM_W_EXTERNSENSORMODE	0x60
#define SS_FAM_R_EXTERNSENSORMODE	0x61

#define SS_FAM_R_SELFTEST    0x70

#define SS_FAM_W_BOOTLOADER	0x80
	#define SS_CMDIDX_SETIV			0x00
	#define SS_CMDIDX_SETAUTH		0x01
	#define SS_CMDIDX_SETNUMPAGES	0x02
	#define SS_CMDIDX_ERASE			0x03
	#define SS_CMDIDX_SENDPAGE		0x04
	#define SS_CMDIDX_ERASE_PAGE	0x05
#define SS_FAM_R_BOOTLOADER	0x81
	#define SS_CMDIDX_BOOTFWVERSION	0x00
	#define SS_CMDIDX_PAGESIZE		0x01

#define SS_FAM_W_BOOTLOADER_CFG	0x82
#define SS_FAM_R_BOOTLOADER_CFG	0x83
	#define SS_CMDIDX_BL_SAVE		0x00
	#define SS_CMDIDX_BL_ENTRY		0x01
		#define SS_BL_CFG_ENTER_BL_MODE		0x00
		#define SS_BL_CFG_EBL_PIN			0x01
		#define SS_BL_CFG_EBL_POL			0x02
	#define SS_CMDIDX_BL_EXIT		0x02
		#define SS_BL_CFG_EXIT_BL_MODE		0x00
		#define SS_BL_CFG_TIMEOUT			0x01


#define SS_FAM_R_IDENTITY	0xFF
	#define SS_CMDIDX_PLATTYPE		0x00
	#define SS_CMDIDX_PARTID		0x01
	#define SS_CMDIDX_REVID			0x02
	#define SS_CMDIDX_FWVERSION		0x03
	#define SS_CMDIDX_AVAILSENSORS	0x04
	#define SS_CMDIDX_DRIVERVER		0x05
	#define SS_CMDIDX_AVAILALGOS	0x06
	#define SS_CMDIDX_ALGOVER		0x07

typedef enum 
{
	SS_SUCCESS = 0x00,

	SS_ERR_COMMAND = 0x01,
	SS_ERR_UNAVAILABLE = 0x02,
	SS_ERR_DATA_FORMAT = 0x03,
	SS_ERR_INPUT_VALUE = 0x04,

	SS_ERR_BTLDR_GENERAL = 0x80,
	SS_ERR_BTLDR_CHECKSUM = 0x81,

	SS_ERR_TRY_AGAIN = 0xFE,
	SS_ERR_UNKNOWN = 0xFF,
} SS_STATUS;

typedef enum 
{
    SS_PLAT_MAX3263X = 0,
    SS_PLAT_MAX32660 = 1,
} SS_PLAT_TYPE;

typedef struct {
    uint8_t addr;
    uint32_t val;
} addr_val_pair;

//self test result masks
#define FAILURE_COMM        0x01
#define FAILURE_INTERRUPT   0x02

#define SS_SMALL_BUF_SIZE   32
#define SS_MED_BUF_SIZE     512
#define SS_LARGE_BUF_SIZE   8224

#define SS_RESET_TIME	10
#define SS_STARTUP_TIME	50

#define SS_MAX_SUPPORTED_SENSOR_NUM	0xFE
#define SS_MAX_SUPPORTED_ALGO_NUM	0xFE
#define SS_MAX_SUPPORTED_ALGO_CFG_NUM	0xFE
#define SS_MAX_SUPPORTED_MODE_NUM	0xFF

gpio_cfg_t MFIO_pin;


void mxc_delay(int us)
{
    TMR_Delay(MXC_TMR5, us);
}


SS_STATUS read_cmd(uint8_t *cmd_bytes, int cmd_bytes_len, uint8_t *rxbuf, int rxbuf_sz, int sleep_ms)
{
	int retries, ret, try_again;
    uint8_t temp[4] = {0, 0, 0, 0};
    
    GPIO_OutClr(&MFIO_pin);
    mxc_delay(500);
    ret = I2CM_Write(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, cmd_bytes, cmd_bytes_len);
    
    mxc_delay(sleep_ms * 1000);

    ret = I2CM_Read(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, rxbuf, rxbuf_sz);
    //ret = I2C_MasterRead(I2C_MASTER, 0xAB, rxbuf, rxbuf_sz, 0);
	try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
    retries = 4;
	while ((ret != rxbuf_sz || try_again) && (retries-- > 0))
    {
		mxc_delay(sleep_ms * 1000);
        ret = I2CM_Read(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, rxbuf, rxbuf_sz);
		try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	}
    
    GPIO_OutSet(&MFIO_pin);
    
    if (ret != rxbuf_sz || try_again) 
    {
        return SS_ERR_UNAVAILABLE;
    }

    return (SS_STATUS)rxbuf[0];
}

SS_STATUS write_cmd(uint8_t *tx_buf, int tx_len, int sleep_ms)
{
    int ret, retries, try_again;
    uint8_t status_byte;
    uint8_t temp[4] = {0, 0, 0, 0};
    
    GPIO_OutClr(&MFIO_pin);
    mxc_delay(500);
    

    
    ret = I2CM_Write(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, tx_buf, tx_len);


    mxc_delay(sleep_ms * 1000);

    ret = I2CM_Read(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, &status_byte, 1);
	try_again = (status_byte == SS_ERR_TRY_AGAIN);
    retries = 4;
	while ((ret != 1 || try_again) && (retries-- > 0))
    {
		mxc_delay(sleep_ms * 1000);
        ret = I2CM_Read(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, &status_byte, 1);
		try_again = (status_byte == SS_ERR_TRY_AGAIN);
	}
    
    //GPIO_OutSet(&MFIO_pin);

    if ((ret != 1) || try_again)
    {
        return SS_ERR_UNAVAILABLE;
    }

	return (SS_STATUS)status_byte;
}


SS_STATUS get_hub_status( uint8_t *response){

	uint8_t cmd_bytes[] = { SS_FAM_R_STATUS, SS_CMDIDX_STATUS };
	uint8_t rxbuf[2] = {0};
	SS_STATUS status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),&rxbuf[0], sizeof(rxbuf), 2);
	if (status == SS_SUCCESS)
		*response = rxbuf[1];

}											
											
											
											
SS_STATUS exit_from_bootloader(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_MODE, SS_CMDIDX_MODE, 0 };
	SS_STATUS status;

    status = write_cmd(cmd_bytes, sizeof(cmd_bytes), 1);
	return status;
}

int in_bootldr_mode(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_MODE, SS_CMDIDX_MODE };
	uint8_t rxbuf[2] = { 0 };

	SS_STATUS status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), &rxbuf[0], sizeof(rxbuf), 2);
	if (status != SS_SUCCESS) return -1;

	return (rxbuf[1] & SS_MASK_MODE_BOOTLDR);
}

SS_STATUS get_ss_fw_version(uint32_t* fw_version)
{
    uint8_t cmd_bytes[2];
    uint8_t rxbuf[4];
    SS_STATUS status;

	int bootldr = in_bootldr_mode();

	if (bootldr > 0) 
    {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
	} 
    else if (bootldr == 0) 
    {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_FWVERSION;
	}

    status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), &rxbuf[0], sizeof(rxbuf), 2);
    *fw_version = ((uint32_t)(rxbuf[0]) << 24) + 
                    ((uint32_t)(rxbuf[1]) << 16) + 
                    ((uint32_t)(rxbuf[2]) << 8) + 
                    ((uint32_t)(rxbuf[3]) << 0);
    return status;
}

SS_STATUS get_reg(int idx, uint8_t addr, uint32_t *val)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_REGATTRIBS, (uint8_t)idx };
	uint8_t rx_reg_attribs[3] = {0};
	uint8_t cmd_bytes2[] = { SS_FAM_R_READREG, (uint8_t)idx, addr };
	uint8_t rxbuf[5] = {0};
	SS_STATUS status;
    int reg_width;
    
    status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), &rx_reg_attribs[0], sizeof(rx_reg_attribs), 2);
	if (status != SS_SUCCESS) return status;

	reg_width = rx_reg_attribs[1];
	status = read_cmd(&cmd_bytes2[0], sizeof(cmd_bytes2), &rxbuf[0], reg_width + 1, 2);

	if (status == SS_SUCCESS) 
    {
		*val = 0;
		for (int i = 0; i < reg_width; i++) 
        {
			*val = (*val << 8) | rxbuf[i + 1];
		}
	}

	return status;
}

SS_STATUS set_reg(int idx, uint8_t addr, uint32_t val, int byte_size)
{
    int i;
	uint8_t cmd_bytes[] = { SS_FAM_W_WRITEREG, (uint8_t)idx, addr };
	uint8_t data_bytes[4];
    uint8_t txbuf[sizeof(cmd_bytes) + 4];
    SS_STATUS status;
    
	for (i = 0; i < byte_size; i++) 
    {
		data_bytes[i] = (val >> (8 * (byte_size - 1)) & 0xFF);
	}

    for(i = 0; i < sizeof(cmd_bytes); i++)
    {
        txbuf[i] = cmd_bytes[i];
    }
    
    for(i = 0; i < byte_size; i++)
    {
        txbuf[sizeof(cmd_bytes) + i] = data_bytes[i];
    }
    
	status = write_cmd(txbuf, sizeof(cmd_bytes) + byte_size, 2);

	return status;
}



SS_STATUS dump_reg(int idx, addr_val_pair* reg_vals, int reg_vals_sz, int* num_regs)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_REGATTRIBS, (uint8_t)idx };
	uint8_t rx_reg_attribs[3] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), &rx_reg_attribs[0], sizeof(rx_reg_attribs), 2);

	if (status != SS_SUCCESS) return status;

	int reg_width = rx_reg_attribs[1];
	*num_regs = rx_reg_attribs[2];
    
	int dump_reg_sz = (*num_regs) * (reg_width + 1) + 1; //+1 to reg_width for address, +1 for status byte

	uint8_t rxbuf[512];
	cmd_bytes[0] = SS_FAM_R_DUMPREG;
	status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), &rxbuf[0], dump_reg_sz, SS_DUMP_REG_SLEEP_MS);

	if (status != SS_SUCCESS) return status;

	//rxbuf format is [status][addr0](reg_width x [val0])[addr1](reg_width x [val1])...
	for (int reg = 0; reg < *num_regs; reg++) 
    {
		reg_vals[reg].addr = rxbuf[(reg * (reg_width + 1)) + 1];
		uint32_t *val = &(reg_vals[reg].val);
		*val = 0;
		for (int byte = 0; byte < reg_width; byte++) 
        {
			*val = (*val << 8) | rxbuf[(reg * (reg_width + 1)) + byte + 2];
		}
	}

	return SS_SUCCESS;
}

SS_STATUS enable_sensor(int idx, int mode)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, (uint8_t)idx, (uint8_t)mode };
	SS_STATUS status;

    status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), SS_ENABLE_SENSOR_SLEEP_MS);
	return status;
}

SS_STATUS set_accel_mode(int mode, int ext)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, SS_SENSORIDX_ACCEL, (uint8_t)mode, (uint8_t)ext };
	SS_STATUS status;

    status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), SS_ENABLE_SENSOR_SLEEP_MS);
	return status;
}

SS_STATUS set_spo2_continue_mode(int mode)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOCONFIG, SS_ALGOIDX_SPO2, 0x02, (uint8_t)mode};
	SS_STATUS status;

    status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), 40);
	return status;
}

SS_STATUS set_SCD_enable(int en)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOCONFIG, SS_ALGOIDX_WHRM, 0x0C, (uint8_t)en};
	SS_STATUS status;

    status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), 40);
	return status;
}

SS_STATUS disable_sensor(int idx)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, (uint8_t)idx, 0 };
	SS_STATUS status;
    
    status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), SS_ENABLE_SENSOR_SLEEP_MS);
	return status;
}

SS_STATUS enable_accel(int idx, int mode)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, (uint8_t)idx, (uint8_t)mode };
	SS_STATUS status;

    status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), SS_ENABLE_SENSOR_SLEEP_MS);
	return status;
}

SS_STATUS enable_algo(int idx, int mode)
{
    uint8_t cmd_bytes[] = { SS_FAM_W_ALGOMODE, (uint8_t)idx, (uint8_t)mode };
	SS_STATUS status;
 
    status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), SS_ENABLE_SENSOR_SLEEP_MS);
	return status;
}

SS_STATUS turn_off_AGC(void)
{
    uint8_t cmd_bytes[] = { 0x52, 0x00, 0x00 };
	SS_STATUS status;
 
    status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), SS_ENABLE_SENSOR_SLEEP_MS);
	return status;
}

SS_STATUS disable_algo(int idx)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOMODE, (uint8_t)idx, 0 };
	SS_STATUS status;

    status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), SS_ENABLE_SENSOR_SLEEP_MS);
	return status;
}

SS_STATUS set_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOCONFIG, (uint8_t)algo_idx, (uint8_t)cfg_idx };
    uint8_t txbuf[16];
    SS_STATUS status;
    int i;
    
    for(i = 0; i < sizeof(cmd_bytes); i++)
    {
        txbuf[i] = cmd_bytes[i];
    }
    
    for(i = 0; i < cfg_sz; i++)
    {
        txbuf[sizeof(cmd_bytes) + i] = cfg[i];
    }
    
	status = write_cmd(txbuf, sizeof(cmd_bytes) + cfg_sz, 2);

	return status;
}

SS_STATUS get_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_ALGOCONFIG, (uint8_t)algo_idx, (uint8_t)cfg_idx };
	SS_STATUS status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), cfg, cfg_sz, 2);

	return status;
}

SS_STATUS set_data_type(int data_type, uint8_t sc_en)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_COMMCHAN, SS_CMDIDX_OUTPUTMODE };
	uint8_t data_bytes[] = { (uint8_t)((sc_en ? SS_MASK_OUTPUTMODE_SC_EN : 0) |
							((data_type << SS_SHIFT_OUTPUTMODE_DATATYPE) & SS_MASK_OUTPUTMODE_DATATYPE)) };
    int i;
    uint8_t txbuf[sizeof(cmd_bytes) + sizeof(data_bytes)];
                            
	if((data_type < 0) && (data_type > 3)) return SS_ERR_INPUT_VALUE;

    for(i = 0; i < sizeof(cmd_bytes); i++)
    {
        txbuf[i] = cmd_bytes[i];
    }
    
    for(i = 0; i < sizeof(data_bytes); i++)
    {
        txbuf[sizeof(cmd_bytes) + i] = data_bytes[i];
    }
    
	SS_STATUS status = write_cmd(txbuf, sizeof(txbuf), 2);

	return status;
}

SS_STATUS get_data_type(int *data_type, uint8_t *sc_en)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_COMMCHAN, SS_CMDIDX_OUTPUTMODE };
	uint8_t rxbuf[2] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), &rxbuf[0], sizeof(rxbuf), 2);
	if (status == SS_SUCCESS)
    {
		*data_type = (rxbuf[1] & SS_MASK_OUTPUTMODE_DATATYPE) >> SS_SHIFT_OUTPUTMODE_DATATYPE;
		*sc_en = (uint8_t)((rxbuf[1] & SS_MASK_OUTPUTMODE_SC_EN) >> SS_SHIFT_OUTPUTMODE_SC_EN);
	}

	return status;
}

SS_STATUS set_fifo_thresh(int thresh)
{
	uint8_t cmd_bytes[3] = { SS_FAM_W_COMMCHAN, SS_CMDIDX_FIFOAFULL, (uint8_t)thresh};
	SS_STATUS status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), 2);
	return status;
}

SS_STATUS get_fifo_thresh(int *thresh)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_COMMCHAN, SS_CMDIDX_FIFOAFULL };
	uint8_t rxbuf[2] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), &rxbuf[0], sizeof(rxbuf), 2);

	if (status == SS_SUCCESS) 
    {
		*thresh = rxbuf[1];
	}

	return status;
}

SS_STATUS ss_comm_check(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_IDENTITY, SS_CMDIDX_PLATTYPE };
	uint8_t rxbuf[2];

	SS_STATUS status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), &rxbuf[0], sizeof(rxbuf), 2);

	int tries = 4;
	while ((status == SS_ERR_TRY_AGAIN) && (tries--))
    {
		mxc_delay(1000000);
		status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), &rxbuf[0], sizeof(rxbuf), 2);
	}

	return status;
}

//void fifo_sample_size(int data_type, int *sample_size)
//{
//	*sample_size = 0;

//	if (data_type == SS_DATATYPE_RAW || data_type == SS_DATATYPE_BOTH) {
//		for (int i = 0; i < SS_MAX_SUPPORTED_SENSOR_NUM; i++) {
//			if (sensor_enabled_mode[i]) {
//				assert_msg(sensor_data_reqs[i], "no ss_data_req found for enabled sensor");
//				*sample_size += sensor_data_reqs[i]->data_size;
//			}
//		}
//	}

//	if (data_type == SS_DATATYPE_ALGO || data_type == SS_DATATYPE_BOTH) {
//		for (int i = 0; i < SS_MAX_SUPPORTED_ALGO_NUM; i++) {
//			if (algo_enabled_mode[i]) {
//				assert_msg(algo_data_reqs[i], "no ss_data_req found for enabled algo");
//				*sample_size += algo_data_reqs[i]->data_size;
//			}
//		}
//	}
//}



SS_STATUS num_avail_samples(uint8_t *num_samples)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_OUTPUTFIFO, SS_CMDIDX_OUT_NUMSAMPLES };
	uint8_t rxbuf[2] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), &rxbuf[0], sizeof(rxbuf), 2);

	if (status == SS_SUCCESS) 
    {
		*num_samples = rxbuf[1];
	}

	return status;
}

SS_STATUS read_fifo_data(int num_samples, int sample_size, uint8_t* databuf, int databuf_sz)
{
	int bytes_to_read = num_samples * sample_size + 1; //+1 for status byte
	uint8_t cmd_bytes[] = { SS_FAM_R_OUTPUTFIFO, SS_CMDIDX_READFIFO };

	SS_STATUS status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), databuf, bytes_to_read, 10);

	return status;
}

SS_STATUS read_fifo_status(uint8_t* stt)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_STATUS, SS_CMDIDX_STATUS };
	uint8_t rxbuf[2] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), &rxbuf[0], sizeof(rxbuf), 6);

	if (status == SS_SUCCESS) 
    {
		*stt = rxbuf[1];
	}

	return status;
}
/*
SS_STATUS set_sample_rate(int sample_rate)
{
	uint8_t cmd_bytes[5] = { 0x55, 0x02, 0x00, (uint8_t)(sample_rate >> 8), (uint8_t)sample_rate};
	SS_STATUS status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), 8);
	return status;
}

SS_STATUS set_spo2_calibration_cofficient(uint32_t a, uint32_t b, uint32_t c)
{
	uint8_t cmd_bytes[15];
    
    cmd_bytes[0] = 0x50;
    cmd_bytes[1] = 0x05;
    cmd_bytes[2] = 0x00;
    
    cmd_bytes[3] = (uint8_t)(a >> 24);
    cmd_bytes[4] = (uint8_t)(a >> 16);
    cmd_bytes[5] = (uint8_t)(a >> 8);
    cmd_bytes[6] = (uint8_t)(a >> 0);
    
    cmd_bytes[7] = (uint8_t)(a >> 24);
    cmd_bytes[8] = (uint8_t)(a >> 16);
    cmd_bytes[9] = (uint8_t)(a >> 8);
    cmd_bytes[10] = (uint8_t)(a >> 0);
    
    cmd_bytes[11] = (uint8_t)(a >> 24);
    cmd_bytes[12] = (uint8_t)(a >> 16);
    cmd_bytes[13] = (uint8_t)(a >> 8);
    cmd_bytes[14] = (uint8_t)(a >> 0);
    
	SS_STATUS status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), 40);
	return status;
}

SS_STATUS set_default_height(int height)
{
	uint8_t cmd_bytes[5] = { 0x55, 0x02, 0x07, (uint8_t)(height >> 8), (uint8_t)height};
	SS_STATUS status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), 8);
	return status;
}

SS_STATUS set_default_weight(int weight)
{
	uint8_t cmd_bytes[5] = { 0x55, 0x02, 0x08, (uint8_t)(weight >> 8), (uint8_t)weight};
	SS_STATUS status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), 8);
	return status;
}

SS_STATUS set_default_age(int age)
{
	uint8_t cmd_bytes[4] = { 0x55, 0x02, 0x09, (uint8_t)age};
	SS_STATUS status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), 8);
	return status;
}
*/

SS_STATUS set_init_hr(int init_hr)
{
	uint8_t cmd_bytes[5] = { 0x55, 0x02, 0x0a, (uint8_t)(init_hr >> 8), (uint8_t)init_hr};
	SS_STATUS status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), 8);
	return status;
}


void Reset_module(void)
{
    gpio_cfg_t Module_rst_pin;
    Module_rst_pin.port = PORT_5;
    Module_rst_pin.mask = PIN_6;
    Module_rst_pin.func = GPIO_FUNC_GPIO;
    Module_rst_pin.pad = GPIO_PAD_NORMAL;
    GPIO_Config(&Module_rst_pin);
    
    GPIO_OutClr(&Module_rst_pin);
    mxc_delay(20000);
    
    GPIO_OutSet(&Module_rst_pin);
    mxc_delay(10000);
}

void MFIO_init_output(uint8_t value)
{
    MFIO_pin.port = PORT_5;
    MFIO_pin.mask = PIN_4;
    MFIO_pin.func = GPIO_FUNC_GPIO;
    MFIO_pin.pad = GPIO_PAD_NORMAL;
    GPIO_Config(&MFIO_pin);
    
    if(value == 0)
    {
        GPIO_OutClr(&MFIO_pin);
    }
    else
    {
        GPIO_OutSet(&MFIO_pin);
    }
}

void MFIO_Input(void)
{
    //gpio_cfg_t MFIO_pin;
    MFIO_pin.port = PORT_5;
    MFIO_pin.mask = PIN_4;
    MFIO_pin.func = GPIO_FUNC_GPIO;
    MFIO_pin.pad = GPIO_PAD_INPUT;
    GPIO_Config(&MFIO_pin);
}

uint8_t MFIO_get_state(void)
{
    uint8_t st;
    //gpio_cfg_t MFIO_pin;
    MFIO_pin.port = PORT_5;
    MFIO_pin.mask = PIN_4;
    MFIO_pin.func = GPIO_FUNC_GPIO;
    MFIO_pin.pad = GPIO_PAD_INPUT;
    st = GPIO_InGet(&MFIO_pin);
    return st;
}


#define MXC_UARTn   MXC_UART_GET_UART(CONSOLE_UART)
#define UART_FIFO   MXC_UART_GET_FIFO(CONSOLE_UART)
void UART_PutChar1(const uint8_t data)
{
    // Wait for TXFIFO to not be full
    while ((MXC_UARTn->tx_fifo_ctrl & MXC_F_UART_TX_FIFO_CTRL_FIFO_ENTRY) == MXC_F_UART_TX_FIFO_CTRL_FIFO_ENTRY);
    MXC_UARTn->intfl = MXC_F_UART_INTFL_TX_DONE; // clear DONE flag for UART_PrepForSleep
    UART_FIFO->tx = data;
}
/*
void Get_Algo_Raw_White_Card_Red_LED(void)
{
	uint8_t temp;
    uint8_t stt;
    uint8_t buff[512];
    SS_STATUS status;        
    uint8_t cmd_bytes[10];
    uint32_t setcnt, setdone;

    cmd_bytes[0] = 0x50;                            //Turn off sensor AGC
    cmd_bytes[1] = 0x05;
    cmd_bytes[2] = 0x03; 
    cmd_bytes[3] = 0x00; 
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x10;                            //Set output mode to Sensor only
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x03; 
	status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x10;                            //set FIFO almost full
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x05; 
	status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x44;                            //Enable AFE (e.g. MAX86141) *** with Sensor Hub Samples
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x00; 
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x44;                            //Enable Accelerometer 
    cmd_bytes[1] = 0x04;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x01; 
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x52;                            //enable SPO2 algo
    cmd_bytes[1] = 0x05;
    cmd_bytes[2] = 0x01;
	status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    
    setdone = 0;
    setcnt = 0;
    while(1)
    {
        while(MFIO_get_state());
        
        setcnt++;
        if(setcnt >= 3)
        {
            setcnt = 21;
            if(setdone != 1)
            {
                setdone = 1;
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Integration time: 117us and ADC 1/2 range: 32uA
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x11; 
                cmd_bytes[3] = 0x3f;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set sample rate of MAX86141 to 100 Hz with 1 sample averaging 
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x12; 
                cmd_bytes[3] = 0x18;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel1 = LED1 (Green), Channel2 = LED2(IR)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x20; 
                cmd_bytes[3] = 0x21;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel3 = LED3 (Red)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x21; 
                cmd_bytes[3] = 0x03;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel4, 5, 6 = none
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x22; 
                cmd_bytes[3] = 0x00;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Green = 124mA (changes referred to the white card test configurations)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x23; 
                cmd_bytes[3] = 0x00;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 IR = 0
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x24; 
                cmd_bytes[3] = 0x00;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //variable, start Red LED current with 50mA and tune this based on PD current
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x25; 
                cmd_bytes[3] = 0x68;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 LED driver range to 124mA
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x2a; 
                cmd_bytes[3] = 0x3F;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
            }
        }
        
        temp = read_fifo_status(&stt);                      //AA 00 00 AB 00 08     read status
        while((temp != 0x00) || (stt != 0x08))
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        }
        mxc_delay(10);
        
        num_avail_samples(&temp);                           //AA 12 00 AB 00 0F    read number of sample in FIFO
        mxc_delay(10);
        
        read_fifo_data(temp, 33, buff, 0);                  //AA 12 01 AB 00 .......    dump FIFO data
        
        {
            uint8_t i, j, temp1;
            uint8_t ss[3];
            ss[2] = ' ';
            for(i = 0; i < temp; i++)
            {
                for(j = 0; j < 33; j++)
                {
                    temp1 = buff[j + 1 + (i * 33)];
                    
                    ss[0] = (temp1 >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = (temp1 & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                {
                    ss[0] = ((i + 1) >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = ((i + 1) & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                UART_PutChar1(13);
                UART_PutChar1(10);
            }
        }
    }
}

void Get_Algo_Raw_White_Card_Green_LED(void)
{
	uint8_t temp;
    uint8_t stt;
    uint8_t buff[512];
    SS_STATUS status;        
    uint8_t cmd_bytes[10];
    uint32_t setcnt, setdone;

    cmd_bytes[0] = 0x50;                    //Turn off Algorithm AEC
    cmd_bytes[1] = 0x02;
    cmd_bytes[2] = 0x0b; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x50;                    //Turn off Algorithm SCD.
    cmd_bytes[1] = 0x02;
    cmd_bytes[2] = 0x0c; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x10;                            //Set output mode to Sensor only
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x10;                            //set FIFO almost full
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x0f; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable AFE MAX86141
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable Accelerometer 
    cmd_bytes[1] = 0x04;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x52;                            //enable WHRM algo
    cmd_bytes[1] = 0x02;
    cmd_bytes[2] = 0x01;
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);

    setdone = 0;
    setcnt = 0;
    while(1)
    {
        while(MFIO_get_state());
        
        setcnt++;
        if(setcnt >= 10)
        {
            setcnt = 21;
            if(setdone == 0)
            {
                setdone = 1;
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Integration time: 117us and ADC 1/2 range: 32uA
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x11; 
                cmd_bytes[3] = 0x3f;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set sample rate of MAX86141 to 100 Hz with 1 sample averaging 
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x12; 
                cmd_bytes[3] = 0x1a;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set MAX86141 LED sequence LED3(red) -> LED2 (ir)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x20; 
                cmd_bytes[3] = 0x21;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set MAX86141 LED sequence LED3(red) -> LED2 (ir)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x21; 
                cmd_bytes[3] = 0x00;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set MAX86141 LED sequence LED3(red) -> LED2 (ir)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x22; 
                cmd_bytes[3] = 0x00;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set MAX86141 LED1(green) current to 50mA
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x23; 
                cmd_bytes[3] = 0x68;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set MAX86141 LED1(green) current to 50mA 
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x2a; 
                cmd_bytes[3] = 0x3F;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
            }
        }
        
        temp = read_fifo_status(&stt);                      //AA 00 00 AB 00 08     read status
        while((temp != 0x00) || (stt != 0x08))
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        }
        mxc_delay(10);
        
        num_avail_samples(&temp);                           //AA 12 00 AB 00 0F    read number of sample in FIFO
        mxc_delay(10);
        
        read_fifo_data(temp, 24, buff, 0);                  //AA 12 01 AB 00 .......    dump FIFO data
        
        {
            uint8_t i, j, temp1;
            uint8_t ss[3];
            ss[2] = ' ';
            
            for(i = 0; i < temp; i++)
            {
                for(j = 0; j < 24; j++)
                {
                    temp1 = buff[j + 1 + (i * 24)];
                    
                    ss[0] = (temp1 >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = (temp1 & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                UART_PutChar1(13);
                UART_PutChar1(10);
            }
        }
    }
}


void Get_Algo_Raw_CrossTalk(void)
{
	uint8_t temp;
    uint8_t stt;
    uint8_t buff[512];
    SS_STATUS status;        
    uint8_t cmd_bytes[10];
    uint32_t setcnt, setdone;

    cmd_bytes[0] = 0x50;                    //Turn off Algorithm AEC
    cmd_bytes[1] = 0x02;
    cmd_bytes[2] = 0x0b; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x50;                    //Turn off Algorithm SCD.
    cmd_bytes[1] = 0x02;
    cmd_bytes[2] = 0x0c; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x10;                            //Set output mode to Sensor only
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x10;                            //set FIFO almost full
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x0f; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable AFE MAX86141
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable Accelerometer 
    cmd_bytes[1] = 0x04;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x52;                            //enable WHRM algo
    cmd_bytes[1] = 0x02;
    cmd_bytes[2] = 0x01;
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);

    setdone = 0;
    setcnt = 0;
    while(1)
    {
        while(MFIO_get_state());
        
        setcnt++;
        if(setcnt >= 10)
        {
            setcnt = 21;
            if(setdone == 0)
            {
                setdone = 1;
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Integration time: 117us and ADC 1/2 range: 32uA
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x11; 
                cmd_bytes[3] = 0x3f;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set sample rate of MAX86141 to 100 Hz with 1 sample averaging 
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x12; 
                cmd_bytes[3] = 0x18;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel1 = LED1 (Green), Channel2 = LED2(IR)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x20; 
                cmd_bytes[3] = 0x21;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel3 = LED3 (Red)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x21; 
                cmd_bytes[3] = 0x03;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel4, 5, 6 = none
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x22; 
                cmd_bytes[3] = 0x00;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set MAX86141 Green = 124mA (changes referred to the white card test configurations)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x23; 
                cmd_bytes[3] = 0xff;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set MAX86141 IR = 0
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x24; 
                cmd_bytes[3] = 0x00;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set MAX86141 Red = 124mA
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x25; 
                cmd_bytes[3] = 0xff;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set MAX86141 LED driver range to 124mA
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x2a; 
                cmd_bytes[3] = 0x3F;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
            }
        }
        
        temp = read_fifo_status(&stt);                      //AA 00 00 AB 00 08     read status
        while((temp != 0x00) || (stt != 0x08))
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        }
        mxc_delay(10);
        
        num_avail_samples(&temp);                           //AA 12 00 AB 00 0F    read number of sample in FIFO
        mxc_delay(10);
        
        read_fifo_data(temp, 24, buff, 0);                  //AA 12 01 AB 00 .......    dump FIFO data
        
        {
            uint8_t i, j, temp1;
            uint8_t ss[3];
            ss[2] = ' ';
            
            for(i = 0; i < temp; i++)
            {
                for(j = 0; j < 24; j++)
                {
                    temp1 = buff[j + 1 + (i * 24)];
                    
                    ss[0] = (temp1 >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = (temp1 & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                UART_PutChar1(13);
                UART_PutChar1(10);
            }
        }
    }
}

void Get_Pure_Raw_White_Card_Red_LED(void)
{
	uint8_t temp;
    uint8_t stt;
    uint8_t buff[512];
    SS_STATUS status;        
    uint8_t cmd_bytes[10];

    cmd_bytes[0] = 0x10;                            //Set output mode to Sensor only
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x10;                            //set FIFO almost full
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x05; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable AFE MAX86141
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 30);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

//    cmd_bytes[0] = 0x44;                            //Enable Accelerometer 
//    cmd_bytes[1] = 0x04;
//    cmd_bytes[2] = 0x01; 
//    cmd_bytes[3] = 0x00; 
//    status = write_cmd(&cmd_bytes[0], 4, 30);
//    if(SS_SUCCESS != status) while(1);
//    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                            //Set MAX86141 Integration time: 117us and ADC 1/2 range: 32uA
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x11; 
    cmd_bytes[3] = 0x3f;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                            //Set sample rate of MAX86141 to 100 Hz with 1 sample averaging 
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x12; 
    cmd_bytes[3] = 0x18;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel1 = LED1 (Green), Channel2 = LED2(IR)
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x20; 
    cmd_bytes[3] = 0x21;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

//    cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel3 = LED3 (Red)
//    cmd_bytes[1] = 0x00;
//    cmd_bytes[2] = 0x21; 
//    cmd_bytes[3] = 0x03;   
//    status = write_cmd(&cmd_bytes[0], 4, 10);
//    if(SS_SUCCESS != status) while(1);

//    cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel4, 5, 6 = none
//    cmd_bytes[1] = 0x00;
//    cmd_bytes[2] = 0x22; 
//    cmd_bytes[3] = 0x00;   
//    status = write_cmd(&cmd_bytes[0], 4, 10);
//    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Green = 0
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x23; 
    cmd_bytes[3] = 0x00;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 IR = 0
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x24;
    cmd_bytes[3] = 0x00;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //variable, start Red LED current with 50mA and tune this based on PD current
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x25; 
    cmd_bytes[3] = 0x7f;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 LED driver range to 124mA
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x2a; 
    cmd_bytes[3] = 0x3f;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 FIFO Interrupt threshold to 124
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x09; 
    cmd_bytes[3] = 0x7c;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 FIFO configuration
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x0a; 
    cmd_bytes[3] = 0x0e;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Interrupt configuration
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x02; 
    cmd_bytes[3] = 0xc6;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    while(1)
    {
        //while(MFIO_get_state());
        
        temp = read_fifo_status(&stt);                      //AA 00 00 AB 00 08     read status
        while((temp != 0x00) || (stt != 0x08))
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        }
        
        num_avail_samples(&temp);                           //AA 12 00 AB 00 0F    read number of sample in FIFO
        
        read_fifo_data(temp, 24, buff, 0);                  //AA 12 01 AB 00 .......    dump FIFO data
        
        {
            uint8_t i, j, temp1;
            uint8_t ss[3];
            ss[2] = ' ';
            
            for(i = 0; i < temp; i++)
            {
                for(j = 0; j < 24; j++)
                {
                    temp1 = buff[j + 1 + (i * 24)];
                    
                    ss[0] = (temp1 >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = (temp1 & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                UART_PutChar1(13);
                UART_PutChar1(10);
            }
        }
    }
}

void Get_Pure_Raw_White_Card_Red_LED1(void)
{
	uint8_t temp;
    uint8_t stt;
    uint8_t buff[512];
    SS_STATUS status;        
    uint8_t cmd_bytes[10];

    cmd_bytes[0] = 0x10;                            //Set output mode to Sensor only
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x10;                            //set FIFO almost full
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x05; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable AFE MAX86141
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 30);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

//    cmd_bytes[0] = 0x44;                            //Enable Accelerometer 
//    cmd_bytes[1] = 0x04;
//    cmd_bytes[2] = 0x01; 
//    cmd_bytes[3] = 0x00; 
//    status = write_cmd(&cmd_bytes[0], 4, 30);
//    if(SS_SUCCESS != status) while(1);
//    mxc_delay(200);

    cmd_bytes[0] = 0x40;                            //Set sample rate of MAX86141 to 100 Hz with 1 sample averaging 
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x12; 
    cmd_bytes[3] = 0x18;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    
    cmd_bytes[0] = 0x40;                            //Set MAX86141 Integration time: 117us and ADC 1/2 range: 32uA
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x11; 
    cmd_bytes[3] = 0x3f;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Green = 0
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x23; 
    cmd_bytes[3] = 0x00;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 IR = 0
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x24;
    cmd_bytes[3] = 0x00;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //variable, start Red LED current with 50mA and tune this based on PD current
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x25; 
    cmd_bytes[3] = 0x7f;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 LED driver range to 124mA
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x2a; 
    cmd_bytes[3] = 0x3c;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel1 = LED1 (Green), Channel2 = LED2(IR)
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x20; 
    cmd_bytes[3] = 0x23;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    
//    {
//        rbuff[0] = 0;
//        rbuff[1] = 0;
//        rbuff[2] = 0;
//        rbuff[3] = 0;
//        cmd_bytes[0] = 0x41;                    //Set MAX86141 Channel3 = LED3 (Red)
//        cmd_bytes[1] = 0x00;
//        cmd_bytes[2] = 0x20;  
//        status = read_cmd(&cmd_bytes[0], 3, rbuff, 3, 100);
//        if(SS_SUCCESS != status) while(1);
//    }

//    cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel3 = LED3 (Red)
//    cmd_bytes[1] = 0x00;
//    cmd_bytes[2] = 0x21; 
//    cmd_bytes[3] = 0x03;   
//    status = write_cmd(&cmd_bytes[0], 4, 10);
//    if(SS_SUCCESS != status) while(1);

//    {
//        rbuff[0] = 0;
//        rbuff[1] = 0;
//        rbuff[2] = 0;
//        rbuff[3] = 0;
//        cmd_bytes[0] = 0x41;                    //Set MAX86141 Channel3 = LED3 (Red)
//        cmd_bytes[1] = 0x00;
//        cmd_bytes[2] = 0x21;  
//        status = read_cmd(&cmd_bytes[0], 3, rbuff, 3, 100);
//        if(SS_SUCCESS != status) while(1);
//    }

//        cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel4, 5, 6 = none
//        cmd_bytes[1] = 0x00;
//        cmd_bytes[2] = 0x22; 
//        cmd_bytes[3] = 0x00;   
//        status = write_cmd(&cmd_bytes[0], 4, 10);
//        if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 FIFO Interrupt threshold to 124
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x09; 
    cmd_bytes[3] = 0x7c;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 FIFO configuration
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x0a; 
    cmd_bytes[3] = 0x0e;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Interrupt configuration
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x02; 
    cmd_bytes[3] = 0xc6;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    while(1)
    {
        while(MFIO_get_state());
        
        temp = read_fifo_status(&stt);                      //AA 00 00 AB 00 08     read status
        while((temp != 0x00) || (stt != 0x08))
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        }
        
        num_avail_samples(&temp);                           //AA 12 00 AB 00 0F    read number of sample in FIFO
        
        read_fifo_data(temp, 24, buff, 0);                  //AA 12 01 AB 00 .......    dump FIFO data
        
        {
            uint8_t i, j, temp1;
            uint8_t ss[3];
            ss[2] = ' ';
            
            for(i = 0; i < temp; i++)
            {
                for(j = 0; j < 24; j++)
                {
                    temp1 = buff[j + 1 + (i * 24)];
                    
                    ss[0] = (temp1 >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = (temp1 & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                UART_PutChar1(13);
                UART_PutChar1(10);
            }
        }
    }
}


void Get_Pure_Raw_White_Card_Red_LED2(void)
{
	uint8_t temp;
    uint8_t stt;
    uint8_t buff[512];
    SS_STATUS status;        
    uint8_t cmd_bytes[10];

    cmd_bytes[0] = 0x10;                            //Set output mode to Sensor only
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x10;                            //set FIFO almost full
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x05; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable AFE MAX86141
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 30);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable Accelerometer 
    cmd_bytes[1] = 0x04;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 30);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x40;                            //Set sample rate of MAX86141 to 100 Hz with 1 sample averaging 
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x12; 
    cmd_bytes[3] = 0x18;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    
    cmd_bytes[0] = 0x40;                            //Set MAX86141 Integration time: 117us and ADC 1/2 range: 32uA
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x11; 
    cmd_bytes[3] = 0x3f;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Green = 0
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x23; 
    cmd_bytes[3] = 0;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 IR = 0
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x24;
    cmd_bytes[3] = 70;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //variable, start Red LED current with 50mA and tune this based on PD current
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x25; 
    cmd_bytes[3] = 58;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 LED driver range to 124mA
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x2a;
    cmd_bytes[3] = 0x3f;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel1 = LED1 (Green), Channel2 = LED2(IR)
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x20;
    cmd_bytes[3] = 0x23;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

//    cmd_bytes[0] = 0x40;                    //LED3 = red
//    cmd_bytes[1] = 0x00;
//    cmd_bytes[2] = 0x21; 
//    cmd_bytes[3] = 0x00;   
//    status = write_cmd(&cmd_bytes[0], 4, 10);
//    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 FIFO Interrupt threshold to 124
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x09;
    cmd_bytes[3] = 0x7c;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 FIFO configuration
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x0a;
    cmd_bytes[3] = 0x0e;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Interrupt configuration
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x02;
    cmd_bytes[3] = 0xc6;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    while(1)
    {
        while(MFIO_get_state());
        
        temp = read_fifo_status(&stt);                      //AA 00 00 AB 00 08     read status
        while((temp != 0x00) || (stt != 0x08))
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        }
        
        num_avail_samples(&temp);                           //AA 12 00 AB 00 0F    read number of sample in FIFO
        
        read_fifo_data(temp, 24, buff, 0);                  //AA 12 01 AB 00 .......    dump FIFO data
        
        {
            uint8_t i, j, temp1;
            uint8_t ss[3];
            ss[2] = ' ';
            
            for(i = 0; i < temp; i++)
            {
                for(j = 0; j < 24; j++)
                {
                    temp1 = buff[j + 1 + (i * 24)];
                    
                    ss[0] = (temp1 >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = (temp1 & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                UART_PutChar1(13);
                UART_PutChar1(10);
            }
        }
    }
}


void Get_Pure_Raw_White_Card_Green_LED(void)
{
	uint8_t temp;
    uint8_t stt;
    uint8_t buff[512];
    SS_STATUS status;        
    uint8_t cmd_bytes[10];

    cmd_bytes[0] = 0x10;                            //Set output mode to Sensor only
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x10;                            //set FIFO almost full
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x08; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable AFE MAX86141
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 30);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable Accelerometer 
    cmd_bytes[1] = 0x04;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 30);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                            //Set MAX86141 Integration time: 117us and ADC 1/2 range: 32uA
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x11; 
    cmd_bytes[3] = 0x3f;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                            //Set sample rate of MAX86141 to 100 Hz with 1 sample averaging 
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x12; 
    cmd_bytes[3] = 0x18;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel1 = LED1 (Green), Channel2 = LED2(IR)
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x20; 
    cmd_bytes[3] = 0x21;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel3 = LED3 (Red)
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x21; 
    cmd_bytes[3] = 0x03;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel4, 5, 6 = none
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x22;
    cmd_bytes[3] = 0x00;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Green = 0
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x23; 
    cmd_bytes[3] = 0x30;
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 IR = 0
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x24; 
    cmd_bytes[3] = 0x00;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //variable, start Red LED current with 50mA and tune this based on PD current
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x25; 
    cmd_bytes[3] = 0x00;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 LED driver range to 124mA
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x2a; 
    cmd_bytes[3] = 0x3f;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 FIFO Interrupt threshold to 124
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x09; 
    cmd_bytes[3] = 0x7c;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 FIFO configuration
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x0a; 
    cmd_bytes[3] = 0x0e;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Interrupt configuration
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x02; 
    cmd_bytes[3] = 0xc6;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    while(1)
    {
        while(MFIO_get_state());
        
        temp = read_fifo_status(&stt);                      //AA 00 00 AB 00 08     read status
        while((temp != 0x00) || (stt != 0x08))
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        }
        
        num_avail_samples(&temp);                           //AA 12 00 AB 00 0F    read number of sample in FIFO
        
        read_fifo_data(temp, 24, buff, 0);                  //AA 12 01 AB 00 .......    dump FIFO data
        
        {
            uint8_t i, j, temp1;
            uint8_t ss[3];
            ss[2] = ' ';
            
            for(i = 0; i < temp; i++)
            {
                for(j = 0; j < 24; j++)
                {
                    temp1 = buff[j + 1 + (i * 24)];
                    
                    ss[0] = (temp1 >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = (temp1 & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                UART_PutChar1(13);
                UART_PutChar1(10);
            }
        }
    }
}

void Get_Pure_Raw_CrossTalk(void)
{
	uint8_t temp;
    uint8_t stt;
    uint8_t buff[512];
    SS_STATUS status;        
    uint8_t cmd_bytes[10];

    cmd_bytes[0] = 0x10;                            //Set output mode to Sensor only
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x10;                            //set FIFO almost full
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x08; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable AFE MAX86141
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 30);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable Accelerometer 
    cmd_bytes[1] = 0x04;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 30);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                            //Set MAX86141 Integration time: 117us and ADC 1/2 range: 32uA
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x11; 
    cmd_bytes[3] = 0x3f;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                            //Set sample rate of MAX86141 to 100 Hz with 1 sample averaging 
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x12; 
    cmd_bytes[3] = 0x18;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel1 = LED1 (Green), Channel2 = LED2(IR)
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x20; 
    cmd_bytes[3] = 0x21;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel3 = LED3 (Red)
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x21; 
    cmd_bytes[3] = 0x03;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel4, 5, 6 = none
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x22; 
    cmd_bytes[3] = 0x00;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Green = 124mA (changes referred to the white card test configurations)
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x23; 
    cmd_bytes[3] = 0xff;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 IR = 0
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x24; 
    cmd_bytes[3] = 0x00;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Red = 124mA
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x25; 
    cmd_bytes[3] = 0xff;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 LED driver range to 124mA
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x2a; 
    cmd_bytes[3] = 0x3f;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 FIFO Interrupt threshold to 124
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x09; 
    cmd_bytes[3] = 0x7c;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 FIFO configuration
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x0a; 
    cmd_bytes[3] = 0x0e;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    cmd_bytes[0] = 0x40;                    //Set MAX86141 Interrupt configuration
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x02; 
    cmd_bytes[3] = 0xc6;   
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);

    while(1)
    {
        while(MFIO_get_state());
        
        temp = read_fifo_status(&stt);                      //AA 00 00 AB 00 08     read status
        while((temp != 0x00) || (stt != 0x08))
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        }
        
        num_avail_samples(&temp);                           //AA 12 00 AB 00 0F    read number of sample in FIFO
        
        read_fifo_data(temp, 24, buff, 0);                  //AA 12 01 AB 00 .......    dump FIFO data
        
        {
            uint8_t i, j, temp1;
            uint8_t ss[3];
            ss[2] = ' ';
            
            for(i = 0; i < temp; i++)
            {
                for(j = 0; j < 24; j++)
                {
                    temp1 = buff[j + 1 + (i * 24)];
                    
                    ss[0] = (temp1 >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = (temp1 & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                UART_PutChar1(13);
                UART_PutChar1(10);
            }
        }
    }
}
*/
uint32_t g_fw_version;

void WHRM_Algorithm(void)
{
	uint8_t temp;
    uint8_t stt;
    uint8_t buff[512];
    SS_STATUS status;        
    uint8_t cmd_bytes[10];

    cmd_bytes[0] = 0x10;                            //Set output mode to Sensor only
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x03; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x10;                            //set FIFO almost full
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x08; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Enable AFE MAX86141
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 30);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x44;                            //Host Accelerometer 
    cmd_bytes[1] = 0x04;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 4, 30);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x52;                            //Set MAX86174 Integration time: 117us and ADC 1/2 range: 32uA
    cmd_bytes[1] = 0x02;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 30);
    if(SS_SUCCESS != status) while(1);

    while(1)
    {
        while(MFIO_get_state());
        
        temp = read_fifo_status(&stt);                      //AA 00 00 AB 00 08     read status
        while((temp != 0x00) || (stt != 0x08))
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        }
        
        num_avail_samples(&temp);                           //AA 12 00 AB 00 0F    read number of sample in FIFO
        
        read_fifo_data(temp, 30, buff, 0);                  //AA 12 01 AB 00 .......    dump FIFO data
        
        {
            uint8_t i, j, temp1;
            uint8_t ss[3];
            ss[2] = ' ';
            
            for(i = 0; i < temp; i++)
            {
                for(j = 0; j < 30; j++)
                {
                    temp1 = buff[j + 1 + (i * 30)];
                    
                    ss[0] = (temp1 >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = (temp1 & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                UART_PutChar1(13);
                UART_PutChar1(10);
            }
        }
    }
}

void Get_Algo_Raw_White_Card_Red_LED_new(void)
{
	uint8_t temp;
    uint8_t stt;
    uint8_t buff[512];
    SS_STATUS status;        
    uint8_t cmd_bytes[10];
    uint32_t setcnt, setdone;

    cmd_bytes[0] = 0x10;                            //Set the output mode to Sensor Only.
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
	status = write_cmd(&cmd_bytes[0], 3, 2);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);
    
    cmd_bytes[0] = 0x10;                            //Set the sensor hub interrupt threshold.
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x01; 
	status = write_cmd(&cmd_bytes[0], 3, 2);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);
    
    cmd_bytes[0] = 0x10;                            //Set the report rate to be one report per every sensor sample.
    cmd_bytes[1] = 0x02;
    cmd_bytes[2] = 0x01; 
	status = write_cmd(&cmd_bytes[0], 3, 2);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);
    
//    cmd_bytes[0] = 0x44;                            //Set the report rate to be one report per every sensor sample.
//    cmd_bytes[1] = 0x00;
//    cmd_bytes[2] = 0x01; 
//    cmd_bytes[3] = 0x00; 
//    status = write_cmd(&cmd_bytes[0], 4, 100);
//    if(SS_SUCCESS != status) while(1);
//    mxc_delay(100);

//    cmd_bytes[0] = 0x44;                            //Set the report rate to be one report per every sensor sample.
//    cmd_bytes[1] = 0x04;
//    cmd_bytes[2] = 0x01; 
//    cmd_bytes[3] = 0x00; 
//    status = write_cmd(&cmd_bytes[0], 4, 100);
//    if(SS_SUCCESS != status) while(1);
//    mxc_delay(100);
    
    cmd_bytes[0] = 0x50;                            //Set the algorithm operation mode to Continuous HRM and Continuous SpO2 or as needed.
    cmd_bytes[1] = 0x07;
    cmd_bytes[2] = 0x0A; 
    cmd_bytes[3] = 0x00;  
    //cmd_bytes[3] = 0x06; 
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);
    
    cmd_bytes[0] = 0x50;                            //Disable Automatic Exposure Control (AEC).
    cmd_bytes[1] = 0x07;
    cmd_bytes[2] = 0x0B; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);

    cmd_bytes[0] = 0x50;                            //Disable Auto PD Current Calculation.
    cmd_bytes[1] = 0x07;
    cmd_bytes[2] = 0x12; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);

    cmd_bytes[0] = 0x50;                            //Disable SCD.
    cmd_bytes[1] = 0x07;
    cmd_bytes[2] = 0x0C; 
    cmd_bytes[3] = 0x00; 
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);
    
    cmd_bytes[0] = 0x52;                            //Enable the wearable heart-rate monitor (WHRM) and SpO2 algorithm.
    cmd_bytes[1] = 0x07;
    cmd_bytes[2] = 0x01; 
	status = write_cmd(&cmd_bytes[0], 3, 600);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);
    
    setdone = 0;
    setcnt = 0;
    while(1)
    {
        //while(MFIO_get_state());
        mxc_delay(40000);
        
        setcnt++;
        if(setcnt >= 5)
        {
            setcnt = 21;
            if(setdone != 1)
            {
                setdone = 1;
                
//                cmd_bytes[0] = 0x40;                    //Set MAX86141 Integration time: 117us and ADC 1/2 range: 32uA
//                cmd_bytes[1] = 0x00;
//                cmd_bytes[2] = 0x11; 
//                cmd_bytes[3] = 0x3f;   
//                status = write_cmd(&cmd_bytes[0], 4, 10);
//                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set sample rate of MAX86141 to 100 Hz with 1 sample averaging 
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x12; 
                cmd_bytes[3] = 0x2c;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
//                cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel1 = LED1 (Green), Channel2 = LED2(IR)
//                cmd_bytes[1] = 0x00;
//                cmd_bytes[2] = 0x20; 
//                cmd_bytes[3] = 0x21;   
//                status = write_cmd(&cmd_bytes[0], 4, 10);
//                if(SS_SUCCESS != status) while(1);
//                
//                cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel3 = LED3 (Red)
//                cmd_bytes[1] = 0x00;
//                cmd_bytes[2] = 0x21; 
//                cmd_bytes[3] = 0x03;   
//                status = write_cmd(&cmd_bytes[0], 4, 10);
//                if(SS_SUCCESS != status) while(1);
//                
//                cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel4, 5, 6 = none
//                cmd_bytes[1] = 0x00;
//                cmd_bytes[2] = 0x22; 
//                cmd_bytes[3] = 0x00;   
//                status = write_cmd(&cmd_bytes[0], 4, 10);
//                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Green = 124mA (changes referred to the white card test configurations)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x23; 
                cmd_bytes[3] = 0x1f;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 IR = 0
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x24; 
                cmd_bytes[3] = 0x00;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //variable, start Red LED current with 50mA and tune this based on PD current
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x25; 
                cmd_bytes[3] = 0x00;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
//                cmd_bytes[0] = 0x40;                    //Set MAX86141 LED driver range to 124mA
//                cmd_bytes[1] = 0x00;
//                cmd_bytes[2] = 0x2a; 
//                cmd_bytes[3] = 0x3F;   
//                status = write_cmd(&cmd_bytes[0], 4, 10);
//                if(SS_SUCCESS != status) while(1);
            }
        }
        
        temp = read_fifo_status(&stt);                      //AA 00 00 AB 00 08     read status
        while((temp != 0x00)/* || (stt != 0x08)*/)
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        }
        mxc_delay(10);
        
        num_avail_samples(&temp);                           //AA 12 00 AB 00 0F    read number of sample in FIFO
        mxc_delay(10);
        
        read_fifo_data(temp, 24, buff, 0);                  //AA 12 01 AB 00 .......    dump FIFO data
        
        {
            uint8_t i, j, temp1;
            uint8_t ss[3];
            ss[2] = ' ';
            for(i = 0; i < temp; i++)
            {
                for(j = 0; j < 24; j++)
                {
                    temp1 = buff[j + 1 + (i * 24)];
                    
                    ss[0] = (temp1 >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = (temp1 & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                {
                    ss[0] = ((i + 1) >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = ((i + 1) & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                UART_PutChar1(13);
                UART_PutChar1(10);
            }
        }
    }
}


void Get_raw_data(uint8_t mode)
{
	uint8_t temp;
    uint8_t stt;
    uint8_t buff[512];
    SS_STATUS status;        
    uint8_t cmd_bytes[10];
    uint32_t setcnt, setdone;

    cmd_bytes[0] = 0x10;                            //Set the output mode to Sensor Only.
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
	status = write_cmd(&cmd_bytes[0], 3, 2);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);
    
    cmd_bytes[0] = 0x10;                            //Set the sensor hub interrupt threshold.
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x01; 
	status = write_cmd(&cmd_bytes[0], 3, 2);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);
    
    cmd_bytes[0] = 0x10;                            //Set the report rate to be one report per every sensor sample.
    cmd_bytes[1] = 0x02;
    cmd_bytes[2] = 0x01; 
	status = write_cmd(&cmd_bytes[0], 3, 2);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);
    
    if(mode == 0)                                   //pure raw data
    {
        cmd_bytes[0] = 0x44;                            //Set the report rate to be one report per every sensor sample.
        cmd_bytes[1] = 0x00;
        cmd_bytes[2] = 0x01; 
        cmd_bytes[3] = 0x00; 
        status = write_cmd(&cmd_bytes[0], 4, 100);
        if(SS_SUCCESS != status) while(1);
        mxc_delay(100);

        cmd_bytes[0] = 0x44;                            //Set the report rate to be one report per every sensor sample.
        cmd_bytes[1] = 0x04;
        cmd_bytes[2] = 0x01; 
        cmd_bytes[3] = 0x00; 
        status = write_cmd(&cmd_bytes[0], 4, 100);
        if(SS_SUCCESS != status) while(1);
        mxc_delay(100);
    }
    else
    {
        cmd_bytes[0] = 0x50;                            //Set the algorithm operation mode to Continuous HRM and Continuous SpO2 or as needed.
        cmd_bytes[1] = 0x07;
        cmd_bytes[2] = 0x0A; 
        cmd_bytes[3] = 0x00;  
        status = write_cmd(&cmd_bytes[0], 4, 10);
        if(SS_SUCCESS != status) while(1);
        mxc_delay(100);
        
        cmd_bytes[0] = 0x50;                            //Disable Automatic Exposure Control (AEC).
        cmd_bytes[1] = 0x07;
        cmd_bytes[2] = 0x0B; 
        cmd_bytes[3] = 0x00; 
        status = write_cmd(&cmd_bytes[0], 4, 10);
        if(SS_SUCCESS != status) while(1);
        mxc_delay(100);

        cmd_bytes[0] = 0x50;                            //Disable Auto PD Current Calculation.
        cmd_bytes[1] = 0x07;
        cmd_bytes[2] = 0x12; 
        cmd_bytes[3] = 0x00; 
        status = write_cmd(&cmd_bytes[0], 4, 10);
        if(SS_SUCCESS != status) while(1);
        mxc_delay(100);

        cmd_bytes[0] = 0x50;                            //Disable SCD.
        cmd_bytes[1] = 0x07;
        cmd_bytes[2] = 0x0C; 
        cmd_bytes[3] = 0x00; 
        status = write_cmd(&cmd_bytes[0], 4, 10);
        if(SS_SUCCESS != status) while(1);
        mxc_delay(100);
        
        cmd_bytes[0] = 0x52;                            //Enable the wearable heart-rate monitor (WHRM) and SpO2 algorithm.
        cmd_bytes[1] = 0x07;
        cmd_bytes[2] = 0x01; 
        status = write_cmd(&cmd_bytes[0], 3, 600);
        if(SS_SUCCESS != status) while(1);
        mxc_delay(100);
    }
    
    setdone = 0;
    setcnt = 0;
    while(1)
    {
        mxc_delay(40000);
        
        setcnt++;
        if(setcnt >= 5)
        {
            setcnt = 21;
            if(setdone != 1)
            {
                setdone = 1;
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Integration time: 117us and ADC 1/2 range: 32uA
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x11; 
                cmd_bytes[3] = 0x3f;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);

                cmd_bytes[0] = 0x40;                    //Set sample rate of MAX86141 to 100 Hz with 1 sample averaging 
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x12; 
                cmd_bytes[3] = 0x2c;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel1 = LED1 (Green), Channel2 = LED2(IR)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x20; 
                cmd_bytes[3] = 0x21;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel3 = LED3 (Red)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x21; 
                cmd_bytes[3] = 0x03;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Channel4, 5, 6 = none
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x22; 
                cmd_bytes[3] = 0x00;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 Green = 124mA (changes referred to the white card test configurations)
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x23; 
                cmd_bytes[3] = 0x00;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 IR = 0
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x24; 
                cmd_bytes[3] = 0x20;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //variable, start Red LED current with 50mA and tune this based on PD current
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x25; 
                cmd_bytes[3] = 0x00;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
                
                cmd_bytes[0] = 0x40;                    //Set MAX86141 LED driver range to 124mA
                cmd_bytes[1] = 0x00;
                cmd_bytes[2] = 0x2a; 
                cmd_bytes[3] = 0x3F;   
                status = write_cmd(&cmd_bytes[0], 4, 10);
                if(SS_SUCCESS != status) while(1);
            }
        }
        
        temp = read_fifo_status(&stt);                      //AA 00 00 AB 00 08     read status
        while((temp != 0x00)/* || (stt != 0x08)*/)
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        }
        mxc_delay(10);
        
        num_avail_samples(&temp);                           //AA 12 00 AB 00 0F    read number of sample in FIFO
        mxc_delay(10);
        
        read_fifo_data(temp, 24, buff, 0);                  //AA 12 01 AB 00 .......    dump FIFO data
        
        {
            uint8_t i, j, temp1;
            uint8_t ss[3];
            ss[2] = ' ';
            for(i = 0; i < temp; i++)
            {
                for(j = 0; j < 24; j++)
                {
                    temp1 = buff[j + 1 + (i * 24)];
                    
                    ss[0] = (temp1 >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = (temp1 & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                {
                    ss[0] = ((i + 1) >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = ((i + 1) & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                UART_PutChar1(13);
                UART_PutChar1(10);
            }
        }
    }
}


void AEC_Mode_Algorithm(void)
{
	uint8_t temp;
    uint8_t stt;
    uint8_t buff[512];
    SS_STATUS status;        
    uint8_t cmd_bytes[10];

    cmd_bytes[0] = 0x10;                            //Set output mode to Sensor only
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x03; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x10;                            //set FIFO almost full
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);

    cmd_bytes[0] = 0x10;                            //set FIFO almost full
    cmd_bytes[1] = 0x02;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x50;                            //Set the algorithm operation mode to Continuous HRM and Continuous SpO2 or as needed.
    cmd_bytes[1] = 0x07;
    cmd_bytes[2] = 0x0A; 
    cmd_bytes[3] = 0x00;  
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);
        
    cmd_bytes[0] = 0x50;                            //Disable Automatic Exposure Control (AEC).
    cmd_bytes[1] = 0x07;
    cmd_bytes[2] = 0x0B; 
    cmd_bytes[3] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);

    cmd_bytes[0] = 0x50;                            //Disable Auto PD Current Calculation.
    cmd_bytes[1] = 0x07;
    cmd_bytes[2] = 0x12; 
    cmd_bytes[3] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);

    cmd_bytes[0] = 0x50;                            //Disable SCD.
    cmd_bytes[1] = 0x07;
    cmd_bytes[2] = 0x0C; 
    cmd_bytes[3] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);
        
    cmd_bytes[0] = 0x52;                            //Enable the wearable heart-rate monitor (WHRM) and SpO2 algorithm.
    cmd_bytes[1] = 0x07;
    cmd_bytes[2] = 0x01; 
    status = write_cmd(&cmd_bytes[0], 3, 600);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(100);

    while(1)
    {
        //while(MFIO_get_state());
        mxc_delay(40);
        
        temp = read_fifo_status(&stt);                      //AA 00 00 AB 00 08     read status
        while((temp != 0x00) || (stt != 0x08))
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        }
        
        num_avail_samples(&temp);                           //AA 12 00 AB 00 0F    read number of sample in FIFO
        
        read_fifo_data(temp, 44, buff, 0);                  //AA 12 01 AB 00 .......    dump FIFO data
        
        {
            uint8_t i, j, temp1;
            uint8_t ss[3];
            ss[2] = ' ';
            
            for(i = 0; i < temp; i++)
            {
                for(j = 0; j < 44; j++)
                {
                    temp1 = buff[j + 1 + (i * 44)];
                    
                    ss[0] = (temp1 >> 4);
                    if(ss[0] <= 9)
                    {
                        ss[0] += '0';
                    }
                    else 
                    {
                        ss[0] += 55;
                    }
                    
                    ss[1] = (temp1 & 0x0f);
                    if(ss[1] <= 9)
                    {
                        ss[1] += '0';
                    }
                    else 
                    {
                        ss[1] += 55;
                    }
                    
                    UART_PutChar1(ss[0]);
                    UART_PutChar1(ss[1]);
                    UART_PutChar1(ss[2]);
                }
                UART_PutChar1(13);
                UART_PutChar1(10);
            }
        }
    }
}

uint8_t tt;
void pbcallback(void *pb)
{
    if(tt == 0)
    {
        tt = 1;
        MAX20303_led0on(0); //bule
    }
    else
    {
        tt = 0;
        MAX20303_led0on(1); //bule
    }
}

uint8_t value[10];




#ifdef SCDSM_TEST



/*------------------------ SCDSM Updates Start --------------------------*/

const int between_dummy_wait_us    = 45;
const int after_dummy_wait_us      = 175;
#define POLL_PERIOD_25MS   (1)
#define POLL_PERIOD_1000MS (25)

typedef uint8_t bool;
#define false ((uint8_t)0)
#define true  ((uint8_t)1)

static void i_mxc_delay(int us)
{
    TMR_Delay(MXC_TMR5, us);
}

/*
void me11_i2c_wakeup_call(void){

	uint8_t tx_buf[1];
	tx_buf[0] = 0x00;
	I2CM_Write(I2C_MASTER, 0x00, NULL, 0, &tx_buf[0], 1);
	i_mxc_delay(between_dummy_wait_us);
	I2CM_Write(I2C_MASTER, 0x00, NULL, 0, &tx_buf[0], 1);
	i_mxc_delay(between_dummy_wait_us);
	I2CM_Write(I2C_MASTER, 0x00, NULL, 0, &tx_buf[0], 1);
	i_mxc_delay(between_dummy_wait_us);
	I2CM_Write(I2C_MASTER, 0x00, NULL, 0, &tx_buf[0], 1);
	i_mxc_delay(after_dummy_wait_us);

}
*/

#define WAIT_200USEC  ((int)150)
#define WAIT_150USEC  ((int)100)
	
void me11_i2c_wakeup_call(void){

#if 1
	uint8_t tx_buf[1];
	tx_buf[0] = 0x00;
	I2CM_Write(I2C_MASTER, 0x00, NULL, 0, &tx_buf[0], 1);
	i_mxc_delay(WAIT_200USEC);
	tx_buf[0] = 0xD3;
	tx_buf[0] = 0x00;
	I2CM_Write(I2C_MASTER, 0x00, NULL, 0, &tx_buf[0], 1);
	
	//I2CM_Write(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, &tx_buf[0], 1);
	//i_mxc_delay(WAIT_150USEC);
	
#else
	
	/*uint8_t tx_buf[1] = {0xFF};
	I2CM_Write(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, &tx_buf[0], 1);
	I2CM_Write(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, &tx_buf[0], 1);
	I2CM_Write(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, &tx_buf[0], 1);
	I2CM_Write(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, &tx_buf[0], 1);
	*/
	
#endif	
	
}

int me11_write( const uint8_t *data, int length, bool repeated){

    int stop = (repeated) ? 0 : 1;
    int written = I2CM_Write(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, (uint8_t *)data, length);
    return length != written;

}

int me11_read( uint8_t *data, int length, bool repeated){

	  int stop = (repeated) ? 0 : 1;
    int read =  I2CM_Read(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, (uint8_t *)data, length);
    return length != read;

}


SS_STATUS write_cmd(uint8_t *tx_buf, int tx_len, int sleep_ms) {


	me11_i2c_wakeup_call();
	int ret = me11_write(tx_buf, tx_len, false);

	int retries = 4;
	while (ret != 0 && retries-- > 0) {
		  printf("i2c wr retry = %d \r\n" , ret);
		  i_mxc_delay(1000);
	      me11_i2c_wakeup_call();
	      ret = me11_write(tx_buf, tx_len, false);
	}

	if (ret != 0) {
		return SS_ERR_UNAVAILABLE;
	}

	i_mxc_delay(sleep_ms * 1000);
	uint8_t status_byte;

	me11_i2c_wakeup_call();
	ret = me11_read( &status_byte, 1, false);

#if 1	//optional
	bool try_again = (status_byte == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again) && retries-- > 0) {
		   i_mxc_delay(sleep_ms * 1000);
		   me11_i2c_wakeup_call();
		   ret = me11_read( &status_byte, 1, false);
		   try_again = (status_byte == SS_ERR_TRY_AGAIN);
	}
#endif

	return (SS_STATUS)status_byte;

}


SS_STATUS read_cmd(   uint8_t *cmd_bytes, int cmd_bytes_len,
					            uint8_t *rxbuf, int rxbuf_sz,
		                  int sleep_ms)
{

	int retries = 4;
	me11_i2c_wakeup_call();
	int ret = me11_write(cmd_bytes, cmd_bytes_len, false);

	while (ret != 0 && retries-- > 0) {
		  printf("i2c wr retry\r\n");
			i_mxc_delay(1000);
			me11_i2c_wakeup_call();
			ret = me11_write(cmd_bytes, cmd_bytes_len, false);
    }

	 if (ret != 0) {
		 return SS_ERR_UNAVAILABLE;
	 }

	 i_mxc_delay(sleep_ms * 1000);

	 me11_i2c_wakeup_call();
	 ret = me11_read(rxbuf, rxbuf_sz, false);


#if 1 //optional
	 bool try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	 while ((ret != 0 || try_again) && retries-- > 0) {
		 i_mxc_delay(sleep_ms * 1000);
		 me11_i2c_wakeup_call();
		 ret = me11_read(rxbuf, rxbuf_sz, false);
		 try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	 }
#endif

	 return (SS_STATUS)rxbuf[0];

}

uint8_t sh_write_buf[512];
int sh_write_cmd_with_data(uint8_t *cmd_bytes,
		                   int cmd_bytes_len,
                       uint8_t *data,
						           int data_len,
                       int cmd_delay_ms)
{
    memcpy(sh_write_buf, cmd_bytes, cmd_bytes_len);
    memcpy(sh_write_buf + cmd_bytes_len, data, data_len);
    int status = write_cmd(sh_write_buf,cmd_bytes_len + data_len, cmd_delay_ms);
    return status;
}


typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} accel_mode1_data;

typedef struct __attribute__((packed)){
	uint32_t led1 : 24;
	uint32_t led2 : 24 ;
	uint32_t led3 : 24;
	uint32_t led4 : 24;
	uint32_t led5 : 24;
	uint32_t led6 : 24;
} max8614x_mode1_data;


typedef struct __attribute__((packed)){
	uint8_t current_operating_mode; // mode 1 & 2
	// WHRM data
	uint16_t hr;         	// mode 1 & 2
	uint8_t hr_conf;     	// mode 1 & 2
	uint16_t rr;         	// mode 1 & 2
	uint8_t rr_conf;		// mode 1 & 2
	uint8_t activity_class; // mode 1 & 2
	// WSPO2 data
	uint16_t r;						// mode 1 & 2
	uint8_t spo2_conf;		// mode 1 & 2
	uint16_t spo2;			// mode 1 & 2
	uint8_t percentComplete;		// mode 1 & 2
	uint8_t lowSignalQualityFlag;	// mode 1 & 2
	uint8_t motionFlag;				// mode 1 & 2
	uint8_t lowPiFlag;				// mode 1 & 2
	uint8_t unreliableRFlag;		// mode 1 & 2
	uint8_t spo2State;   			// mode 1 & 2
	uint8_t scd_contact_state;
	uint8_t ibi_offset;

} whrm_wspo2_suite_mode1_data;

enum{
	ALGO_REPORT_MODE_BASIC    = 1,
	ALGO_REPORT_MODE_EXTENDED = 2,
};

enum{
	SCDSTATE_UNDETECTED    = 0,
	SCDSTATE_NOSKINCONTACT = 1,
	SCDSTATE_SOMECONTACT   = 2,
	SCDSTATE_SKINCONTACT   = 3
};
	
	
int sh_set_report_period(uint8_t period)
{

	uint8_t cmd_bytes[]  = { 0x10, 0x02 };
	uint8_t data_bytes[] = { (uint8_t)period };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								              &data_bytes[0], sizeof(data_bytes), SS_DEFAULT_CMD_SLEEP_MS );
	return status;
}

int sh_set_data_type(int data_type_, bool sc_en_)
{

	uint8_t cmd_bytes[] = { 0x10, 0x00 };
	uint8_t data_bytes[] = { (uint8_t)((sc_en_ ? SS_MASK_OUTPUTMODE_SC_EN : 0) |
							((data_type_ << SS_SHIFT_OUTPUTMODE_DATATYPE) & SS_MASK_OUTPUTMODE_DATATYPE)) };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								&data_bytes[0], sizeof(data_bytes),
								SS_DEFAULT_CMD_SLEEP_MS);
	return status;
}

int sh_set_fifo_thresh( int threshold ){

	uint8_t cmd_bytes[]  = { 0x10 , 0x01 };
	uint8_t data_bytes[] = { (uint8_t)threshold };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								&data_bytes[0], sizeof(data_bytes),
								SS_DEFAULT_CMD_SLEEP_MS
	                            );
	return status;

}

int sh_enable_algo_(int idx, int mode)
{
  uint8_t cmd_bytes[] = { 0x52, (uint8_t)idx, (uint8_t)mode };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes), 0, 0, 25 * SS_ENABLE_SENSOR_SLEEP_MS);

	return status;
}

int sh_enable_algo_fast(int idx, int mode)
{
  uint8_t cmd_bytes[] = { 0x52, (uint8_t)idx, (uint8_t)mode };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes), 0, 0, 200);

	return status;
}



static bool m_irq_received_ = false;

#define MFIOPIN_GPIO_PORT	5
#define MFIOPIN_GPIO_PIN	PIN_4

static gpio_cfg_t mfioPinConfig ={
	   .port = MFIOPIN_GPIO_PORT,
	   .mask = MFIOPIN_GPIO_PIN, 
	   .func = GPIO_FUNC_GPIO,
	   .pad = GPIO_PAD_INPUT_PULLUP
};


void sh_clear_poll_event_flag(void){
	__disable_irq();
	m_irq_received_ = false;
	__enable_irq();
}

bool sh_has_poll_event(void){
	return m_irq_received_;
}

void ContinuousTimer_handler()
{
    TMR32_ClearFlag(MXC_TMR4);
	  m_irq_received_ = true;
	
}


bool sh_has_mfio_event(void){
	return m_irq_received_;
}

static volatile int mfio_event_count = 0;
void mfio_event_handler(void *cbdata){
	TMR_TO_Start(MXC_TMR3, 2000000);
	++mfio_event_count;
	m_irq_received_ = true;
}	

void mfio_event_handler_mestest(void *cbdata){
	++mfio_event_count;
	m_irq_received_ = true;
}


void sh_clear_mfio_event_flag(void){
	__disable_irq();
	m_irq_received_ = false;
	__enable_irq();
}

int start_hub_event_poll( int pollPeriod_ms){
	
	  int error = 0;
    tmr32_cfg_t cont_cfg;
    uint32_t IntervalTime = 100;//ms
	  //enable timer interrupt
    NVIC_SetVector(TMR4_0_IRQn, ContinuousTimer_handler);
    TMR32_EnableINT(MXC_TMR4); 

	  tmr_prescale_t prescale = TMR_PRESCALE_DIV_2_12;
    error = TMR_Init(MXC_TMR4, prescale, NULL);
	  if(error != E_NO_ERROR)
        return error;
		
	  cont_cfg.mode = TMR32_MODE_CONTINUOUS;
	  //calculate the ticks values for given time
    error = TMR32_TimeToTicks(MXC_TMR4, IntervalTime, TMR_UNIT_MILLISEC, &(cont_cfg.compareCount));
		if(error != E_NO_ERROR)
        return error;
		
		//configure and start the timer
    TMR32_Config(MXC_TMR4, &cont_cfg);
    TMR32_Start(MXC_TMR4);

    return error;
	
	//shubEventPollTimer.attach(&sensor_hub_poll_event , ((float) pollPeriod_ms) / 1000.0);
	
}


typedef struct{
	uint32_t green_led_cnt;
	uint32_t ir_led_cnt;
	uint32_t red_led_cnt;
	uint32_t hr;
	uint32_t hr_conf;
	uint32_t spo2;
	uint32_t spo2_conf;
  uint32_t scd_state;

}mes_repor_t;

void execute_data_poll( mes_repor_t* mesOutput ) {
	
	 const int sensHubReportFifoThresh      = 1;
	 const int MAX_WHRMWSPO2_SAMPLE_COUNT   = 45;
	 const int RESERVED_BYTES_SZ	          = 3; 
   const int WHRMWSPO2_FRAME_SIZE         = sizeof(accel_mode1_data)
																					+ sizeof(max8614x_mode1_data)
																				  + sizeof(whrm_wspo2_suite_mode1_data)
   														            + RESERVED_BYTES_SZ;						
																				
	 static uint8_t databuf[WHRMWSPO2_FRAME_SIZE * MAX_WHRMWSPO2_SAMPLE_COUNT + 1];
	 
   int status;																					 
																					
			 if( sh_has_poll_event()) {
			 
			
			 sh_clear_poll_event_flag();
			 uint8_t hubStatus = 0;
    	 status = read_fifo_status(&hubStatus);
	 		 //printf("hubStatus status = %2X n", hubStatus);
 			 if ( status == SS_SUCCESS && (hubStatus & SS_MASK_STATUS_DATA_RDY) == SS_MASK_STATUS_DATA_RDY ) {
			 
					     uint8_t num_samples = 1;
    		    	 status = num_avail_samples(&num_samples);
				 
				       if(num_samples > MAX_WHRMWSPO2_SAMPLE_COUNT)
								   num_samples = MAX_WHRMWSPO2_SAMPLE_COUNT;
				 
    	    		 if(status == SS_SUCCESS ) {    
			 
									status = read_fifo_data(num_samples, WHRMWSPO2_FRAME_SIZE, &databuf[0], sizeof(databuf));
								  if(status == SS_SUCCESS &&  num_samples > 0 && num_samples < MAX_WHRMWSPO2_SAMPLE_COUNT){  
				 
											max8614x_mode1_data             ppgDataSample;
											accel_mode1_data                accelDataSamp;
											whrm_wspo2_suite_mode1_data     algoDataSamp;

											//uint8_t *ptr = &databuf[1]; 
										
										  int sampleIdx = 0;
										  int ptr =0;
											 while( sampleIdx < num_samples ) {

												 ppgDataSample.led1  			     =  (databuf[ptr+1] << 16) + (databuf[ptr+2] << 8) + (databuf[ptr+3] << 0);
												 ppgDataSample.led2  			     =  (databuf[ptr+4] << 16) + (databuf[ptr+5] << 8) + (databuf[ptr+6] << 0);
												 ppgDataSample.led3  			     =  (databuf[ptr+7] << 16) + (databuf[ptr+8] << 8) + (databuf[ptr+9] << 0);
												 ppgDataSample.led4  				 =  (databuf[ptr+10] << 16) + (databuf[ptr+11] << 8) + (databuf[ptr+12] << 0);
												 ppgDataSample.led5  				 =  (databuf[ptr+13] << 16) + (databuf[ptr+14] << 8) + (databuf[ptr+15] << 0);
												 ppgDataSample.led6  				 =  (databuf[ptr+16] << 16) + (databuf[ptr+17] << 8) + (databuf[ptr+18] << 0);
												 accelDataSamp.x                     =  (databuf[ptr+19] << 8)  + (databuf[ptr+20] << 0);
												 accelDataSamp.y                     =  (databuf[ptr+21] << 8)  + (databuf[ptr+22] << 0);
												 accelDataSamp.z                     =  (databuf[ptr+23] << 8)  + (databuf[ptr+24] << 0);
   											 algoDataSamp.current_operating_mode =  (databuf[ptr+25]);
												 algoDataSamp.hr                     =  (databuf[ptr+26] << 8)  + (databuf[ptr+27] << 0);
												 algoDataSamp.hr_conf                =  (databuf[ptr+28]);
												 algoDataSamp.rr                     =  (databuf[ptr+29] << 8)  + (databuf[ptr+30] << 0);
												 algoDataSamp.rr_conf      			 =  (databuf[ptr+31]);
												 algoDataSamp.activity_class         =  (databuf[ptr+32]);
												 algoDataSamp.r                      =  (databuf[ptr+33] << 8)  + (databuf[ptr+34] << 0);
												 algoDataSamp.spo2_conf              =  (databuf[ptr+35]);
												 algoDataSamp.spo2                   =  (databuf[ptr+36] << 8)  + (databuf[ptr+37] << 0);
												 algoDataSamp.percentComplete 		 =  (databuf[ptr+38]);
												 algoDataSamp.lowSignalQualityFlag   =  (databuf[ptr+39]);
												 algoDataSamp.motionFlag 			 =  (databuf[ptr+40]);
												 algoDataSamp.lowPiFlag 			 =  (databuf[ptr+41]);
												 algoDataSamp.unreliableRFlag 		 =  (databuf[ptr+42]);
												 algoDataSamp.spo2State 			 =  (databuf[ptr+43]);
												 algoDataSamp.scd_contact_state 	 =  (databuf[ptr+44]);
												 algoDataSamp.ibi_offset 	      =  (databuf[ptr+45]);
												 uint8_t reservedByte_1         =  (databuf[ptr+46]); 
												 uint8_t reservedByte_2         =  (databuf[ptr+47]); 
												 uint8_t reservedByte_3         =  (databuf[ptr+48]); 

												 mesOutput->green_led_cnt   			  =  ppgDataSample.led1;
												 mesOutput->ir_led_cnt   			      =  ppgDataSample.led2;
												 mesOutput->red_led_cnt   			    =  ppgDataSample.led3;
												 mesOutput->hr                  		=  algoDataSamp.hr / 10;
												 mesOutput->hr_conf                 =  algoDataSamp.hr_conf;
												 mesOutput->spo2                 	  =  algoDataSamp.spo2 / 10;
												 mesOutput->spo2_conf               =  algoDataSamp.spo2_conf;
												 mesOutput->scd_state               =  algoDataSamp.scd_contact_state;

											  /* printf(" greenCnt= %d , irCnt= %d , redCnt = %d ,"
																" hr= %d , hr_conf= %d , spo2= %d , spo2_conf= %d , skin_contact = %d \r\n"
																, mesOutput->green_led_cnt , mesOutput->ir_led_cnt , mesOutput->red_led_cnt
																, mesOutput->hr , mesOutput->hr_conf , mesOutput->spo2 , mesOutput->spo2_conf , mesOutput->scd_state);
													*/			

												 sampleIdx += 1;
												 ptr += WHRMWSPO2_FRAME_SIZE;

											 } //eof loop reading bytyes from hub report fifo 
										
				          } // eof datas pull request form hub
							 } // eof fifo data count query
			    } // eof hub status query
		  } //eof mfio event query
}	

int start_whrm_measurement(reportPeriod_in40msSteps){
	
	 const int sensHubReportFifoThresh      = 1;
	
	 int status;	
	 status  =  sh_set_report_period(reportPeriod_in40msSteps);
   if( status != SS_SUCCESS )
    	 return -1;
	 
   status = sh_set_data_type( SS_DATATYPE_BOTH , false );
     if( status != SS_SUCCESS )
    	 return -1; 																					
			
   //set fifo threshold for mfio event frequency
   status = sh_set_fifo_thresh(sensHubReportFifoThresh);
   if( status != SS_SUCCESS )
     	 return -1;
		 
	 status = sh_enable_algo_(0x07 , (int) ALGO_REPORT_MODE_BASIC);
   if( status != SS_SUCCESS )
     	 return -1; 
	 
	 return status;
	
}	

void stop_data_poll(){
	
	TMR32_DisableINT(MXC_TMR4);
	TMR32_Stop(MXC_TMR4);
	sh_clear_poll_event_flag();
	
}	

uint8_t execute_data_poll_mfio( mes_repor_t* mesOutput ) {
	
   static uint8_t mfio_event_status =0;
	
	 const int MAX_WHRMWSPO2_SAMPLE_COUNT   = 45;
	 const int RESERVED_BYTES_SZ	          = 3; 
   const int WHRMWSPO2_FRAME_SIZE         = sizeof(accel_mode1_data)
																					+ sizeof(max8614x_mode1_data)
																				  + sizeof(whrm_wspo2_suite_mode1_data)
																					+ RESERVED_BYTES_SZ;
																				
	 static uint8_t databuf[WHRMWSPO2_FRAME_SIZE * MAX_WHRMWSPO2_SAMPLE_COUNT + 1];
	 
   int status;	
	
	 if( sh_has_mfio_event()) {
			 
			 sh_clear_poll_event_flag();
		 
			 uint8_t hubStatus = 0;
    	 status = read_fifo_status(&hubStatus);
	 		 //printf("hubStatus status = %2X n", hubStatus);
		   if(status == SS_SUCCESS)  
						mfio_event_status = hubStatus;
			 
 			 if ( status == SS_SUCCESS && (hubStatus & SS_MASK_STATUS_DATA_RDY) == SS_MASK_STATUS_DATA_RDY ) {
			 
					     uint8_t num_samples = 0;
    		    	 status = num_avail_samples(&num_samples);
				       //printf("hubStatus status = %2X n", hubStatus);

	   	    		 if(status == SS_SUCCESS &&  num_samples > 0 ) {    
								 
			 				     if(num_samples > MAX_WHRMWSPO2_SAMPLE_COUNT)
												num_samples = MAX_WHRMWSPO2_SAMPLE_COUNT;
									 
									status = read_fifo_data(num_samples, WHRMWSPO2_FRAME_SIZE, &databuf[0], sizeof(databuf));
								  if(status == SS_SUCCESS  ){  
				 
											max8614x_mode1_data             ppgDataSample;
											accel_mode1_data                accelDataSamp;
											whrm_wspo2_suite_mode1_data     algoDataSamp;

											//uint8_t *ptr = &databuf[1]; 
										
										  int sampleIdx = 0;
										  int ptr =0;
											while( sampleIdx < num_samples ) {

												 ppgDataSample.led1  			     =  (databuf[ptr+1] << 16) + (databuf[ptr+2] << 8) + (databuf[ptr+3] << 0);
												 ppgDataSample.led2  			     =  (databuf[ptr+4] << 16) + (databuf[ptr+5] << 8) + (databuf[ptr+6] << 0);
												 ppgDataSample.led3  			     =  (databuf[ptr+7] << 16) + (databuf[ptr+8] << 8) + (databuf[ptr+9] << 0);
												 ppgDataSample.led4  				 =  (databuf[ptr+10] << 16) + (databuf[ptr+11] << 8) + (databuf[ptr+12] << 0);
												 ppgDataSample.led5  				 =  (databuf[ptr+13] << 16) + (databuf[ptr+14] << 8) + (databuf[ptr+15] << 0);
												 ppgDataSample.led6  				 =  (databuf[ptr+16] << 16) + (databuf[ptr+17] << 8) + (databuf[ptr+18] << 0);
												 accelDataSamp.x                     =  (databuf[ptr+19] << 8)  + (databuf[ptr+20] << 0);
												 accelDataSamp.y                     =  (databuf[ptr+21] << 8)  + (databuf[ptr+22] << 0);
												 accelDataSamp.z                     =  (databuf[ptr+23] << 8)  + (databuf[ptr+24] << 0);
   											 algoDataSamp.current_operating_mode =  (databuf[ptr+25]);
												 algoDataSamp.hr                     =  (databuf[ptr+26] << 8)  + (databuf[ptr+27] << 0);
												 algoDataSamp.hr_conf                =  (databuf[ptr+28]);
												 algoDataSamp.rr                     =  (databuf[ptr+29] << 8)  + (databuf[ptr+30] << 0);
												 algoDataSamp.rr_conf      			 =  (databuf[ptr+31]);
												 algoDataSamp.activity_class         =  (databuf[ptr+32]);
												 algoDataSamp.r                      =  (databuf[ptr+33] << 8)  + (databuf[ptr+34] << 0);
												 algoDataSamp.spo2_conf              =  (databuf[ptr+35]);
												 algoDataSamp.spo2                   =  (databuf[ptr+36] << 8)  + (databuf[ptr+37] << 0);
												 algoDataSamp.percentComplete 		 =  (databuf[ptr+38]);
												 algoDataSamp.lowSignalQualityFlag   =  (databuf[ptr+39]);
												 algoDataSamp.motionFlag 			 =  (databuf[ptr+40]);
												 algoDataSamp.lowPiFlag 			 =  (databuf[ptr+41]);
												 algoDataSamp.unreliableRFlag 		 =  (databuf[ptr+42]);
												 algoDataSamp.spo2State 			 =  (databuf[ptr+43]);
												 algoDataSamp.scd_contact_state 	 =  (databuf[ptr+44]);
												 algoDataSamp.ibi_offset 	      =  (databuf[ptr+45]);
												 uint8_t reservedByte_1         =  (databuf[ptr+46]); 
												 uint8_t reservedByte_2         =  (databuf[ptr+47]); 
												 uint8_t reservedByte_3         =  (databuf[ptr+48]); 

												 mesOutput->green_led_cnt   			  =  ppgDataSample.led1;
												 mesOutput->ir_led_cnt   			      =  ppgDataSample.led2;
												 mesOutput->red_led_cnt   			    =  ppgDataSample.led3;
												 mesOutput->hr                  		=  algoDataSamp.hr / 10;
												 mesOutput->hr_conf                 =  algoDataSamp.hr_conf;
												 mesOutput->spo2                 	  =  algoDataSamp.spo2 / 10;
												 mesOutput->spo2_conf               =  algoDataSamp.spo2_conf;
												 mesOutput->scd_state               =  algoDataSamp.scd_contact_state;
												 
											  /* printf(" greenCnt= %d , irCnt= %d , redCnt = %d ,"
																" hr= %d , hr_conf= %d , spo2= %d , spo2_conf= %d , skin_contact = %d \r\n"
																, ppgDataSample.led1 , ppgDataSample.led2 , ppgDataSample.led3
																, algoDataSamp.hr / 10, algoDataSamp.hr_conf , algoDataSamp.spo2 / 10 , algoDataSamp.spo2_conf , algoDataSamp.scd_contact_state);
												*/				

												 sampleIdx += 1;
												 ptr += WHRMWSPO2_FRAME_SIZE;

											 } //eof loop reading bytyes from hub report fifo 
										
				          } // eof datas pull request form hub
							 } // eof fifo data count query
			    } // eof hub status query
		  } //eof mfio event query
																
	   return mfio_event_status;
}




int measure_whrm_wspo2_with_mfio_event(  uint8_t algoSuiteOperatingMode){

	
	 printf(" \n WHRM measurement with SemsorHub event reporting over MFIO pin \n");
	
	
	 const int sensHubReportFifoThresh      = 5;
	 const int MAX_WHRMWSPO2_SAMPLE_COUNT   = 45;
   const int RESERVED_BYTES_SZ	          = 3;  
	 const int WHRMWSPO2_FRAME_SIZE         = sizeof(accel_mode1_data)
																					+ sizeof(max8614x_mode1_data)
																				  + sizeof(whrm_wspo2_suite_mode1_data)
																					+ RESERVED_BYTES_SZ;
																				
	 static uint8_t databuf[WHRMWSPO2_FRAME_SIZE * MAX_WHRMWSPO2_SAMPLE_COUNT + 1];

	 // set mfio as input and attach interrupt (ie mfio event)																				
	GPIO_Config(&mfioPinConfig);																	
  GPIO_RegisterCallback(&mfioPinConfig, mfio_event_handler_mestest, NULL);
  GPIO_IntConfig(&mfioPinConfig, GPIO_INT_FALLING_EDGE);
  GPIO_IntEnable(&mfioPinConfig);
  NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(mfioPinConfig.port));																					

  sh_clear_mfio_event_flag();																					
	

  int status;	
	status  =  sh_set_report_period(1);
  if( status != SS_SUCCESS )
    	 return -1;
	
	status = sh_set_data_type( SS_DATATYPE_BOTH , false );
  if( status != SS_SUCCESS )
    	 return -1; 	
	
	status = sh_set_fifo_thresh(sensHubReportFifoThresh);
  if( status != SS_SUCCESS )
     	 return -1;
	
	status = sh_enable_algo_fast(0x07, (int) ALGO_REPORT_MODE_BASIC);
	if( status != SS_SUCCESS )
     	 return -1;
	
	__disable_irq();
	  mfio_event_count = 0;
	__enable_irq();
	
	i_mxc_delay(500000);
	mes_repor_t mesOutput;
  
	
	 while(1){
	
			 if( sh_has_mfio_event()) {
			 printf("evc = %d ", mfio_event_count );
			
			 sh_clear_poll_event_flag();
			 uint8_t hubStatus = 0;
    	 status = read_fifo_status(&hubStatus);
	 		 printf("hs = %d ", hubStatus);
 			 if ( status == SS_SUCCESS && (hubStatus & SS_MASK_STATUS_DATA_RDY) == SS_MASK_STATUS_DATA_RDY ) {
			 
					     uint8_t num_samples = 0;
    		    	 status = num_avail_samples(&num_samples);
				       printf(" numsamps = %d \n", num_samples);

	   	    		 if(status == SS_SUCCESS &&  num_samples > 0 ) {    
								 
			 				     if(num_samples > MAX_WHRMWSPO2_SAMPLE_COUNT)
												num_samples = MAX_WHRMWSPO2_SAMPLE_COUNT;
									 
									status = read_fifo_data(num_samples, WHRMWSPO2_FRAME_SIZE, &databuf[0], sizeof(databuf));
								  if(status == SS_SUCCESS  ){  
				 
											max8614x_mode1_data             ppgDataSample;
											accel_mode1_data                accelDataSamp;
											whrm_wspo2_suite_mode1_data     algoDataSamp;

											//uint8_t *ptr = &databuf[1]; 
										
										  int sampleIdx = 0;
										  int ptr =0;
											while( sampleIdx < num_samples ) {

												 ppgDataSample.led1  			     =  (databuf[ptr+1] << 16) + (databuf[ptr+2] << 8) + (databuf[ptr+3] << 0);
												 ppgDataSample.led2  			     =  (databuf[ptr+4] << 16) + (databuf[ptr+5] << 8) + (databuf[ptr+6] << 0);
												 ppgDataSample.led3  			     =  (databuf[ptr+7] << 16) + (databuf[ptr+8] << 8) + (databuf[ptr+9] << 0);
												 ppgDataSample.led4  				 =  (databuf[ptr+10] << 16) + (databuf[ptr+11] << 8) + (databuf[ptr+12] << 0);
												 ppgDataSample.led5  				 =  (databuf[ptr+13] << 16) + (databuf[ptr+14] << 8) + (databuf[ptr+15] << 0);
												 ppgDataSample.led6  				 =  (databuf[ptr+16] << 16) + (databuf[ptr+17] << 8) + (databuf[ptr+18] << 0);
												 accelDataSamp.x                     =  (databuf[ptr+19] << 8)  + (databuf[ptr+20] << 0);
												 accelDataSamp.y                     =  (databuf[ptr+21] << 8)  + (databuf[ptr+22] << 0);
												 accelDataSamp.z                     =  (databuf[ptr+23] << 8)  + (databuf[ptr+24] << 0);
   											 algoDataSamp.current_operating_mode =  (databuf[ptr+25]);
												 algoDataSamp.hr                     =  (databuf[ptr+26] << 8)  + (databuf[ptr+27] << 0);
												 algoDataSamp.hr_conf                =  (databuf[ptr+28]);
												 algoDataSamp.rr                     =  (databuf[ptr+29] << 8)  + (databuf[ptr+30] << 0);
												 algoDataSamp.rr_conf      			 =  (databuf[ptr+31]);
												 algoDataSamp.activity_class         =  (databuf[ptr+32]);
												 algoDataSamp.r                      =  (databuf[ptr+33] << 8)  + (databuf[ptr+34] << 0);
												 algoDataSamp.spo2_conf              =  (databuf[ptr+35]);
												 algoDataSamp.spo2                   =  (databuf[ptr+36] << 8)  + (databuf[ptr+37] << 0);
												 algoDataSamp.percentComplete 		 =  (databuf[ptr+38]);
												 algoDataSamp.lowSignalQualityFlag   =  (databuf[ptr+39]);
												 algoDataSamp.motionFlag 			 =  (databuf[ptr+40]);
												 algoDataSamp.lowPiFlag 			 =  (databuf[ptr+41]);
												 algoDataSamp.unreliableRFlag 		 =  (databuf[ptr+42]);
												 algoDataSamp.spo2State 			 =  (databuf[ptr+43]);
												 algoDataSamp.scd_contact_state 	 =  (databuf[ptr+44]);
												 algoDataSamp.ibi_offset 	      =  (databuf[ptr+45]);
												 uint8_t reservedByte_1         =  (databuf[ptr+46]); 
												 uint8_t reservedByte_2         =  (databuf[ptr+47]); 
												 uint8_t reservedByte_3         =  (databuf[ptr+48]); 
	
											   printf(" greenCnt= %d , irCnt= %d , redCnt = %d ,"
																" hr= %d , hr_conf= %d , spo2= %d , spo2_conf= %d , skin_contact = %d \r\n"
																, ppgDataSample.led1 , ppgDataSample.led2 , ppgDataSample.led3
																, algoDataSamp.hr / 10, algoDataSamp.hr_conf , algoDataSamp.spo2 / 10 , algoDataSamp.spo2_conf , algoDataSamp.scd_contact_state);
																

												 sampleIdx += 1;
												 ptr += WHRMWSPO2_FRAME_SIZE;

											 } //eof loop reading bytyes from hub report fifo 
										
				          } // eof datas pull request form hub
							 } // eof fifo data count query
			    } // eof hub status query
		  } //eof mfio event query
																					
		

		} //eof main measurement loop
	 
}

int measure_whrm_wspo2(  uint8_t reportPeriod_in40msSteps ,   uint8_t algoSuiteOperatingMode){

	 const int sensHubReportFifoThresh      = 1;
	 const int MAX_WHRMWSPO2_SAMPLE_COUNT   = 45;
   const int WHRMWSPO2_FRAME_SIZE         = sizeof(accel_mode1_data)
																					+ sizeof(max8614x_mode1_data)
																				  + sizeof(whrm_wspo2_suite_mode1_data);
																				
	 static uint8_t databuf[WHRMWSPO2_FRAME_SIZE * MAX_WHRMWSPO2_SAMPLE_COUNT + 1];
   int status;	

	 status = start_whrm_measurement(reportPeriod_in40msSteps);
   if( status != SS_SUCCESS )
     	 return -1; 
	 
	 int poolPeriod_ms =  ((int)reportPeriod_in40msSteps) * 40 * 5;
   start_hub_event_poll(poolPeriod_ms);
   sh_clear_poll_event_flag(); 
   
	 mes_repor_t mesOutput;
	 
	 while(1){
	
		 execute_data_poll( &mesOutput );
		 printf(" greenCnt= %d , irCnt= %d , redCnt = %d ,"
						" hr= %d , hr_conf= %d , spo2= %d , spo2_conf= %d , skin_contact = %d \r\n"
							, mesOutput.green_led_cnt , mesOutput.ir_led_cnt , mesOutput.red_led_cnt
							, mesOutput.hr , mesOutput.hr_conf , mesOutput.spo2 , mesOutput.spo2_conf , mesOutput.scd_state);

		 } //eof main measurement loop
	 
}

SS_STATUS disable_sensors(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, 0xFF, 2, SS_SENSORIDX_ACCEL, 0, 0x00, SS_SENSORIDX_MAX86140, 0, 0x00 };

	SS_STATUS status = write_cmd(&cmd_bytes[0], sizeof(cmd_bytes), 5 * SS_ENABLE_SENSOR_SLEEP_MS);

	return status;
}


SS_STATUS get_scdsm_state(uint8_t* stt)
{
	uint8_t cmd_bytes[] = { 0xBD , (uint8_t) 0x00 };
	uint8_t rxbuf[2] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], sizeof(cmd_bytes), &rxbuf[0], sizeof(rxbuf), 2);

	if (status == SS_SUCCESS) 
    {
		*stt = rxbuf[1];
	}

	return status;
}

SS_STATUS set_scdsm_state(int enable_scdsm)
{
	uint8_t cmd_bytes[2][3] = {{ 0xBB, 0x00, 0x00 },
	                           { 0xBC, 0x01, 0x01 } };
		
	int cmd_set = (enable_scdsm)? 1:0;
 
	SS_STATUS status = write_cmd(&cmd_bytes[cmd_set][0], sizeof(cmd_bytes[0]), 2);
	return status;
}

//GPIO_Config(&gpioLP1);	
int test_scdsm( uint8_t reportPeriod_in40msSteps ){
	
	 
	 const int sensHubReportFifoThresh      = 1;
	 const int MAX_WHRMWSPO2_SAMPLE_COUNT   = 45;
   const int WHRMWSPO2_FRAME_SIZE         = sizeof(accel_mode1_data)
																					+ sizeof(max8614x_mode1_data)
																				  + sizeof(whrm_wspo2_suite_mode1_data);
																					
	 static uint8_t databuf[WHRMWSPO2_FRAME_SIZE * MAX_WHRMWSPO2_SAMPLE_COUNT + 1];																				
	 
	 int status;	  
	 // configure mfio as input pin																				
	 GPIO_Config(&mfioPinConfig);
   																					
	 uint8_t scdsm_op_state;   
	 status = get_scdsm_state(&scdsm_op_state); 			 																
	 if( status != SS_SUCCESS )
    	 return -1;		
	 printf("scdsm operation = %d \r\n", scdsm_op_state);
	 // start with scd state machine off
   if(scdsm_op_state == 1){	 
		   status = set_scdsm_state(0);
	     if( status != SS_SUCCESS )
    	     return -1;			      
		}
	 get_scdsm_state(&scdsm_op_state); 	
	 printf("scdsm operation = %d \r\n", scdsm_op_state);	

	 status = start_whrm_measurement(reportPeriod_in40msSteps);
   if( status != SS_SUCCESS )
     	 return -1; 
	 
	 int poolPeriod_ms =  ((int)reportPeriod_in40msSteps) * 40 * 5;
   start_hub_event_poll(poolPeriod_ms);
   sh_clear_poll_event_flag();
	 
	 mes_repor_t mesOutput;
	 
	 TMR_TO_Start(MXC_TMR3, 8000000);
	 
	 while(1){
		 
		execute_data_poll( &mesOutput );
		printf("sc= %d\r\n",mesOutput.scd_state); 
    if(mesOutput.scd_state != SCDSTATE_SKINCONTACT  && TMR_TO_Check(MXC_TMR3) )		
		{	
			   // activate scdsm
         status = set_scdsm_state(1);
			   i_mxc_delay(100000);
			   get_scdsm_state(&scdsm_op_state); 	
	       //printf("scdsm operation = %d \r\n", scdsm_op_state);	
			   if( status == SS_SUCCESS ){
						stop_data_poll();
					 
					  LP_ClearWakeUpConfig();
					  LP_ClearWakeUpFlags();
					  LP_ConfigGPIOWakeUpDetect(&mfioPinConfig, 0, LP_WEAK_PULL_UP);
            __disable_irq();
            LP_EnterLP1();
            __enable_irq();
            printf(" \r\n\r\n woke up on me11 mfio notification \r\n\r\n");
					  
					 // restart measurement
					 //disable_sensors();
					 start_whrm_measurement(reportPeriod_in40msSteps);
					 int poolPeriod_ms =  ((int)reportPeriod_in40msSteps) * 40 * 5;
           start_hub_event_poll(poolPeriod_ms);
           sh_clear_poll_event_flag();
					 TMR_TO_Start(MXC_TMR3, 8000000);
				 }	 

    }
	  
	}
	
}	

#endif

int test_scdsm_mfio(void){
	
	 
	 const int sensHubReportFifoThresh      = 5;
	 const int MAX_WHRMWSPO2_SAMPLE_COUNT   = 45;
   const int WHRMWSPO2_FRAME_SIZE         = sizeof(accel_mode1_data)
																					+ sizeof(max8614x_mode1_data)
																				  + sizeof(whrm_wspo2_suite_mode1_data);
																					
	 static uint8_t databuf[WHRMWSPO2_FRAME_SIZE * MAX_WHRMWSPO2_SAMPLE_COUNT + 1];																				
	 
	 int status;	  
	 // configure mfio as input pin																				
	 GPIO_Config(&mfioPinConfig);
   																					
	 uint8_t scdsm_op_state;   
	 status = get_scdsm_state(&scdsm_op_state); 			 																
	 if( status != SS_SUCCESS )
    	 return -1;		
	 printf("scdsm operation = %d \r\n", scdsm_op_state);
	 // start with scd state machine off
   if(scdsm_op_state == 1){	 
		   status = set_scdsm_state(0);
	     if( status != SS_SUCCESS )
    	     return -1;			      
		}
	 get_scdsm_state(&scdsm_op_state); 	
	 printf("scdsm operation = %d \r\n", scdsm_op_state);	
		 
		 // set mfio as input and attach interrupt (ie mfio event)																				
	GPIO_Config(&mfioPinConfig);																	
  GPIO_RegisterCallback(&mfioPinConfig, mfio_event_handler, NULL);
  GPIO_IntConfig(&mfioPinConfig, GPIO_INT_FALLING_EDGE);
  GPIO_IntEnable(&mfioPinConfig);
  NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(mfioPinConfig.port));																					

  sh_clear_mfio_event_flag();																					
	
	status  =  sh_set_report_period(1);
  if( status != SS_SUCCESS )
    	 return -1;
	
	status = sh_set_data_type( SS_DATATYPE_BOTH , false );
  if( status != SS_SUCCESS )
    	 return -1; 	
	
	status = sh_set_fifo_thresh(sensHubReportFifoThresh);
  if( status != SS_SUCCESS )
     	 return -1;
	
	status = sh_enable_algo_fast(0x07, (int) ALGO_REPORT_MODE_BASIC);
	if( status != SS_SUCCESS )
     	 return -1;
	
	__disable_irq();
	  mfio_event_count = 0;
	__enable_irq();
	
	status = set_scdsm_state(1);
	
	i_mxc_delay(4000000);
	TMR_TO_Start(MXC_TMR3, 800000000);
	
	//i_mxc_delay(10000000);
	mes_repor_t mesOutput;
	static int prev_mfio_event_count = -1; 
	static int no_event_counter = 0; 	
	
	 while(1){
		 
		uint8_t hub_event = execute_data_poll_mfio( &mesOutput );
		printf("ev = %d  sc= %d   hr = %d  hr_conf = %d spo2 = %d , spo2_conf = %d \r\n",mfio_event_count , 
		                                                                                  mesOutput.scd_state, 
		                                                                                 mesOutput.hr ,
		                                                                                 mesOutput.hr_conf,
		 		                                                                                 mesOutput.spo2 ,
		                                                                                 mesOutput.spo2_conf); 
	
		/*
		 * MXC_TMR3 is used like a watchdog timer to check whether any mfio interrupt from me11 occured within a scecific duration
		 * MXC_TMR3 is setup eachtime when mfio interrupt received. So if mfio interrupts continues to come, below scope does not execute
		 * when interrupts stops , and do not arive within a certain amoun of time Host CPU understands that ME11 is in autonomuous scdsm mode 
		 * And Host CPU sleeps
		 */ 
  	if( TMR_TO_Check(MXC_TMR3) ) {
			  //printf("sleeping \n");
		 
				LP_ClearWakeUpConfig();
				LP_ClearWakeUpFlags();
				LP_ConfigGPIOWakeUpDetect(&mfioPinConfig, 0, LP_WEAK_PULL_UP);
				__disable_irq();
				LP_EnterLP1();
				__enable_irq();
			  i_mxc_delay(4000000);
				TMR_TO_Start(MXC_TMR3, 5000000);		 
        //printf("woke up \n");
		}
	}
	
}	



int main(void)
{
    sys_cfg_i2cm_t i2cm_sys_cfg;
    uint8_t temp;
    
    ioman_cfg_t io_cfg = IOMAN_I2CM1(IOMAN_MAP_A, 1);
    i2cm_sys_cfg.clk_scale = CLKMAN_SCALE_DIV_1;
    i2cm_sys_cfg.io_cfg = io_cfg;
    I2CM_Init(I2C_MASTER, &i2cm_sys_cfg, I2CM_SPEED_400KHZ);
    
    MAX20303_Init();
    
    LED_On(0);
    LED_On(1);
    LED_On(2);
    mxc_delay(2000);
    mxc_delay(2000);
    mxc_delay(2000);
    
    MFIO_init_output(1);
	Reset_module();
    mxc_delay(1000000);
    MFIO_Input();
    
    PB_RegisterCallback(0, pbcallback);
    
    g_fw_version = 0;
    
	if(0 != in_bootldr_mode())          //AA 02 00 AB 00 00     read mode
	{
		exit_from_bootloader();
		mxc_delay(1000000);
	}
    //mxc_delay(1000);
    get_ss_fw_version(&g_fw_version);	//AA FF 03 AB 00 01 07 00   read version
    temp = (uint8_t)(g_fw_version >> 16);
    printf("version = %d.", temp);
    temp = (uint8_t)(g_fw_version >> 8);
    printf("%d.", temp);
    temp = (uint8_t)(g_fw_version >> 0);
    printf("%d\n", temp);
    mxc_delay(1000);
    printf("scdsm sample code \r\n");
	
	  test_scdsm_mfio();
	  //measure_whrm_wspo2_with_mfio_event(0);
	  //measure_whrm_wspo2( POLL_PERIOD_25MS ,   0);
	  //test_scdsm( POLL_PERIOD_25MS );
	
    //Get_Algo_Raw_White_Card_Red_LED();
    //Get_Algo_Raw_White_Card_Green_LED();
    //Get_Algo_Raw_CrossTalk();
    //Get_Pure_Raw_White_Card_Red_LED2();
    //Get_Pure_Raw_White_Card_Green_LED();
    //Get_Pure_Raw_CrossTalk();
    
    //WHRM_Algorithm();
    //Get_Algo_Raw_White_Card_Red_LED_new();
    //Get_raw_data(1);
    //AEC_Mode_Algorithm();
    while(1);
}

