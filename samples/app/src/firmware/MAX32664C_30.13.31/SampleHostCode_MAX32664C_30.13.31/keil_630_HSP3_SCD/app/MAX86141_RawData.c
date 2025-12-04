/***** Includes *****/
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "mxc_config.h"
#include "i2cm.h"
#include "led.h"
#include "tmr_utils.h"


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

void mxc_delay(int us)
{
    TMR_Delay(MXC_TMR5, us);
}

SS_STATUS read_cmd(uint8_t *cmd_bytes, int cmd_bytes_len, uint8_t *rxbuf, int rxbuf_sz, int sleep_ms)
{
	int retries, ret, try_again;

    ret = I2CM_Write(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, cmd_bytes, cmd_bytes_len);
    
//    retries = 4;
//	while ((ret != cmd_bytes_len) && (retries > 0))
//    {
//		mxc_delay(1000);
//    	ret = I2C_MasterWrite(MXC_I2C0, 0xAA, cmd_bytes, cmd_bytes_len, 0);
//        retries--;
//	}
//    if (retries == 0) 
//    {
//        return SS_ERR_UNAVAILABLE;
//    }

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
    
    ret = I2CM_Write(I2C_MASTER, I2C_SLAVE_ADDR, NULL, 0, tx_buf, tx_len);
//	retries = 4;
//	while((ret != tx_len) && (retries > 0))
//    {
//		mxc_delay(1000);
//    	ret = I2C_MasterWrite(MXC_I2C0, 0xAA, tx_buf, tx_len, 0);
//        retries--;
//	}
//    if (ret != tx_len) 
//    {
//        return SS_ERR_UNAVAILABLE;
//    }

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

    if ((ret != 1) || try_again)
    {
        return SS_ERR_UNAVAILABLE;
    }

	return (SS_STATUS)status_byte;
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

SS_STATUS num_avail_samples(int *num_samples)
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
    mxc_delay(10000);
    
    GPIO_OutSet(&Module_rst_pin);
    mxc_delay(10000);
}

void MFIO_init_output(uint8_t value)
{
    gpio_cfg_t MFIO_pin;
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
    gpio_cfg_t MFIO_pin;
    MFIO_pin.port = PORT_5;
    MFIO_pin.mask = PIN_4;
    MFIO_pin.func = GPIO_FUNC_GPIO;
    MFIO_pin.pad = GPIO_PAD_INPUT;
    GPIO_Config(&MFIO_pin);
}

uint8_t MFIO_get_state(void)
{
    uint8_t st;
    gpio_cfg_t MFIO_pin;
    MFIO_pin.port = PORT_5;
    MFIO_pin.mask = PIN_4;
    MFIO_pin.func = GPIO_FUNC_GPIO;
    MFIO_pin.pad = GPIO_PAD_INPUT;
    st = GPIO_InGet(&MFIO_pin);
    return st;
}

void Get_RawData_SPO2(void)
{
	int temp;
    SS_STATUS ret_stt;
    uint8_t stt;
    uint8_t buff[1024];
    int index;
    int16_t accel;
    SS_STATUS status;    
    uint8_t cmd_bytes[10];


    cmd_bytes[0] = 0x40;                    //Set sample rate of MAX86141 to 100 Hz with 1 sample averaging 
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x12; 
    cmd_bytes[3] = 0x18;   
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 Integration time: 117us and ADC 1/2 range: 32uA
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x11; 
    cmd_bytes[3] = 0x3f;   
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 LED1(green) current to 0
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x23; 
    cmd_bytes[3] = 0x00;   
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 LED2(ir) current to half of full scale. Reduce if signal is saturated.
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x24; 
    cmd_bytes[3] = 0x7f;   
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 LED3(red) current to half of full scale. Reduce if signal is saturated.
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x25; 
    cmd_bytes[3] = 0x7f;   
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 LED2&3 current full range = 124mA  
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x2a; 
    cmd_bytes[3] = 0x3c;   
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 LED sequence LED3(red) -> LED2 (ir)
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x20; 
    cmd_bytes[3] = 0x23;   
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 FIFO configuration
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x0a; 
    cmd_bytes[3] = 0x0e;   
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                    //Set MAX86141 Interrupt configuration
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x02; 
    cmd_bytes[3] = 0xc6;   
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x10;                    //Set output mode to Sensor only
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
	status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x10;                    //Set Sensor Hub Interrupt Threshold
    cmd_bytes[1] = 0x01;
    cmd_bytes[2] = 0x05; 
	status = write_cmd(&cmd_bytes[0], 3, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                    //Enable AFE (e.g. MAX86141) *** with Sensor Hub Samples
    cmd_bytes[1] = 0x00;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x00;   
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    cmd_bytes[0] = 0x40;                    //Enable Accelerometer with Sensor Hub 
    cmd_bytes[1] = 0x04;
    cmd_bytes[2] = 0x01; 
    cmd_bytes[3] = 0x00;   
	status = write_cmd(&cmd_bytes[0], 4, 10);
    if(SS_SUCCESS != status) while(1);
    mxc_delay(200);
    
    while(1)
    {
        while(MFIO_get_state());
        
        temp = read_fifo_status(&stt);                  //AA 00 00 AB 00 08     read status
        while((temp != 0x00) || (stt != 0x08))
        {
            mxc_delay(1000);
            temp = read_fifo_status(&stt);              //AA 00 00 AB 00 08     read status
        }
        mxc_delay(20);
        
        num_avail_samples(&temp);                       //AA 12 00 AB 00 0F    read number of sample in FIFO
        mxc_delay(20);
        
        read_fifo_data(temp, 30, buff, 0);              //AA 12 01 AB 00 .......    dump FIFO data
        
        index = 7;
        gs_led_cnt.red_led_cnt = (uint32_t)(buff[0 + index] << 16) + (uint32_t)(buff[1 + index] << 8) + buff[2 + index];
        gs_led_cnt.ir_led_cnt  = (uint32_t)(buff[3 + index] << 16) + (uint32_t)(buff[4 + index] << 8) + buff[5 + index];
        
        index = 19;
        accel = (int16_t)(((uint16_t)buff[0 + index] << 8) + buff[1 + index]);
        gs_accel_data.x = (float)accel / 1000;
        accel = (int16_t)(((uint16_t)buff[2 + index] << 8) + buff[3 + index]);
        gs_accel_data.y = (float)accel / 1000;
        accel = (int16_t)(((uint16_t)buff[4 + index] << 8) + buff[5 + index]);
        gs_accel_data.z = (float)accel / 1000;
        
        index = 25;
        temp = ((uint16_t)buff[0 + index] << 8) + buff[1 + index];
        gs_WHRM_data.hr = (float)temp / 10;
        gs_WHRM_data.hr_confidence = buff[2 + index];
        gs_WHRM_data.status = buff[3 + index];
        //printf("heat rate = %f,  confidence = %d,  SpO2 = %f\n", gs_SpO2_data.heat_rate, gs_SpO2_data.confidence, gs_SpO2_data.SpO2);
    }
}

int main(void)
{
    sys_cfg_i2cm_t i2cm_sys_cfg;
    
    ioman_cfg_t io_cfg = IOMAN_I2CM1(IOMAN_MAP_A, 1);
    i2cm_sys_cfg.clk_scale = CLKMAN_SCALE_DIV_1;
    i2cm_sys_cfg.io_cfg = io_cfg;
    I2CM_Init(I2C_MASTER, &i2cm_sys_cfg, I2CM_SPEED_400KHZ);
        
    MFIO_init_output(1);
	Reset_module();
    mxc_delay(200000);
    MFIO_Input();
    mxc_delay(800000);
    
	if(0 != in_bootldr_mode())          //AA 02 00 AB 00 00     read mode
	{
		mxc_delay(1000);
		exit_from_bootloader();
	}
    mxc_delay(1000);
    get_ss_fw_version(&g_fw_version);	//AA FF 03 AB 00 01 07 00   read version
    mxc_delay(1000);
    
    Get_RawData_SPO2();
}

