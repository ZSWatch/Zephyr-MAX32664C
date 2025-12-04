/*******************************************************************************
 * Copyright (C) 2018 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "mxc_config.h"
#include <i2cm.h>
#include "max20303.h"
#include "board.h"

// Application Processor Interface Related Variables
uint8_t i2cbuffer_[16];
uint8_t appdatainoutbuffer_[8];
uint8_t appcmdoutvalue_;
int MAX20303_LDO1Config();
int MAX20303_LDO2Config();
int MAX20303_BuckBoostEnable();
extern void wait_ms(int ms);
void mxc_delay(int us);

/******************************************************************************/
int MAX20303_Init(void)
{
    uint8_t addr;
    uint8_t data[2];

    /* Setup the I2CM Peripheral to talk to the MAX14690 */
    I2CM_Init(MAX20303_I2CM, &max20303_sys_cfg, I2CM_SPEED_100KHZ);

    /* Attempt to read the ID from the device */
    addr = REG_HARDWARE_ID;
    if (I2CM_Read(MAX20303_I2CM, MAX20303_SLAVE_ADDR, &addr, 1, data, 2) != 2) {
        return E_COMM_ERR;
    }

	MAX20303_LDO1Config();
	MAX20303_LDO2Config();
	

	//max20303.BoostEnable();
	MAX20303_BuckBoostEnable();
	
    return E_NO_ERROR;
}

//******************************************************************************
int MAX20303_LDO1Config(void)
{
	int32_t ret = 0;

	appcmdoutvalue_ = 0x40;
	appdatainoutbuffer_[0] = 0x05;
	appdatainoutbuffer_[1] = 0x34;
	ret = MAX20303_AppWrite(2);
	
	return ret;
}

//******************************************************************************
int MAX20303_LDO2Config(void)
{
	int32_t ret = 0;
	appcmdoutvalue_ = 0x42;
	appdatainoutbuffer_[0] = 0x01;
	appdatainoutbuffer_[1] = 0x15;     // 0.9V + (0.1V * number)   =  3V
	ret = MAX20303_AppWrite(2);

	return ret;
}


//******************************************************************************
int MAX20303_WriteReg(max20303_reg_t reg, uint8_t value)
{
	int32_t ret;

	uint8_t cmdData[2] = {reg, value};

	//ret = m_i2c->write(m_writeAddress, cmdData, sizeof(cmdData));
	ret = I2CM_Write(MAX20303_I2CM, MAX20303_SLAVE_ADDR, NULL, 0, cmdData, sizeof(cmdData));
	//printf("MAX20303 write reg[0x%X]=0x%X, ret=%d\r\n", (uint32_t)reg, value, ret)

	if (ret != sizeof(cmdData))
		return MAX20303_ERROR;

	return MAX20303_NO_ERROR;
}


//******************************************************************************
int MAX20303_ReadReg(max20303_reg_t reg, uint8_t *value)
{
	int32_t ret;

	uint8_t data = reg;

	//ret = m_i2c->write(m_writeAddress, &data, sizeof(data));
	ret = I2CM_Write(MAX20303_I2CM, MAX20303_SLAVE_ADDR, NULL, 0, &data, sizeof(data));
	if (ret != sizeof(data)) {
//		printf("%s - failed - ret: %d\n", __func__);
		return MAX20303_ERROR;
	}

	//ret = m_i2c->read(m_readAddress, &data, sizeof(data));
	ret = I2CM_Read(MAX20303_I2CM, MAX20303_SLAVE_ADDR, NULL, 0, &data, sizeof(data));
	if (ret != sizeof(data)) {
//		printf("%s - failed - ret: %d\n", __func__);
		return MAX20303_ERROR;
	}

	*value = data;
//	printf("MAX20303 read reg[0x%X]=0x%X, ret=%d\r\n", (uint32_t)reg, *value, ret);
	return MAX20303_NO_ERROR;
}

//******************************************************************************
int MAX20303_ReadRegMulti(max20303_reg_t reg, uint8_t *value, uint8_t len){
	int32_t ret;
	uint8_t data = reg;

	//ret = m_i2c->write(m_writeAddress, &data, sizeof(data));
	ret = I2CM_Write(MAX20303_I2CM, MAX20303_SLAVE_ADDR, NULL, 0, &data, sizeof(data));
	
	if (ret != sizeof(data)) {
//		printf("%s - failed - ret: %d\n", __func__);
		return MAX20303_ERROR;
	}

	//ret = m_i2c->read(m_readAddress, (char *)value, len);
	ret = I2CM_Read(MAX20303_I2CM, MAX20303_SLAVE_ADDR, NULL, 0, value, len);
	if (ret != sizeof(data)) {
//		printf("%s - failed - ret: %d\n", __func__);
		return MAX20303_ERROR;
	}

//	printf("MAX20303 read reg[0x%X]=0x%X, ret=%d\r\n", (uint32_t)reg, value, ret);
	return MAX20303_NO_ERROR;
}

//******************************************************************************
int MAX20303_WriteRegMulti(max20303_reg_t reg, uint8_t *value, uint8_t len){
	int32_t ret;
	i2cbuffer_[0] = reg;
	memcpy(&i2cbuffer_[1], value, len);

	//ret = m_i2c->write(m_writeAddress, (char *)i2cbuffer_, (len+1));
	ret = I2CM_Write(MAX20303_I2CM, MAX20303_SLAVE_ADDR, NULL, 0, i2cbuffer_, len+1);
	
	//printf("MAX20303 write multi-reg[0x%X]=0x%X, ret=%d\r\n", (uint32_t)reg, value[0], ret);

	if (ret != (len+1))
		return MAX20303_ERROR;

	return MAX20303_NO_ERROR;
}
//******************************************************************************
int MAX20303_mv2bits(int mV)
{
    int regBits;

    if (( MAX20303_LDO_MIN_MV <= mV) && (mV <= MAX20303_LDO_MAX_MV)) {
        regBits = (mV -  MAX20303_LDO_MIN_MV) /  MAX20303_LDO_STEP_MV;
    } else {
        return -1;
    }

    return regBits;
}
//******************************************************************************
int MAX20303_PowerOffthePMIC(){
	int ret;
	appdatainoutbuffer_[0] = 0xB2;
	appcmdoutvalue_ = 0x80;
	ret = MAX20303_AppWrite(1);

	if(appcmdoutvalue_ != 0x80){
		ret |= MAX20303_ERROR;
	}

	return ret;
}
//******************************************************************************
int MAX20303_PowerOffDelaythePMIC(){
	int ret;
	appdatainoutbuffer_[0] = 0xB2;
	appcmdoutvalue_ = 0x84;
	ret = MAX20303_AppWrite(1);

	if(appcmdoutvalue_ != 0x80){
		ret |= MAX20303_ERROR;
	}

	return ret;
}

//******************************************************************************
int MAX20303_SoftResetthePMIC(){
	int ret;
	appdatainoutbuffer_[0] = 0xB3;
	appcmdoutvalue_ = 0x81;
	ret = MAX20303_AppWrite(1);

	if(appcmdoutvalue_ != 0x81){
		ret |= MAX20303_ERROR;
	}

	return ret;
}
//******************************************************************************
int MAX20303_HardResetthePMIC(){
	int ret;
	appdatainoutbuffer_[0] = 0xB4;
	appcmdoutvalue_ = 0x82;
	ret = MAX20303_AppWrite(1);

	if(appcmdoutvalue_ != 0x82){
		ret |= MAX20303_ERROR;
	}

	return ret;
}

//******************************************************************************
int MAX20303_AppWrite(uint8_t dataoutlen){
	int ret;

	ret = MAX20303_WriteRegMulti(REG_AP_DATOUT0, appdatainoutbuffer_, dataoutlen);
	//printf("write multi reg  ret is %d \n", ret);
	
	ret |= MAX20303_WriteReg(REG_AP_CMDOUT, appcmdoutvalue_);
	//printf("write reg  ret is %d \n", ret);
	
	mxc_delay(10000);
	ret |= MAX20303_ReadReg(REG_AP_RESPONSE, &appcmdoutvalue_);
	//printf("APP wirte multi reg  ret is %d \n", ret);
	
	if(ret != 0)
		return MAX20303_ERROR;

	return MAX20303_NO_ERROR;
}


//******************************************************************************
int MAX20303_AppRead(uint8_t datainlen){
	int ret;

	ret = MAX20303_WriteReg(REG_AP_CMDOUT, appcmdoutvalue_);
	mxc_delay(10000);
	ret |= MAX20303_ReadRegMulti(REG_AP_RESPONSE, i2cbuffer_, datainlen);
	if(ret != 0)
		return MAX20303_ERROR;

	return MAX20303_NO_ERROR;
}

//******************************************************************************
char MAX20303_CheckPMICHWID(){
	int ret;
	uint8_t value = 0x00;

	ret = MAX20303_ReadReg(REG_HARDWARE_ID, &value);
	if(ret != MAX20303_NO_ERROR)
		return false;

	if(value == 0x02)
		return true;
	else
		return false;
}

//******************************************************************************
int MAX20303_CheckPMICStatusRegisters(unsigned char buf_results[5]){
	int ret;
	ret  = MAX20303_ReadReg(REG_STATUS0, &buf_results[0]);
	ret |= MAX20303_ReadReg(REG_STATUS1, &buf_results[1]);
	ret |= MAX20303_ReadReg(REG_STATUS2, &buf_results[2]);
	ret |= MAX20303_ReadReg(REG_STATUS3, &buf_results[3]);
	ret |= MAX20303_ReadReg(REG_SYSTEM_ERROR, &buf_results[4]);
	return ret;
}

//******************************************************************************
int Max20303_BatteryGauge(unsigned char *batterylevel){
    int ret;
    uint8_t data[2];

    data[0] = 0x04;
    //ret = m_i2c->write(MAX20303_I2C_ADDR_FUEL_GAUGE, data, 1);
	ret = I2CM_Write(MAX20303_I2CM, MAX20303_SLAVE_ADDR, NULL, 0, data, 1);
	
    if(ret != 0){
//        printf("Max20303_FuelGauge has failed\r\n");
    }

    //ret = m_i2c->read(MAX20303_I2C_ADDR_FUEL_GAUGE | 1, data, 2);
	ret = I2CM_Read(MAX20303_I2CM, MAX20303_SLAVE_ADDR, NULL, 0, data, 2);
	
    if(ret != 0){
//        printf("Max20303_FuelGauge has failed\r\n");
    }

    // if the level is more than 100 assume the battery is not connected
    if(data[0] > 100){
        *batterylevel = 0;
    } else{

        *batterylevel = data[0];
    }
    return 0;
}


//******************************************************************************
int MAX20303_led0on(char enable) {

	if(enable)
		return MAX20303_WriteReg(REG_LED0_DIRECT, 0x21);
	else
		return MAX20303_WriteReg(REG_LED0_DIRECT, 0x01);
}

//******************************************************************************
int MAX20303_led1on(char enable) {
	if(enable)
		return MAX20303_WriteReg(REG_LED1_DIRECT, 0x21);
	else
		return MAX20303_WriteReg(REG_LED1_DIRECT, 0x01);
}

//******************************************************************************
int MAX20303_led2on(char enable) {
	if(enable)
		return MAX20303_WriteReg(REG_LED2_DIRECT, 0x21);
	else
		return MAX20303_WriteReg(REG_LED2_DIRECT, 0x01);
}


//******************************************************************************
int MAX20303_BoostEnable(void) {
	MAX20303_WriteReg(REG_AP_DATOUT3, 0x00);	// 00 : 5V
	MAX20303_WriteReg(REG_AP_DATOUT0, 0x01);	// Boost Enabled
	MAX20303_WriteReg(REG_AP_CMDOUT, 0x30);
	return MAX20303_NO_ERROR;
}

//******************************************************************************
int MAX20303_BuckBoostEnable(void)
{
	int ret = 0;

	ret |= MAX20303_WriteReg( REG_AP_DATOUT0,  0x00);    // Reserved = 0x00
	ret |= MAX20303_WriteReg( REG_AP_DATOUT1,  0x04);    // BBstlSet = 0b'100   Buck Boost Peak current Limit = 200mA
	ret |= MAX20303_WriteReg( REG_AP_DATOUT2,  0x19);    // BBstVSet = 0b'11001  Buck Boost Output Voltage = 5V
	ret |= MAX20303_WriteReg( REG_AP_DATOUT3,  0x01);    // BBstRipRed = 1 Ripple Reduction
	// BBstAct    = 1 Actively discharged in Hard-Reset or Enable Low
	// BBstPas    = 1 Passively discharged in Hard-Reset or Enable Low
	// BBstMd     = 1 Damping Enabled
	// BBstInd    = 0  Inductance is 4.7uH
	// BBstEn     = 0b'01 Enabled
	ret |= MAX20303_WriteReg( REG_AP_CMDOUT, 0x70);
	if (ret != 0)
		return MAX20303_ERROR;

	return MAX20303_NO_ERROR;
}
