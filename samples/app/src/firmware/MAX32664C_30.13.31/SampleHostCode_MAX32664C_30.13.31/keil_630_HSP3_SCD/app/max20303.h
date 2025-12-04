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
#ifndef __MAX20303_H_
#define __MAX20303_H_


#define MAX20303_SLAVE_ADDR		    (0x50 >> 1)
#define MAX20303_SLAVE_WR_ADDR		((MAX20303_SLAVE_ADDR << 1))
#define MAX20303_SLAVE_RD_ADDR		((MAX20303_SLAVE_ADDR << 1) | 1)


#define MAX20303_NO_ERROR   0
#define MAX20303_ERROR      -1

#define MAX20303_I2C_ADDR_FUEL_GAUGE    0x6C

#define MAX20303_LDO_MIN_MV 800
#define MAX20303_LDO_MAX_MV 3600
#define MAX20303_LDO_STEP_MV 100

#define MAX20303_OFF_COMMAND 0xB2

typedef enum{
	REG_HARDWARE_ID		= 0x00,		///< HardwareID Register
	REG_FIRMWARE_REV	= 0x01,		///< FirmwareID Register
	//					= 0x02,		///<
	REG_INT0			= 0x03,		///< Int0 Register
	REG_INT1			= 0x04,		///< Int1 Register
	REG_INT2			= 0x05,		///< Int2 Register
	REG_STATUS0			= 0x06,		///< Status Register 0
	REG_STATUS1			= 0x07,		///< Status Register 1
	REG_STATUS2			= 0x08,		///< Status Register 2
	REG_STATUS3			= 0x09,		///< Status Register 2
	//					= 0x0A,		///<
	REG_SYSTEM_ERROR	= 0x0B,		///< SystemError Register
	REG_INT_MASK0		= 0x0C,		///< IntMask0 Register
	REG_INT_MASK1		= 0x0D,		///< IntMask1 Register
	REG_INT_MASK2		= 0x0E,		///< IntMask1 Register
	REG_AP_DATOUT0		= 0x0F,     ///< APDataOut0 Register
	REG_AP_DATOUT1		= 0x10,     ///< APDataOut1 Register
	REG_AP_DATOUT2		= 0x11,     ///< APDataOut2 Register
	REG_AP_DATOUT3		= 0x12,     ///< APDataOut3 Register
	REG_AP_DATOUT4		= 0x13,     ///< APDataOut4 Register
	REG_AP_DATOUT5		= 0x14,     ///< APDataOut5 Register
	REG_AP_DATOUT6		= 0x15,     ///< APDataOut6 Register
	REG_AP_CMDOUT		= 0x17,     ///< APCmdOut Register
	REG_AP_RESPONSE		= 0x18,     ///< APResponse Register
	REG_AP_DATAIN0		= 0x19,
	REG_AP_DATAIN1		= 0x1A,
	REG_AP_DATAIN2		= 0x1B,
	REG_AP_DATAIN3		= 0x1C,
	REG_AP_DATAIN4		= 0x1D,
	REG_AP_DATAIN5		= 0x1E,
	//					= 0x1F,		///<
	REG_LDO_DIRECT		= 0x20,
	REG_MPC_DIRECTWRITE	= 0x21,
	REG_MPC_DIRECTRED	= 0x22,

	REG_LED_STEP_DIRECT	= 0x2C,
	REG_LED0_DIRECT		= 0x2D,
	REG_LED1_DIRECT		= 0x2E,
	REG_LED2_DIRECT		= 0x2F,


	REG_LDO1_CONFIG_WRITE = 0x40,
	REG_LDO1_CONFIG_READ  = 0x41,
	REG_LDO2_CONFIG_WRITE = 0x42,
	REG_LDO2_CONFIG_READ  = 0x43
}max20303_reg_t;
	


int MAX20303_led0on(char enable);
int MAX20303_led1on(char enable);
int MAX20303_led2on(char enable);
int MAX20303_BoostEnable(void);
int MAX20303_BuckBoostEnable(void);

/// @brief Enable the 1.8V output rail **/
int MAX20303_LDO1Config(void);

/// @brief Enable the 3V output rail **/
int MAX20303_LDO2Config(void);


int MAX20303_mv2bits(int mV);

/** @brief Power Off the board
 */
int MAX20303_PowerOffthePMIC(void);

/** @brief Power Off the board with 30ms delay
 */
int MAX20303_PowerOffDelaythePMIC(void);

/** @brief Soft reset the PMIC
 */
int MAX20303_SoftResetthePMIC(void);

/** @brief Hard reset the PMIC
 */
int MAX20303_HardResetthePMIC(void);

/** @brief check if can communicate with max20303
*/
char Max20303_CheckPMICHWID(void);

/** @brief CheckPMICStatusRegisters
*/
int MAX20303_CheckPMICStatusRegisters(unsigned char buf_results[5]);

int Max20303_BatteryGauge(unsigned char *batterylevel);

int MAX20303_WriteReg(max20303_reg_t reg, uint8_t value);
int MAX20303_ReadReg(max20303_reg_t reg, uint8_t *value);

int MAX20303_WriteRegMulti(max20303_reg_t reg, uint8_t *value, uint8_t len);
int MAX20303_ReadRegMulti(max20303_reg_t reg, uint8_t *value, uint8_t len);


/** @brief API Related Functions ***/

/***
 *  @brief  starts writing from ApResponse register 0x0F
 *  check the datasheet to determine the value of dataoutlen
 */
int MAX20303_AppWrite(uint8_t dataoutlen);

/** @brief starts reading from ApResponse register 0x18
 *  check the datasheet to determine the value of datainlen
 *  the result values are written into i2cbuffer
 *
*/
int MAX20303_AppRead(uint8_t datainlen);

int MAX20303_Init(void);

#endif /* __MAX20303_H_ */
