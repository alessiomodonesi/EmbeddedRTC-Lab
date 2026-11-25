/*
 * SX1509_Utils.h
 *
 * Created on: May 5, 2025
 * Author: Alberto Morato
 *
 */

#ifndef INC_SX1509_UTILS_H_
#define INC_SX1509_UTILS_H_

#include "main.h"
#include "SX1509_Registers.h"

#define HAL_TIMEOUT 100

#define I2C_TIMEOUT 200

#define SX1509_I2C_ADDR1 0x3E //	SX1509 Proxy Sensors I2C address
#define SX1509_I2C_ADDR2 0x3F //	SX1509 Keypad I2C address

extern I2C_HandleTypeDef hi2c1;

void KeypadInit(void)
{
	uint8_t data;
	HAL_StatusTypeDef status;

	/***
	 * SX1509 KEYPAD
	 */

	/* Disable EXTI4_IRQ during SX1509 initialization */
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);

	/* Software reset */
	data = 0x12;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
	//   if (status != HAL_OK)
	//     printf("I2C communication error (%X).\n", status);

	data = 0x34;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	HAL_Delay(100);

	/* Set KeyPad scanning engine */

	/* Set RegClock to 0x40 (enable internal oscillator; 2MHz freq) */
	data = 0x40;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_CLOCK, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Set Bank A RegDir to 0xF0 (IO[0:3] as out) */
	data = 0xF0;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DIR_A, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Set Bank B RegDir to 0x0F (IO[8:11] as in) */
	data = 0x0F;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DIR_B, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Set Bank A RegOpenDrain to 0x0F (IO[0:3] as open-drain outputs) */
	data = 0x0F;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_OPEN_DRAIN_A, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Set Bank B RegPullup to 0x0F (pull-ups enabled on inputs IO[8:11]) */
	data = 0x0F;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_PULL_UP_B, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Set Bank B RegDebounceEnable to 0x0F (enable debouncing on IO[8:11]) */
	data = 0x0F;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DEBOUNCE_ENABLE_B, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Set RegDebounceConfig to 0x05 (16ms debounce time) */
	data = 0x05;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DEBOUNCE_CONFIG, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Set RegKeyConfig1 to 0x7D (8s auto-sleep; 32ms scan time per row) */
	data = 0x7D;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_KEY_CONFIG_1, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Set RegKeyConfig2 to 0x1B (4 rows; 4 columns) */
	data = 0x1B;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_KEY_CONFIG_2, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Enable EXTI4_IRQ after SX1509 initialization */
	HAL_Delay(100);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void LineSensorInit(void)
{
	uint8_t data;
	HAL_StatusTypeDef status;

	/***
	 * SX1509 LINE SENSOR
	 */

	/* Disable EXTI2_IRQ during SX1509 initialization */
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

	/* Software reset */
	data = 0x12;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	data = 0x34;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	HAL_Delay(100);

	/* Set RegDirA to 0xFF (all IO of Bank A configured as inputs) */
	data = 0xFF; // 0 = out; 1 = in
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_DIR_A, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Set RegDirB to 0xFF (all IO of Bank B configured as inputs) */
	data = 0xFF; // 0 = out; 1 = in
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_DIR_B, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Set RegInterruptMaskA to 0x00 (all IO of Bank A will trigger an interrupt) */
	data = 0x00;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_INTERRUPT_MASK_A, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Set RegSenseHighA to 0xAA (IO[7:4] of Bank A will trigger an interrupt on falling edge) */
	data = 0xAA;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_SENSE_HIGH_A, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Set RegSenseLowA to 0xAA (IO[3:0] of Bank A will trigger an interrupt on falling edge) */
	data = 0xAA;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_SENSE_LOW_A, 1, &data, 1, I2C_TIMEOUT);
	// if (status != HAL_OK)
	// 	printf("I2C communication error (%X).\n", status);

	/* Enable EXTI2_IRQ after SX1509 initialization */
	HAL_Delay(100);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

void Init_SX1509(void)
{
	// printf("Initilizing SX1509\n");

	KeypadInit();

	LineSensorInit();
}

#endif /* INC_SX1509_UTILS_H_ */
