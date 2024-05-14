/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVER 0
uint8_t data_rx[512] = { 0 };
uint8_t data_tx[512] = { 0 };
uint8_t last_data_received = 0;
uint16_t last_data_num;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void write_reg(uint8_t reg, uint8_t data) {

	__disable_irq();
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
	reg = reg | 0x80;
	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
	__enable_irq();
}

uint8_t read_reg(uint8_t reg) {
	__disable_irq();
	uint8_t data = 0;
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
	HAL_SPI_Receive(&hspi1, &data, 1, 1000);
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
	__enable_irq();
	return data;
}

void SetOpMode(uint8_t opMode) {
	write_reg( REG_OPMODE, (read_reg( REG_OPMODE) & RF_OPMODE_MASK) | opMode);
}

void SetRfTxPower(int8_t power, uint32_t freq) {
	uint8_t paConfig = 0;
	uint8_t paDac = 0;

	paConfig = read_reg( REG_PACONFIG);
	paDac = read_reg( REG_PADAC);

	paConfig =
			(paConfig & RF_PACONFIG_PASELECT_MASK) | RF_PACONFIG_PASELECT_RFO;

	if ((paConfig & RF_PACONFIG_PASELECT_PABOOST)
			== RF_PACONFIG_PASELECT_PABOOST) {
		if (power > 17) {
			paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_ON;
		} else {
			paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_OFF;
		}
		if ((paDac & RF_PADAC_20DBM_ON) == RF_PADAC_20DBM_ON) {
			if (power < 5) {
				power = 5;
			}
			if (power > 20) {
				power = 20;
			}
			paConfig = (paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK)
					| (uint8_t) ((uint16_t) (power - 5) & 0x0F);
		} else {
			if (power < 2) {
				power = 2;
			}
			if (power > 17) {
				power = 17;
			}
			paConfig = (paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK)
					| (uint8_t) ((uint16_t) (power - 2) & 0x0F);
		}
	} else {
		if (power < -1) {
			power = -1;
		}
		if (power > 14) {
			power = 14;
		}
		paConfig = (paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK)
				| (uint8_t) ((uint16_t) (power + 1) & 0x0F);
	}
	write_reg( REG_PACONFIG, paConfig);
	write_reg( REG_PADAC, paDac);
}

void set_for_rx(uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
		uint32_t bandwidthAfc, uint16_t preambleLen, uint16_t symbTimeout,
		uint8_t fixLen, uint8_t payloadLen, uint8_t crcOn, uint8_t freqHopOn,
		uint8_t hopPeriod, uint8_t iqInverted, uint8_t rxContinuous) {

	SetOpMode( RF_OPMODE_SLEEP);
	write_reg( REG_OPMODE,
			(read_reg( REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK)
					| RFLR_OPMODE_LONGRANGEMODE_ON);

	write_reg( REG_DIOMAPPING1, 0x00);
	write_reg( REG_DIOMAPPING2, 0x00);
	write_reg(REG_LR_FIFORXBASEADDR, 0x00); //half of fifo for rx

	uint8_t LowDatarateOptimize = 0;
	if (datarate > 12) {
		datarate = 12;
	} else if (datarate < 6) {
		datarate = 6;
	}

	if (((bandwidth == 0) && ((datarate == 11) || (datarate == 12)))
			|| ((bandwidth == 1) && (datarate == 12))) {
		LowDatarateOptimize = 0x01;
	} else {
		LowDatarateOptimize = 0x00;
	}

	write_reg( REG_LR_MODEMCONFIG1,
			(read_reg( REG_LR_MODEMCONFIG1) &
			RFLR_MODEMCONFIG1_BW_MASK &
			RFLR_MODEMCONFIG1_CODINGRATE_MASK &
			RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
			RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
			RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK) | (bandwidth << 6)
					| (coderate << 3) | (fixLen << 2) | (crcOn << 1)
					| LowDatarateOptimize);

	write_reg( REG_LR_MODEMCONFIG2,
			(read_reg( REG_LR_MODEMCONFIG2) &
			RFLR_MODEMCONFIG2_SF_MASK &
			RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) | (datarate << 4)
					| ((symbTimeout >> 8)
							& ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK));

	write_reg( REG_LR_SYMBTIMEOUTLSB, (uint8_t) (symbTimeout & 0xFF));

	write_reg( REG_LR_PREAMBLEMSB, (uint8_t) ((preambleLen >> 8) & 0xFF));
	write_reg( REG_LR_PREAMBLELSB, (uint8_t) (preambleLen & 0xFF));

	if (fixLen == 1) {
		write_reg( REG_LR_PAYLOADLENGTH, payloadLen);
	}

	if (freqHopOn) {
		write_reg( REG_LR_PLLHOP,
				(read_reg( REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK)
						| RFLR_PLLHOP_FASTHOP_ON);
		write_reg( REG_LR_HOPPERIOD, hopPeriod);
	}

	if (datarate == 6) {
		write_reg( REG_LR_DETECTOPTIMIZE, (read_reg( REG_LR_DETECTOPTIMIZE) &
		RFLR_DETECTIONOPTIMIZE_MASK) |
		RFLR_DETECTIONOPTIMIZE_SF6);
		write_reg( REG_LR_DETECTIONTHRESHOLD,
		RFLR_DETECTIONTHRESH_SF6);
	} else {
		write_reg( REG_LR_DETECTOPTIMIZE, (read_reg( REG_LR_DETECTOPTIMIZE) &
		RFLR_DETECTIONOPTIMIZE_MASK) |
		RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
		write_reg( REG_LR_DETECTIONTHRESHOLD,
		RFLR_DETECTIONTHRESH_SF7_TO_SF12);
	}
}

void set_for_tx(int8_t power, uint32_t fdev, uint32_t bandwidth,
		uint32_t datarate, uint8_t coderate, uint16_t preambleLen,
		uint8_t fixLen, uint8_t crcOn, uint8_t freqHopOn, uint8_t hopPeriod,
		uint8_t iqInverted, uint32_t timeout) {
	SetOpMode( RF_OPMODE_SLEEP);
	write_reg( REG_OPMODE,
			(read_reg( REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK)
					| RFLR_OPMODE_LONGRANGEMODE_ON);

	write_reg( REG_DIOMAPPING1, 0x00);
	write_reg( REG_DIOMAPPING2, 0x00);
	write_reg(REG_LR_FIFOTXBASEADDR, 0x80); //half of fifo for tx

	uint8_t LowDatarateOptimize = 0;
	if (datarate > 12) {
		datarate = 12;
	} else if (datarate < 6) {
		datarate = 6;
	}
	if (((bandwidth == 0) && ((datarate == 11) || (datarate == 12)))
			|| ((bandwidth == 1) && (datarate == 12))) {
		LowDatarateOptimize = 0x01;
	} else {
		LowDatarateOptimize = 0x00;
	}

	if (freqHopOn) {
		write_reg( REG_LR_PLLHOP,
				(read_reg( REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK)
						| RFLR_PLLHOP_FASTHOP_ON);
		write_reg( REG_LR_HOPPERIOD, hopPeriod);
	}

	write_reg( REG_LR_MODEMCONFIG1,
			(read_reg( REG_LR_MODEMCONFIG1) &
			RFLR_MODEMCONFIG1_BW_MASK &
			RFLR_MODEMCONFIG1_CODINGRATE_MASK &
			RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
			RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
			RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK) | (bandwidth << 6)
					| (coderate << 3) | (fixLen << 2) | (crcOn << 1)
					| LowDatarateOptimize);

	write_reg( REG_LR_MODEMCONFIG2, (read_reg( REG_LR_MODEMCONFIG2) &
	RFLR_MODEMCONFIG2_SF_MASK) | (datarate << 4));

	write_reg( REG_LR_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
	write_reg( REG_LR_PREAMBLELSB, preambleLen & 0xFF);

	if (datarate == 6) {
		write_reg( REG_LR_DETECTOPTIMIZE, (read_reg( REG_LR_DETECTOPTIMIZE) &
		RFLR_DETECTIONOPTIMIZE_MASK) |
		RFLR_DETECTIONOPTIMIZE_SF6);
		write_reg( REG_LR_DETECTIONTHRESHOLD,
		RFLR_DETECTIONTHRESH_SF6);
	} else {
		write_reg( REG_LR_DETECTOPTIMIZE, (read_reg( REG_LR_DETECTOPTIMIZE) &
		RFLR_DETECTIONOPTIMIZE_MASK) |
		RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
		write_reg( REG_LR_DETECTIONTHRESHOLD,
		RFLR_DETECTIONTHRESH_SF7_TO_SF12);
	}

}

void set_channel(uint32_t freq) {
	freq = (uint32_t) ((double) freq / (double) FREQ_STEP);
	write_reg( REG_FRFMSB, (uint8_t) ((freq >> 16) & 0xFF));
	write_reg( REG_FRFMID, (uint8_t) ((freq >> 8) & 0xFF));
	write_reg( REG_FRFLSB, (uint8_t) (freq & 0xFF));
}

void Write(uint8_t addr, uint8_t *buffer, uint8_t size) {
	addr = addr | 0x80;
	__disable_irq();
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &addr, 1, 1000);
	HAL_SPI_Transmit(&hspi1, buffer, size, 1000);
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
	__enable_irq();

}

void WriteFifo(uint8_t *buffer, uint8_t size) {
	Write(0, buffer, size);
}

void lora_tx(uint8_t freqHopOn) {

	if (freqHopOn) {
		write_reg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
		RFLR_IRQFLAGS_RXDONE |
		RFLR_IRQFLAGS_PAYLOADCRCERROR |
		RFLR_IRQFLAGS_VALIDHEADER |
		//RFLR_IRQFLAGS_TXDONE |
				RFLR_IRQFLAGS_CADDONE |
				//RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
				RFLR_IRQFLAGS_CADDETECTED);

		// DIO0=TxDone, DIO2=FhssChangeChannel
		write_reg( REG_DIOMAPPING1,
				(read_reg( REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK
						& RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_01
						| RFLR_DIOMAPPING1_DIO2_00);
	} else {
		write_reg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
		RFLR_IRQFLAGS_RXDONE |
		RFLR_IRQFLAGS_PAYLOADCRCERROR |
		RFLR_IRQFLAGS_VALIDHEADER |
		//RFLR_IRQFLAGS_TXDONE |
				RFLR_IRQFLAGS_CADDONE |
				RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
				RFLR_IRQFLAGS_CADDETECTED);

		// DIO0=TxDone
		write_reg( REG_DIOMAPPING1,
				(read_reg( REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK)
						| RFLR_DIOMAPPING1_DIO0_01);
	}

	SetOpMode( RF_OPMODE_TRANSMITTER);
}

void lora_send(uint8_t *buffer, uint8_t size, uint8_t freqHopOn,
		uint8_t IqInverted) {
	if (IqInverted) {
		write_reg( REG_LR_INVERTIQ,
				((read_reg( REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK
						& RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF
						| RFLR_INVERTIQ_TX_ON));
		write_reg( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
	} else {
		write_reg( REG_LR_INVERTIQ,
				((read_reg( REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK
						& RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF
						| RFLR_INVERTIQ_TX_OFF));
		write_reg( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
	}

	// Initializes the payload size
	write_reg( REG_LR_PAYLOADLENGTH, size);

	// Full buffer used for Tx
	write_reg( REG_LR_FIFOTXBASEADDR, 0);
	write_reg( REG_LR_FIFOADDRPTR, 0);

	// FIFO operations can not take place in Sleep mode
	if ((read_reg( REG_OPMODE) & ~RF_OPMODE_MASK) == RF_OPMODE_SLEEP) {
		SetOpMode(RF_OPMODE_STANDBY);
		HAL_Delay(5);
	}
	// Write payload buffer
	WriteFifo(buffer, size);

	lora_tx(freqHopOn);
}

void lora_rx(uint32_t timeout, uint8_t IqInverted, uint8_t RxContinuous,
		uint8_t freqHopOn) {
	if (IqInverted) {
		write_reg( REG_LR_INVERTIQ,
				((read_reg( REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK
						& RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_ON
						| RFLR_INVERTIQ_TX_OFF));
		write_reg( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
	} else {
		write_reg( REG_LR_INVERTIQ,
				((read_reg( REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK
						& RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF
						| RFLR_INVERTIQ_TX_OFF));
		write_reg( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
	}

	if (freqHopOn) {
		write_reg( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
				//RFLR_IRQFLAGS_RXDONE |
				//RFLR_IRQFLAGS_PAYLOADCRCERROR |
				RFLR_IRQFLAGS_VALIDHEADER |
				RFLR_IRQFLAGS_TXDONE |
				RFLR_IRQFLAGS_CADDONE |
				//RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
						RFLR_IRQFLAGS_CADDETECTED);

		// DIO0=RxDone, DIO2=FhssChangeChannel
		write_reg( REG_DIOMAPPING1,
				(read_reg( REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK
						& RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_00
						| RFLR_DIOMAPPING1_DIO2_00);
	} else {
		write_reg( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
				//RFLR_IRQFLAGS_RXDONE |
				//RFLR_IRQFLAGS_PAYLOADCRCERROR |
				RFLR_IRQFLAGS_VALIDHEADER |
				RFLR_IRQFLAGS_TXDONE |
				RFLR_IRQFLAGS_CADDONE |
				RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
				RFLR_IRQFLAGS_CADDETECTED);

		// DIO0=RxDone
		write_reg( REG_DIOMAPPING1,
				(read_reg( REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK)
						| RFLR_DIOMAPPING1_DIO0_00);
	}
	write_reg( REG_LR_FIFORXBASEADDR, 0);
	write_reg( REG_LR_FIFOADDRPTR, 0);

	if (RxContinuous) {
		for (int i = 0; i < 0x3e; ++i) {
			asm("NOP");
			uint8_t val = read_reg(i);
			asm("NOP");
			asm("NOP");
		}

		SetOpMode( RFLR_OPMODE_RECEIVER);
	} else {
		SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE);
	}
}

void read_rx_data() {
	//check error flags
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
	uint8_t flags = read_reg(REG_LR_IRQFLAGS);
	uint8_t status = read_reg(1);
	if ((flags & 0x40)) {				//rx_done

		uint8_t nbytes = read_reg(REG_LR_RXNBBYTES);
		if (nbytes > 0) {
			write_reg(REG_LR_FIFOADDRPTR, read_reg(REG_LR_FIFORXCURRENTADDR));

			uint8_t address = REG_LR_FIFO;
			HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, &address, 1, 1000);
			HAL_SPI_Receive(&hspi1, data_rx, nbytes, 1000);
			last_data_received = 1;
		}
		//write_reg(REG_LR_IRQFLAGS, 0x40);//reset rx_done interrupt
	}
	write_reg(REG_LR_IRQFLAGS, 0xff); //clear interrupts

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */

	set_channel(RF_FREQUENCY);

	set_for_rx(LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
	LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
	LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
	LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
	LORA_IQ_INVERSION_ON, 1);
	set_for_tx(TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
	LORA_SPREADING_FACTOR, LORA_CODINGRATE,
	LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
	LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
	LORA_IQ_INVERSION_ON, 2000);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	uint8_t receive_buffer[32] = { 0 };
	uint8_t receive_buffer_ptr = 0;
	uint8_t last_receive_ptr = 0;
	while (1) {

		uint8_t buffer[70] =
				"Tw=21.9;Ta=22.6;Ma=37.7;Lo=2547;GN=50.088032;GE=14.452941;GT=102908\n\r";
		HAL_UART_Transmit(&huart2, buffer, 70, 1000);
		HAL_Delay(10000);
		//quickly send data

		if (last_data_received) {
			uint8_t numdata_to_send = snprintf(data_tx, 15, "LAST : %s\n\r",
					data_rx);
			lora_send(data_tx, numdata_to_send, LORA_FHSS_ENABLED,
			LORA_IQ_INVERSION_ON);
			last_data_received = 0;
		} else {
			lora_send(buffer, 70, LORA_FHSS_ENABLED, LORA_IQ_INVERSION_ON);
		}

		uint8_t status_val = read_reg(1);
		while ((status_val & 0xf) == 0x3) { //while transmission in progess
			status_val = read_reg(1);
		}
		//CLEAR TX COMPLETE INTERRUPT
		write_reg(REG_LR_IRQFLAGS, 0x8);
		lora_rx(1000, LORA_IQ_INVERSION_ON, 1, LORA_FHSS_ENABLED);

		HAL_Delay(4000);
		SetOpMode(RF_OPMODE_STANDBY);

		status_val = read_reg(1);
		asm("NOP");

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LORA_CS_Pin */
	GPIO_InitStruct.Pin = LORA_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LORA_CS_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
