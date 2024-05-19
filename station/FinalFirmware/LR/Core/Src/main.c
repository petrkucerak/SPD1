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
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVER 0
uint8_t data_rx [512] = {0};
uint8_t data_tx [512] = {0};
uint8_t last_data_received = 0;
uint16_t last_data_num;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart4_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM21_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float last_valid_latt = 0;
float last_valid_long = 0;
float last_valid_time = 0;
char last_valid_ns = 0;
char last_valid_we = 0;

uint16_t last_transmission_ID = 0;
uint8_t last_transmission_Checksum = 0;
uint8_t last_transmission_BitwiseParity = 0;
uint8_t last_transmission_confirmed = 0;





#define RXBUFLEN  4096
#define ADCDATALEN 2
char rx_data [RXBUFLEN] = {0};
uint16_t last_rx = 0;
#define RXLINBUFLEN 128
char rx_lin_buf [RXLINBUFLEN+1]= {0};

char tempbuffer [] = "$GNGLL,5005.2816,N,01427.1765,E,102908.000,A,D*48\r\n";

#define TXBUFLEN 110
char txbuf [TXBUFLEN] = {0};

uint16_t adcdata [ADCDATALEN] = {0};


enum msg_types {msg_unknown, msg_gnss_pos, msg_gnss_sat, msg_gps_pos, msg_gps_sat};

char* str_msg_gnss_pos = "$GNGLL";
char* str_msg_gps__pos = "$GPGLL";


//functions

uint8_t random_bit(){
	return adcdata[1]&1;
}

uint16_t random_uin16_t(){
	uint16_t random_code = 0;
	for(int i=0; i<16; ++i){
		random_code = random_code<<1;
		random_code = random_code|random_bit();
		HAL_Delay(5);
	}
	return random_code;
}

uint8_t strings_match_six(char *a, char* b){
	for(int i=0; i<6; ++i){
		if(a[i]!= b[i]){
			return 0;
		}
	}
	return 1;
}



uint8_t get_msg_type(){
	uint8_t msg_type = msg_unknown;
	if(rx_lin_buf[0]=='$'){
		if(strings_match_six(rx_lin_buf, str_msg_gnss_pos)){
			msg_type = msg_gnss_pos;
		}
		if(strings_match_six(rx_lin_buf, str_msg_gps__pos)){
			msg_type = msg_gps_pos;
		}

	}
	return msg_type;
}

int transfer_to_lin_buf(){
	//transfers one gps message into linear buffer
	int i=0;
	int end_found = 0;
	for(i=0; i<RXLINBUFLEN;++i){
		int idx = (last_rx+i)%RXBUFLEN;
		if(rx_data[idx] == '*'){
			if(rx_data[(idx+1)%RXBUFLEN]!=0){
				if(rx_data[(idx+2)%RXBUFLEN]!=0){
					end_found = 1;
				}
			}
			break;
		}
	}
	if(end_found==1){
		for(int j=0; j<=i+4; ++j){
			int idx = (last_rx+j)%RXBUFLEN;
			rx_lin_buf[j] = rx_data[idx];
			rx_data[idx] = 0;
		}
		for(int j=i+4+1; j<RXLINBUFLEN; ++j){
			rx_lin_buf[j] = 0;
		}
		last_rx = (last_rx+i+5)%RXBUFLEN;
	}
	return end_found;
}

uint8_t decode_gps(float *lat, float *lon, float* time, char *ns, char *we){
	//returns 1 if data is valid
	char valid = 0;

	char *endptr;
	char* parsedtext = &rx_lin_buf[7];
	*lat = strtof(parsedtext, &endptr);
	if(endptr[1]!=','){
		*ns = endptr[1];
		endptr= endptr+3;
	}else{
		endptr = endptr+2;
	}
	parsedtext = endptr;
	*lon = strtof(parsedtext, &endptr);
	if(endptr[1]!=','){
		*we = endptr[1];
		endptr= endptr+3;
	}else{
		endptr = endptr+2;
	}

	parsedtext = endptr;
	*time = strtof(parsedtext, &endptr);
	if(endptr[1]!=','){
		valid = endptr[1];
		endptr= endptr+3;
	}else{
		endptr = endptr+2;
	}

	//convert the values to degrees
	*lat = *lat/100;
	*lat = (floor(*lat)+(*lat-floor(*lat))/60.0*100.0);
	*lon = *lon/100;
	*lon = (floor(*lon)+(*lon-floor(*lon))/60.0*100.0);
	return(valid == 'A');
}

void handle_gps(){
	while(transfer_to_lin_buf()){
		uint8_t msg_type=  get_msg_type();
		if(msg_type == msg_gnss_pos || msg_type== msg_gps_pos){
			float temp_lat = 0;
			float temp_time = 0;
			float temp_long = 0;
			char temp_ns =0;
			char temp_we =0;
			if(decode_gps(&temp_lat, &temp_long, &temp_time, &temp_ns, &temp_we)){
				last_valid_latt = temp_lat;
				last_valid_long = temp_long;
				last_valid_time = temp_time;
				last_valid_ns = temp_ns;
				last_valid_we = temp_we;
			}
		}
	}

}






void delay_us (uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim21,0);
    while ((__HAL_TIM_GET_COUNTER(&htim21))<us);
}

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	GPIOB->BRR |= TEMPSIG_Pin; //reset pin
	delay_us (480);   // delay according to datasheet
	GPIOB->BSRR |= TEMPSIG_Pin; //reset pin
	delay_us (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (TEMPSIG_GPIO_Port, TEMPSIG_Pin))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = 0;

	delay_us (400); // 480 us delay totally.

	return Response;
}

void DS18B20_Write (uint8_t data)
{

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1
			GPIOB->BRR |= TEMPSIG_Pin; //reset pin
			delay_us (1);  // wait for 1 us

			GPIOB->BSRR |= TEMPSIG_Pin; //reset pin
			delay_us (70);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0
			GPIOB->BRR |= TEMPSIG_Pin; //reset pin
			delay_us (70);  // wait for 60 us

			GPIOB->BSRR |= TEMPSIG_Pin; //reset pin
			delay_us (1);  // wait for 1 us
		}
	}

}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;
	HAL_GPIO_WritePin(TEMPSIG_GPIO_Port, TEMPSIG_Pin, GPIO_PIN_SET);

	for (int i=0;i<8;i++)
	{
		GPIOB->BRR |= TEMPSIG_Pin; //reset pin
		//HAL_GPIO_WritePin(TEMPSIG_GPIO_Port, TEMPSIG_Pin, GPIO_PIN_RESET);
		delay_us (1);  // wait for 3 us
		GPIOB->BSRR |= TEMPSIG_Pin; //reset pin
		delay_us (1);
		//HAL_GPIO_WritePin(TEMPSIG_GPIO_Port, TEMPSIG_Pin, GPIO_PIN_SET);
		if (HAL_GPIO_ReadPin (TEMPSIG_GPIO_Port, TEMPSIG_Pin))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay_us (70);  // wait for 60 us
	}
	return value;
}

float get_DS18B20_temp(void){

	float floattemp = 0;

	__disable_irq();  // disable all interrupts
		if(DS18B20_Start ()){ //sensor detected
		__enable_irq();   // enable all interrupts
		HAL_Delay (1);
		__disable_irq();  // disable all interrupts
		DS18B20_Write (0xCC);  // skip ROM
		DS18B20_Write (0x44);  // convert t
		__enable_irq();   // enable all interrupts
		HAL_Delay (800);
		__disable_irq();  // disable all interrupts
		if(DS18B20_Start ()){
			__enable_irq();   // enable all interrupts
					HAL_Delay (1);
					__disable_irq();  // disable all interrupts
			DS18B20_Write (0xCC);  // skip ROM
			DS18B20_Write (0xBE);  // Read Scratch-pad
			uint8_t Temp_byte1 = DS18B20_Read();
			uint8_t Temp_byte2 = DS18B20_Read();
			__enable_irq();   // enable all interrupts
			int16_t temp = Temp_byte1 | Temp_byte2<<8;
			floattemp = temp/8.0;
			floattemp = floattemp*0.5;
			asm("NOP");
		}

	}
		__enable_irq();   // enable all interrupts


	return floattemp;
}



void write_reg(uint8_t reg, uint8_t data){

	__disable_irq();
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
	reg = reg|0x80;
	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
	__enable_irq();
}

uint8_t read_reg(uint8_t reg){
	__disable_irq();
	uint8_t data =0;
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
	HAL_SPI_Receive(&hspi1, &data, 1, 1000);
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
	__enable_irq();
	return data;
}

void SetOpMode( uint8_t opMode )
{
    write_reg( REG_OPMODE, (read_reg( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}

void SetRfTxPower( int8_t power ,uint32_t freq)
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = read_reg( REG_PACONFIG );
    paDac = read_reg( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | RF_PACONFIG_PASELECT_RFO;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    write_reg( REG_PACONFIG, paConfig );
    write_reg( REG_PADAC, paDac );
}



void set_for_rx(uint32_t bandwidth,
        uint32_t datarate, uint8_t coderate,
        uint32_t bandwidthAfc, uint16_t preambleLen,
        uint16_t symbTimeout, uint8_t fixLen,
        uint8_t payloadLen,
		uint8_t crcOn, uint8_t freqHopOn, uint8_t hopPeriod,
		uint8_t iqInverted, uint8_t rxContinuous ){

    	SetOpMode( RF_OPMODE_SLEEP );
	    write_reg( REG_OPMODE, (read_reg( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

	    write_reg( REG_DIOMAPPING1, 0x00 );
	    write_reg( REG_DIOMAPPING2, 0x00 );
		write_reg(REG_LR_FIFORXBASEADDR, 0x00); //half of fifo for rx

	uint8_t LowDatarateOptimize=0;
    if( datarate > 12 )
    {
        datarate = 12;
    }
    else if( datarate < 6 )
    {
        datarate = 6;
    }

    if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
    {
    	LowDatarateOptimize = 0x01;
    }
    else
    {
    	LowDatarateOptimize = 0x00;
    }

    write_reg( REG_LR_MODEMCONFIG1,
                 ( read_reg( REG_LR_MODEMCONFIG1 ) &
                   RFLR_MODEMCONFIG1_BW_MASK &
                   RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                   RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
                   RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
                   RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK ) |
                   ( bandwidth << 6 ) | ( coderate << 3 ) |
                   ( fixLen << 2 ) | ( crcOn << 1 ) |
				   LowDatarateOptimize );

    write_reg( REG_LR_MODEMCONFIG2,
                 ( read_reg( REG_LR_MODEMCONFIG2 ) &
                   RFLR_MODEMCONFIG2_SF_MASK &
                   RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                   ( datarate << 4 ) |
                   ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

    write_reg( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

    write_reg( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
    write_reg( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

    if( fixLen == 1 )
    {
        write_reg( REG_LR_PAYLOADLENGTH, payloadLen );
    }

    if(freqHopOn)
    {
        write_reg( REG_LR_PLLHOP, ( read_reg( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
        write_reg( REG_LR_HOPPERIOD, hopPeriod );
    }

    if( datarate == 6 )
    {
        write_reg( REG_LR_DETECTOPTIMIZE,
                     ( read_reg( REG_LR_DETECTOPTIMIZE ) &
                       RFLR_DETECTIONOPTIMIZE_MASK ) |
                       RFLR_DETECTIONOPTIMIZE_SF6 );
        write_reg( REG_LR_DETECTIONTHRESHOLD,
                     RFLR_DETECTIONTHRESH_SF6 );
    }
    else
    {
        write_reg( REG_LR_DETECTOPTIMIZE,
                     ( read_reg( REG_LR_DETECTOPTIMIZE ) &
                     RFLR_DETECTIONOPTIMIZE_MASK ) |
                     RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
        write_reg( REG_LR_DETECTIONTHRESHOLD,
                     RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
    }
}

void set_for_tx(int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
						uint8_t fixLen, uint8_t crcOn, uint8_t freqHopOn,
                        uint8_t hopPeriod, uint8_t iqInverted, uint32_t timeout )
{
	SetOpMode( RF_OPMODE_SLEEP );
	write_reg( REG_OPMODE, (read_reg( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

	write_reg( REG_DIOMAPPING1, 0x00 );
	write_reg( REG_DIOMAPPING2, 0x00 );
	write_reg(REG_LR_FIFOTXBASEADDR, 0x80); //half of fifo for tx



	uint8_t LowDatarateOptimize=0;
	if( datarate > 12 )
	{
	    datarate = 12;
		}
	else if( datarate < 6 )
	{
	    datarate = 6;
	}
	if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
	( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
	{
		LowDatarateOptimize = 0x01;
	}
	else
	{
		LowDatarateOptimize = 0x00;
	}

	if(freqHopOn)
	{
		write_reg( REG_LR_PLLHOP, ( read_reg( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
	    write_reg( REG_LR_HOPPERIOD,hopPeriod );
	}

	write_reg( REG_LR_MODEMCONFIG1,
	                         ( read_reg( REG_LR_MODEMCONFIG1 ) &
	                           RFLR_MODEMCONFIG1_BW_MASK &
	                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
	                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
	                           RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
	                           RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK ) |
	                           ( bandwidth << 6 ) | ( coderate << 3 ) |
	                           ( fixLen << 2 ) | ( crcOn << 1 ) |
							   LowDatarateOptimize);

	write_reg( REG_LR_MODEMCONFIG2,
	                        ( read_reg( REG_LR_MODEMCONFIG2 ) &
	                          RFLR_MODEMCONFIG2_SF_MASK ) |
	                          ( datarate << 4 ) );


	write_reg( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
	write_reg( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

	if( datarate == 6 )
	{
		write_reg( REG_LR_DETECTOPTIMIZE,
				( read_reg( REG_LR_DETECTOPTIMIZE ) &
						RFLR_DETECTIONOPTIMIZE_MASK ) |
						RFLR_DETECTIONOPTIMIZE_SF6 );
	            write_reg( REG_LR_DETECTIONTHRESHOLD,
	                    RFLR_DETECTIONTHRESH_SF6 );
	}
	else
	{
		write_reg( REG_LR_DETECTOPTIMIZE,
				( read_reg( REG_LR_DETECTOPTIMIZE ) &
						RFLR_DETECTIONOPTIMIZE_MASK ) |
	                    RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
		write_reg( REG_LR_DETECTIONTHRESHOLD,
				RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
	}

}

void set_channel(uint32_t freq){
	freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
	    write_reg( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
	    write_reg( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) &  0xFF ) );
	    write_reg( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}

void Write( uint8_t addr, uint8_t *buffer, uint8_t size )
{
	addr = addr | 0x80;
	__disable_irq();
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, 1000);
    HAL_SPI_Transmit(&hspi1, buffer, size, 1000);
    HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
    __enable_irq();

}

void WriteFifo( uint8_t *buffer, uint8_t size )
{
    Write( 0, buffer, size );
}


void lora_tx(uint8_t freqHopOn)
{

            if(freqHopOn)
            {
                write_reg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone, DIO2=FhssChangeChannel
                write_reg( REG_DIOMAPPING1, (read_reg( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                write_reg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                write_reg( REG_DIOMAPPING1, (read_reg( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }




	SetOpMode( RF_OPMODE_TRANSMITTER );
}

void lora_send(uint8_t *buffer, uint8_t size , uint8_t freqHopOn, uint8_t IqInverted)
{
                if(IqInverted)
            {
                write_reg( REG_LR_INVERTIQ, ( (read_reg( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
                write_reg( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                write_reg( REG_LR_INVERTIQ, ( (read_reg( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                write_reg( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            // Initializes the payload size
            write_reg( REG_LR_PAYLOADLENGTH, size );

            // Full buffer used for Tx
            write_reg( REG_LR_FIFOTXBASEADDR, 0 );
            write_reg( REG_LR_FIFOADDRPTR, 0 );

            // FIFO operations can not take place in Sleep mode
            if( (read_reg( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
            {
                SetOpMode(RF_OPMODE_STANDBY);
                HAL_Delay(5);
            }
            // Write payload buffer
            WriteFifo( buffer, size );



            lora_tx( freqHopOn);
}

void lora_rx( uint32_t timeout, uint8_t IqInverted, uint8_t RxContinuous,  uint8_t freqHopOn)
{
    if(IqInverted)
	{
		write_reg( REG_LR_INVERTIQ, ( (read_reg( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
		write_reg( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
	}
	else
	{
		write_reg( REG_LR_INVERTIQ, ( (read_reg( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
		write_reg( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
	}

	if(freqHopOn)
	{
		write_reg( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
				//RFLR_IRQFLAGS_RXDONE |
				//RFLR_IRQFLAGS_PAYLOADCRCERROR |
				RFLR_IRQFLAGS_VALIDHEADER |
				RFLR_IRQFLAGS_TXDONE |
				RFLR_IRQFLAGS_CADDONE |
				//RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
				RFLR_IRQFLAGS_CADDETECTED );

		// DIO0=RxDone, DIO2=FhssChangeChannel
		write_reg( REG_DIOMAPPING1, (read_reg( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
	}
	else
	{
		write_reg( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
				//RFLR_IRQFLAGS_RXDONE |
				//RFLR_IRQFLAGS_PAYLOADCRCERROR |
				RFLR_IRQFLAGS_VALIDHEADER |
				RFLR_IRQFLAGS_TXDONE |
				RFLR_IRQFLAGS_CADDONE |
				RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
				RFLR_IRQFLAGS_CADDETECTED );

		// DIO0=RxDone
		write_reg( REG_DIOMAPPING1, (read_reg( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
	}
	write_reg( REG_LR_FIFORXBASEADDR, 0 );
	write_reg( REG_LR_FIFOADDRPTR, 0 );




        if( RxContinuous)
        {
            for(int i=0; i<0x3e; ++i){
            			  asm("NOP");
            			  uint8_t val = read_reg(i);
            			  asm("NOP");
            			  asm("NOP");
            		  }

            SetOpMode( RFLR_OPMODE_RECEIVER );
        }
        else
        {
            SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
}

void check_ack(){
	uint8_t byte_offset = 4;
	uint16_t received_ID =( data_rx[byte_offset]) | (data_rx[byte_offset+1]<<8);
	uint8_t received_Checksum = data_rx[byte_offset+5];
	uint8_t received_BitwiseParity = data_rx[byte_offset+6];
	if(received_ID != last_transmission_ID){
		return;
	}
	if(received_Checksum != last_transmission_Checksum){
		return;
	}
	if(received_BitwiseParity != last_transmission_BitwiseParity){
		return;
	}
	if((data_rx[byte_offset+2] != 'A') ||(data_rx[byte_offset+3] != 'C')||(data_rx[byte_offset+4] != 'K')){
		return;
	}
	last_transmission_confirmed = 1;
}

void read_rx_data(){
	//check error flags
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
	uint8_t flags = read_reg(REG_LR_IRQFLAGS);
	uint8_t status = read_reg(1);
	if((flags&0x40)){//rx_done

		uint8_t nbytes = read_reg(REG_LR_RXNBBYTES);
		if(nbytes>0){
			write_reg(REG_LR_FIFOADDRPTR, read_reg(REG_LR_FIFORXCURRENTADDR));
			uint8_t address = REG_LR_FIFO;
			HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, &address, 1, 1000);
			HAL_SPI_Receive(&hspi1, data_rx, nbytes, 1000);
			check_ack();
		}
		//write_reg(REG_LR_IRQFLAGS, 0x40);//reset rx_done interrupt
	}
	write_reg(REG_LR_IRQFLAGS, 0xff); //clear interrupts


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	read_rx_data();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_DMA_Init();
  MX_USART4_UART_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM21_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart4, rx_data, RXBUFLEN);
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
  HAL_GPIO_WritePin(TEMPMOISTSENON_GPIO_Port, TEMPMOISTSENON_Pin, GPIO_PIN_SET); //turn on moisture sensor
  HAL_GPIO_WritePin(TEMPPWRT_GPIO_Port, TEMPPWRT_Pin, GPIO_PIN_SET); //turn on temp sensor
  HAL_Delay(100);
  HAL_ADC_Start_DMA(&hadc, adcdata, ADCDATALEN);
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_Base_Start(&htim21);





  set_channel(RF_FREQUENCY);

  set_for_rx(LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                           LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                           LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                           LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                           LORA_IQ_INVERSION_ON, 1 );
  set_for_tx(TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                       LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                       LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                       LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                       LORA_IQ_INVERSION_ON, 2000 );




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */




  uint8_t receive_buffer[32] = {0};
  uint8_t receive_buffer_ptr = 0;
  uint8_t last_receive_ptr = 0;
  while (1)
  	  {

	  	  //prepare data_to_send
	  	  float temp =  get_DS18B20_temp();
	  	  uint8_t i2c_data [10] = {0};
	  	  HAL_I2C_Mem_Read(&hi2c2, 0x44<<1, 0x2C06, 2, i2c_data, 6, 1000);
	  	  float moisttemp = -45+175*((i2c_data[0]<<8)|i2c_data[1])/((1<<16)-1.0);
	  	  float moistmois = 100*((i2c_data[3]<<8)|i2c_data[4])/((1<<16)-1.0);


	  	  int j = snprintf(txbuf, TXBUFLEN, "    Tw=%.1f;"
			  	  	  	  	  	  	  	  "Ta=%.1f;"
			  	  	  	  	  	  	  	  "Ma=%.1f;"
			  	  	  	  	  	  	  	  "Lo=%u;"
			  	  	  	  	  	  	  	  "G%c=%.6f;"
	  			  	  	  	  	  	  	  "G%c=%.6f  ",
			  	  	  	  	  	  	  	   temp, moisttemp, moistmois,
										  	  adcdata[0],
											  last_valid_ns,last_valid_latt,
											  last_valid_we,last_valid_long);

	  	  //quickly send data
	  	  uint16_t msg_len = j;
	  	  uint16_t random_ID = random_uin16_t();
	  	  last_transmission_ID = random_ID;

	  	  txbuf[0] = random_ID&0xff;
	  	  txbuf[1] = random_ID>>8;
	  	  txbuf[2] = msg_len>>8;
	  	  txbuf[3] = msg_len&0xff;
	  	  //calculate checksum
	  	  uint8_t sum = 0;
	  	  for(int i=0; i<msg_len-2; ++i){
	  		  sum = sum+txbuf[i];
	  	  }
	  	  txbuf[msg_len-2] = 256-sum;
	  	  last_transmission_Checksum = 256-sum;

	  	  //calculate bitwise parity
	  	  uint8_t parity =0;
	  	  for(int i=0; i<msg_len-1; ++i){
	  		parity = parity^txbuf[i];
	  	  }
	  	  txbuf[msg_len-1] = parity;
	  	  last_transmission_BitwiseParity=parity;
	  	  last_transmission_confirmed = 0;


	  	  //send until transmission confirmed, maximum 5 times
	  	  for(int transmissionNo = 0; transmissionNo<5; transmissionNo++){
	  		lora_send(txbuf, j, LORA_FHSS_ENABLED, LORA_IQ_INVERSION_ON);
	  		uint8_t status_val = read_reg(1);
	  		while((status_val &0xf) ==0x3){//while transmission in progess
	  		 	  status_val = read_reg(1);
	  		}
	  		//CLEAR TX COMPLETE INTERRUPT
	  		write_reg(REG_LR_IRQFLAGS, 0x8);
	  		lora_rx(1000, LORA_IQ_INVERSION_ON, 1, LORA_FHSS_ENABLED);
	  		HAL_Delay(200+100*transmissionNo);
	  		if(last_transmission_confirmed){
	  			break;
	  		}
	  	  }
	  	  HAL_Delay(10000);



		  SetOpMode(RF_OPMODE_STANDBY);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00506682;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 24000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 24-1;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 65535;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TEMPSIG_Pin|TEMPPWRT_Pin|TEMPMOISTSENON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEMPSIG_Pin */
  GPIO_InitStruct.Pin = TEMPSIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEMPSIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TEMPPWRT_Pin TEMPMOISTSENON_Pin LORA_CS_Pin */
  GPIO_InitStruct.Pin = TEMPPWRT_Pin|TEMPMOISTSENON_Pin|LORA_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
