/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "string.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
typedef enum {
	NRF24L01_Transmit_Status_Lost = 0x00, /*!< Message is lost, reached maximum number of retransmissions */
	NRF24L01_Transmit_Status_Ok = 0x01, /*!< Message sent successfully */
	NRF24L01_Transmit_Status_Sending = 0xFF /*!< Message is still sending */
} NRF24L01_Transmit_Status_t;
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BLINK_LED(n) for(uint8_t i=0;i<n;i++){\
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);\
	HAL_Delay(10); HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);\
	HAL_Delay(10);}

#define Check_bit(reg, bit)       (reg & (1 << bit))

#define CE_LOW				HAL_GPIO_WritePin(GPIOB, nrf24_CE_Pin, GPIO_PIN_RESET)
#define CE_HIGH				HAL_GPIO_WritePin(GPIOB, nrf24_CE_Pin, GPIO_PIN_SET)
#define CSN_LOW				HAL_GPIO_WritePin(GPIOB, nrf24_CS_Pin, GPIO_PIN_RESET)
#define CSN_HIGH			HAL_GPIO_WritePin(GPIOB, nrf24_CS_Pin, GPIO_PIN_SET)

#define Clear_IRQ   do { W_Register(0x07, 0x70); } while (0)

#define nrf24l01_CONFIG			((1 << EN_CRC) | (0 << CRCO))

/* Configuration register*/
#define RX_DR		6
#define TX_DS		5
#define MAX_RT		4
#define EN_CRC		3
#define CRCO		2
#define PWR_UP		1
#define PRIM_RX		0

/* FIFO status*/
#define RX_EMPTY	0

/*Dynamic length*/
#define NRF24L01_DPL_P0			0
#define NRF24L01_DPL_P1			1
#define NRF24L01_DPL_P2			2
#define NRF24L01_DPL_P3			3
#define NRF24L01_DPL_P4			4
#define NRF24L01_DPL_P5			5

/* NRF24L01+ registers*/
#define STATUS 			0x07
#define RF_CH 			0x05
#define RF_SETUP 		0x06
#define FLUSH_TX 		0xE1
#define FLUSH_RX 		0xE2
#define RX_ADDR_P0 		0x0A
#define RX_ADDR_P1 		0x0B
#define TX_ADDR 		0x10
#define CONFIG 			0x00
#define RX_PW_P0 		0x11
#define RX_PW_P1 		0x12
#define W_TX_PAYLOAD 	0xA0
#define R_RX_PAYLOAD 	0x61
#define NOP 			0xFF
#define FIFO_STATUS 	0x17
#define RXFIFO_EMPTY 	0x01
#define W_REGISTER 		0x20
#define EN_RXADDR 		0x02
#define EN_AA 			0x01
#define SETUP_RETR 		0x04
#define SETUP_AW 		0x03
#define DYNPD 			0x1C
#define FEATURE 		0x1D
#define R_RX_PL_WID 	0x60
/* USER CODE END PTD */

uint8_t nrf24_Init();
void NRF24L01_InitPins(void);
void WriteBit(uint8_t reg, uint8_t bit, uint8_t value);
uint8_t ReadBit(uint8_t reg, uint8_t bit);
void Flush(uint8_t data);
void CheckReg();
uint8_t DataReady();
uint8_t RxFifoEmpty();
uint8_t GetStatus();
uint8_t ReadReg(uint8_t reg, uint8_t param);
uint8_t ReadBit(uint8_t reg, uint8_t bit);
void Transmit(uint8_t *data);
void GetData(uint8_t* data) ;
void PowerUpRx();
void PowerUpTx();
NRF24L01_Transmit_Status_t GetTransmissionStatus(void) ;
void uart_disp(char *text);
void uart_int_disp(uint8_t value);
void SetAddress(uint8_t reg, uint8_t *addr, uint8_t len);
void uart_float_disp(float value);

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
volatile enum {
	CMD_READY, WAITING_FOR_CMD, SEND_DATA
} RF_state;

/* Transmission status enumeration*/


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == nrf24_IRQ_Pin)
		if (DataReady())
			RF_state = CMD_READY;

}

GPIO_InitTypeDef GPIO_InitStruct;

void GPIO_SwitchModeInput()
{
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);
}


void GPIO_SwitchModeOutput()
{
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);
}

void delayus(uint32_t us) {
 volatile uint32_t counter = 8*us;
 while(counter--);
}

HAL_StatusTypeDef DS18B20_ResetPulse()
{
	uint8_t PRESENCE = 0;
	GPIO_SwitchModeOutput();
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
	delayus(480);

	GPIO_SwitchModeInput();
	delayus(80);


	if (HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin)== GPIO_PIN_RESET)
		PRESENCE = 1;

	delayus(400);

	if ((HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin) == GPIO_PIN_SET) & (PRESENCE == 1))
		return HAL_OK;

	return HAL_ERROR;
}

void DS18B20_write(uint8_t data)
{
	for (int i=0; i<8; i++)
	{
		GPIO_SwitchModeOutput();
		if ((data & (1<<i))!=0)
		{
			HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
			delayus(1);
			GPIO_SwitchModeInput();
			delayus(60);
		}
		else
		{
			HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
			delayus(60);
			GPIO_SwitchModeInput();
		}
	}
}


uint8_t DS18B20_read()
{
	uint8_t value=0;
	GPIO_SwitchModeInput();

	for (int i=0;i<8;i++)
	{
		GPIO_SwitchModeOutput();
		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
		delayus(2);
		GPIO_SwitchModeInput();
		if (HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin))
		{
			value |= 1<<i;
		}
		delayus(60);
	}
	return value;
}

float DS18B20_GetTemperature()
{
	uint8_t temp_l, temp_h;
	uint16_t temp;
	float temperature;

	DS18B20_ResetPulse();

	DS18B20_write(0xCC);  // Skip ROM
	DS18B20_write(0x44);  // Convert T

	DS18B20_ResetPulse();

	DS18B20_write(0xCC);  // Skip ROM
	DS18B20_write(0xBE);  // Read Scratchpad

	temp_l = DS18B20_read();
	temp_h = DS18B20_read();

	temp = (temp_h<<8)|temp_l;
	temperature = (float)temp/16;

	return temperature;
}





/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
	uint8_t TxAddress[] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
	uint8_t MyAddress[] = { 0x7E, 0x7E, 0x7E, 0x7E, 0x7E };
	uint8_t dataIn[32];

	NRF24L01_Transmit_Status_t transmissionStatus;

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  SPI_HandleTypeDef hspi1;

  UART_HandleTypeDef huart1;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  /* USER CODE END 2 */

  nrf24_Init();
  CheckReg();
  WriteBit(CONFIG, TX_DS, 1);
  WriteBit(CONFIG, MAX_RT, 1);
  SetAddress(RX_ADDR_P0, TxAddress, 5);
  SetAddress(TX_ADDR, TxAddress, 5);
  SetAddress(RX_ADDR_P1, MyAddress, 5);

  RF_state = WAITING_FOR_CMD;

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  BLINK_LED(1);

	  uint8_t dataOut[32] = {DS18B20_GetTemperature()};

    /* USER CODE END WHILE */
	  switch (RF_state) {
	  		case CMD_READY:
	  			do {
	  				GetData(dataIn);
	  				WriteBit(STATUS, RX_DR, 1);

	  			} while (!ReadBit(FIFO_STATUS,RX_EMPTY));

	  			if(dataIn[0] == 1) uart_disp("Slave sends temperature successfully\n\r");
	  			if(dataIn[0] == 0) uart_disp("Slave sends the temperature unsuccessfully\n\r");
	  			for( int i=0; i<32; i++)
	  				dataIn[i] = 0;


	  		case SEND_DATA:
	  			Transmit(dataOut);

	  			do {
	  				transmissionStatus = GetTransmissionStatus();
	  			} while (transmissionStatus == NRF24L01_Transmit_Status_Sending);
	  			PowerUpRx();
	  			if (transmissionStatus == NRF24L01_Transmit_Status_Ok) {
	  				uart_disp("ACK, Master received data\n\r");
	  			} else if (transmissionStatus == NRF24L01_Transmit_Status_Lost) {
	  				uart_disp("NOACK, Lost ACK from Master\n\r");
	  			} else {
	  				uart_disp("SENDING\n\r");
	  			}
	  			RF_state = WAITING_FOR_CMD;
	  			__HAL_TIM_SET_COUNTER(&htim2, 0);

	  		case WAITING_FOR_CMD:
	  			HAL_Delay(1);
	  			if ( __HAL_TIM_GET_COUNTER(&htim2) > 1000) {
	  				uart_disp("No command for more than 1s\n\r");
	  				__HAL_TIM_SET_COUNTER(&htim2, 0);
	  			}
	  			break;
	  		}
		}

    /* USER CODE BEGIN 3 */
 }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

  /* USER CODE END Error_Handler_Debug */
}

void W_Register(uint8_t reg, uint8_t value)
{
	 reg = reg | W_REGISTER;
	 CSN_LOW;
	 HAL_SPI_Transmit(&hspi1, &reg,1,1000);
	 HAL_SPI_Transmit(&hspi1, &value,1,1000);
	 CSN_HIGH;
}

void NRF24L01_InitPins()
{
	/* CSN high = disable SPI */
	CSN_HIGH;

	/* CE low = disable TX/RX */
	CE_LOW;
}

uint8_t nrf24_Init()
{
	NRF24L01_InitPins();
    W_Register(RF_CH,0x5A);
    W_Register(RF_SETUP, 0x06); // TX_PWR:0dBm, Datarate:2Mbps
    W_Register(CONFIG,nrf24l01_CONFIG);
    W_Register(EN_AA, 0x3F); //// Enable Auto.Ack:Pipe0
    W_Register(EN_RXADDR, 0x3F); //  Enable data pipe 0.
    /* Auto retransmit delay: 1000 (4x250) us and Up to 15 retransmit trials */
    W_Register(SETUP_RETR, 0x4F); //odpowiada za automatyczna retransmisje ???
    W_Register(RX_PW_P0, 32);
    W_Register(RX_PW_P1, 32);
    /* Dynamic length configurations: No dynamic length */
    W_Register(DYNPD, (0 << NRF24L01_DPL_P0) | (0 << NRF24L01_DPL_P1) | (0 << NRF24L01_DPL_P2) | (0 << NRF24L01_DPL_P3) | (0 << NRF24L01_DPL_P4) | (0 << NRF24L01_DPL_P5));

    Flush(FLUSH_RX);
    Flush(FLUSH_TX);

    Clear_IRQ;

   	PowerUpRx();

   	/* Return OK */
   	return 1;
}

void PowerUpRx()
{
	CE_LOW;

	Flush(FLUSH_RX);;

	Clear_IRQ;

	/* RX mode */
	WriteBit(CONFIG, PWR_UP, 1);
	WriteBit(CONFIG, PRIM_RX, 1);

	/* Start listening */
	CE_HIGH;
}

void PowerUpTx()
{
	Clear_IRQ;

	WriteBit(CONFIG, PWR_UP, 1);
	WriteBit(CONFIG, PRIM_RX, 0);
}


void WriteBit(uint8_t reg, uint8_t bit, uint8_t value)
{
	uint8_t tmp;

	tmp = ReadReg(reg,1);

	if (value) {
		tmp |= 1 << bit;
	} else {
		tmp &= ~(1 << bit);
	}

	W_Register(reg, tmp);
}

void Flush(uint8_t data)
{
	 CSN_LOW;
	 HAL_SPI_Transmit(&hspi1, &data, 1, 200);
	 CSN_HIGH;
}

uint8_t DataReady()
{
	uint8_t status = GetStatus();

	if (Check_bit(status, RX_DR)) {
		return 1;
	}
	return !RxFifoEmpty();
}

uint8_t RxFifoEmpty()
{
	uint8_t reg = ReadReg(FIFO_STATUS,1);
	return Check_bit(reg, RX_EMPTY);
}

uint8_t GetStatus()
{
	uint8_t status;
	uint8_t reg = NOP;
	CSN_LOW;

	/* First received byte is always status register */
	HAL_SPI_TransmitReceive(&hspi1, &reg, &status , 1, 100);

	CSN_HIGH;

	return status;
}

uint8_t ReadReg(uint8_t reg, uint8_t param)
{
	uint8_t data[16];

	CSN_LOW;

	HAL_SPI_Transmit(&hspi1, &reg, 1,100);
	HAL_SPI_Receive(&hspi1,data,param,100);

	CSN_HIGH;

	return data[param-1];
}

void Transmit(uint8_t *data)
{
	uint8_t count = 32;
	uint8_t cmd;

	CE_LOW;

	PowerUpTx();

	cmd = FLUSH_TX;
	CSN_LOW;
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	CSN_HIGH;

	CSN_LOW;

	cmd = W_TX_PAYLOAD;
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);

	HAL_SPI_Transmit(&hspi1, data, count, 100);

	CSN_HIGH;

	CE_HIGH;
}

void GetData(uint8_t* data)
{
	uint8_t cmd = R_RX_PAYLOAD;

	CSN_LOW;

	HAL_SPI_Transmit(&hspi1, &cmd , 1, 100);

	HAL_SPI_Receive(&hspi1, data, 32, 100);

	CSN_HIGH;

	W_Register(STATUS, (1 << RX_DR));
}

uint8_t ReadBit(uint8_t reg, uint8_t bit)
{
	uint8_t tmp;
	tmp = ReadReg(reg,1);
	if (!Check_bit(tmp, bit)) {
		return 0;
	}
	return 1;
}

void SetAddress(uint8_t reg, uint8_t *addr, uint8_t len)
{
	 reg=reg+W_REGISTER;
	 CSN_LOW;
	 HAL_SPI_Transmit(&hspi1, &reg,1,1000);

	 for(int i=0; i<len; i++)
	 {
		 HAL_SPI_Transmit(&hspi1,&addr[i],1,1000);
		 HAL_Delay(10);
	 }

	 CSN_HIGH;
 }


void CheckReg()
{
	/* Check  register value*/
	 if(ReadReg(RF_CH,1)==0x5A)
	 {
		 uart_disp("1.RF_CH poprawna wartosc");
	 }
	 if(ReadReg(RF_SETUP,1)==0x06)
	 {
		 uart_disp("2.RF_SETUP poprawna wartosc");
	 }
	 if(ReadReg(CONFIG,1)== 0x0B)
	 {
		 uart_disp("3.CONFIG poprawna wartosc");
	 }
	 if(ReadReg(EN_AA,1)==0x3F)
	 {
		 uart_disp("4.EN_AA poprawna wartosc");
	 }
	 if(ReadReg(SETUP_RETR,1)==0x4F)
	 {
		 uart_disp("5.SETUP_RETR poprawna wartosc");
	 }
	 if(ReadReg(RX_PW_P0,1)==32)
	 {
		 uart_disp("6.RX_PW_P0 poprawna wartosc");
	 }
	 if(ReadReg(TX_ADDR,1)==0xE7)
	 {
		 uart_disp("TX_ADDR[0] poprawna wartosc");
	 }
	 if(ReadReg(TX_ADDR,2)==0xE7)
	 {
		 uart_disp("TX_ADDR[1] poprawna wartosc");
	 }
	 if(ReadReg(TX_ADDR,3)==0xE7)
	 {
	 	 uart_disp("TX_ADDR[2] poprawna wartosc");
	 }
	 if(ReadReg(TX_ADDR,4)==0xE7)
	 {
	 	 uart_disp("TX_ADDR[3] poprawna wartosc");
	 }
	 if(ReadReg(TX_ADDR,5)==0xE7)
	 {
		 uart_disp("TX_ADDR[4] poprawna wartosc");
	 }
	 if(ReadReg(RX_ADDR_P1,1)==0x7E)
	 {
	 	 uart_disp("RX_ADDR_P1[0] poprawna wartosc");
	 }
	 if(ReadReg(RX_ADDR_P1,2)==0x7E)
	 {
		 uart_disp("RX_ADDR_P1[1] poprawna wartosc");
	 }
	 if(ReadReg(RX_ADDR_P1,3)==0x7E)
	 {
	  	 uart_disp("RX_ADDR_P1[2] poprawna wartosc");
	 }
	 if(ReadReg(RX_ADDR_P1,4)==0x7E)
	 {
	  	 uart_disp("RX_ADDR_P1[3] poprawna wartosc");
	 }
	 if(ReadReg(RX_ADDR_P1,5)==0x7E)
	 {
	  	 uart_disp("RX_ADDR_P1[4] poprawna wartosc");
 	 }
}

NRF24L01_Transmit_Status_t GetTransmissionStatus(void) {
	uint8_t status = GetStatus();
	if (Check_bit(status, TX_DS)) {
		/* Successfully sent */
		return NRF24L01_Transmit_Status_Ok;
	} else if (Check_bit(status, MAX_RT)) {
		/* Message lost */
		return NRF24L01_Transmit_Status_Lost;
	}
	/* Still sending */
	return NRF24L01_Transmit_Status_Sending;
}

void uart_disp(char *text)
{
	 HAL_UART_Transmit(&huart1, text, strlen(text), 1000);
	 HAL_UART_Transmit(&huart1, "\r\n", 2, 1000);
}

void uart_int_disp(uint8_t value)
{
	char buffer[16];
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "%d", value), 1000);
	HAL_UART_Transmit(&huart1, "\r\n", 2, 1000);
}

void uart_float_disp(float value)
{
	char buffer[16];
	//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "%f", value), 1000);
	HAL_UART_Transmit(&huart1, "\r\n", 2, 1000);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
