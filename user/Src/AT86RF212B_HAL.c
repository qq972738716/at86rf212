/*
 * AT86RF212B_HAL.c
 *
 *  Created on: Feb 15, 2018
 *      Author: owner
 */

#include <string.h>
#include <stdint.h>
#include <stdio.h>
//#include <unistd.h>
#include "../Inc/AT86RF212B.h"
#include "../Inc/AT86RF212B_HAL.h"

#include "../Inc/ErrorsAndLogging.h"
#include "../Inc/Terminal.h"
#include "../Inc/Buffer.h"
#include "../Settings/AT86RF212B_Settings.h"
#include "../Settings/HAL_Settings.h"


#include "stm32f1xx_hal.h"
#include "usart.h"
#include "main.h"
#include "spi.h"

#define SPI_NSS_PORT NSS_GPIO_Port
#define SPI_NSS_PIN NSS_Pin

#define CLKM_PORT CLK_GPIO_Port
#define CLKM_PIN CLK_Pin

#define IRQ_PORT IRQ_GPIO_Port
#define IRQ_PIN IRQ_Pin

#define SLP_TR_PORT SLP_TR_GPIO_Port
#define SLP_TR_PIN SLP_TR_Pin

#define RST_PORT RST_GPIO_Port
#define RST_PIN RST_Pin

#define DIG2_PORT DIG2_GPIO_Port
#define DIG2_PIN DIG2_Pin

uint32_t timeout = 1000;
extern SPI_HandleTypeDef hspi1;

SPI_HandleTypeDef hspi;
extern uint8_t aRxBuffer1;


//-----------------Implement changing of GPIO pin
void AT86RF212B_WritePinHAL(uint8_t pin, uint8_t state){
	uint16_t GPIO_PIN;


	GPIO_TypeDef * GPIO_PORT;
	switch(pin){
		case AT86RF212B_PIN_CLKM:
			GPIO_PORT = CLKM_PORT;
			GPIO_PIN = CLKM_PIN;
			break;
		case AT86RF212B_PIN_IRQ:
			GPIO_PORT = IRQ_PORT;
			GPIO_PIN = IRQ_PIN;
			break;
		case AT86RF212B_PIN_SLP_TR:
			GPIO_PORT = SLP_TR_PORT;
			GPIO_PIN = SLP_TR_PIN;
			break;
		case AT86RF212B_PIN_RST:
			GPIO_PORT = RST_PORT;
			GPIO_PIN = RST_PIN;
			break;
		case AT86RF212B_PIN_DIG2:
			GPIO_PORT = DIG2_PORT;
			GPIO_PIN = DIG2_PIN;
			break;
		default:
			ASSERT(0);
			LOG(LOG_LVL_ERROR, "Unknown Pin");
			return;
	}
	(state) ? HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN, GPIO_PIN_RESET);

	return;
}

//-----------------Implement reading GPIO pin
uint8_t AT86RF212B_ReadPinHAL(uint8_t pin){
	uint16_t GPIO_PIN;

	GPIO_TypeDef * GPIO_PORT;
	switch(pin){
		case AT86RF212B_PIN_CLKM:
			GPIO_PORT = CLKM_PORT;
			GPIO_PIN = CLKM_PIN;
			break;
		case AT86RF212B_PIN_IRQ:
			GPIO_PORT = IRQ_PORT;
			GPIO_PIN = IRQ_PIN;
			break;
		case AT86RF212B_PIN_SLP_TR:
			GPIO_PORT = SLP_TR_PORT;
			GPIO_PIN = SLP_TR_PIN;
			break;
		case AT86RF212B_PIN_RST:
			GPIO_PORT = RST_PORT;
			GPIO_PIN = RST_PIN;
			break;
		case AT86RF212B_PIN_DIG2:
			GPIO_PORT = DIG2_PORT;
			GPIO_PIN = DIG2_PIN;
			break;
		default:
			ASSERT(0);
			LOG(LOG_LVL_ERROR, (uint8_t*)"Unknown Pin");
			return 0;
	}
	return HAL_GPIO_ReadPin(GPIO_PORT, GPIO_PIN);
}

//-----------------Implement platform specific initialization
//TODO: Change the returns from void to an indicator, that means the functions need to validate a successful operation or not
void AT86RF212B_OpenHAL(uint32_t time_out)
	{

	hspi = hspi1;
	timeout = time_out;
	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SLP_TR_PORT, SLP_TR_PIN, GPIO_PIN_RESET);

  }


//-----------------Implement SPI read/write
void AT86RF212B_SPIreadAndWriteHAL(uint8_t * pTxData, uint8_t * pRxData, uint16_t size){

	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi , pTxData, pRxData, size, timeout);
	//TODO: This probably needs to be changed, could lock up here.
	while(hspi.State == HAL_SPI_STATE_BUSY);
	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_SET);

}

//-----------------Implement Interrupt Callback, need to call AT86RF212B_ISR_Callback() when an interrupt is detected

//This function overrides the default callback for the STM32 HAL and its name should not be changed
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == IRQ_PIN){
		AT86RF212B_ISR_Callback();
	}
}


//-----------------Implement mili sec count
uint32_t HALGetMs(void){

	return HAL_GetTick();

}

//-----------------Implement micro sec count
uint32_t HALGetUs(void){

	//Start the counter
	DWT->CTRL |= 1;
	return DWT->CYCCNT;
}

//-----------------Implement mili sec delay
void HALDelayMs(uint32_t timeMs){
	
	HAL_Delay(timeMs);
	
	return;
}

//-----------------Implement micro sec delay
void HALDelayUs(uint32_t timeUs){

	//Clear the counter
	DWT->CYCCNT = 0;
	uint32_t stopTime = timeUs*(HAL_RCC_GetHCLKFreq()/1000000);
	//Start the counter
	DWT->CTRL |= 1;
	while(DWT->CYCCNT < stopTime);

	return;
}

//-----------------Implement reading from hardware data input to the TX buffer
void ReadInputHAL(){
#if RASPBERRY_PI
	char inChar;
	while(read(0, &inChar, 1) > 0){
		PushToInputBuffer(inChar);
	}
#endif

#if STM32
	//CDC_Enable_USB_Packet();
#endif
}

//-----------------Implement writing RX buffer to the hardware output
void WriteToOutputHAL(uint8_t * pTxData, uint32_t length){
	
	HAL_UART_Transmit(&huart2,pTxData, length,0xFFFF) ;
		
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
  */
	if( &huart2 == huart)
	{
		PushToInputBuffer(aRxBuffer1);
		HAL_UART_Receive_IT(&huart2,&aRxBuffer1,1);      // 重新使能串口2接收中断
	}
}
