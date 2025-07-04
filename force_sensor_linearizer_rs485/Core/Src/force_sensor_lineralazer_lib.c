#include "force_sensor_lineralazer_lib.h"
#include "globals.h"
#include "stdio.h"
#include "string.h"
#include <math.h>
#include "gpio.h"
#include "crc.h"
#include "adc.h"

void transmitData(USART_TypeDef *USARTx, uint8_t *data, int length)
{
	LL_GPIO_SetOutputPin(RE_DE_GPIO_Port, RE_DE_Pin); // Set MAX485 to Tx mode
	for (int i = 0; i < length; i++)
	{
		while (!LL_USART_IsActiveFlag_TXE(USART1));
		LL_USART_TransmitData8(USARTx, data[i]);
	}
	while (!LL_USART_IsActiveFlag_TC(USARTx))
		;
	LL_USART_ClearFlag_TC(USART1);
	LL_GPIO_ResetOutputPin(RE_DE_GPIO_Port, RE_DE_Pin); // Set MAX485 to Rx mode
}

void clearBufferAndCounter(void)
{
	memset(rxBuffer, 0, sizeof(rxBuffer));
	rxBufferCounter = 0;
	memset(txBuffer, 0, sizeof(txBuffer));
	memset(adcForTransmitt, 0, sizeof(adcForTransmitt));
	memset(adcBuffer, 0, sizeof(adcBuffer));
	recieved_CRC_32 = 0;
}

void readADCchannels(void)
{
	memset(adcBuffer, 0, sizeof(adcBuffer));

    for (uint8_t i = 0; i < 4; i++)
		{
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			adcBuffer[i] = HAL_ADC_GetValue(&hadc1);
		}

    HAL_ADC_Stop(&hadc1);
}

