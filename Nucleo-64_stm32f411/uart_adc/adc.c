#include <stdint.h>
#include "stm32f4xx.h"
#include "adc.h"



#define ADC1EN (1u<<8)
#define GPIOAEN (1u<<0)
#define ADC_CH1 (1u<<0)
#define ADC_SEQ_LEN 0x00
#define ADC_ENABLE (1u<<0)
#define ADC_START (1u<<30)
#define ADC_CON (1u<<1)




void pa1_adc_init(void){
	RCC->AHB1ENR |= GPIOAEN; // enable clk access to GPIOa
	GPIOA->MODER |= (1u<<2) | (1u<<3); // change the pin to analog
	RCC->APB1ENR |= ADC1EN; // Enable the adc on apb1
	ADC1->SQR3 = ADC_CH1;// start sequance channel one
	ADC1->SQR1 = ADC_SEQ_LEN; // define the sequance length
	ADC1->CR2 |= ADC_ENABLE; // ADC enable bit

}

void start_adc(void){
	ADC1->CR2 |= ADC_CON;
	ADC1->CR2 |= ADC_START;

}


uint32_t adc_read(void){

	while(!(ADC1->SR & (1u<<1))){}
	return ADC1->DR;


}

