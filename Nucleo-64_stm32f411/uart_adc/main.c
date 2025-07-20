#include <stdio.h>
#include "uart2.h"
#include "adc.h"

#define LED_PIN (1u<<5)
char rec_data;

uint32_t sensor_value;
int main (void)
{
	// enable a led to confirm the received data
	GPIOA->MODER |=(1u<<10);
	GPIOA->MODER &=~(1u<<11);
	uart_tx_rx_init();
	pa1_adc_init();
	start_adc();

	while(1){

		sensor_value = adc_read();
		printf("Sensor value :%d \n\r", (int)sensor_value);
	}
}



