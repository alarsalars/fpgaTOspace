#include <stdio.h>
#include "uart2.h"
#include "adc.h"
#include "timer.h"

#define LED_PIN (1u<<5)
char rec_data;

uint32_t sensor_value;
int timestamp = 0;

int main (void)
{
	// enable a led to confirm the received data

	uart_tx_rx_init();
	pa1_adc_init();
	start_adc();
	timer2_1_sec_init();
	timer2_compare(); // toggle the led with direct timer

	timer3_capture(); // you need a wire to connect the PA5 to PA6

	while(1){
		while(!(TIM3->SR & SR_CC1IF)){}

			timestamp = TIM3->CCR1;

}

}



