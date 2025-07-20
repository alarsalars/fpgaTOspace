#include "uart2.h"
#define GPIOAEN 	(1u<<0)
#define UART2EN		(1u<<17)

#define sys_clk			16000000
#define div_baudrate 115200


static void uart_set_baudrate(USART_TypeDef *USARTX,uint32_t clk,uint32_t baudrate );
static uint16_t uart_cal_BR(uint32_t clk,uint32_t baudrate);

void uart2_write(int ch);


int io_char(int ch){
	uart2_write(ch);
	return ch;
}


void uart_tx_rx_init(void){
	// enable GPIOA clk access
	RCC->AHB1ENR |= GPIOAEN;
	// change GPIOA mode pa2 and pa3 to alternate
	GPIOA->MODER &= ~(1u<<4);
	GPIOA->MODER |= (1u<<5);
	GPIOA->MODER &= ~(1u<<6);
	GPIOA->MODER |= (1u<<7);
	// configure alternate function of PA2 and PA3 to AF7 as in the alternate function map in the datasheet
	GPIOA->AFR[0] |= (1u<<8);
	GPIOA->AFR[0] |= (1u<<9);
	GPIOA->AFR[0] |= (1u<<10);
	GPIOA->AFR[0] &= ~(1u<<11);
	GPIOA->AFR[0] |= (1u<<12);
	GPIOA->AFR[0] |= (1u<<13);
	GPIOA->AFR[0] |= (1u<<14);
	GPIOA->AFR[0] &= ~(1u<<15);
	// enable UART2
	RCC->APB1ENR |= UART2EN;
	// configure baudrate
	uart_set_baudrate(USART2,sys_clk,div_baudrate );
	// configure uart CR, one bit start 8 bit data one bit stop
	USART2->CR1 |=(1u<<2); // RX
	USART2->CR1 |=(1u<<3); //TX
	USART2->CR1 |=(1u<<13); //uart enable

}

void uart2_write(int ch){
	while(!(USART2->SR & (1u<<7))){} // wait until is empty
	USART2->DR = (ch & 0xFF);
}

char uart2_read(){
	while(!(USART2->SR & (1u<<5))){} // wait until the receive register is not empty
	return USART2->DR ;
}



static void uart_set_baudrate(USART_TypeDef *USARTX,uint32_t clk,uint32_t baudrate ){
	USARTX->BRR = uart_cal_BR( clk, baudrate);
}

static uint16_t uart_cal_BR(uint32_t clk,uint32_t baudrate){
	return ((clk + (baudrate/2u))/baudrate);
}




