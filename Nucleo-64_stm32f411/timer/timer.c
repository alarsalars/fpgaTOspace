
#include "stm32f4xx.h"

#define TIMER2EN	(1u<<0)
#define TIMER3EN	(1u<<1)
#define CR1_CEN  	(1u<<0)
#define CCER_CC1S    (1u<<0)


void timer2_1_sec_init(void){
	RCC->APB1ENR |= TIMER2EN;
	TIM2->PSC = 1600-1;
	TIM2->ARR = 10000-1;
	TIM2->CNT=0;
	TIM2->CR1= CR1_CEN;
}

void timer2_compare(void){
	RCC->AHB1ENR |= (1u<<0);
	GPIOA->MODER |=(1u<<11);
	GPIOA->MODER &=~(1u<<10);
	GPIOA->AFR[0] |= (1u<<20); //AFR5 timer
	RCC->APB1ENR |= TIMER2EN;
	TIM2->PSC = 1600-1;
	TIM2->ARR = 10000-1;
	TIM2->CCMR1 = (1u<<4) | (1u<<5);
	TIM2->CCER = (1u<<0);
	TIM2->CNT=0;
	TIM2->CR1= CR1_CEN;
}

void timer3_capture(void){
	RCC->AHB1ENR |= (1u<<0);
	GPIOA->MODER |=(1u<<12);
	GPIOA->MODER &=~(1u<<13);
	GPIOA->AFR[0] |=(1u<<25);
	RCC->APB1ENR |= TIMER3EN;
	TIM3->PSC = 16000-1;
	TIM3->CCMR1 = CCER_CC1S; // input capture
	TIM3->CCER  = (1u<<0);
	TIM3->CR1= CR1_CEN;




}
