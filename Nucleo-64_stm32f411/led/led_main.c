// Start with Led toggle


#define PERIPH_BASE 		0x40000000
#define AHB1_OFFSET 		0x00020000
#define AHB1PERIPH_BASE 	(PERIPH_BASE + AHB1_OFFSET)
#define GPIOA_OFFSET 		0x00000000
#define GPIOA_BASE 			(AHB1PERIPH_BASE + GPIOA_OFFSET)
#define RCC_OFFSET 			0x3800 // reset clock control
#define RCC_BASE 			(AHB1PERIPH_BASE + RCC_OFFSET)
#define AHB1EN_RCC_offset 	0x30
#define RCC_AHB1EN_BASE		(*(volatile unsigned int*)(RCC_BASE + AHB1EN_RCC_offset))
#define GPIO_MODE_OFFSET	0x00
#define GPIOA_MODE_BASE 	(*(volatile unsigned int*)(GPIOA_BASE + GPIO_MODE_OFFSET))
#define ODR_OFFSET 			0x14		// output data register
#define GPIOA_ODR_BASE		(*(volatile unsigned int*)(GPIOA_BASE + ODR_OFFSET))


#define GPIOAEN_CLK			1<<0 // 0x00000001
#define LED_PIN5			1<<5



int main (void)
{
	RCC_AHB1EN_BASE |= GPIOAEN_CLK;
	GPIOA_MODE_BASE |= (1<<10); // bit 10 to '1'
	GPIOA_MODE_BASE &= ~(1<<11); // bit 11 to '0'
	while(1){
		GPIOA_ODR_BASE ^= LED_PIN5;
		for (int i=0; i<1000000;i++){
	};
}


}
