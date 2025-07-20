#include "systick.h"


#define SYSTICK_LOAD_VAL		 16000
#define ENABLE_SYS				(1u<<0)
#define CLKSRC_SYS				(1u<<2)
#define FLAG_SYS				(1u<<16)


void systick_delay(int delay){
	SysTick->LOAD = SYSTICK_LOAD_VAL; // load the NR. clocks value
	SysTick-> VAL = 0; // clear the current value
	SysTick->CTRL = ENABLE_SYS | CLKSRC_SYS;

	for (int i=0; i <delay; i++){
		while((SysTick->CTRL & FLAG_SYS)== 0){} // wait the count flag

	}
	SysTick->CTRL = 0;
}
