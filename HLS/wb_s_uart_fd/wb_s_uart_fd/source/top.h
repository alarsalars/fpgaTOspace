#pragma once

#include <ap_int.h>

typedef enum {idle,write,read} wb_fsm;


#define CLOCK_FREQUENCY 100000000 // 100 MHz
#define BAUD_RATE 112500          // 112,500 bps
#define DIVISOR (CLOCK_FREQUENCY / (16 * BAUD_RATE)) // Baud rate divisor

