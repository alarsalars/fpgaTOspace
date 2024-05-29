#include <stdio.h>
//#include "platform.h"
#include "xil_printf.h"
#include "xgpio.h"
#include "xmbox.h"
#include "sleep.h"

#define GPIO_DEVICE_ID XPAR_GPIO_0_DEVICE_ID
#define MBOX_DEVICE_ID XPAR_MBOX_0_DEVICE_ID
#define LED_DELAY 1000000 // 1 second delay

XGpio Gpio; // GPIO instance
XMbox Mbox; // Mailbox instance

const char *hello = "Hello from CPU0";
const char *ledon = "LED ON";
const char *ledoff = "LED OFF";

int main() {
    int Status;
    XGpio_Config *GPIOConfigPtr;
    XMbox_Config *ConfigPtr;

    // Initialize platform
    //init_platform();

    // Initialize GPIO
    GPIOConfigPtr = XGpio_LookupConfig(GPIO_DEVICE_ID);
    if (GPIOConfigPtr == NULL) {
        xil_printf("GPIO LookupConfig failed.\n");
        return XST_FAILURE;
    }

    Status = XGpio_CfgInitialize(&Gpio, GPIOConfigPtr, GPIOConfigPtr->BaseAddress);
    if (Status != XST_SUCCESS) {
        xil_printf("GPIO CfgInitialize failed.\n");
        return XST_FAILURE;
    }

    XGpio_SetDataDirection(&Gpio, 1, 0x0); // Set GPIO direction to output

    // Initialize Mailbox
    ConfigPtr = XMbox_LookupConfig(MBOX_DEVICE_ID);
    if (ConfigPtr == NULL) {
        xil_printf("Mbox LookupConfig failed.\n");
        return XST_FAILURE;
    }

    Status = XMbox_CfgInitialize(&Mbox, ConfigPtr, ConfigPtr->BaseAddress);
    if (Status != XST_SUCCESS) {
        xil_printf("Mbox CfgInitialize failed.\n");
        return XST_FAILURE;
    }

    // Write initial message to mailbox
    XMbox_WriteBlocking(&Mbox, (u32*)((u8*)hello), strlen(hello));

    // Main loop
    while (1) {
        XGpio_DiscreteWrite(&Gpio, 1, 0x01); // Turn on LED
        XMbox_WriteBlocking(&Mbox, (u32*)((u8*)ledon), strlen(ledon)); // Send LED ON message
        usleep(LED_DELAY); // Delay

        XGpio_DiscreteWrite(&Gpio, 1, 0x00); // Turn off LED
        XMbox_WriteBlocking(&Mbox, (u32*)((u8*)ledoff), strlen(ledoff)); // Send LED OFF message
        usleep(LED_DELAY); // Delay
    }

    // Clean up platform (usually not reached due to infinite loop)
    //cleanup_platform();

    return 0;
}
