#include <stdio.h>
#include <string.h> // Include for strlen and memset
#include "platform.h"
#include "xil_printf.h"
#include "xmbox.h" // Include for mailbox functions

#define MSGSIZ 32 // Define message size (adjust as needed)

XMbox Mbox; // Mailbox instance
char RecvMsg[MSGSIZ]; // Buffer for received messages
const char *hello = "Hello from ARM"; // Example message to send
const char *ledon = "LED ON"; // Example message to receive

int main() {
    int Status;
    XMbox_Config *ConfigPtr;

    // Initialize platform
    init_platform();

    // Initialize Mailbox
    ConfigPtr = XMbox_LookupConfig(XPAR_MBOX_0_DEVICE_ID);
    if (ConfigPtr == NULL) {
        xil_printf("Mbox LookupConfig failed.\n");
        return XST_FAILURE;
    }

    Status = XMbox_CfgInitialize(&Mbox, ConfigPtr, ConfigPtr->BaseAddress);
    if (Status != XST_SUCCESS) {
        xil_printf("Mbox CfgInitialize failed.\n");
        return XST_FAILURE;
    }

    printf("hello from ARM, %d\n\r", strlen(hello));

    // Read initial message from the mailbox
    XMbox_ReadBlocking(&Mbox, (u32*)RecvMsg, strlen(hello));
    printf("Rcvd the message --> \r\n\r\n\t--[%s]--\r\n\r\n", RecvMsg);

    // Clear the buffer
    memset(RecvMsg, 0, MSGSIZ);

    while (1) {
        XMbox_ReadBlocking(&Mbox, (u32*)RecvMsg, strlen(ledon));
        printf("Recv the message --> \r\n\r\n\t--[%s]--\r\n\r\n", RecvMsg);

        // Clear the buffer
        memset(RecvMsg, 0, MSGSIZ);
    }

    // Clean up platform (usually not reached due to infinite loop)
    cleanup_platform();

    return 0;
}
