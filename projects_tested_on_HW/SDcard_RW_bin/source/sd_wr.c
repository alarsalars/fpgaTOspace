
// refrence  Vipin Kizheppatt

#include "ff.h" 
#include "xstatus.h" 
#include <stdlib.h> 
#include "xil_printf.h" 
#include "xil_cache.h" 

FATFS fatfs; 

static int SD_Init(); 
static int SD_Eject(); 
static int ReadFile(char *FileName, u32 DestinationAddress);
static int WriteFile(char *FileName, u32 size, u32 SourceAddress); 

#define inputImageWidth 512 
#define inputImageHeight 512 
//  char imageBuffer[inputImageWidth * inputImageHeight * 3]; // Declare a character array to store image data
#define BUFFER_SIZE 3101640
char imageBuffer[BUFFER_SIZE]; 

int main(){
    int Status;

    // Initialize the SD card
    Status = SD_Init(&fatfs);
    if (Status != XST_SUCCESS) {
        print("file system init failed\n\r");
        return XST_FAILURE; 
    }
    xil_printf("done111...");
    // Read a file named "lenacolor.bin" from the SD card into the imageBuffer
    Status = ReadFile("boot.bin", (u32)imageBuffer);
    if (Status != XST_SUCCESS) {
        print("file read failed\n\r");
        return XST_FAILURE;
    }
    xil_printf("done222...");
    // Write data from imageBuffer to a file named "lend.bin" on the SD card
    //Status = WriteFile("lend.bin", (inputImageWidth * inputImageHeight * 3), (u32)imageBuffer);
    Status = WriteFile("bootcp2.bin", (3101640), (u32)imageBuffer);
    if (Status != XST_SUCCESS) {
        print("file write failed\n\r"); 
        return XST_FAILURE; 
    }
    xil_printf("done333...");
    // Eject/unmount the SD card, this might not work cuz the unmount is not possible with the board
    Status = SD_Eject(&fatfs);
    if (Status != XST_SUCCESS) {
        print("SD card unmount failed\n\r"); 
        return XST_FAILURE; 
    }

    xil_printf("done..."); 
    return 0;
}

// Function to initialize the SD card
static int SD_Init()
{
    FRESULT rc; // Declare FatFs result variable
    TCHAR *Path = "0:/"; // Declare path to the drive
    rc = f_mount(&fatfs, Path, 0); // Mount the logical drive
    if (rc) {
        xil_printf(" ERROR : f_mount returned %d\r\n", rc); 
        return XST_FAILURE;
    }
    return XST_SUCCESS; 
}

// Function to eject/unmount the SD card
static int SD_Eject()
{
    FRESULT rc; 
    TCHAR *Path = "0:/";
    rc = f_mount(0, Path, 1); // Unmount the logical drive
    if (rc) {
        xil_printf(" ERROR : f_mount returned %d\r\n", rc); 
        return XST_FAILURE; 
    }
    return XST_SUCCESS; 
}

// Function to read a file from the SD card
static int ReadFile(char *FileName, u32 DestinationAddress)
{
    FIL fil; 
    FRESULT rc; 
    UINT br; 
    u32 file_size; 
    rc = f_open(&fil, FileName, FA_READ); // Open the file for reading
    if (rc) {
        xil_printf(" ERROR : f_open returned %d\r\n", rc); 
        return XST_FAILURE; 
    }
    //file_size = fil.fsize; // Get the file size
    file_size = f_size(&fil); 
    rc = f_lseek(&fil, 0); 
    if (rc) {
        xil_printf(" ERROR : f_lseek returned %d\r\n", rc);
        return XST_FAILURE; 
    }
    rc = f_read(&fil, (void*) DestinationAddress, file_size, &br);
    if (rc) {
        xil_printf(" ERROR : f_read returned %d\r\n", rc);
        return XST_FAILURE; 
    }
    rc = f_close(&fil); 
    if (rc) {
        xil_printf(" ERROR : f_close returned %d\r\n", rc);
        return XST_FAILURE; 
    }
    Xil_DCacheFlush(); // Flush the data cache
    return XST_SUCCESS; 
}

// Function to write data to a file on the SD card
static int WriteFile(char *FileName, u32 size, u32 SourceAddress)
{
    UINT btw; 
    static FIL fil; 
    FRESULT rc; 
    rc = f_open(&fil, (char *)FileName, FA_OPEN_ALWAYS | FA_WRITE); // Open the file for writing (create if not exists)
    if (rc) {
        xil_printf(" ERROR : f_open returned %d\r\n", rc); 
        return XST_FAILURE; 
    }
    rc = f_write(&fil, (const void*)SourceAddress, size, &btw); // Write data to the file
    if (rc) {
        xil_printf(" ERROR : f_write returned %d\r\n", rc); 
        return XST_FAILURE;
    }
    rc = f_close(&fil); 
    if (rc) {
        xil_printf(" ERROR : f_write returned %d\r\n", rc); 
        return XST_FAILURE; 
    }
    return XST_SUCCESS; 
}
