
// Vipin

#include "ff.h" // Include FatFs library header file
#include "xstatus.h" // Include Xilinx status header file
#include <stdlib.h> // Include standard library header file
#include "xil_printf.h" // Include Xilinx printf header file
#include "xil_cache.h" // Include Xilinx cache control header file

FATFS fatfs; // Declare a global variable of type FATFS for file system management

static int SD_Init(); // Function prototype for SD card initialization
static int SD_Eject(); // Function prototype for SD card ejection
static int ReadFile(char *FileName, u32 DestinationAddress); // Function prototype for reading a file from the SD card
static int WriteFile(char *FileName, u32 size, u32 SourceAddress); // Function prototype for writing data to a file on the SD card

#define inputImageWidth 512 // Define constant for input image width
#define inputImageHeight 512 // Define constant for input image height
//  char imageBuffer[inputImageWidth * inputImageHeight * 3]; // Declare a character array to store image data
#define BUFFER_SIZE 3101640
char imageBuffer[BUFFER_SIZE]; // Declare a character array to store image data

int main(){
    int Status; // Declare variable to hold function return statuses

    // Initialize the SD card
    Status = SD_Init(&fatfs);
    if (Status != XST_SUCCESS) {
        print("file system init failed\n\r"); // Print error message if initialization fails
        return XST_FAILURE; // Return failure status
    }
    xil_printf("done111...");
    // Read a file named "lenacolor.bin" from the SD card into the imageBuffer
    Status = ReadFile("boot.bin", (u32)imageBuffer);
    if (Status != XST_SUCCESS) {
        print("file read failed\n\r"); // Print error message if reading file fails
        return XST_FAILURE; // Return failure status
    }
    xil_printf("done222...");
    // Write data from imageBuffer to a file named "lend.bin" on the SD card
    //Status = WriteFile("lend.bin", (inputImageWidth * inputImageHeight * 3), (u32)imageBuffer);
    Status = WriteFile("bootcp2.bin", (3101640), (u32)imageBuffer);
    if (Status != XST_SUCCESS) {
        print("file write failed\n\r"); // Print error message if writing file fails
        return XST_FAILURE; // Return failure status
    }
    xil_printf("done333...");
    // Eject/unmount the SD card, this might not work cuz the unmount is not possible with the board
    Status = SD_Eject(&fatfs);
    if (Status != XST_SUCCESS) {
        print("SD card unmount failed\n\r"); // Print error message if ejection fails
        return XST_FAILURE; // Return failure status
    }

    xil_printf("done..."); // Print "done..." message indicating successful completion
    return 0; // Return success status
}

// Function to initialize the SD card
static int SD_Init()
{
    FRESULT rc; // Declare FatFs result variable
    TCHAR *Path = "0:/"; // Declare path to the drive
    rc = f_mount(&fatfs, Path, 0); // Mount the logical drive
    if (rc) {
        xil_printf(" ERROR : f_mount returned %d\r\n", rc); // Print error message if mounting fails
        return XST_FAILURE; // Return failure status
    }
    return XST_SUCCESS; // Return success status
}

// Function to eject/unmount the SD card
static int SD_Eject()
{
    FRESULT rc; // Declare FatFs result variable
    TCHAR *Path = "0:/"; // Declare path to the drive
    rc = f_mount(0, Path, 1); // Unmount the logical drive
    if (rc) {
        xil_printf(" ERROR : f_mount returned %d\r\n", rc); // Print error message if unmounting fails
        return XST_FAILURE; // Return failure status
    }
    return XST_SUCCESS; // Return success status
}

// Function to read a file from the SD card
static int ReadFile(char *FileName, u32 DestinationAddress)
{
    FIL fil; // Declare a FatFs file object
    FRESULT rc; // Declare FatFs result variable
    UINT br; // Declare variable to store number of bytes read
    u32 file_size; // Declare variable to store file size
    rc = f_open(&fil, FileName, FA_READ); // Open the file for reading
    if (rc) {
        xil_printf(" ERROR : f_open returned %d\r\n", rc); // Print error message if opening fails
        return XST_FAILURE; // Return failure status
    }
    //file_size = fil.fsize; // Get the file size
    file_size = f_size(&fil); // Get the file size
    rc = f_lseek(&fil, 0); // Move the file pointer to the beginning of the file
    if (rc) {
        xil_printf(" ERROR : f_lseek returned %d\r\n", rc); // Print error message if seeking fails
        return XST_FAILURE; // Return failure status
    }
    rc = f_read(&fil, (void*) DestinationAddress, file_size, &br); // Read data from the file
    if (rc) {
        xil_printf(" ERROR : f_read returned %d\r\n", rc); // Print error message if reading fails
        return XST_FAILURE; // Return failure status
    }
    rc = f_close(&fil); // Close the file
    if (rc) {
        xil_printf(" ERROR : f_close returned %d\r\n", rc); // Print error message if closing fails
        return XST_FAILURE; // Return failure status
    }
    Xil_DCacheFlush(); // Flush the data cache
    return XST_SUCCESS; // Return success status
}

// Function to write data to a file on the SD card
static int WriteFile(char *FileName, u32 size, u32 SourceAddress)
{
    UINT btw; // Declare variable to store number of bytes written
    static FIL fil; // Declare a static FatFs file object
    FRESULT rc; // Declare FatFs result variable
    rc = f_open(&fil, (char *)FileName, FA_OPEN_ALWAYS | FA_WRITE); // Open the file for writing (create if not exists)
    if (rc) {
        xil_printf(" ERROR : f_open returned %d\r\n", rc); // Print error message if opening fails
        return XST_FAILURE; // Return failure status
    }
    rc = f_write(&fil, (const void*)SourceAddress, size, &btw); // Write data to the file
    if (rc) {
        xil_printf(" ERROR : f_write returned %d\r\n", rc); // Print error message if writing fails
        return XST_FAILURE; // Return failure status
    }
    rc = f_close(&fil); // Close the file
    if (rc) {
        xil_printf(" ERROR : f_write returned %d\r\n", rc); // Print error message if closing fails
        return XST_FAILURE; // Return failure status
    }
    return XST_SUCCESS; // Return success status
}
