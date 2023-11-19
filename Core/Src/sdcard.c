/*
 * sdcard.c
 *
 *  Created on: Nov 15, 2023
 *      Author: esteb
 */


/*
 * sdcard.c
 *
 *  Created on: Nov 14, 2023
 *      Author: esteb
 */

#include <stdio.h>
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>

#define QUEUE_SIZE 500

FATFS FatFs;
FIL fil;
FRESULT fres;

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;


volatile uint8_t queue[QUEUE_SIZE][60];  // Queue to store lines
volatile uint8_t queueFront = 0;          // Front of the queue
volatile uint8_t queueRear = 0;           // Rear of the queue
volatile uint8_t lineNumber = 1;
volatile uint8_t dequeueNumber = 1;
volatile uint8_t queueFlag = 1;
volatile uint8_t isPrinting = 1;

void myprintf(const char *fmt, ...)
{
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);
}
void OpenFilesystem(void)
  {
	HAL_Delay(1000);
  	fres = f_mount(&FatFs, "", 1); //1=mount now
  	  if (fres != FR_OK)
  	  {
  		osDelay(50);
  		while(1);
  	  }

  	  //Let's get some statistics from the SD card
  	  DWORD free_clusters, free_sectors, total_sectors;

  	  FATFS* getFreeFs;

  	  fres = f_getfree("", &free_clusters, &getFreeFs);
  	  if (fres != FR_OK) {
  		myprintf("f_getfree error (%i)\r\n", fres);
  		while(1);
  	  }

  	  //Formula comes from ChaN's documentation
  	  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  	  free_sectors = free_clusters * getFreeFs->csize;

  	  myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

  }
void RandomPrinting(void *argument)//only used for debugging purposes.
{
	while(true)
		{
        isPrinting = !isPrinting;  // Toggle the value
        osDelay(500);
		}

}
void Enqueue(const char* line)// add gcode lines from the test.txt files to the queue.
{
    // Add the line to the queue
    strncpy((char*)queue[queueRear], line, sizeof(queue[queueRear]));
    queueRear = (queueRear + 1) % QUEUE_SIZE;
    queueFlag = 1;  // Set the flag to indicate there's something in the queue
}
void Dequeue(void) {//if there is something in the queue, it will dequeue, if there is nothing on the queue it will show printing is finished in serial monitor.

    if (queueFlag) {
        // Dequeue the first item
        myprintf("Dequeued G Code Line %lu: %s\r", lineNumber, queue[queueFront]);
        queueFront = (queueFront + 1) % QUEUE_SIZE;
        lineNumber++;
        // Reset the flag if the queue is empty
        if (queueFront == queueRear) {
            queueFlag = 0;
            myprintf("Finished Printing.");

        }
    }
}
void GCodeEnqueueFromCard(void)// will add all the lines from test.txt file to the queueue, so they can m=be read and dequeued once they are printed.
  {
	myprintf("Starting Queues.\r\n");
	HAL_Delay(2000);
  	fres = f_open(&fil, "test.txt", FA_READ);
  	  if (fres != FR_OK) {
  	      myprintf("f_open error (%i)\r\n", fres);
  	      while (1);
  	  }
  	  myprintf("Reading from 'test.txt'\r\n");

  	  // Read lines from "test.txt" on the SD card
  	  BYTE readBuf[50];
  	  TCHAR* rres;

  	  while (1) {


  	      rres = f_gets((TCHAR*)readBuf, sizeof(readBuf), &fil);

  	      // Check if a line was successfully read
  	      if (rres != 0) {

  	          // Add the line to the queue
  	          Enqueue((const char*)readBuf);
  	          myprintf("Enqueued Gcode Line %lu: %s\r", lineNumber, readBuf);

  	          // Optionally, introduce a delay before reading the next line
  	          lineNumber++;
  	      } else {
  	          // If we reach the end of the file, break out of the loop
  	          if (f_eof(&fil)) {
  	        	  lineNumber=1;
  	        	  myprintf("Finished Enqueing");
  	              break;
  	          } else {
  	              myprintf("f_gets error (%i)\r\n", fres);
  	              break;  // Exit the loop on error
  	          }
  	      }
  	  }

  	  //Be a tidy kiwi - don't forget to close your file!
  	  f_close(&fil);


  }
