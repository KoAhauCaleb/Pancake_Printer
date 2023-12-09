/*
 * interpreter.c
 *
 *  Created on: Nov 18, 2023
 *      Author: Ryan
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "main.h"
#include "interpreter.h"
#include <stdarg.h>

extern osMessageQueueId_t commandQueueHandle;
extern osMessageQueueId_t coordQueueHandle;
extern UART_HandleTypeDef huart2;

void myprintc(const char *fmt, ...)
{
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);
}

void InterpretLine(const char* line) {

    // Copy the line to a mutable buffer for modification
    char buffer[100];
    strncpy(buffer, line, sizeof(buffer));

    // Tokenize the line using space as a delimiter
    char* token = strtok(buffer, " ");

    int gCode = -1;
    int mCode = -1;

    float xCoord = 0.0;
    float yCoord = 0.0;

    int speedSet = 0;
    int pauseDur = 0;

    // Iterate through tokens
    while (token != NULL) {
        // Check for G-code command
        if (strncmp(token, "G", 1) == 0) {
            // Extract the G-code command
            gCode = atoi(token + 1); // Skip the 'G' character
        }
        // Check for X-coordinate
        else if (strncmp(token, "X", 1) == 0) {
            // Extract the X-coordinate
            xCoord = atof(token + 1); // Skip the 'X' character
        }
        // Check for Y-coordinate
        else if (strncmp(token, "Y", 1) == 0) {
            // Extract the Y-coordinate
            yCoord = atof(token + 1); // Skip the 'Y' character
        }
        else if (strncmp(token, "F", 1) == 0) {
			// Extract the speed
			speedSet = atof(token + 1); // Skip the 'F' character
		}
        else if (strncmp(token, "P", 1) == 0) {
			// Extract the pause duration
			pauseDur = atof(token + 1); // Skip the 'P' character
		}
        else if (strncmp(token, "M", 1) == 0) {
			// Extract the pause duration
			mCode = atof(token + 1); // Skip the 'M' character
		}
        else if (strncmp(token, ";", 1) == 0) {
			// if we hit a ';', end parsing
			token = NULL;
		}

        // Get the next token
        token = strtok(NULL, " ");
    }



    //TODO: will this wait like it is expected to when the queue is full
    switch (gCode) {
      case 1:

        // Add code to handle G1 Set Speed
        osMessageQueuePut(commandQueueHandle, &gCode, 100, 0U);
        osMessageQueuePut(commandQueueHandle, &speedSet, 100, 0U);
        break;
      case 4:

        // Add code to handle G4 Pause
        osMessageQueuePut(commandQueueHandle, &gCode, 100, 0U);
        osMessageQueuePut(commandQueueHandle, &pauseDur, 100, 0U);
        break;
      case 0:
        // Add command to queue or call move(xCoord, yCoord)
        // osMessageQueuePut(commandQueueHandle, &msg, 0U, 0U);
        osMessageQueuePut(commandQueueHandle, &gCode, 100, 0U);
        osMessageQueuePut(coordQueueHandle, &xCoord, 100, 0U);
        osMessageQueuePut(coordQueueHandle, &yCoord, 100, 0U);
        break;
      case 28:

        // Add code to handle G28 Home
        osMessageQueuePut(commandQueueHandle, &gCode, 100, 0U);
        break;

      default:
        printf("Unknown G-code command: %d\n", gCode);
        break;
    }

    switch (mCode) {
		case 106:

			// Add code to handle M106 Start Extrusion
			// Add command to queue or start_extrusion()
		  osMessageQueuePut(commandQueueHandle, &mCode, 100, 0U);
			break;
		case 107:

			// Add code to handle M107 Stop Extrusion
			// Add command to queue or call stop_extrusion()
		  osMessageQueuePut(commandQueueHandle, &mCode, 100, 0U);
			break;
		default:
			printf("Unknown M command: %d\n", mCode);
			break;
    }
}


