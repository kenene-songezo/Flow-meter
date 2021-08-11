
/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "em_common.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"

#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "sl_uartdrv_instances.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>


// GPIO pin defines
#define GPIO_LED0   3
#define GPIO_PWRKEY 14

// GPIO Ports
#define GPIOA gpioPortA
#define GPIOB gpioPortB
#define GPIOC gpioPortC
#define GPIOD gpioPortD
#define GPIOF gpioPortF


#define DELAY1       1000
#define DELAY2       1000

#define onesecond    DELAY1*11

int getProgress = 1;
char r_buffer[500];                   // read buffer
char sendQHTTPURL[];
char response_QIFGCNT[25];
char response_QICSGP[32];
char response_QIACT1[32];
char response_QIACT2[32];
char response_QIACT3[32];
char response_QHTTPURL[32];
char response_QHTTPGET[32];
char response_QHTTPREAD[200];
char response_URL[34];
char responses_QHTTPCFG[35];
char checkIfON[25];

int step = 0;
bool respone_correct = false;        // flag to check if response received matches the expected
int buffer_size;                     // size of read buffer
uint8_t* sendBuffer;
uint8_t* receiveBuffer;
uint8_t* message;
int counter = 0;
FILE *out;
char newBuffer[];
char *analysed;

//AT Commands
char at_commands[20][10]={"AT","ATV1","ATE1","AT+CMEE=2","AT+IPR?","ATI","AT+GSN","AT+CPIN?","AT+CIMI","AT+QCCID","AT+CSQ","AT+CREG?","AT+CGREG?"};


/**************************************************************************//**
 * All other functions to be defined here
 *****************************************************************************/

//callback functions
static void processTxDone()
{
}

static void processRxDone()
{
  respone_correct = true;

}

// Delay functions
void delay(void){
  for (int i = 0; i < DELAY1; i++) {
          for (int j = 0; j < DELAY2; ++j) {
              __NOP();
          }
      }
}

void uartdelay(void){
  for (int i = 0; i < 100; i++) {
          for (int j = 0; j < 1000; ++j) {
              __NOP();
          }
      }
}

void oneseconddelay(){
  for (int i = 0; i < onesecond; ++i) {
          for (int j = 0; j < DELAY2; ++j) {
              __NOP();
          }
      }
}

//Power up modem
void on_PWRKEY(){
  // check if the modem is on
  sendBuffer = (uint8_t*) "\r\nAT\r\n";
  UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 6,processTxDone);
  UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) checkIfON, 10, processRxDone);
  int    ch = '\r';
  if (strchr(checkIfON, ch)==NULL) {

      //If modem is off switch it on (see datasheet)
      GPIO_PinOutSet(GPIOD, GPIO_PWRKEY);
      delay();  //100ms delay
      GPIO_PinOutClear(GPIOD, GPIO_PWRKEY);
      oneseconddelay();
      GPIO_PinOutSet(GPIOD, GPIO_PWRKEY);
      //The process of switching on ends here (see datasheet)

      oneseconddelay();
      UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) checkIfON, 17, processRxDone);
  }

}

//Initialization
void init_GPIO(){
  /**************************************************************************//**
   * @brief GPIO initialization
   *****************************************************************************/

  // Enable GPIO clock
  CMU_ClockEnable (cmuClock_GPIO, true);

  // Configure GPIO as output
  GPIO_PinModeSet (GPIOF, GPIO_LED0, gpioModePushPull, 0);
  GPIO_PinModeSet (GPIOD, GPIO_PWRKEY, gpioModePushPull, 0);
}


//get request
void getRequest(uint8_t *url){
  int url_length = strlen((char*)url);
  sprintf(sendQHTTPURL,"\r\nAT+QHTTPURL=%d,30\r\n",url_length);
  //int sendQHTTPURL_length = strlen(sendQHTTPURL);
  int getProgressMaxsteps = 10;
  for (int i = 0; i < getProgressMaxsteps; i++) {
    switch (getProgress) {

      case 1:
        sendBuffer = (uint8_t*) "\r\nAT+QIACT?\r\n";
        UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 16,processTxDone);
        UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_QIACT1, 17, processRxDone);
        getProgress++;
        break;
      case 2:
        sendBuffer = (uint8_t*) "\r\nAT+QICSGP=1,\"CMNET\"\r\n";
        UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 25,processTxDone);
        UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_QICSGP, 29, processRxDone);
        getProgress++;
        break;
      case 3:
        sendBuffer = (uint8_t*) url;
        UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 14,processTxDone);
        UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_QIACT2,11, processRxDone);
        getProgress++;
        break;

      case 4:
        sendBuffer = (uint8_t*) "\r\nAT+QIACT?\r\n";
        UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 13,processTxDone);
        UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_QIACT3,13, processRxDone);
        getProgress++;
        break;
      case 5:
        sendBuffer = (uint8_t*) "\r\nAT+QHTTPURL=50,30\r\n";
        UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 21,processTxDone);
        UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_QHTTPURL,30, processRxDone);
        getProgress++;
        break;
      case 6:

        UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, url, url_length,processTxDone);
        UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_URL, 4, processRxDone);
        getProgress++;
        break;
      case 7:
        sendBuffer = (uint8_t*) "\r\nAT+QHTTPGET=80\r\n";
        UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 18,processTxDone);
        UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_QHTTPGET, 30, processRxDone);
        getProgress++;
        break;
      case 8:
        sendBuffer = (uint8_t*) "\r\nAT+QHTTPREAD=90\r\n";
        UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 19,processTxDone);
        UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_QHTTPREAD, 100, processRxDone);
        getProgress++;
        break;

      default:
        //to show that the count has been updated on the server successfully
        GPIO_PinOutSet(GPIOF, GPIO_LED0);
        delay();
        delay();
        GPIO_PinOutClear(GPIOF, GPIO_LED0);
        break;
    }
    uartdelay();  // a delay to allow one reading via the uart to finish before moving to the next read.
  }


}

void postRequest(){
  switch (getProgress) {
    case 1:
      sendBuffer = (uint8_t*) "\r\nAT+QIFGCNT=0\r\n";
      UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 16,processTxDone);
      UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_QIFGCNT, 20, processRxDone);
      getProgress++;
      break;
    case 2:
      sendBuffer = (uint8_t*) "\r\nAT+QICSGP=1,\"CMNET\"\r\n";
      UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 25,processTxDone);
      UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_QICSGP, 29, processRxDone);
      getProgress++;
      break;
    case 3:
      sendBuffer = (uint8_t*) "\r\nAT+QHTTPURL=31,30\r\n";
      UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 21,processTxDone);
      UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_QHTTPURL,30, processRxDone);
      getProgress++;
      break;
    case 4:
      sendBuffer = (uint8_t*) "\r\nhttp://41.78.128.25/httpds?\r\n";
      UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 28,processTxDone);
      UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_URL, 30, processRxDone);
      getProgress++;
      break;
    case 5:
      sendBuffer = (uint8_t*) "\r\nAT+QHTTPPOST=18,50,10\r\n";
      UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, 25,processTxDone);
      UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_QHTTPGET, 30, processRxDone);
      getProgress++;
      break;
    case 6:
      message = (uint8_t*) "\r\nRX1=whatever \r\n";
      UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, message, 14,processTxDone);
      UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) response_QHTTPREAD, 54, processRxDone);
      getProgress++;
      break;

    default:
      GPIO_PinOutSet(GPIOF, GPIO_LED0);
      step=5;
      break;
  }
  uartdelay();          // a delay to allow one reading via the uart to finish before moving to the next read.
}

char* analyse(char *buffer, int size){
  bool flag = false;
  char* ptr = buffer;
  int buffer_size = size;
  int j = 0;
  for (int i = 0; i < buffer_size; i++) {
    if(strcmp(ptr,"\r")!=0){
        ptr++;
        if (strcmp(ptr,"\n")==0) {
          if (flag) {
              newBuffer[j] = '\n';
          }
        }else{
            ptr--;
        }

    }else{
        newBuffer[j]=(char) ptr[i];
        flag = true;
    }
    j++;
    ptr++;
  }
  newBuffer[j]='\0';
  for (int i = 0; i < j; ++i) {
    ptr--;
  }

  return ptr;
}

void mainModemProgram(uint8_t *url){
  switch (step) {
    case 0:       //switch modem on
      on_PWRKEY();
      GPIO_PinOutClear(GPIOF, GPIO_LED0);
      step=3;

      //UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, receiveBuffer, lengths[counter], processRxDone);
      break;

    case 1:       //write AT command to the modem
      //sendBuffer = (uint8_t*) getRequestCommands[counter];
      //UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, sendBuffer, lengths2[counter],processTxDone);

      step++;
      break;

    case 2:

      UARTDRV_Receive(sl_uartdrv_usart_inst0_handle, (uint8_t*) r_buffer, 500, processRxDone);
      counter++;
      break;

    case 3:
      getRequest(url);

      break;
    case 4:
      postRequest();

      break;

    default:
      GPIO_PinOutToggle(GPIOF, GPIO_LED0);
      delay();
      break;

  }


  //UART transfer data

  //GPIO_PinOutToggle(gpioPortF, GPIO_LED0);
  //delay();
  //delay();
  //UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, (uint8_t*) r_buffer, strlen(r_buffer),processTxDone);
  //delay();
}


void USART0_RX_IRQHandler (void)
{
  USART_IntClear (USART0, USART_IF_RXDATAV);
}


