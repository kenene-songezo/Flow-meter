/*
 * httpget.h
 *
 *  Created on: 24 Feb 2021
 *      Author: Songezo Kenene
 */

#ifndef HTTPGET_H_
#define HTTPGET_H_

void init_GPIO();
void delay(void);
void uartdelay(void);
void oneseconddelay();
void on_PWRKEY();
void mainModemProgram();
void getRequest();
void postRequest();
char* analyse(char *buffer, int size);



#endif /* HTTPGET_H_ */
