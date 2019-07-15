/*
 * terminal.h
 *
 *  Created on: Feb 12, 2018
 *      Author: owner
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include<stdint.h>

typedef void(*functionPointerType)(uint8_t* arg1, uint8_t* arg2);

void TerminalOpen(void);
void TermianlClose(void);
void TerminalRead(void);
void TerminalWrite(uint8_t *txStr);
void TerminalWriteLength(char *txStr, uint32_t length);
void TerminalMain(void);

#endif /* TERMINAL_H_ */
