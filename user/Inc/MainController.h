#ifndef MYINC_MAINCONTROLLER_H_
#define MYINC_MAINCONTROLLER_H_
#include "stdint.h"

void MainControllerOpen(void);
void MainControllerLoop(void);
void MainControllerSetMode(uint8_t newMode);
uint8_t MainControllerGetMode(void);

#endif /* MYINC_MAINCONTROLLER_H_ */
