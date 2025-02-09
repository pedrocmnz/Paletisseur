/*
 * bsp.h
 *
 *  Created on: Dec 28, 2023
 *      Author: Laurent
 */

#ifndef INC_BSP_H_
#define INC_BSP_H_

#include "stm32f0xx.h"
#include "main.h"

/*
 * LED driver functions
 */

// Green LED on Nucleo Board
void	BSP_LED_Init			(void);
void	BSP_LED_On				(void);
void	BSP_LED_Off				(void);
void	BSP_LED_Toggle			(void);

// RGB LED
void 	BSP_RGB_LED_PWM_Init	(void);
void	BSP_RGB_LED_SetRED		(uint8_t duty);
void	BSP_RGB_LED_SetGREEN	(uint8_t duty);
void	BSP_RGB_LED_SetBLUE		(uint8_t duty);


/*
 * Push-Button driver functions
 */
void       BSP_PB_Init			(void);
uint8_t    BSP_PB_GetState		(void);


/*
 * Debug Console init
 */
void	BSP_Console_Init		(void);


/*
 * Timer init
 */
void 	BSP_TIMER_Timebase_Init	(void);



#endif /* INC_BSP_H_ */
