#ifndef __GPIO_H__
#define	__GPIO_H__

#include "stm32f4xx.h"

 
#define LED1_PIN					GPIO_Pin_12			// LED1×´Ì¬µÆ
#define LED1_GPIO					GPIOB

#define LED2_PIN					GPIO_Pin_13			// LED2×´Ì¬µÆ
#define LED2_GPIO					GPIOB

											

#define LED1(a)									if (a)	\
																	GPIO_ResetBits(LED1_GPIO,LED1_PIN);\
																else		\
																	GPIO_SetBits(LED1_GPIO,LED1_PIN)
																
#define LOGGLE_LED1 							LED1_GPIO->ODR ^= (LED1_PIN)	

																
#define LED2(a)									if (a)	\
																	GPIO_ResetBits(LED2_GPIO,LED2_PIN);\
																else		\
																	GPIO_SetBits(LED2_GPIO,LED2_PIN)
																
#define LOGGLE_LED2							LED2_GPIO->ODR ^= (LED2_PIN)																	
																
//-----------------------------------------------------------------------------------------
void 	IWDG_Configuration(void) ;
void 	IWDG_Feed(void)  ; 

void 	GPIO_Config(void) ;
																
#endif
					
					
//------------------end of file
