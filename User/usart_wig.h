#ifndef __WEIGHT_USART__
#define __WEIGHT_USART__

#include "stm32f4xx.h"

//COM2 Define

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define USART_WIG UART7
#define WIG_USART_CLK RCC_APB1Periph_UART7
#define WIG_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define WIG_USART_BAUDRATE 9600 //串口波特率

#define WIG_USART_RX_GPIO_PORT GPIOE
#define WIG_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define WIG_USART_RX_PIN GPIO_Pin_7
#define WIG_USART_RX_AF GPIO_AF_UART7
#define WIG_USART_RX_SOURCE GPIO_PinSource7

#define WIG_USART_TX_GPIO_PORT GPIOE
#define WIG_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define WIG_USART_TX_PIN GPIO_Pin_8
#define WIG_USART_TX_AF GPIO_AF_UART7
#define WIG_USART_TX_SOURCE GPIO_PinSource8

#define WIG_USART_IRQ UART7_IRQn
#define WIG_USART_IRQHandler UART7_IRQHandler

//----------------------------------------------------------------
#define WIG_BAUDRATE 96
#define WIG_REG_LEN 1
#define WIG_STATION 1

#define WIG_CUR_VAL wReg[31] //位移传感器值1

#define WIG_COM_FAIL wReg[38] //位移传感器通讯失败计数
#define WIG_COM_SUCS wReg[39] //位移传感器通讯成功计算
#define WIG_COM_CNT wReg[40]  //位移传感器通讯通讯计时

void WIG_Init(void);
void WIG_TxCmd(void);
void WIG_Task(void);

void WIG_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
