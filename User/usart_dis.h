#ifndef __DISPLACEMENT_USART__
#define __DISPLACEMENT_USART__

#include "stm32f4xx.h"

//COM2 Define

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define USART_DIS USART3

#define DIS_USART_CLK RCC_APB1Periph_USART3
#define DIS_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define DIS_USART_BAUDRATE 9600 //串口波特率

#define DIS_USART_RX_GPIO_PORT GPIOB
#define DIS_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOB
#define DIS_USART_RX_PIN GPIO_Pin_10
#define DIS_USART_RX_AF GPIO_AF_USART3
#define DIS_USART_RX_SOURCE GPIO_PinSource10

#define DIS_USART_TX_GPIO_PORT GPIOB
#define DIS_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOB
#define DIS_USART_TX_PIN GPIO_Pin_11
#define DIS_USART_TX_AF GPIO_AF_USART3
#define DIS_USART_TX_SOURCE GPIO_PinSource11

#define DIS_USART_IRQ USART3_IRQn
#define DIS_USART_IRQHandler USART3_IRQHandler

//----------------------------------------------------------------
#define DIS_BAUDRATE 96
#define DIS_REG_LEN 2
#define DIS_STATION 1

#define DIS_CUR_DIS wReg[21]  //位移传感器值：位移
#define DIS_CUR_SPD wReg[22] //位移传感器值：平均速度
#define DIS_CUR_DISMM wReg[23] //油缸位移-mm/10
#define DIS_CUR_SPDMM wReg[24] //油缸速度-mm/10/s

#define DIS_COM_FAIL wReg[28] //位移传感器通讯失败计数
#define DIS_COM_SUCS wReg[29] //位移传感器通讯成功计算
#define DIS_COM_CNT wReg[30]  //位移传感器通讯通讯计时

void DIS_Init(void);
void DIS_TxCmd(void);
void DIS_Task(void);

void DIS_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
