#ifndef __SPEED1_USART__
#define __SPEED1_USART__

#include "stm32f4xx.h"

//COM2 Define


/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define USART_SPD1 USART6

#define SPD1_USART_CLK RCC_APB2Periph_USART6
#define SPD1_USART_APBxClkCmd RCC_APB2PeriphClockCmd
#define SPD1_USART_BAUDRATE 9600 //串口波特率

#define SPD1_USART_RX_GPIO_PORT GPIOC
#define SPD1_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define SPD1_USART_RX_PIN GPIO_Pin_6
#define SPD1_USART_RX_AF GPIO_AF_USART6
#define SPD1_USART_RX_SOURCE GPIO_PinSource6

#define SPD1_USART_TX_GPIO_PORT GPIOC
#define SPD1_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define SPD1_USART_TX_PIN GPIO_Pin_7
#define SPD1_USART_TX_AF GPIO_AF_USART6
#define SPD1_USART_TX_SOURCE GPIO_PinSource7

#define SPD1_USART_IRQ USART6_IRQn
#define SPD1_USART_IRQHandler USART6_IRQHandler

//----------------------------------------------------------------
#define SPD1_BAUDRATE 384
#define SPD1_REG_LEN 1
#define SPD1_STATION 1


#define SPD1_CUR_ANG wReg[0]  //编码器当前角度
#define SPD1_CUR_TICK wReg[1] //编码器当前角度计时
#define SPD1_CUR_DETA wReg[2] //编码器速度变化
#define SPD1_CUR_SPD wReg[3]  //编码器当前速度
#define SPD1_LST_ANG wReg[4]  //编码器上次角度
#define SPD1_LST_TICK wReg[5] //编码器上次角度计时
#define SPD1_LST_DETA wReg[6] //编码器角度变化量
#define SPD1_AVG_SPD wReg[7]  //编码器10次平均速度
#define SPD1_COM_FAIL wReg[8] //编码器通讯失败计数
#define SPD1_COM_SUCS wReg[9] //编码器通讯成功计算
#define SPD1_COM_CNT wReg[10] //编码器通讯通讯计时

#define SPD1_COUNTER wReg[11] //编码器圈数计数器

void SPD1_Init(void);
void SPD1_TxCmd(void);
void SPD1_Task(void);

void SPD1_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
