#include "usart_wig.h"
#include "spd_comm.h"
#include "Modbus_svr.h"
#include "SysTick.h"
#include "stm32f4xx_conf.h"

extern u8 bChanged;
extern u16 wReg[];

uint8_t WIG_frame[8] = {0x01, 0x03, 0x00, 0x55, 0x00, 0x01, 0x00, 0x00};
u8 WIG_buffer[256];
u8 WIG_curptr;
u8 WIG_bRecv;
u8 WIG_frame_len = 85;
u8 WIG_bFirst = 1;
u32 ulWIGTick = 0;

SpeedValueQueue qWIG;

//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void WIG_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = WIG_USART_IRQ;
    /* 抢断优先级为1 */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    /* 子优先级为1 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

//-------------------------------------------------------------------------------
//	@brief	串口初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void WIG_Config(u16 wBaudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(WIG_USART_RX_GPIO_CLK | WIG_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 USART 时钟 */
    WIG_USART_APBxClkCmd(WIG_USART_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = WIG_USART_TX_PIN;
    GPIO_Init(WIG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = WIG_USART_RX_PIN;
    GPIO_Init(WIG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(WIG_USART_RX_GPIO_PORT, WIG_USART_RX_SOURCE, WIG_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(WIG_USART_TX_GPIO_PORT, WIG_USART_TX_SOURCE, WIG_USART_TX_AF);

    /* 配置串WIG_USART 模式 */
    /* 波特率设置：WIG_USART_BAUDRATE */
    USART_InitStructure.USART_BaudRate = wBaudrate * 100;
    /* 字长(数据位+校验位)：8 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    /* 停止位：1个停止位 */
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    /* 校验位选择：不使用校验 */
    USART_InitStructure.USART_Parity = USART_Parity_No;
    /* 硬件流控制：不使用硬件流 */
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* USART模式控制：同时使能接收和发送 */
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* 完成USART初始化配置 */
    USART_Init(USART_WIG, &USART_InitStructure);

    /* 嵌套向量中断控制器NVIC配置 */
    WIG_NVIC_Configuration();

    /* 使能串口接收中断 */
    USART_ITConfig(USART_WIG, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART_WIG, ENABLE);
}

/****************************************************************
 *	@brief:	    WIG通信初始化程序
 *	@param:	    None
 *	@retval:	None
 ****************************************************************/
void WIG_Init(void)
{
    WIG_Config(WIG_BAUDRATE);

    WIG_curptr = 0;
    WIG_bRecv = 0;
    WIG_COM_FAIL = 0;
    WIG_frame_len = 2 * WIG_REG_LEN + 5;
    ulWIGTick = GetCurTicks();

    SpdQueueInit(&qWIG);
}

//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void WIG_TxCmd(void)
{
    u16 uCRC;

    if (WIG_bRecv == 1) //如果当前未完成接收，则通信错误计数器递增
        WIG_COM_FAIL++;

    WIG_curptr = 0;
    WIG_bRecv = 1;

    if (WIG_bFirst)
    {
        uCRC = CRC16(WIG_frame, 6);
        WIG_frame[6] = uCRC & 0x00FF;        //CRC low
        WIG_frame[7] = (uCRC & 0xFF00) >> 8; //CRC high
        WIG_bFirst = 0;
    }

    Usart_SendBytes(USART_WIG, WIG_frame, 8);
}

/*
 *	@brief	接收数据处理
 *	@param	None
 *	@retval	None
 */
void WIG_Task(void)
{
    u32 tick;

    if (WIG_curptr < WIG_frame_len)
        return;

    if (WIG_buffer[0] != WIG_STATION || WIG_buffer[1] != 0x03) //站地址判断
        return;

    if (WIG_buffer[2] != 2 * WIG_REG_LEN) //数值长度判读
        return;

    tick = GetCurTicks();

    WIG_CUR_VAL = WIG_buffer[3] << 0x08 | WIG_buffer[4]; //本次位移传感器器值1
    WIG_COM_CNT = tick - ulWIGTick;                      //本次计时器值
    ulWIGTick = tick;                                     //保存计时器

    WIG_COM_SUCS++;
    WIG_bRecv = 0;
    WIG_curptr = 0;
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void WIG_USART_IRQHandler(void)
{
    u8 ch;

    if (USART_GetITStatus(USART_WIG, USART_IT_RXNE) != RESET) //判断读寄存器是否非空
    {
        ch = USART_ReceiveData(USART_WIG); //将读寄存器的数据缓存到接收缓冲区里
        WIG_buffer[WIG_curptr++] = ch;
    }

    if (USART_GetITStatus(USART_WIG, USART_IT_TXE) != RESET)
    {
        USART_ITConfig(USART_WIG, USART_IT_TXE, DISABLE);
    }
}

//-----------------------------End of file--------------------------------------------------
