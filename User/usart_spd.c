#include "usart_spd.h"
#include "spd_comm.h"
#include "Modbus_svr.h"
#include "SysTick.h"
#include "stm32f4xx_conf.h"

extern u8 bChanged;
extern u16 wReg[];

uint8_t SPD1_frame[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
u8 SPD1_buffer[256];
u8 SPD1_curptr;
u8 SPD1_bRecv;
u8 SPD1_frame_len = 85;
u8 SPD1_bFirst = 1;
u32 ulSpd1Tick = 0;

SpeedValueQueue qspd1;

//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SPD1_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = SPD1_USART_IRQ;
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
static void SPD1_Config(u16 wBaudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(SPD1_USART_RX_GPIO_CLK | SPD1_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 USART 时钟 */
    SPD1_USART_APBxClkCmd(SPD1_USART_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SPD1_USART_TX_PIN;
    GPIO_Init(SPD1_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SPD1_USART_RX_PIN;
    GPIO_Init(SPD1_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(SPD1_USART_RX_GPIO_PORT, SPD1_USART_RX_SOURCE, SPD1_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(SPD1_USART_TX_GPIO_PORT, SPD1_USART_TX_SOURCE, SPD1_USART_TX_AF);

    /* 配置串SPD1_USART 模式 */
    /* 波特率设置：SPD1_USART_BAUDRATE */
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
    USART_Init(USART_SPD1, &USART_InitStructure);

    /* 嵌套向量中断控制器NVIC配置 */
    SPD1_NVIC_Configuration();

    /* 使能串口接收中断 */
    USART_ITConfig(USART_SPD1, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART_SPD1, ENABLE);
}

/****************************************************************
 *	@brief:	    SPD1通信初始化程序
 *	@param:	    None
 *	@retval:	None
 ****************************************************************/
void SPD1_Init(void)
{
    SPD1_Config(SPD1_BAUDRATE);

    SPD1_curptr = 0;
    SPD1_bRecv = 0;
    SPD1_COM_FAIL = 0;
    SPD1_frame_len = 2 * SPD1_REG_LEN + 5;
    ulSpd1Tick = GetCurTicks();

    SpdQueueInit(&qspd1);
}

//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD1_TxCmd(void)
{
    u16 uCRC;

    if (SPD1_bRecv == 1) //如果当前未完成接收，则通信错误计数器递增
        SPD1_COM_FAIL++;

    SPD1_curptr = 0;
    SPD1_bRecv = 1;

    if ( SPD1_bFirst )
    {
        uCRC = CRC16(SPD1_frame, 6);
        SPD1_frame[6] = uCRC & 0x00FF;        //CRC low
        SPD1_frame[7] = (uCRC & 0xFF00) >> 8; //CRC high
        SPD1_bFirst = 0;
    }

    Usart_SendBytes(USART_SPD1, SPD1_frame, 8);
}

/*
 *	@brief	接收数据处理
 *	@param	None
 *	@retval	None
 */
void SPD1_Task(void)
{
    u32 tick;

    if (SPD1_curptr < SPD1_frame_len)
        return;

    if (SPD1_buffer[0] != SPD1_STATION || SPD1_buffer[1] != 0x03) //站地址判断
        return;

    if (SPD1_buffer[2] != 2 * SPD1_REG_LEN) //数值长度判读
        return;

    tick = GetCurTicks();
    SPD1_LST_ANG = SPD1_CUR_ANG;   //上次编码器值
    SPD1_LST_TICK = SPD1_CUR_TICK; //上次计时器值
    SPD1_LST_DETA = SPD1_CUR_DETA; //上次角度变化值

    SPD1_CUR_ANG = SPD1_buffer[3] << 0x08 | SPD1_buffer[4]; //本次编码器值
    SPD1_CUR_TICK = tick - ulSpd1Tick;                      //本次计时器值
    SPD1_COM_CNT = SPD1_CUR_TICK;
    ulSpd1Tick = tick;                           //保存计时器
    SPD1_CUR_DETA = SPD1_CUR_ANG - SPD1_LST_ANG; //本次角度变化量
    if (SPD1_CUR_ANG < 1024 && SPD1_LST_ANG > 3072)
    {
        SPD1_CUR_DETA = SPD1_CUR_ANG - SPD1_LST_ANG + 4096;
        SPD1_COUNTER++;
    }
    if (SPD1_CUR_ANG > 3072 && SPD1_LST_ANG < 1024)
    {
        SPD1_CUR_DETA = SPD1_CUR_ANG - SPD1_LST_ANG - 4096;
        SPD1_COUNTER--;
    }
    if (SPD1_CUR_TICK != 0)
        SPD1_CUR_SPD = SPD1_CUR_DETA * 1000 / SPD1_CUR_TICK; //本次速度

    SpdQueueIn(&qspd1, SPD1_CUR_DETA, SPD1_CUR_TICK);
    SPD1_AVG_SPD = SpdQueueAvgVal(&qspd1); //10次平均速度

    SPD1_COM_SUCS++;
    SPD1_bRecv = 0;
    SPD1_curptr = 0;
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD1_USART_IRQHandler(void)
{
    u8 ch;

    if (USART_GetITStatus(USART_SPD1, USART_IT_RXNE) != RESET) //判断读寄存器是否非空
    {
        ch = USART_ReceiveData(USART_SPD1); //将读寄存器的数据缓存到接收缓冲区里
        SPD1_buffer[SPD1_curptr++] = ch;
    }

    if (USART_GetITStatus(USART_SPD1, USART_IT_TXE) != RESET)
    {
        USART_ITConfig(USART_SPD1, USART_IT_TXE, DISABLE);
    }
}

//-----------------------------End of file--------------------------------------------------
