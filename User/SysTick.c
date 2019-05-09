/******************** (C) COPYRIGHT 2012 WildFire Team ***************************
 * 文件名  ：SysTick.c
 * 描述    ：SysTick 系统滴答时钟10us中断函数库,中断时间可自由配置，
**********************************************************************************/
#include "SysTick.h"
#include "Modbus_svr.h"
#include "..\user\Modbus_svr.h"

#define TIMER_NUM  10

extern short wReg[] ;

static __IO u32 TimingDelay = 0 ;  	// 延时定时器计数器
__IO u16	TimePre[TIMER_NUM] ;		//	计数器预置值
__IO u16	TimeCur[TIMER_NUM] ;		//	计数器预置值
__IO u8		TimerFlag[TIMER_NUM] ;
static __IO u32 nCurTicks = 0 ;

//---------SysTick Initialize----------------------------------------------------
void SysTick_Init(void)
{
	int i ;
	/* SystemFrequency / 10000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
	if (SysTick_Config(SystemCoreClock / 10000))	// ST3.5.0库版本
	{ 
		while (1);
	}	
	// Enable SysTick
	SysTick->CTRL |= ~SysTick_CTRL_ENABLE_Msk;
	for( i = 0 ; i < TIMER_NUM ; i++)
	{
		TimePre[i] = 0 ;
		TimeCur[i] = 0 ;
		TimerFlag[i] = 0 ;
	}
}

//------------- Delay ms-----------------------------------------------------------
void Delay_ms(__IO u32 nTime)
{ 
	TimingDelay = nTime;	
	while(TimingDelay != 0);
}

//------SysTick Handler --------------------------------------------------------------
void SysTick_Handler(void)
{
	int i ;
	
	ModbusTimer() ;
	nCurTicks++ ;

	if (!TimingDelay)
		TimingDelay-- ;
	
	for( i = 0 ; i < TIMER_NUM ; i++)
	{
		if ( TimeCur[i] )
		{
			if ( TimeCur[i] == 1 )
				TimerFlag[i] = 1 ;
			TimeCur[i]-- ;			
		}
		else
			TimeCur[i] = TimePre[i] ;
	}	
}

//------SysTick Handler --------------------------------------------------------------
void SetTimer(u8 no, u16 val)
{
	if ( no < TIMER_NUM )
	{
		TimePre[no]  = val ;
		TimeCur[no] = val ;
	}
}

//------SysTick Handler --------------------------------------------------------------
u16 GetTimer(u8 no)
{
	
	if ( no < TIMER_NUM && TimerFlag[no] )
	{
		TimerFlag[no] = 0 ;
		return 1 ;
	}
	else
		return 0 ;
		
}

//----------Get current ticks-----------------------------------------------------
u32 GetCurTicks(void)
{
	return nCurTicks ;
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/

