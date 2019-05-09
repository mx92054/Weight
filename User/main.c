/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   用1.5.1版本库建的工程模板
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "SysTick.h"
#include "Modbus_svr.h"
#include "gpio.h"
#include "usart_spd.h"
#include "usart_dis.h"
#include "usart_wig.h"
#include "bsp_innerflash.h"

extern short wReg[];
extern u8 bChanged;

extern uint8_t cpt_frame[];

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{

	SysTick_Init();
	GPIO_Config();

	Flash_Read16BitDatas(FLASH_USER_START_ADDR, 100, wReg + 100) ;

	Modbus_init();
	SPD1_Init() ;
	DIS_Init();
	WIG_Init();

	SetTimer(0, 500);
	SetTimer(1, 100);
	SetTimer(2, 2000);

	IWDG_Configuration();

	while (1)
	{
		Modbus_task();
		SPD1_Task() ;
		DIS_Task();
		WIG_Task();

		if (GetTimer(0))
		{
			IWDG_Feed();
			LOGGLE_LED2;
		}
		
		if ( GetTimer(1) )
		{
			SPD1_TxCmd() ;
			DIS_TxCmd();
			WIG_TxCmd();
		}


		if (GetTimer(2) && bSaved )
		{
			Flash_Write16BitDatas(FLASH_USER_START_ADDR, 100, wReg + 100) ;
			bSaved = 0 ;
		}
	}
}

/*********************************************END OF FILE**********************/
