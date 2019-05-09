#ifndef __SPD_COMM__
#define __SPD_COMM__

#include "stm32f4xx.h"

#define SPD1_QUEUE_LEN 10
#define PID_MAX_OUT 0x01F40000
#define PID_ZERO_ZONE 160 //零区域电压设置，应该设置为0x50死区电压设置

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

//速度計算值隊列
typedef struct _tag_speed_queue
{
  u16 ptr_head;
  u16 ptr_tail;
  long lSum_ang;
  long lSum_tim;
  short queue_ang[SPD1_QUEUE_LEN];
  short queue_tim[SPD1_QUEUE_LEN];
} SpeedValueQueue;

//pid调节器结构
typedef struct _tag_PID_Module
{
  short *pParaAdr;
  //参数0：输入变量地址 0 -100
  //参数1：输出变量地址 100-200
  //参数2：输出死区，每个0x0CCC是1V电压左右
  //参数3：设定值
  //参数4：比例  %
  //参数5：积分  ms
  //参数6：微分  ms
  //参数7：输出限幅值 0-100%
  //参数8：作用方式
  //参数9：调节器工作 0-停止   1-工作
  //                2-正并联 3 - 反并联
  //  输出寄存器地址和输出寄存器下一个寄存器
  short valIn;  //参数输入值
  short valOut; //参数输出值

  short usLastFunc; //上一次调节器的作用方式

  long int vOutL1; //上一次计算值
  long int vOutL2; //上二次计算值
  short sDeltaL1;  //上一次偏差值
  short sDeltaL2;  //上二次偏差值
} PID_Module;

//-----------------------------------------------------------------
void SpdQueueInit(SpeedValueQueue *svq);
void SpdQueueIn(SpeedValueQueue *svq, short ang, short tim);
short SpdQueueAvgVal(SpeedValueQueue *svq);

//--------------------------------------------------------------------
void PIDMod_initialize(PID_Module *pPid, int no); //PID模块初始化
void PIDMod_step(PID_Module *pPid);               //PID模块计算
void Thruster_step(PID_Module *pPid);             //推进器模块计算
void PIDMod_update_para(PID_Module *pPid);        //PID模块参数更新

short ForceToDigitout(short force); //推进力到数字量的换算
float Fuzzy_trimf(short x, short a, short b, short c);

float Fuzzy_trimf(short x, short a, short b, short c); //三角函数运算
short Fuzzy_controller(PID_Module *pPid);              //模糊控制器
short PID_controller(PID_Module *pPid);                //PID控制器
void Thruster_out(PID_Module *pPid, short force);      //推进力输出函数
void Fuzzy_PIDParameter_step(PID_Module *pPid);        //模糊算法调整PID参数

#endif
/*------------------end of file------------------------*/
