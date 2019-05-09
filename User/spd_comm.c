#include "spd_comm.h"

extern short wReg[];

//-------------------------------------------------------------------------------
//	@brief	速度值計算隊列初始化
//	@param	svq:隊列指針
//	@retval	None
//-------------------------------------------------------------------------------
void SpdQueueInit(SpeedValueQueue *svq)
{
    int i;

    svq->ptr_head = 0;
    svq->ptr_tail = 0;
    svq->lSum_ang = 0;
    svq->lSum_tim = 0;
    for (i = 0; i < SPD1_QUEUE_LEN; i++)
    {
        svq->queue_ang[i] = 0;
        svq->queue_tim[i] = 0;
    }
}

//-------------------------------------------------------------------------------
//	@brief	速度值計算隊列初始化
//	@param	svq:隊列指針
//          val:插入隊列的值
//	@retval	None
//-------------------------------------------------------------------------------
void SpdQueueIn(SpeedValueQueue *svq, short ang, short tim)
{
    svq->lSum_ang += ang;
    svq->lSum_ang -= svq->queue_ang[svq->ptr_head];
    svq->lSum_tim += tim;
    svq->lSum_tim -= svq->queue_tim[svq->ptr_head];

    svq->queue_ang[svq->ptr_head] = ang;
    svq->queue_tim[svq->ptr_head] = tim;

    svq->ptr_head++;
    if (svq->ptr_head >= SPD1_QUEUE_LEN)
        svq->ptr_head = 0;
}

//-------------------------------------------------------------------------------
//	@brief	速度值計算隊列初始化
//	@param	svq:隊列指針
//	@retval	隊列中保存數據的平均值
//-------------------------------------------------------------------------------
short SpdQueueAvgVal(SpeedValueQueue *svq)
{
    if (svq->lSum_tim == 0)
        return 0;
    return svq->lSum_ang * 1000 / svq->lSum_tim;
}

/****************************************************************
 *	@brief	PID模块初始化
 *	@param	pPid模块指针
 *          no 参数起始地址
 *	@retval	None
 ****************************************************************/
void PIDMod_initialize(PID_Module *pPid, int no)
{
    pPid->pParaAdr = &wReg[no];

    pPid->usLastFunc = 0;

    pPid->vOutL1 = 0;
    pPid->vOutL2 = 0;
    pPid->sDeltaL1 = 0;
    pPid->sDeltaL2 = 0;
}

/****************************************************************
 *	@brief	PID模块参数检测
 *	@param	pPid模块指针
 *          no 参数起始地址
 *	@retval	None
 ****************************************************************/
void PIDMod_update_para(PID_Module *pPid)
{
    if (pPid->pParaAdr[0] > 130 || pPid->pParaAdr[0] < 0) //判断输入寄存器地址
        pPid->pParaAdr[0] = 0;
    if (pPid->pParaAdr[1] >= 200 || pPid->pParaAdr[1] < 100) //判断输出寄存器地址
        pPid->pParaAdr[1] = 199;

    if (pPid->pParaAdr[7] < 0)
        pPid->pParaAdr[7] = 0;
    if (pPid->pParaAdr[7] > 100)
        pPid->pParaAdr[7] = 100;

    if (pPid->pParaAdr[9] < 0 || pPid->pParaAdr[9] > 3)
        pPid->pParaAdr[9] = 0;
}

/****************************************************************
 *	@brief	PID模块计算
 *	@param	pPid模块指针
 *	@retval	None
 ****************************************************************/
void PIDMod_step(PID_Module *pPid)
{
    long int pid_u, pid_out;
    long int curDelta, tmp, val;

    if (pPid->pParaAdr[9] == 0 && pPid->usLastFunc != 0)
    {
        wReg[pPid->pParaAdr[1]] = 0x8000; //单回路PID
        if (pPid->usLastFunc == 2 || pPid->usLastFunc == 3)
            wReg[pPid->pParaAdr[1] + 1] = 0x8000; //正并联或反并联

        pPid->sDeltaL1 = 0;
        pPid->sDeltaL2 = 0;

        pPid->vOutL2 = 0;
        pPid->vOutL1 = 0;
    }
    pPid->usLastFunc = pPid->pParaAdr[9];

    if (pPid->pParaAdr[9] == 0)
        return;

    curDelta = pPid->pParaAdr[3] - wReg[pPid->pParaAdr[0]]; //当前偏差值

    pid_u = pPid->pParaAdr[4] * (curDelta - pPid->sDeltaL1) +
            pPid->pParaAdr[5] * pPid->sDeltaL1 +
            pPid->pParaAdr[6] * (curDelta - 2 * pPid->sDeltaL1 + pPid->sDeltaL2);
    pPid->sDeltaL2 = pPid->sDeltaL1;
    pPid->sDeltaL1 = curDelta;

    pid_out = pPid->vOutL1;
    if (pPid->pParaAdr[8] == 0) //根据作用方式确定是增量还是减量
        pid_out -= pid_u;
    else
        pid_out += pid_u;

    //输出值限幅，避免调节器饱和
    if (pid_out > PID_MAX_OUT)
        pid_out = PID_MAX_OUT;
    if (pid_out < -PID_MAX_OUT)
        pid_out = -PID_MAX_OUT;

    //输出限幅
    tmp = 0x8000 * pPid->pParaAdr[7] / 100 - 1;
    val = pid_out / 1000;
    if (val > tmp)
        val = tmp;
    if (val < -tmp)
        val = -tmp;

    //死区调整
    tmp = (int)pPid->pParaAdr[2];
    if (val <= wReg[PID_ZERO_ZONE] && val >= -wReg[PID_ZERO_ZONE])
        val = 0;
    if (val > wReg[PID_ZERO_ZONE])
        val = tmp + val * (0x7FFF - tmp) / 0x7FFF;
    if (val < -wReg[PID_ZERO_ZONE])
        val = -tmp + val * (0x8000 - tmp) / 0x8000;

    //输出方式选择
    val &= 0x0000FFFF;
    wReg[pPid->pParaAdr[1]] = 0x8000 + val; //单回路PID

    //输出方式选择
    if (pPid->pParaAdr[9] == 2)
        wReg[pPid->pParaAdr[1] + 1] = 0x8000 + val; //正向并联PID
    if (pPid->pParaAdr[9] == 3)
        wReg[pPid->pParaAdr[1] + 1] = 0x8000 - val; //反向并联PID

    //保存中间结果
    pPid->vOutL2 = pPid->vOutL1;
    pPid->vOutL1 = pid_out;
}

/****************************************************************
 *	@brief	推进器模块的计算
 *	@param	pPid模块指针
 *	@retval	None
 ****************************************************************/
void Thruster_step(PID_Module *pPid)
{
    short force;

    if (pPid->pParaAdr[9] == 0 && pPid->usLastFunc != 0)
    {
        wReg[pPid->pParaAdr[1]] = 0x8000; //单回路PID
        if (pPid->usLastFunc == 2 || pPid->usLastFunc == 3)
            wReg[pPid->pParaAdr[1] + 1] = 0x8000; //正并联或反并联

        pPid->sDeltaL1 = 0;
        pPid->sDeltaL2 = 0;

        pPid->vOutL2 = 0;
        pPid->vOutL1 = 0;
    }
    pPid->usLastFunc = pPid->pParaAdr[9];

    if (pPid->pParaAdr[9] == 0)
        return;

    if (pPid->pParaAdr[2] == 0) //PID控制
        force = PID_controller(pPid);

    if (pPid->pParaAdr[2] == 1) //模糊控制
        force = Fuzzy_controller(pPid);

    if (pPid->pParaAdr[2] == 2) //误差小于100采用PID控制，反之采用模糊控制
        if (pPid->sDeltaL1 < 300 && pPid->sDeltaL1 > -300)
            force = PID_controller(pPid);
        else
            force = Fuzzy_controller(pPid);

    if (pPid->pParaAdr[2] == 3) //PID控制，采用模糊规则调节PID参数
    {
        force = PID_controller(pPid);
        Fuzzy_PIDParameter_step(pPid);
    }

    wReg[161] = force;

    Thruster_out(pPid, force);
}

/* ******************************************************
 * Desc:PID控制器
 * Param: PID_Module
 * Retval: None
 * *******************************************************/
short PID_controller(PID_Module *pPid)
{
    short curDelta;
    int pid_out;
    int tmp;
    short pid_u;

    curDelta = pPid->pParaAdr[3] - wReg[pPid->pParaAdr[0]]; //当前偏差值 设定值-实际值

    pid_u = pPid->pParaAdr[4] * (curDelta - pPid->sDeltaL1) +
            pPid->pParaAdr[5] * pPid->sDeltaL1 +
            pPid->pParaAdr[6] * (curDelta - 2 * pPid->sDeltaL1 + pPid->sDeltaL2);
    pPid->sDeltaL2 = pPid->sDeltaL1;
    pPid->sDeltaL1 = curDelta;

    pid_out = pPid->vOutL1;
    pid_out += pid_u;
    //输出值限幅，避免调节器饱和
    //输出的最大推进力为130kgf
    if (pid_out > 1300000)
        pid_out = 1300000;
    if (pid_out < -1300000)
        pid_out = -1300000;

    //输出限幅
    tmp = pid_out * pPid->pParaAdr[7] / 100;

    //缩放到-130 - 130范围内
    pid_u = tmp / 10000;
    //出现正偏差,且发生负输出时，输出为正回推进力
    if (curDelta > 0 && pid_u < 0)
    {
        pid_u = wReg[164];
        //pid_out = wReg[164] * 10000;
    }

    //出现负偏差,且发生正输出时，输出为负回正推进力
    if (curDelta < 0 && pid_u > 0)
    {
        pid_u = -wReg[164];
        //pid_out = -wReg[164] * 10000;
    }

    //在小偏差范围内，关断输出
    if (curDelta < wReg[163] && curDelta > -wReg[163])
        pid_u = 0;

    pPid->vOutL2 = pPid->vOutL1;
    pPid->vOutL1 = pid_out;

    return pid_u;
}

/* ******************************************************
 * Desc:推进力到数字输出的转换
 * Param: PID_Module
 * Retval: None
 * *******************************************************/
short ForceToDigitout(short force)
{
    float fout;
    float fin = (force > 0) ? force : -force;
    int val;

    //根据推进力曲线，将推力转化到输出电压
    //Voltage(v):2  	3  		4       5       6       7       8       9       10
    //Force(kgf):2.0  3.0  4.5     9.8     21.7    46.3    70.8    97.2    130
    //根据拟合曲线计算
    // vol = 0.0294*f^3 - 7.057*f^2 + 609.4*f + 8100.0
    fout = fin * 0.0294f - 7.057f;
    fout = fin * fout + 609.4f;
    fout = fin * fout + 8100.0f;

    if (force < 2 && force > -2)
        fout = 0.0f;

    if (force > 0)
        val = (int)fout;
    else
        val = -(int)fout;

    if (val > 32767)
        val = 32767;
    if (val < -32767)
        val = -32767;

    return (short)val;
}

//----------------------模糊控制规则------------------------------------
int F_rulelist[7][7] = {
    {-3, -3, -2, -2, -1, 0, 1},
    {-3, -3, -2, -1, -1, 0, 1},
    {-2, -2, -2, -1, 0, 1, 1},
    {-2, -2, -1, 0, 1, 2, 2},
    {-1, -1, 0, 1, 1, 2, 2},
    {-1, 0, 1, 2, 2, 2, 3},
    {0, 0, 2, 2, 2, 3, 3}};

/* ******************************************************
 * Desc:三角函数
 * Param: Fuzzy_ctl_block
 * Retval: None
 * *******************************************************/
float Fuzzy_trimf(short x, short a, short b, short c)
{
    float u;
    if (x >= a && x <= b)
        u = (float)(x - a) / (float)(b - a);
    else if (x > b && x <= c)
        u = (float)(c - x) / (float)(c - b);
    else
        u = 0.0f;

    return u;
}

/* ******************************************************
 * Desc:模糊控制计算
 * Param: Fuzzy_ctl_block
 * Retval:  输出值
 * *******************************************************/
short Fuzzy_controller(PID_Module *pPid)
{
    float u_e[7];  //误差隶属度
    float u_de[7]; //误差变化率隶属度
    float u_u;     //输出隶属度

    float den, num;

    int u_e_index[3];  //每个输入只激活3个模糊子集
    int u_de_index[3]; //每个输入只激活3个模糊子集
    short curDelta;
    short dcurDelta;

    int i, j;
    short start;  //三角函数起点
    short intval; //三角函数间隔

    curDelta = pPid->pParaAdr[3] - wReg[pPid->pParaAdr[0]]; //当前偏差值 设定值-实际值
    if (curDelta >= 1800)
        curDelta = 1799;
    if (curDelta <= -1800)
        curDelta = -1799;

    //e模糊化，计算误差隶属度
    start = -1800;
    intval = 450; // =1800/4 ;
    j = 0;
    for (i = 0; i < 7; i++)
    {
        u_e[i] = Fuzzy_trimf(curDelta, start, start + intval, start + 2 * intval);
        if (u_e[i] > 0.001f)
            u_e_index[j++] = i;
        start += intval;
    }
    for (; j < 3; j++)
        u_e_index[j] = 0;

    dcurDelta = curDelta - pPid->sDeltaL1;
    pPid->sDeltaL1 = curDelta;

    if (dcurDelta <= -wReg[165])
        dcurDelta = -wReg[165] + 1;
    if (dcurDelta >= wReg[165])
        dcurDelta = wReg[165] - 1;

    //de模糊化，计算误差变化率隶属度
    start = -wReg[165];
    intval = wReg[165] / 4;
    j = 0;
    for (i = 0; i < 7; i++)
    {
        u_de[i] = Fuzzy_trimf(dcurDelta, start, start + intval, start + 2 * intval);
        if (u_de[i] > 0.001f)
            u_de_index[j++] = i;
        start += intval;
    }
    for (; j < 3; j++)
        u_de_index[j] = 0;

    den = 0.0f;
    num = 0.0f;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
        {
            num += u_e[u_e_index[i]] * u_de[u_de_index[j]] * (float)F_rulelist[u_e_index[i]][u_de_index[j]];
            den += u_e[u_e_index[i]] * u_de[u_de_index[j]];
        }

    u_u = (num / den) * 130.0f * pPid->pParaAdr[7] / 300.0f;

    return (short)u_u;
}

/* ******************************************************
 * Desc:推进力输出函数
 * Param: Fuzzy_ctl_block
 * Retval:  输出值
 * *******************************************************/
void Thruster_out(PID_Module *pPid, short force)
{
    short dig_out;
    short act_out;

    dig_out = ForceToDigitout(force);
    wReg[162] = dig_out;

    if (pPid->pParaAdr[8])
        act_out = dig_out;
    else
        act_out = -dig_out;

    wReg[pPid->pParaAdr[1]] = 0x8000 + act_out; //单回路PID

    //输出方式选择
    if (pPid->pParaAdr[9] == 2)
        wReg[pPid->pParaAdr[1] + 1] = 0x8000 + act_out; //正向并联PID
    if (pPid->pParaAdr[9] == 3)
        wReg[pPid->pParaAdr[1] + 1] = 0x8000 - act_out; //反向并联PID
}

//---------根据模糊规则修正PID参数----------------------------
int P_rulelist[7][7] = {
    {PB, PB, PM, PM, PS, ZO, ZO},
    {PB, PB, PM, PS, PS, ZO, ZO},
    {PM, PM, PM, PS, ZO, NS, NS},
    {PM, PM, PS, ZO, NS, NM, NM},
    {PS, PS, ZO, NS, NS, NM, NM},
    {PS, ZO, NS, NM, NM, NM, NB},
    {ZO, ZO, NM, NM, NM, NB, NB}};
int I_rulelist[7][7] = {
    {PS, NS, NB, NB, NB, NM, PS},
    {PS, NS, NB, NM, NM, NS, ZO},
    {ZO, NS, NM, NM, NS, NS, ZO},
    {ZO, NS, NS, NS, NS, NS, ZO},
    {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
    {PB, NS, PS, PS, PS, PS, PB},
    {PB, PM, PM, PM, PS, PS, PB}};
int D_rulelist[7][7] = {
    {NB, NB, NM, NM, NS, ZO, ZO},
    {NB, NB, NM, NS, NS, ZO, ZO},
    {NB, NM, NS, NS, ZO, PS, PS},
    {NM, NM, NS, ZO, PS, PM, PM},
    {NM, NS, ZO, PS, PS, PM, PB},
    {ZO, ZO, PS, PS, PM, PB, PB},
    {ZO, ZO, PS, PM, PM, PB, PB}};

void Fuzzy_PIDParameter_step(PID_Module *pPid)
{
    float u_e[7];  //误差隶属度
    float u_de[7]; //误差变化率隶属度
    float u_u;     //输出隶属度

    float den, num;

    int u_e_index[3];  //每个输入只激活3个模糊子集
    int u_de_index[3]; //每个输入只激活3个模糊子集
    short curDelta;
    short dcurDelta;

    int i, j;
    short start;  //三角函数起点
    short intval; //三角函数间隔

    curDelta = pPid->sDeltaL1; //当前偏差值 设定值-实际值
    if (curDelta >= 1800)
        curDelta = 1799;
    if (curDelta <= -1800)
        curDelta = -1799;

    //e模糊化，计算误差隶属度
    start = -1800;
    intval = 450; // =1800/4 ;
    j = 0;
    for (i = 0; i < 7; i++)
    {
        u_e[i] = Fuzzy_trimf(curDelta, start, start + intval, start + 2 * intval);
        if (u_e[i] > 0.001f)
            u_e_index[j++] = i;
        start += intval;
    }
    for (; j < 3; j++)
        u_e_index[j] = 0;

    dcurDelta = pPid->sDeltaL2 - pPid->sDeltaL1;

    if (dcurDelta <= -wReg[165])
        dcurDelta = -wReg[165] + 1;
    if (dcurDelta >= wReg[165])
        dcurDelta = wReg[165] - 1;

    //de模糊化，计算误差变化率隶属度
    start = -wReg[165];
    intval = wReg[165] / 4;
    j = 0;
    for (i = 0; i < 7; i++)
    {
        u_de[i] = Fuzzy_trimf(dcurDelta, start, start + intval, start + 2 * intval);
        if (u_de[i] > 0.001f)
            u_de_index[j++] = i;
        start += intval;
    }
    for (; j < 3; j++)
        u_de_index[j] = 0;

    den = 0.0f;
    num = 0.0f;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
        {
            num += u_e[u_e_index[i]] * u_de[u_de_index[j]] * (float)P_rulelist[u_e_index[i]][u_de_index[j]];
            den += u_e[u_e_index[i]] * u_de[u_de_index[j]];
        }
    u_u = num / den;
    pPid->pParaAdr[4] += (short)u_u * 5;
    if (pPid->pParaAdr[4] < 100)
        pPid->pParaAdr[4] = 100;
    if (pPid->pParaAdr[4] > 2000)
        pPid->pParaAdr[4] = 2000;

    num = 0.0f;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
        {
            num += u_e[u_e_index[i]] * u_de[u_de_index[j]] * (float)I_rulelist[u_e_index[i]][u_de_index[j]];
        }
    u_u = num / den;
    pPid->pParaAdr[5] += (short)u_u;
    if (pPid->pParaAdr[5] < 1)
        pPid->pParaAdr[5] = 1;
    if (pPid->pParaAdr[5] > 20)
        pPid->pParaAdr[5] = 20;

    num = 0.0f;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
        {
            num += u_e[u_e_index[i]] * u_de[u_de_index[j]] * (float)D_rulelist[u_e_index[i]][u_de_index[j]];
        }
    u_u = num / den;
    pPid->pParaAdr[6] += (short)u_u * 5;
    if (pPid->pParaAdr[6] < 1000)
        pPid->pParaAdr[6] = 1000;
    if (pPid->pParaAdr[6] > 10000)
        pPid->pParaAdr[6] = 10000;
}

/*------------------end of file------------------------*/
