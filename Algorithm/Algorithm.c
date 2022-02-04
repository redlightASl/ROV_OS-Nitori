#include "Algorithm.h"
#include "HardwareAccelerate.h"

#define GET_BIT_N_VAL(data, n) (0x1 & (( *((data) + (n)/8) & (0x1 << ((n)%8)) ) >> ((n)%8)))

/**
 * @brief 逐位加和运算
 * @param  CacString        待校验数据
 * @param  CacStringSize    待校验数据长度
 * @return u32 加和运算结果
 */
static u32 SumCalculate(u8* CacString, u8 CacStringSize)
{
    u32 CacResult = CacString[0];
    for (u8 i = 0; i < CacStringSize; i++)
    {
        CacResult += CacString[i];
    }
    return CacResult;
}

/**
 * @brief CRC校验
 * @param  CacString        待校验数据
 * @param  CacStringSize    待校验数据长度
 * @return u32 CRC运算结果
 */
static u32 CrcCalculate(u8* CacString, u8 CacStringSize)
{
    u32 CacResult;
    for (int num = CacStringSize; num > 0; num--)
    {
        CacResult = CacResult ^ (*CacString++ << 8);
        for (u8 i = 0; i < 8; i++)
        {
            if (CacResult & 0x8000)
            {
                CacResult = (CacResult << 1) ^ 0x1021;
            }
            else
            {
                CacResult <<= 1;
            }
        }
        CacResult &= 0xFFFF;
    }
    return CacResult;
}

/**
 * @brief 逐位奇偶运算
 * @param  CacString        待校验数据
 * @param  CacStringSize    待校验数据长度
 * @return u8 奇偶运算结果
 */
static u8 ParityCalculate(u8* CacString, u8 CacStringSize)
{
    u32 CacResult = 0;
    for (u8 i = 0; i < CacStringSize; i++)
    {
        CacResult += CacResult + GET_BIT_N_VAL((CacString), i);
    }
    if (CacResult % 2 == 0)
    {
        return 1u;
    }
    else
    {
        return 0u;
    }
}

/**
 * @brief 逐位异或运算
 * @param  CacString        待校验数据
 * @param  CacStringSize    待校验数据长度
 * @return u8 异或运算结果
 */
static u8 XorCalculate(u8* CacString, u8 CacStringSize)
{
    u8 CacResult = CacString[0];
    for (u8 i = 0; i < CacStringSize; ++i)
    {
        CacResult ^= CacString[i];
    }
    return CacResult;
}

/**
 * @brief 加和校验
 * @param  CacString        待校验字符串
 * @param  CalLength        待校验字符串长度
 * @param  CacBit           校验位
 * @return u8 成功返回1，失败返回0
 */
ROV_ALWAYS_INLINE u8 SumCheck(u8* CacString, u8 CalLength, u8 CacBit)
{
#ifdef DATA_CHECK
#ifdef HARDWARE_ACCELERATE_SUM
    u8 Res = 0;
    rov_sum_check(CacString, &Res, CalLength);
    return ((CacString[CacBit] == Res) ? 1 : 0);
#else
    return ((CacString[CacBit] == SumCalculate(CacString, CalLength)) ? 1 : 0);
#endif
#else
    return 1; //不开启校验时默认成功
#endif
}

/**
 * @brief CRC校验
 * @param  CacString        待校验字符串
 * @param  CalLength        待校验字符串长度
 * @param  CacBit           校验位
 * @return u8 成功返回1，失败返回0
 */
ROV_ALWAYS_INLINE u8 CrcCheck(u8* CacString, u8 CalLength, u8 CacBit)
{
#ifdef DATA_CHECK
#ifdef HARDWARE_ACCELERATE_CRC
    u8 Res = 0;
    rov_crc_check(CacString, &Res, CalLength);
    return ((CacString[CacBit] == Res) ? 1 : 0);
#else
    return ((CacString[CacBit] == CrcCalculate(CacString, CalLength)) ? 1 : 0);
#endif
#else
    return 1; //不开启校验时默认成功
#endif
}

/**
 * @brief 奇偶校验
 * @param  CacString        待校验字符串
 * @param  CalLength        待校验字符串长度
 * @param  CacBit           校验位
 * @return u8 成功返回1，失败返回0
 */
ROV_ALWAYS_INLINE u8 ParityCheck(u8* CacString, u8 CalLength, u8 CacBit)
{
#ifdef DATA_CHECK
#ifdef HARDWARE_ACCELERATE_PARITY
    u8 Res = 0;
    rov_parity_check(CacString, &Res, CalLength);
    return ((CacString[CacBit] == Res) ? 1 : 0);
#else
    return ((CacString[CacBit] == ParityCalculate(CacString, CalLength)) ? 1 : 0);
#endif
#else
    return 1; //不开启校验时默认成功
#endif
}

/**
 * @brief 异或校验
 * @param  CacString        待校验字符串
 * @param  CalLength        待校验字符串长度
 * @param  CacBit           校验位
 * @return u8 成功返回1，失败返回0
 */
ROV_ALWAYS_INLINE u8 XorCheck(u8* CacString, u8 CalLength, u8 CacBit)
{
#ifdef DATA_CHECK
#ifdef HARDWARE_ACCELERATE_XOR
    u8 Res = 0;
    rov_xor_check(CacString, &Res, CalLength);
    return ((CacString[CacBit] == Res) ? 1 : 0);
#else
    return ((CacString[CacBit] == XorCalculate(CacString, CalLength)) ? 1 : 0);
#endif
#else
    return 1; //不开启校验时默认成功
#endif
}

/**
 * @brief 初始化卡尔曼滤波结构体
 * @param  Kalman           卡尔曼滤波结构体
 * @param  Q                测量误差初值
 * @param  R                预测误差初值
 * @param  P                卡尔曼滤波矩阵初值
 */
void InitKalman(Algorithm_Kalman_t Kalman, f32 Q, f32 R, KalmanMatrix_t P)
{
    //DEBUG:卡尔曼动态ADT
    Kalman->Q = Q; //定义测量误差初值
    Kalman->KalmanMatrix->A = P->A; //定义卡尔曼滤波矩阵初值
    Kalman->KalmanMatrix->B = P->B;
    Kalman->KalmanMatrix->C = P->C;
    Kalman->KalmanMatrix->D = P->D;
    Kalman->R = R; //定义预测误差初值
    Kalman->Measure = 0;
    Kalman->Measure_Pre = 0;
}

/**
 * @brief 进行一次卡尔曼滤波迭代
 * @param  Kalman           卡尔曼滤波结构体
 * @param  Measure          测量值
 * @return f32 本次迭代预测值
 */
f32 KalmanIterOnce(Algorithm_Kalman_t Kalman, f32 Measure)
{
#ifdef HARDWARE_ACCELERATE_KALMAN
    //TODO:PID硬件加速
#else
    f32 error = 0;
    f32 A = Kalman->KalmanMatrix->A;
    f32 B = Kalman->KalmanMatrix->B;
    f32 C = Kalman->KalmanMatrix->C;
    f32 D = Kalman->KalmanMatrix->D;

    f32 E = A + B + C + D + Kalman->Q;
    error = Measure - Kalman->Measure_Pre - Kalman->R;
    Kalman->Measure_Pre = Kalman->Measure_Pre + Kalman->R + (E - Kalman->R) * error / E;
    Kalman->R = Kalman->R + (C + D) * error / E;

    Kalman->KalmanMatrix->A = Kalman->R * (E - Kalman->R) / E;
    Kalman->KalmanMatrix->B = Kalman->R * (B + D) / E;
    Kalman->KalmanMatrix->C = Kalman->R * (C + D) / E;
    Kalman->KalmanMatrix->D = ((A * D) + (B * D) - (B * C)) / E;

    return Kalman->Measure_Pre;
#endif
}

/**
 * @brief 初始化PID,填充结构体参数
 * @param  mode             PID模式
 * @param  Pid              PID结构体
 */
void InitPID(u8 mode, Algorithm_PID_t Pid)
{
    //DEBUG:PID动态ADT
    Pid->mode = mode;
    Pid->Ref = 0.0f;
    Pid->FeedBack = 0.0f;
    Pid->Error = 0.0f;
    Pid->DError = 0.0f;
    Pid->DDError = 0.0f;
    Pid->PreError = 0.0f;
    Pid->PreDError = 0.0f;
    Pid->Kp = 0.0f;
    Pid->Ki = 0.0f;
    Pid->Kd = 0.0f;
    Pid->KiDomain = 0.0f;
    Pid->KdDomain = 0.0f;
    Pid->MaxOutput = 0.0f;
    Pid->MinOutput = 0.0f;
    Pid->MaxIntegral = 0.0f;
    Pid->MinIntegral = 0.0f;
    Pid->Output = 0.0f;
}

/**
 * @brief 计算PID值并输出
 * @param  Pid              PID结构体
 * @param  feedback         反馈值
 * @return u32 运行返回1
 */
u32 PIDCal(Algorithm_PID_t Pid, f32 feedback)
{
#ifdef HARDWARE_ACCELERATE_PID
    //TODO:PID硬件加速
#else
    static f32 zero_2 = 0.01f;
    static f32 zero_3 = 0.001f;
    f32 Kp = 0.0f;
    f32 Ki = 0.0f;
    f32 Kd = 0.0f;

    switch (Pid->mode)
    {
        //TODO:整型PID，PID计算Debug
    case PID_IF: //增量浮点型
        Pid->FeedBack = feedback;
        //pid->Error = pid->Ref - pid->FeedBack;
        rov_sub_f32(&Pid->Ref, &Pid->FeedBack, &Pid->Error, 1);
        //pid->DError = pid->Error - pid->PreError;
        rov_sub_f32(&Pid->Error, &Pid->PreError, &Pid->DError, 1);
        //pid->DDError = pid->DError - pid->PreDError;
        rov_sub_f32(&Pid->DError, &Pid->PreDError, &Pid->DDError, 1);
        //把设定参数赋到处理后的实际参数上
        Pid->PreError = Pid->Error;
        Pid->PreDError = Pid->DError;
        //PID分开运算
        //P一直开
        rov_mult_f32(&Pid->Kp, &zero_2, &Kp, 1);
        rov_mult_f32(&Kp, &Pid->DError, &Kp, 1);
        //I在误差大于KiDomain时关闭,KiDomain == 0则关闭该功能	
        if ((fabsf(Pid->Error) < Pid->KiDomain) || !Pid->KiDomain)
        {
            rov_mult_f32(&Pid->Ki, &zero_3, &Ki, 1);
            rov_mult_f32(&Ki, &Pid->Error, &Ki, 1);
        }
        //D在误差小于KdDomain时关闭,KdDomain == 0则关闭该功能	
        if ((fabsf(Pid->Error) > Pid->KdDomain) || !Pid->KdDomain)
        {
            rov_mult_f32(&Pid->Kd, &Pid->DDError, &Kd, 1);
        }
        //加和进行PID公式计算
        rov_add_f32(&Pid->Output, &Kp, &Pid->Output, 1);
        rov_add_f32(&Ki, &Kd, &Ki, 1);
        rov_add_f32(&Pid->Output, &Ki, &Pid->Output, 1);
        //pid->Out += (pid->Kp * pid->DError + pid->Ki * pid->Error / 10 + pid->Kd * pid->DDError * 100); //做积分限幅
        //输出限幅
        if (Pid->Output > Pid->MaxOutput)
        {
            Pid->Output = (Pid->MaxOutput);
        }
        else if (Pid->Output < Pid->MinOutput)
        {
            Pid->Output = Pid->MinOutput;
        }
        break;
    case PID_PF: //位置浮点型
        Pid->FeedBack = feedback;
        //pid->Error = pid->Ref - pid->FeedBack;
        rov_sub_f32(&Pid->Ref, &Pid->FeedBack, &Pid->Error, 1);
        //pid->integral	+=	pid->Error;
        rov_add_f32(&Pid->DeltaIntegral, &Pid->Error, &Pid->DeltaIntegral, 1);
        //积分限幅
        if (Pid->DeltaIntegral > (Pid->MaxIntegral))
        {
            Pid->DeltaIntegral = (Pid->MaxIntegral);
        }
        else if (Pid->DeltaIntegral < (Pid->MinIntegral))
        {
            Pid->DeltaIntegral = (Pid->MinIntegral);
        }
        //pid->Out = pid->Kp * pid->Error + pid->Ki * pid->integral / 10 + pid->Kd * (pid->Error - pid->DError)*100;
        //P一直开
        rov_mult_f32(&Pid->Kp, &zero_2, &Kp, 1);
        rov_mult_f32(&Kp, &Pid->Error, &Kp, 1);
        //I在误差较小时开启	
        if ((fabsf(Pid->Error) < Pid->KiDomain) || !Pid->KiDomain)
        {
            rov_mult_f32(&Pid->Ki, &zero_3, &Ki, 1);
            rov_mult_f32(&Ki, &Pid->DeltaIntegral, &Ki, 1);
        }
        //D在误差较小时开启
        if ((fabs(Pid->Error) < Pid->KdDomain) || !Pid->KdDomain)
        {
            rov_sub_f32(&Pid->Error, &Pid->DError, &Pid->DDError, 1);
            rov_mult_f32(&Pid->Kd, &Pid->DDError, &Kd, 1);
        }
        //加和
        rov_add_f32(&Ki, &Kd, &Ki, 1);
        rov_add_f32(&Kp, &Ki, &Pid->Output, 1);
        Pid->DError = Pid->Error;
        //输出限幅
        if (Pid->Output > Pid->MaxOutput)
        {
            Pid->Output = (Pid->MaxOutput);
        }
        else if (Pid->Output < Pid->MinOutput)
        {
            Pid->Output = Pid->MinOutput;
        }
        break;
    }
    // return Pid->Output;
    return 1;
#endif
}

/**
 * @brief 通用姿态解算函数
 * @param  straight_num     前后
 * @param  rotate_num       旋转
 * @param  vertical_num     垂直
 * @param  horizental_mode_num 水平模式
 * @param  vertical_mode_num 垂直模式
 * @return AttitudeControl_t 姿态控制结构体
 * @note 可直接输出给推进器驱动，随时根据硬件调整
 */
AttitudeControl_t CommonThrusterControl(u16 straight_num, u16 rotate_num, u16 vertical_num, u8 horizental_mode_num, u8 vertical_mode_num)
{
    AttitudeControl_t ThrusterTemp;

    switch (horizental_mode_num)
    {
    case ROTATE_MODE: //转向模式
        u8 AFlag = (rotate_num > straight_num);
        u8 BFlag = ((rotate_num + straight_num) > 3000);
        u8 CFlag = (rotate_num > 1500);
        u8 DFlag = (straight_num > 1500);
        u8 SFlag = AFlag << 3 + BFlag << 2 + CFlag << 1 + DFlag;

        switch (SFlag)
        {
        case 0b0000:
        case 0b1111:
            ThrusterTemp.HorizontalThruster_RightFront = (vu32)(rotate_num);
            ThrusterTemp.HorizontalThruster_RightRear = (vu32)(rotate_num);
            ThrusterTemp.HorizontalThruster_LeftFront = (vu32)(1500 - rotate_num + straight_num);
            ThrusterTemp.HorizontalThruster_LeftRear = (vu32)(1500 - rotate_num + straight_num);
            break;
        case 0b0111:
        case 0b1000:
            ThrusterTemp.HorizontalThruster_RightFront = (vu32)(straight_num);
            ThrusterTemp.HorizontalThruster_RightRear = (vu32)(straight_num);
            ThrusterTemp.HorizontalThruster_LeftFront = (vu32)(1500 - rotate_num + straight_num);
            ThrusterTemp.HorizontalThruster_LeftRear = (vu32)(1500 - rotate_num + straight_num);
            break;
        case 0b0100:
        case 0b1010:
            ThrusterTemp.HorizontalThruster_RightFront = (vu32)(rotate_num + straight_num - 1500);
            ThrusterTemp.HorizontalThruster_RightRear = (vu32)(rotate_num + straight_num - 1500);
            ThrusterTemp.HorizontalThruster_LeftFront = (vu32)(straight_num);
            ThrusterTemp.HorizontalThruster_LeftRear = (vu32)(straight_num);
            break;
        case 0b0001:
        case 0b1110:
            ThrusterTemp.HorizontalThruster_RightFront = (vu32)(rotate_num + straight_num - 1500);
            ThrusterTemp.HorizontalThruster_RightRear = (vu32)(rotate_num + straight_num - 1500);
            ThrusterTemp.HorizontalThruster_LeftFront = (vu32)(3000 - rotate_num);
            ThrusterTemp.HorizontalThruster_LeftRear = (vu32)(3000 - rotate_num);
            break;
        }
        break;
    case SIDEPUSH_MODE: //侧推模式
        ThrusterTemp.HorizontalThruster_RightFront = (vu32)(rotate_num);
        ThrusterTemp.HorizontalThruster_RightRear = (vu32)(3000 - rotate_num);
        ThrusterTemp.HorizontalThruster_LeftFront = (vu32)(rotate_num);
        ThrusterTemp.HorizontalThruster_LeftRear = (vu32)(3000 - rotate_num);
        break;
    case HORIZENTAL_MIX_MODE:
        //水平推进器保持不动
        break;
    default:
        break;
    }

#if (NUMBER_OF_VERTICAL_THRUSTER == 2) && (NUMBER_OF_HORIZENTAL_THRUSTER == 4) /* 六轴 */
    ThrusterTemp.VerticalThruster_RightFront = (vu32)(vertical_num);
    ThrusterTemp.VerticalThruster_RightRear = (vu32)(vertical_num);
    ThrusterTemp.VerticalThruster_LeftFront = (vu32)(vertical_num);
    ThrusterTemp.VerticalThruster_LeftRear = (vu32)(vertical_num);
#endif
#if (NUMBER_OF_VERTICAL_THRUSTER == 4) && (NUMBER_OF_HORIZENTAL_THRUSTER == 4) /* 八轴 */
    switch (vertical_mode_num)
    {
    case UPDOWN_MODE: //垂直模式
        ThrusterTemp.VerticalThruster_RightFront = (vu32)(vertical_num);
        ThrusterTemp.VerticalThruster_RightRear = (vu32)(vertical_num);
        ThrusterTemp.VerticalThruster_LeftFront = (vu32)(vertical_num);
        ThrusterTemp.VerticalThruster_LeftRear = (vu32)(vertical_num);
        break;
    case PITCH_MODE: //俯仰模式
        ThrusterTemp.VerticalThruster_RightFront = (vu32)(vertical_num);
        ThrusterTemp.VerticalThruster_RightRear = (vu32)(3000 - vertical_num);
        ThrusterTemp.VerticalThruster_LeftFront = (vu32)(vertical_num);
        ThrusterTemp.VerticalThruster_LeftRear = (vu32)(3000 - vertical_num);
        break;
    case ROLL_MODE: //横滚模式
        ThrusterTemp.VerticalThruster_RightFront = (vu32)(vertical_num);
        ThrusterTemp.VerticalThruster_RightRear = (vu32)(vertical_num);
        ThrusterTemp.VerticalThruster_LeftFront = (vu32)(3000 - vertical_num);
        ThrusterTemp.VerticalThruster_LeftRear = (vu32)(3000 - vertical_num);
        break;
    case VERTICAL_MIX_MODE: //DEBUG:自由翻滚模式测试
        u8 AFlag = (rotate_num > straight_num);
        u8 BFlag = ((rotate_num + straight_num) > 3000);
        u8 CFlag = (rotate_num > 1500);
        u8 DFlag = (straight_num > 1500);
        u8 SFlag = AFlag << 3 + BFlag << 2 + CFlag << 1 + DFlag;

        switch (SFlag)
        {
        case 0b0000:
        case 0b1111:
            ThrusterTemp.VerticalThruster_RightFront = (vu32)(rotate_num);
            ThrusterTemp.VerticalThruster_RightRear = (vu32)(rotate_num);
            ThrusterTemp.VerticalThruster_LeftFront = (vu32)(1500 - rotate_num + straight_num);
            ThrusterTemp.VerticalThruster_LeftRear = (vu32)(1500 - rotate_num + straight_num);
            break;
        case 0b0111:
        case 0b1000:
            ThrusterTemp.VerticalThruster_RightFront = (vu32)(straight_num);
            ThrusterTemp.VerticalThruster_RightRear = (vu32)(straight_num);
            ThrusterTemp.VerticalThruster_LeftFront = (vu32)(1500 - rotate_num + straight_num);
            ThrusterTemp.VerticalThruster_LeftRear = (vu32)(1500 - rotate_num + straight_num);
            break;
        case 0b0100:
        case 0b1010:
            ThrusterTemp.VerticalThruster_RightFront = (vu32)(rotate_num + straight_num - 1500);
            ThrusterTemp.VerticalThruster_RightRear = (vu32)(rotate_num + straight_num - 1500);
            ThrusterTemp.VerticalThruster_LeftFront = (vu32)(straight_num);
            ThrusterTemp.VerticalThruster_LeftRear = (vu32)(straight_num);
            break;
        case 0b0001:
        case 0b1110:
            ThrusterTemp.VerticalThruster_RightFront = (vu32)(rotate_num + straight_num - 1500);
            ThrusterTemp.VerticalThruster_RightRear = (vu32)(rotate_num + straight_num - 1500);
            ThrusterTemp.VerticalThruster_LeftFront = (vu32)(3000 - rotate_num);
            ThrusterTemp.VerticalThruster_LeftRear = (vu32)(3000 - rotate_num);
            break;
        }
        break;
    default:
        break;
    }
#endif
    return ThrusterTemp;
}


