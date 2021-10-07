#include "Algorithm.h"

static u8 SumCalculate(u8* CacString, u8 CacStringSize);
static u8 CrcCalculate(u8* CacString, u8 CacStringSize);
static u8 ParityCalculate(u8* CacString, u8 CacStringSize);
static u8 XorCalculate(u8* CacString, u8 CacStringSize);

/**
 * @brief 加和校验
 * @param  CacString        待校验字符串
 * @param  CalLength        待校验字符串长度
 * @param  CacBit           校验位
 * @return u8 成功返回1，失败返回0
 */
u8 SumCheck(u8* CacString, u8 CalLength, u8 CacBit)
{
#ifdef DATA_CHECK
    if (CacString[CacBit] == SumCalculate(CacString, CalLength))
    {
        return 1;
    }
    else
    {
        return 0;
    }
#else
    return 1; //不开启奇偶校验时默认成功
#endif
}

u8 CrcCheck(u8* CacString, u8 CalLength, u8 CacBit);
u8 ParityCheck(u8* CacString, u8 CalLength, u8 CacBit);


/**
 * @brief 异或校验
 * @param  CacString        待校验字符串
 * @param  CalLength        待校验字符串长度
 * @param  CacBit           校验位
 * @return u8 成功返回1，失败返回0
 */
u8 XorCheck(u8* CacString, u8 CalLength, u8 CacBit)
{
#ifdef DATA_CHECK
    if (CacString[CacBit] == XorCalculate(CacString, CalLength))
    {
        return 1;
    }
    else
    {
        return 0;
    }
#else
    return 1; //不开启奇偶校验时默认成功
#endif
}

//TODO：用于定深的卡尔曼滤波
//NOTE：定深算法
//使用PID，根据设定深度输出一个推进器控制值
//
// u16 KalmanFilter(u16 original_value, u16 measure_value)
// {
//     for (int i = 0;i < 20;i++)
//     {
//         x_mid = x_last;    //x_last=x(k-1|k-1),x_mid=x(k|k-1)
//         p_mid = p_last + Q;  //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声

//         kg = p_mid / (p_mid + R); //kg为kalman filter，R为噪声
//         x_now = x_mid + kg * (measure_value - x_mid); //估计出的最优值
//         p_now = (1 - kg) * p_mid;//最优值对应的covariance

//         p_last = p_now;  //更新covariance值
//         x_last = x_now;  //更新系统状态值
//     }

//     return x_now; //返回kalman估计值
// }





// int main()
// {
//     float x_last = 0;
//     float p_last = 0.02;
//     float Q = 0.018;
//     float R = 0.542;
//     float kg;
//     float x_mid;
//     float x_now;
//     float p_mid;
//     float p_now;
//     float z_real = 0.56;//0.56
//     float z_measure;
//     float sumerror_kalman = 0;
//     float sumerror_measure = 0;
//     int i;
//     x_last = z_real + frand() * 0.03;
//     x_mid = x_last;

//     for (i = 0;i < 20;i++)
//     {
//         x_mid = x_last;    //x_last=x(k-1|k-1),x_mid=x(k|k-1)
//         p_mid = p_last + Q;  //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
//         kg = p_mid / (p_mid + R); //kg为kalman filter，R为噪声
//         z_measure = z_real + frand() * 0.03;//测量值
//         x_now = x_mid + kg * (z_measure - x_mid);//估计出的最优值
//         p_now = (1 - kg) * p_mid;//最优值对应的covariance

//         printf("Real     position: %6.3f \n", z_real);  //显示真值
//         printf("Mesaured position: %6.3f [diff:%.3f]\n", z_measure, fabs(z_real - z_measure));  //显示测量值以及真值与测量值之间的误差
//         printf("Kalman   position: %6.3f [diff:%.3f]\n", x_now, fabs(z_real - x_now));  //显示kalman估计值以及真值和卡尔曼估计值的误差

//         sumerror_kalman += fabs(z_real - x_now);  //kalman估计值的累积误差
//         sumerror_measure += fabs(z_real - z_measure);  //真值与测量值的累积误差
//         p_last = p_now;  //更新covariance值
//         x_last = x_now;  //更新系统状态值
//     }

//     printf("总体测量误差      : %f\n", sumerror_measure);  //输出测量累积误差
//     printf("总体卡尔曼滤波误差: %f\n", sumerror_kalman);   //输出kalman累积误差

//     printf("卡尔曼误差所占比例: %d%% \n", 100 - (int)((sumerror_kalman / sumerror_measure) * 100));

//     return 0;
// }



u16 PositionalPID(u16 target_value, u16 actual_value)
{
#ifdef HARDWARE_ACCELERATE_PID

#else
    return 1; //不开启校验时默认成功
#endif
}

u16 IncrementalPID(u16 target_value, u16 actual_value)
{
#ifdef HARDWARE_ACCELERATE_PID
    //当前误差
    static float Ek;
    //前一次误差
    static float Ek1;
    //累计积分位置
    static float LocSum;
    //数据清空标志位
    static u8 PIDData = 0;

    if (ModeType == 4) //定深模式
    {
        PIDData = 0;
        u16 PIDLoc;
        Ek = (float)(SetValue - ActualValue);
        LocSum += Ek;
        PIDLoc =
            (u16)(1500
                + BASICCTRL_RANGE(
                    (int16_t)(PID_D_Kp * Ek + (PID_D_Ki * LocSum) + PID_D_Kd * (Ek1 - Ek)),
                    -1000, 1000));
        return PIDLoc;
    }
    else if (ModeType == 2) //定向模式
    {
        PIDData = 0;
        u16 PIDLoc;
        Ek = (float)(SetValue - ActualValue);
        LocSum += Ek;
        PIDLoc =
            (u16)(1500
                + BASICCTRL_RANGE(
                    (int16_t)(PID_O_Kp * Ek + (PID_O_Ki * LocSum) + PID_O_Kd * (Ek1 - Ek)),
                    -1000, 1000));
        return PIDLoc;
    }
    else
    {
        if (!PIDData)
        {
            PIDData = 1;
            Ek = 0;
            LocSum = 0;
        }
        return 0;
    }
#else
    return 1; //不开启校验时默认成功
#endif
}

/**
 * @brief 初始化PID,填充结构体参数
 * @param  mode             PID模式
 * @param  Pid              PID结构体
 */
void InitPID(u8 mode, Algorithm_PID_t Pid)
{
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
 * @return u32 
 */
u32 PIDCal(Algorithm_PID_t Pid, f32 feedback)
{
    static f32 zero_2 = 0.01f;
    static f32 zero_3 = 0.001f;
    f32 Kp = 0.0f;
    f32 Ki = 0.0f;
    f32 Kd = 0.0f;

    switch (Pid->mode)
    {
    //TODO:整型PID
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
    //TODO：PID计算Debug
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
        //D在误差较小时开启闭	
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
    case MIX_MODE:
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
    case MIX_MODE: //TODO:自由翻滚模式测试
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

/**
 * @brief 逐位加和运算
 * @param  CacString        待校验数据
 * @param  CacStringSize    待校验数据长度
 * @return u8 加和运算结果，如果不开启数据校验则默认返回1
 */
static u8 SumCalculate(u8* CacString, u8 CacStringSize)
{
#ifdef DATA_CHECK
    u32 CacResult = CacString[0];
    for (u8 i = 0; i < CacStringSize; i++)
    {
        CacResult += CacString[i];
    }
    return CacResult;
#else
    return 1;
#endif
}

static u8 CrcCalculate(u8* CacString, u8 CacStringSize);
static u8 ParityCalculate(u8* CacString, u8 CacStringSize);

/**
 * @brief 逐位异或运算
 * @param  CacString        待校验数据
 * @param  CacStringSize    待校验数据长度
 * @return u8 异或运算结果，如果不开启数据校验则默认返回1
 */
static u8 XorCalculate(u8* CacString, u8 CacStringSize)
{
#ifdef DATA_CHECK
    u8 CacResult = CacString[0];
    for (u8 i = 0; i < CacStringSize; ++i)
    {
        CacResult ^= CacString[i];
    }
    return CacResult;
#else
    return 1;
#endif
}
