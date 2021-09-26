#include "Algorithm.h"

static u8 XorCaculate(u8* CacString, u8 CacStringSize);
static u32 SumCaculate(u8* CacString, u32 CacStringSize);
static u8 IdTest(u8* String, u8 Format, u8 SendUpLength, u8 SendDownLength);



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



// #include "stdio.h"
// #include "stdlib.h"
// #include "math.h"

// double frand()
// {
//     return 2 * ((rand() / (double)RAND_MAX) - 0.5);  //随机噪声
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

#else
    return 1; //不开启校验时默认成功
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
    case MIX_MODE: //TODO:自由翻滚模式
    
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
    case MIX_MODE: //TODO:自由翻滚模式

    break;
    default:
        break;
}
#endif

    return ThrusterTemp;
}

/**
 * @brief 异或运算位检查(奇偶校验)
 * @param  String           待校验的数据
 * @param  Format           上传格式为1；下传格式为0
 * @param  SendUpLength     上传数据长度
 * @param  SendDownLength   下传指令长度
 * @return u8 正确为1；错误为0，如果不开启奇偶校验默认为1
 */
static u8 IdTest(u8* String, u8 Format, u8 SendUpLength, u8 SendDownLength)
{
#ifdef DATA_CHECK
    if (Format) //上传数据格式
    {
        if (*(String + SendUpLength) == XorCaculate(String, SendUpLength))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else //下传指令格式
    {
        if (*(String + SendDownLength) == XorCaculate(String, SendDownLength))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    return 1;
#else
    return 1; //不开启奇偶校验时默认成功
#endif
}

/**
 * @brief 逐位异或运算
 * @param  CacString        待校验数据
 * @param  CacStringSize    待校验数据长度
 * @return u8 异或运算结果，如果不开启数据校验则默认返回0
 */
static u8 XorCaculate(u8* CacString, u8 CacStringSize)
{
#ifdef DATA_CHECK
    u8 CacResult = CacString[0];
    for (u8 i = 0; i < CacStringSize; ++i)
    {
        CacResult ^= CacString[i];
    }
    return CacResult;
#else
    return 0;
#endif
}

/**
 * @brief 逐位加和运算
 * @param  CacString        待校验数据
 * @param  CacStringSize    待校验数据长度
 * @return u32 加和运算结果，如果不开启数据校验则默认返回0
 */
static u32 SumCaculate(u8* CacString, u32 CacStringSize)
{
#ifdef DATA_CHECK
    u32 CacResult = CacString[0];
    for (u8 i = 0; i < CacStringSize; i++)
    {
        CacResult += CacString[i];
    }
    return CacResult;
#else
    return 0;
#endif   
}