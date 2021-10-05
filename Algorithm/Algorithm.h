/**
 * @file Algorithm.h
 * @brief 提供常用的软件算法 
 * @author RedlightASl (dddbbbdd@foxmail.com)
 * @version 1.0
 * @date 2021-09-13
 * 
 * @copyright Copyright (c) 2021  RedlightASl
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-09-13 <td>1.0     <td>wangh     <td>Content
 * </table>
 */
#ifndef __ROV_ALGORITHM_H
#define __ROV_ALGORITHM_H
#include "Port.h"
#include <Defines.h>
#include "Setup.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef PID_FIXED
/* 恒定PID参数 */
//定深比例系数
#define PID_DEEP_Kp 200
//定深积分系数
#define PID_DEEP_Ki 2
//定深微分系数
#define PID_DEEP_Kd 300
//定向比例系数
#define PID_ORBIT_Kp 200
//定向积分系数
#define PID_ORBIT_Ki 2
//定向微分系数
#define PID_ORBIT_Kd 300
#else
#define PID_ADJUSTABLE //使用上位机下传的参数
#endif

#define ALGORITHM_MAX(x, y) ({\
							typeof(x) _x = (x);\
							typeof(y) _y = (y);\
							(void)(&_x == &_y);\
							_x > _y ? _x : _y;\
							})

#define ALGORITHM_MIN(x, y) ({\
							typeof(x) _x = (x);\
							typeof(y) _y = (y);\
							(void)(&_x == &_y);\
							_x < _y ? _x : _y;\
							})

#define ALGORITHM_RANGE(x, a, b) (BASICCTRL_MIN((BASICCTRL_MAX(x, a)), b))

/**
 * @brief 检查PWM控制数据是否正确
 * @param  pwm_value        待检查的PWM控制数据
 * @param  top              上限
 * @param  bottom           下限
 * @return u8 1正确；0错误
 */
#define CheckPwmValue(pwm_value, bottom_value, top_value) (((pwm_value <= top_value)\
                                                        &&(pwm_value >= bottom_value))?\
                                                        (1):(0))

/**
 * @brief 将一个数据从某个区间映射到另一个目标区间
 * @param origin_data 原始数据
 * @param origin_bottom 原始数据下限
 * @param origin_top 原始数据上限
 * @param tatget_bottom 目标数据下限
 * @param target_top 目标数据上限
 * @return 目标区间上的数据值
 */
#define MAP(origin_data,origin_bottom,origin_top,tatget_bottom,target_top) \
			(((origin_data) - (origin_bottom)) * \
			((target_top) - (tatget_bottom)) / \
			((origin_top) - (origin_bottom))) + \
			(tatget_bottom)

struct AttitudeControl
{
	vu32 HorizontalThruster_RightFront;
	vu32 HorizontalThruster_RightRear;
	vu32 HorizontalThruster_LeftFront;
	vu32 HorizontalThruster_LeftRear;
	vu32 VerticalThruster_RightFront;
	vu32 VerticalThruster_RightRear;
	vu32 VerticalThruster_LeftFront;
	vu32 VerticalThruster_LeftRear;
};
typedef struct AttitudeControl AttitudeControl_t;

u8 SumCheck(u8* CacString, u8 CalLength, u8 CacBit);
u8 CrcCheck(u8* CacString, u8 CalLength, u8 CacBit);
u8 ParityCheck(u8* CacString, u8 CalLength, u8 CacBit);
u8 XorCheck(u8* CacString, u8 CalLength, u8 CacBit);

u16 KalmanFilter(u16 original_value);
u16 PositionalPID(u16 target_value,u16 actual_value);
u16 IncrementalPID(u16 target_value,u16 actual_value);

AttitudeControl_t CommonThrusterControl(u16 straight_num, u16 rotate_num, u16 vertical_num, u8 horizental_mode_num, u8 vertical_mode_num);

extern u32 CrcCalculate(u8* CacString, u32 CacStringSize);

#ifdef __cplusplus
}
#endif

#endif
