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
#include <Port.h>
#include <Defines.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef PID_FIXED /* 恒定PID参数 */
#define PID_DEEP_Kp 200 //定深比例系数
#define PID_DEEP_Ki 2 //定深积分系数
#define PID_DEEP_Kd 300 //定深微分系数
#define PID_ORBIT_Kp 200 //定向比例系数
#define PID_ORBIT_Ki 2 //定向积分系数
#define PID_ORBIT_Kd 300 //定向微分系数
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

	struct Algorithm_PID
	{
		u8 mode; //PID模式设置

		f32 Ref; //参考值
		f32 FeedBack; //反馈
		f32 Error; //误差
		f32 DError; //误差与上次差
		f32 DDError; //误差上次与上上次差
		f32 PreError; //上次误差
		f32 PreDError; //上次误差差

		f32 DeltaIntegral; //误差的积分

		f32 KiDomain; //Ki作用域，设为0关闭此功能
		f32 KdDomain; //Kd作用域，设为0关闭此功能

		f32 Kp; //PID参数p
		f32 Ki; //PID参数i
		f32 Kd; //PID参数d

		u32 MaxOutput; //输出限幅
		u32 MinOutput;

		f32 MaxIntegral; //积分限幅
		f32 MinIntegral;

		f32 Output; //输出值
	};
	typedef struct Algorithm_PID* Algorithm_PID_t; /* PID控制ADT */

	struct Algorithm_Kalman
	{
		f32 MeasureNoise_R; //测量噪声
		f32 ProcessNoise_Q; //过程噪声

		f32 X_Output;
		f32 X_last; //上一轮结果
		f32 X_mid; //本次结果

		f32 Measure; //测量值

		f32 P_mid;
		f32 P_now;

		f32 sumerror_kalman;
		f32 sumerror_measure;
	};
	typedef struct Algorithm_Kalman* Algorithm_Kalman_t; /* 一维卡尔曼滤波控制ADT */

	u8 SumCheck(u8* CacString, u8 CalLength, u8 CacBit);
	u8 CrcCheck(u8* CacString, u8 CalLength, u8 CacBit);
	u8 ParityCheck(u8* CacString, u8 CalLength, u8 CacBit);
	u8 XorCheck(u8* CacString, u8 CalLength, u8 CacBit);

	u16 KalmanFilter(u16 original_value);
	u16 PositionalPID(u16 target_value, u16 actual_value);
	u16 IncrementalPID(u16 target_value, u16 actual_value);

	AttitudeControl_t CommonThrusterControl(u16 straight_num, u16 rotate_num, u16 vertical_num, u8 horizental_mode_num, u8 vertical_mode_num);

	

	void InitPID(u8 mode, Algorithm_PID_t Pid);
	u32 PIDCal(Algorithm_PID_t Pid, f32 feedback);
	
#ifdef __cplusplus
}
#endif

#endif
