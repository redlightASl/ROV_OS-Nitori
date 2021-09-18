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
#include "Defines.h"
#include "Setup.h"

#ifdef __cplusplus
extern "C" {
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

//TODO:map(origin,origin_bottom,origin_top,tatget_bottom,target_top)函数



u8 ParityCheck(u8* CacString, u8 CacStringSize);
u16 PositionalPID(u16 target_value,u16 actual_value);
u16 IncrementalPID(u16 target_value,u16 actual_value);
u16 KalmanFilter(u16 original_value);

#ifdef __cplusplus
}
#endif

#endif
