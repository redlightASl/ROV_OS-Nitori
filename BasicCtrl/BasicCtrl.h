#ifndef __ROV_BASIC_CTRL_H
#define __ROV_BASIC_CTRL_H
#include "Defines.h"
#include "Setup.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* 推进器类 */
    struct rov_Thruster
    {
        vu32 HorizontalThruster[NUMBER_OF_HORIZENTAL_THRUSTER]; //水平方向推进器
        vu32 VerticalThruster[NUMBER_OF_VERTICAL_THRUSTER]; //垂直方向推进器

        u8(*rov_Thruster_Init)(rov_PwmDevice_t pwm_device); //推进器初始化
        u8(*rov_Thruster_MoveControl)(u16 straight_num, u16 rotate_num, u16 vertical_num, u8 mode_num); //根据控制数据解算出PWM输出值
    };
    typedef struct rov_Thruster* rov_Thruster_t;

    /* 算法控制类 */
    struct rov_AutoControl
    {

    };
    typedef struct rov_AutoControl* rov_AutoControl_t;



#ifdef __cplusplus
}
#endif

#endif
