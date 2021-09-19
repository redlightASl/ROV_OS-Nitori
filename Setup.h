#ifndef __ROV_SETUP_H
#define __ROV_SETUP_H

/* 选择运行仓位 */
#define CONTROL_CARBIN //控制仓
//#define MOVE_CARBIN //PWM仓、电源仓

/* 数据校验开关 */
#define ROV_MESSAGE_DATA_CHECK //上传数据校验
#define ROV_COMMAND_DATA_CHECK //下传指令校验

/* PID模式 */
#define PID_FIXED //恒定参数
#define PID_ADJUSTABLE //使用上位机下传的参数

/* PID参数 */
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

/* 推进器数量 */
#define NUMBER_OF_VERTICAL_THRUSTER 2
#define NUMBER_OF_HORIZENTAL_THRUSTER 4

/* 传感器数据最大长度 */
#define MAX_LENGTH_OF_SENSOR_DATA 8







#endif
