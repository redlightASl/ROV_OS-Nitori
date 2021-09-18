#ifndef __ROV_SENSOR_H
#define __ROV_SENSOR_H
#include "Defines.h"
#include "Setup.h"

#ifdef __cplusplus
extern "C" {
#endif

    struct rov_SensorData
    {
        u8* name; //数据位对应的名字
        u8 count; //数据位位置
        u8 hex_num; //所收取十六进制位的内容

        struct rov_SensorData* previous;
        struct rov_SensorData* next;

    };
    /* 传感器数据类型管理类 */
    typedef struct rov_SensorData* rov_SensorData_t;

    struct rov_Sensor
    {
        rov_SensorData_t data_head; //传感器数据头指针
        u8* huart; //传感器接收串口硬件设备
        ROV_STABLE_MEMORY_SPACE u8* buffer; //传感器接收缓存区指针
        u8 buffer_length; //传感器接收缓存区大小

        void (*ROV_Sensor_Init)(void); //传感器初始化
        void (*ROV_Sensor_Receive)(struct rov_Sensor* sensor); //传感器数据接收

        struct rov_Sensor* previous;
        struct rov_Sensor* next;

    };
    /* 传感器类 */
    typedef struct rov_Sensor* rov_Sensor_t;

    struct rov_SensorInformation
    {
        rov_Sensor_t sensor_head; //传感器头指针
        u8* huart; //上传串口硬件设备
        ROV_STABLE_MEMORY_SPACE u8* buffer; //上传缓存区指针
        u8 buffer_length; //上传缓存区大小

        void (*ROV_SendUp_Init)(void); //上传串口与传感器初始化
        void (*ROV_SendUp)(struct rov_SensorInformation* SendUpUart); //上传数据

        struct rov_SensorInformation* previous;
        struct rov_SensorInformation* next;
    };
    /* 传感器上传类 */
    typedef struct rov_SensorInformation* rov_SensorInformation_t;


#ifdef __cplusplus
}
#endif

#endif