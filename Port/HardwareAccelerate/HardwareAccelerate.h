/**
 * @file HardwareAccelerate.h
 * @brief 在该文件及对应.c文件中编写硬件加速相关代码
 * @author RedlightASl (dddbbbdd@foxmail.com)
 * @version 1.0
 * @date 2021-09-30
 *
 * @copyright Copyright (c) 2021  RedlightASl
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-09-30 <td>1.0     <td>wangh     <td>Content
 * </table>
 */
#ifndef __ROV_HARDWRE_ACC_H
#define __ROV_HARDWRE_ACC_H
#include <Defines.h>

#ifdef NITORI_ON_FPGA_THIRD_PARTY
 /* 这里加入第三方FPGA加速库的头文件 */
#else
 /* 自定义硬件加速库 */
#ifdef HARDWARE_ACCELERATE_PID
//TODO:PID硬件加速库
#endif
#ifdef HARDWARE_ACCELERATE_KALMAN
//TODO:卡尔曼滤波硬件加速库
#endif
#ifdef HARDWARE_ACCELERATE_CRC
ROV_ALWAYS_INLINE void rov_crc_check(u8* pData, u32* pRes, u32 dataSize);
#endif
#ifdef HARDWARE_ACCELERATE_PARITY
ROV_ALWAYS_INLINE void rov_parity_check(u8* pData, u32* pRes, u32 dataSize);
#endif
#ifdef HARDWARE_ACCELERATE_XOR
ROV_ALWAYS_INLINE void rov_xor_check(u8* pData, u32* pRes, u32 dataSize);
#endif
#ifdef HARDWARE_ACCELERATE_SUM
ROV_ALWAYS_INLINE void rov_sum_check(u8* pData, u32* pRes, u32 dataSize);
#endif
#ifdef HARDWARE_ACCELERATE_SENSOR
//TODO:传感器硬件加速库
#endif
ROV_ALWAYS_INLINE void rov_add_f32(f32* pSrcA, f32* pSrcB, f32* pDst, u32 blockSize);
ROV_ALWAYS_INLINE void rov_sub_f32(f32* pSrcA, f32* pSrcB, f32* pDst, u32 blockSize);
ROV_ALWAYS_INLINE void rov_mult_f32(f32* pSrcA, f32* pSrcB, f32* pDst, u32 blockSize);
ROV_ALWAYS_INLINE void rov_abs_f32(f32* pSrc, f32* pDst, u32 blockSize);
#endif


#endif
