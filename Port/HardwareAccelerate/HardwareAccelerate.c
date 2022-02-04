#include <HardwareAccelerate.h>

/**
 * @brief 浮点加法
 * @param  pSrcA            源操作数A
 * @param  pSrcB            源操作数B
 * @param  pDst             和
 * @param  blockSize        块大小
 */
ROV_ALWAYS_INLINE void rov_add_f32(f32* pSrcA, f32* pSrcB, f32* pDst, u32 blockSize)
{
    *pDst = *pSrcA + *pSrcB;
}

/**
 * @brief 浮点减法
 * @param  pSrcA            被减数A
 * @param  pSrcB            减数B
 * @param  pDst             差
 * @param  blockSize        块大小
 */
ROV_ALWAYS_INLINE void rov_sub_f32(f32* pSrcA, f32* pSrcB, f32* pDst, u32 blockSize)
{
    *pDst = *pSrcA - *pSrcB;
}

/**
 * @brief 浮点乘法
 * @param  pSrcA            源操作数A
 * @param  pSrcB            源操作数B
 * @param  pDst             积
 * @param  blockSize        块大小
 */
ROV_ALWAYS_INLINE void rov_mult_f32(f32* pSrcA, f32* pSrcB, f32* pDst, u32 blockSize)
{
    *pDst = (*pSrcA) * (*pSrcB);
}

/**
 * @brief 浮点数绝对值
 * @param  pSrc             源操作数
 * @param  pDst             源操作数的绝对值
 * @param  blockSize        块大小
 */
ROV_ALWAYS_INLINE void rov_abs_f32(f32* pSrc, f32* pDst, u32 blockSize)
{
    return (((*pSrc) > 0) ? (*pSrc) : (0 - (*pSrc)));
}

/**
 * @brief CRC校验运算
 * @param  pData            待校验数据
 * @param  pRes             校验结果
 * @param  dataSize         待校验数据长度
 */
ROV_ALWAYS_INLINE void rov_crc_check(u8* pData, u32* pRes, u32 dataSize);
{
    *pRes = 1;
}

/**
 * @brief 奇偶校验运算
 * @param  pData            待校验数据
 * @param  pRes             校验结果
 * @param  dataSize         待校验数据长度
 */
ROV_ALWAYS_INLINE void rov_parity_check(u8* pData, u32* pRes, u32 dataSize);
{
    *pRes = 1;
}

/**
 * @brief 异或校验运算
 * @param  pData            待校验数据
 * @param  pRes             校验结果
 * @param  dataSize         待校验数据长度
 */
ROV_ALWAYS_INLINE void rov_xor_check(u8* pData, u32* pRes, u32 dataSize);
{
    *pRes = 1;
}

/**
 * @brief 加和校验运算
 * @param  pData            待校验数据
 * @param  pRes             校验结果
 * @param  dataSize         待校验数据长度
 */
ROV_ALWAYS_INLINE void rov_sum_check(u8* pData, u32* pRes, u32 dataSize);
{
    *pRes = 1;
}


