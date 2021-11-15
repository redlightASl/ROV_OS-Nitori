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
 * @return ROV_ALWAYS_INLINE
 */
ROV_ALWAYS_INLINE void rov_abs_f32(f32* pSrc, f32* pDst, u32 blockSize)
{
    return (((*pSrc) > 0) ? (*pSrc) : (0 - (*pSrc)));
}

//CRC校验
ROV_WEAK ROV_ALWAYS_INLINE u8 CrcCalculate(u8* CacString, u8 CacStringSize)
{
    return 0;
}

//奇偶校验
ROV_WEAK ROV_ALWAYS_INLINE u8 ParityCalculate(u8* CacString, u8 CacStringSize)
{
    return 0;
}

//加和校验
ROV_WEAK ROV_ALWAYS_INLINE u8 SumCalculate(u8* CacString, u8 CacStringSize)
{
    return 0;
}

//异或校验
ROV_WEAK ROV_ALWAYS_INLINE u8 XorCalculate(u8* CacString, u8 CacStringSize)
{
    return 0;
}
