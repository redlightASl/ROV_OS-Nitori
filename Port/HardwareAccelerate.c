#include <HardwareAccelerate.h>

//浮点加法
ROV_ALWAYS_INLINE void rov_add_f32(f32* pSrcA, f32* pSrcB, f32* pDst, u32 blockSize)
{
    *pDst = *pSrcA + *pSrcB;
}

//浮点减法   
ROV_ALWAYS_INLINE void rov_sub_f32(f32* pSrcA, f32* pSrcB, f32* pDst, u32 blockSize)
{
    *pDst = *pSrcA - *pSrcB;
}

//浮点乘法
ROV_ALWAYS_INLINE void rov_mult_f32(f32* pSrcA, f32* pSrcB, f32* pDst, u32 blockSize)
{
    *pDst = (*pSrcA) * (*pSrcB);
}

//浮点绝对值
ROV_ALWAYS_INLINE void rov_abs_f32(f32* pSrc, f32* pDst, u32 blockSize)
{

}