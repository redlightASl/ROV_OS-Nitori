#include "Algorithm.h"

static u8 XorCaculate(u8* CacString, u8 CacStringSize);
static u8 IdTest(u8* String, u8 Format, u8 SendUpLength, u8 SendDownLength);




/**
 * @brief 异或运算位检查(奇偶校验)
 * @param  String           待校验的数据
 * @param  Format           上传格式为1；下传格式为0
 * @param  SendUpLength     上传数据长度
 * @param  SendDownLength   下传指令长度
 * @return u8 正确为1；错误为0，如果不开启奇偶校验默认为1
 */
static u8 IdTest(u8* String, u8 Format, u8 SendUpLength, u8 SendDownLength)
{
#ifdef DataIdentify
    if (Format) //上传数据格式
    {
        if (*(String + SendUpLength) == XorCaculate(String, SendUpLength))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else //下传指令格式
    {
        if (*(String + SendDownLength) == XorCaculate(String, SendDownLength))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    return 1;
#else
    return 1; //不开启奇偶校验时默认成功
#endif
}

/**
 * @brief 异或运算
 * @param  CacString        待校验数据
 * @param  CacStringSize    待校验数据长度
 * @return u8 异或运算结果，如果不开启数据校验则默认返回0
 */
static u8 XorCaculate(u8* CacString, u8 CacStringSize)
{
#ifdef DataIdentify
    u8 CacResult = CacString[0];
    for (u8 i = 0; i < CacStringSize; ++i)
    {
        CacResult ^= CacString[i];
    }
    return CacResult;
#else
    return 0;
#endif
}

