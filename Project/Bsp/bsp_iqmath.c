#include "stdint.h"
#include "bsp_iqmath.h"

//定点数转换到浮点数
float _IQ24toF(long A)
{
    return ((float)A/(1L<<24));
}

float _IQ16toF(long A)
{
    return ((float)A/(1L<<16));
}

//定点数乘法
long __IQmpy(long A, long B, int Q)
{ 
    return ((uint64_t)A*B >> Q);
}

//定点数乘Int32
long _IQ16mpyI32int(long A, long B)
{
    return ((uint64_t)A*B);
}

//
long _IQ16int(long A)
{
    return ((uint64_t)A>>16);
}