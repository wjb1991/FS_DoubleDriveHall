/*
命名法则
u   无符号型
无  有符号型

f   浮点型
d   双精度浮点型

l   长整形  64位
n   整形    32位
s   短整形  16位
c   字符型  8位
b   布尔    1位  
v   无效(void)

    
a   数组
p   指针
*/


#ifndef __BSP_H__
#define __BSP_H__


#include "bsp_cpu.h"

#include "bsp_iqmath.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_svpwm_2shunt.h"
#include "bsp_hall.h"
#include "bsp_dac.h"
#include "bsp_systick.h"

#include "bsp_bitfilter.h"
#include "bsp_key.h"
#include "bsp_cpu_flash.h"

#define   Bsp_Printf    printf
#define   TRAG_DBG      printf
#endif
