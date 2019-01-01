#include "mc_estHallAngle.h"

// 0 2 4 3 6 1 5    sec
// 0 1 2 3 4 5 6    hall 

// CCW 1-2-3-4-5-6-1  --5--1--3--2--6--4    101--001--011--010--110--100
//                                          010--110--100--101--001--011    b3 hallB  b2 hallA  b2 hallC                     

const uint8_t	ucaMC_HallSecTable[8] = {0,2,4,3,6,1,5,0 };
                                        
uint8_t  ucMC_HallValue = 0;
uint8_t  ucMC_HallValueLast = 0;                             //上一个霍尔值
uint8_t  ucMC_HallValueErrTick = 0;                          //霍尔错误计时
uint8_t  ucMC_HallSector = 0;

int16_t  usMC_HallEva_IntervalTime = 16872;                  //一个电角度的时间    

uint8_t  ucMC_HallEva_OneRoundFlag = 0;                      //一圈标记清0 电度角
uint8_t  ucMC_HallEva_EdgeTickDir = 0;                       //一圈计数 数组索引清0 电度角

uint8_t  ucMC_HallEva_EvaReadyFlag = 0;                      //事件就绪标志位清0  角度小于0度标记 再换向的时候防止溢出  
uint8_t  ucMC_HallEva_DirChangedFlag = 1;                    //置1是默认 表示电机方向不变
                
int8_t   cMC_HallEva_SectorDiff = 0;                         //两侧霍尔扇区相差的角度
uint8_t  ucMC_HallEva_Direction = 0;                         //电机方向 1 CCW  / 0 CW
uint8_t  ucMC_HallEva_DirectionLast = 0;                     //上一次的方向    

uint8_t  ucMC_HallEva_EdgeTick = 0;                          //电机稳定运行计数 连续6次大于46rpm 计数
uint16_t usMC_HallTick = 0;                                  //两次霍尔边沿的计时  
uint16_t usaMC_HallEva_InvTimeBuf[6] = {0};                  //存放最近6次霍尔跳变的间隔时间 6个usMC_HallTick 

uint8_t  ucMC_HallEva_LowSpeedorEdged = 0;

uint16_t usMC_HallEva_SectCenterAngle = 0;                   //扇区中心角
uint16_t usMC_HallEva_SecStartAngle = 0;                     //扇区启动角
uint16_t usMC_HallEva_AngleDownLim = 0;                      //扇区最小启动角
uint16_t usMC_HallEva_AngleUpLim = 0;                        //扇区最大启动角
uint16_t usMC_HallEva_AngleLimitforLowSpd = 0;               //扇区最大估算角范围 低速运行的状态下
uint16_t usMC_HallEva_AngleLimitforNormalSpd = 0;            //扇区最大估算角范围 普通运行的状态下

uint8_t  ucMC_HallEva_Over180Flag = 0;                       //超过180度标记

uint16_t usMC_AngleEva = 0;                                  //估算角度
int16_t  sMC_HallEva_DistanceLeading = 0;                    //估算角度偏差值
uint16_t usMC_HallEva_AngleLimit = 0;                        //当前扇区的最大估算范围

uint8_t  ucMC_HallSectorErr = 0;                             //霍尔扇区错误标记
uint8_t  ucMC_HallEva_LowSpeedFlag = 0;                      //低速标记
uint16_t usMC_HallEva_Interpolation = 0;                     //估算时的插入角
uint16_t usMC_HallEva_IntervalTimeLast = 0;                  //上一次电周期的时间

uint16_t usMC_AngleFb = 0;

int16_t sMC_HallEstSpeed_RPM = 0;

// 40 20 20 还可以
#define     DEF_ANGLE_LIMIT_SPEED_RPM               (44L)     //94.7RPM
#define     DEF_LOWSPEED_CHANGE_SPEED_RPM           (21L)     //71.1RPM
#define     DEF_HIGHSPEED_CHANGE_SPEED_RPM          (21L)     //71.1RPM

#define     DEF_ANGLE_LIMIT_SPEED_FREQUENCY         (60*16000/10/(DEF_ANGLE_LIMIT_SPEED_RPM))           //1013
#define     DEF_LOWSPEED_CHANGE_FREQUENCY           (60*16000/10/(DEF_LOWSPEED_CHANGE_SPEED_RPM))     //1350
#define     DEF_HIGHSPEED_CHANGE_FREQUENCY          (60*16000/10/(DEF_HIGHSPEED_CHANGE_SPEED_RPM))     //1350
       
//获取角度估算角度模块
void vMC_EST_HallAngle(void)
{
    /**
    电机停止or超低速检测
    */
    if ( usMC_HallTick < 2812 )    //2812*64us = 179968us   180ms
    {
        ++usMC_HallTick;
    }
    if ( usMC_HallTick >= 2812 )   // long time without edge of hall -->  reset    长时间没有霍尔边沿
    {
        usMC_HallEva_IntervalTime = 16872;
        ucMC_HallEva_DirChangedFlag = 0;
        ucMC_HallEva_OneRoundFlag = 0;
        ucMC_HallEva_EdgeTickDir = 0;
    }
    /**
    电机低速检测
    */
    if ( (usMC_HallEva_IntervalTime > DEF_ANGLE_LIMIT_SPEED_FREQUENCY)         // 电周期时间 > 1012 *64us = 12.954ms
        || ((5 * usMC_HallTick > DEF_ANGLE_LIMIT_SPEED_FREQUENCY)-1 ))             // (1/(6*15)) *60 / 12.954ms   = 51 rpm  
    {
        // long time?  adjust the limit angle   时间太长 转速太低 小于51rpm 调整限制角度
        usMC_HallEva_AngleLimit = usMC_HallEva_AngleLimitforLowSpd; 
    }

    
    /**
    
    */
    if ( (usMC_HallEva_IntervalTime <= DEF_HIGHSPEED_CHANGE_FREQUENCY) && (6 * usMC_HallTick <= DEF_HIGHSPEED_CHANGE_FREQUENCY) ) // 225*64us = 14.4ms  (1/(6*15)) *60 / 14.4ms   = 46 rpm   
    {
        //loc_8003244 时间小于 1350  转速大于 46rpm 
        if ( ucMC_HallEva_EdgeTick >= 6 )                       //连续6次大于46rpm
        {   // speed enough
            ucMC_HallEva_LowSpeedorEdged = 0; 
            ucMC_HallEva_LowSpeedFlag = 0;        //清除低速标记
        }
    } 
    else if ( (usMC_HallEva_IntervalTime >= (DEF_LOWSPEED_CHANGE_FREQUENCY)) && (6 * usMC_HallTick >= (DEF_LOWSPEED_CHANGE_FREQUENCY)) )
    {
        ucMC_HallEva_LowSpeedorEdged = 1;                       //设置低速标记
        ucMC_HallEva_EdgeTick = 0;                              //连续次数清0
    }

    
    /**
    定时估算角度 角度插值
    可能是每隔30°就换一次范围
    */
    if ( ucMC_HallEva_Direction )  
    {   
        //CCW   SECTOR IS increasing 1->2->3->4->5->6->1    
        //这里的CCW 逆时针 是指FOC的 iq 的旋转方向 并不一定是电机的旋转方向
        if ( (ucMC_HallSector == 6) 
            && (usMC_HallEva_AngleLimit != 0xffff))    
        {   // last sector for one round  判断是否是最后一个扇区
            //这里把角度分为两段 
            //0xffff - usMC_HallEva_AngleLimit :这一段表示没有过扇区边界 0 ~ -30 
            //usMC_HallEva_AngleLimit - 0      :这一段表示在本扇区       30 ~ 0
            
            if ( ucMC_HallEva_Over180Flag )    
            {
                //是最后一个扇区
                if ( usMC_HallEva_AngleLimit - usMC_HallEva_Interpolation <= usMC_AngleEva )
                {   // 估算角度 + 插入角度 >= 限制值    
                    //估算角度超过限制
                    usMC_AngleEva = usMC_HallEva_AngleLimit;
                }
                else
                {   //估算角度加上插入角度
                    usMC_AngleEva += usMC_HallEva_Interpolation;
                }
            }     
            else      
            {
                // 估算角度 + 插入角度 > 0xFFFF (这里用减法做判断 不会溢出)
                // 0xFFFF可能表示180
                if ( 0xFFFF - usMC_HallEva_Interpolation < usMC_AngleEva )
                    ucMC_HallEva_Over180Flag = 1;
                //估算角度加上插入角度
                usMC_AngleEva += usMC_HallEva_Interpolation;
            }
        }    
        else 
        {   // 普通扇区
            // 感觉这里和上面一样 
            // 就是把ucMC_HallEva_Over180Flag 换成了 ucMC_HallEva_EvaReadyFlag 切极性相反
            
            //这里把角度分为两段 
            //0xffff - usMC_HallEva_AngleLimit :这一段表示没有过扇区边界 0 ~ -30 
            //usMC_HallEva_AngleLimit - 0      :这一段表示在本扇区       30 ~ 0        
            if ( ucMC_HallEva_EvaReadyFlag )    
            {   
                // 估算角度 + 插入角度 > 0xFFFF (这里用减法做判断 不会溢出)
                if ( 0xFFFF - usMC_HallEva_Interpolation < usMC_AngleEva )
                {
                    ucMC_HallEva_EvaReadyFlag = 0;
                }
                // consider the acc 估算角度加上插入角度
                usMC_AngleEva += usMC_HallEva_Interpolation;
            }    
            else 
            {
                if ( usMC_HallEva_AngleLimit - usMC_HallEva_Interpolation <= usMC_AngleEva )    {
                    usMC_AngleEva = usMC_HallEva_AngleLimit;
                }    
                else    
                {
                    usMC_AngleEva += usMC_HallEva_Interpolation;
                }
            }
        }
    }  
    else 
    {   
        //CW  sector is decreasing 6->5->4->3->2->1->6
        //这里的CW 顺时针 是指FOC的 iq 的旋转方向 并不一定是电机的旋转方向
        //基本就和CCW一样 只不过加角度变成了减角度 符号有些变化
        if ( ucMC_HallSector == 1 
            && usMC_HallEva_AngleLimit )  
        {
            // last sector for one round
            if ( ucMC_HallEva_Over180Flag )   
            {
                //loc_80033C0
                if ( usMC_HallEva_AngleLimit + usMC_HallEva_Interpolation >= usMC_AngleEva )
                {
                    usMC_AngleEva = usMC_HallEva_AngleLimit;
                }
                else
                {
                    usMC_AngleEva -= usMC_HallEva_Interpolation;
                }
            }   
            else    
            {
                if ( usMC_AngleEva < usMC_HallEva_Interpolation ){
                    ucMC_HallEva_Over180Flag = 1;
                }
                usMC_AngleEva -= usMC_HallEva_Interpolation;
            }
        }  
        else
        { //// normal sector
            if ( ucMC_HallEva_EvaReadyFlag )	
            {
                if ( usMC_AngleEva < usMC_HallEva_Interpolation )
                {
                    ucMC_HallEva_EvaReadyFlag = 0;
                }
                usMC_AngleEva -= usMC_HallEva_Interpolation;
            }  
            else  
            {
                if ( usMC_HallEva_AngleLimit + usMC_HallEva_Interpolation >= usMC_AngleEva )  
                {
                    usMC_AngleEva = usMC_HallEva_AngleLimit;
                }  
                else  
                {
                    usMC_AngleEva -= usMC_HallEva_Interpolation;
                }
                //loc_80033EC
            }
        }
    }


    /**
    霍尔信号读取
    */
    ucMC_HallValue = ucBSP_ReadHall();                      //读取霍尔 


    if(ucMC_HallValue == 7 || ucMC_HallValue == 0)          //全0 全1 hall出错
    {
        if( ucMC_HallValueErrTick < 30 )                    //小于30次 16K/30 ~= 0.5k 200ms
        {
            ucMC_HallValueErrTick++;                        //
            ucMC_HallValue = ucMC_HallValueLast;            //霍尔值恢复上一次的值
        }
    }
    else
    {
        ucMC_HallValueErrTick = 0;
    }   
    
    /**
    霍尔换向处理
    */
    if( ucMC_HallValueLast != ucMC_HallValue )              //上一次的霍尔值不等于这次的霍尔值
    {
        
        ucMC_HallEva_EvaReadyFlag = 0;                      //事件就绪标志位清0        
        ucMC_HallEva_DirChangedFlag = 1;                    //置1是默认 表示电机方向不变
        
        /** 计算扇区差值 本次的扇区值 - 上次的扇区值 */
        cMC_HallEva_SectorDiff = ucaMC_HallSecTable[ucMC_HallValue] - ucMC_HallSector;//
        
        /** 电机方向检测  */
        if ( (cMC_HallEva_SectorDiff == 1) || cMC_HallEva_SectorDiff == -5 )    
        {
            //等于1或者等于-5   CCW 1->2->3->4->5->6->1  1-6=-5    2-1 =1
            ucMC_HallEva_Direction = 1;      
        }  
        else if ( cMC_HallEva_SectorDiff == -1 || cMC_HallEva_SectorDiff == 5 )
        {
            //等于-1或5		    CW  6->5->4->3->2->1->6  6-1=5     1-2=-1
            ucMC_HallEva_Direction = 0;
        }                
        else  
        {
            //跳扇区
        }
        
        /** 电机方向改变检测 上次的方向 != 这次的方向 */
        if ( ucMC_HallEva_DirectionLast != ucMC_HallEva_Direction )    
        {
            // reset 重置变量
            usMC_HallEva_IntervalTime = 16872;                   //一个电角度的时间    
            ucMC_HallEva_DirChangedFlag = 0;                    //清0表示方向变换

            ucMC_HallEva_OneRoundFlag = 0;                      //一圈标记清0 电度角
            ucMC_HallEva_EdgeTickDir = 0;                       //一圈计数 数组索引清0 电度角
        }    
        ucMC_HallEva_DirectionLast = ucMC_HallEva_Direction;    //最后的方向等于当前方向    
        
        /** 电机稳定运行计数 连续6次大于46rpm 计数 */
        if ( ucMC_HallEva_EdgeTick < 6 )
        {
            ++ucMC_HallEva_EdgeTick;
        }

        /** 记录距离上一次霍尔中断的时间 存到一个数组里 并清0*/
        usaMC_HallEva_InvTimeBuf[ucMC_HallEva_EdgeTickDir] = usMC_HallTick;
        usMC_HallTick = 0;
        
        if ( ucMC_HallEva_OneRoundFlag )    
        {   
            //一圈完成 把数组里记录的时间相加算出 一圈的时间
            uint16_t sumT = 0;
            uint8_t i;
            for ( i = 0; i < 6; i++  ) 
                sumT += usaMC_HallEva_InvTimeBuf[i];
            usMC_HallEva_IntervalTime = sumT;  //计算一圈的时间        
        }      
        else    
        {   
            //没有到一圈
            if ( ucMC_HallEva_DirChangedFlag )              //判断方向1是方向未改变 
            {
                //方向未改变 本次记录的换向间隔时间*6 预估一周的时间
                usMC_HallEva_IntervalTime = 6 * usaMC_HallEva_InvTimeBuf[ucMC_HallEva_EdgeTickDir];
            }   
            else      
            {
                //方向改变 变量重置？
                usMC_HallEva_IntervalTime = 16872; // ???  first round before continuous moving 
                ucMC_HallEva_LowSpeedorEdged = 1;
            }
            //记录6个数据就认为电角度完成1周  数组索引=5 表示记录了6个霍尔边沿的间隔
            if ( ucMC_HallEva_EdgeTickDir == 5 )
            {
                ucMC_HallEva_OneRoundFlag = 1;  //置旋转1周标记   这里的一周是指电角度一周
            }
        }

        // 数组索引自增 范围 0~5 表示保存6个霍尔边沿的间隔
        if (++ucMC_HallEva_EdgeTickDir >= 6 )
            ucMC_HallEva_EdgeTickDir = 0; 

        // 速度限制 过滤噪声 电角度周期不能小于46*64us 说明电角度频率不能大于1/46*64us 
        if ( usMC_HallEva_IntervalTime < 46 )  
             usMC_HallEva_IntervalTime = 46;
        
        //CCW 1-2-3-4-5-6-1 CW 6-5-4-3-2-1-6
        ucMC_HallSector = ucaMC_HallSecTable[ucMC_HallValue];  //读取霍尔扇区 
        ucMC_HallValueLast = ucMC_HallValue;
                               
/**
 \     |     /
  \    |    /
   \   |   /
    \  |  /
     \ | /
      \|/
--------------------------------
      /|\
     / | \
    /  |  \
   /   |   \  
  /    |    \
 /     |     \         
*/

        /**根据扇区选择起始角 中心角 高低速最大限制角 最大启动角 最小起始角*/
        if ( ucMC_HallSector )     //扇区有效监测  0扇区表示霍尔 工作不正常
        { 
            if ( ucMC_HallEva_Direction )     
            {   // CCW
                switch ( ucMC_HallSector )    
                {
                case 1u:
                    usMC_HallEva_SectCenterAngle = 5461;            // 30 degree, sector down limit
                    usMC_HallEva_SecStartAngle = 0;                 // 0 degree 
                    usMC_HallEva_AngleDownLim = 60438;              // -28 degree, next sector center when CW
                    usMC_HallEva_AngleUpLim = 5097;                 // 28 degree
                    usMC_HallEva_AngleLimitforLowSpd = 10922;       // 60 degree sector up limit
                    usMC_HallEva_AngleLimitforNormalSpd = 16019;    // 88 degree, next sector center when CCW
                    if ( !ucMC_HallEva_Over180Flag )
                        ucMC_HallEva_EvaReadyFlag = 1;
                    break;
                case 2u:
                    usMC_HallEva_SectCenterAngle = 0x3FFF;          // 90 degree, sector down limit
                    usMC_HallEva_SecStartAngle = 10922;             // 60 degree
                    usMC_HallEva_AngleDownLim = 5825;               // 32 degree
                    usMC_HallEva_AngleUpLim = 16019;                // 88 degree
                    usMC_HallEva_AngleLimitforLowSpd = 21845;       // 120 degree
                    usMC_HallEva_AngleLimitforNormalSpd = 26942;    // 148 degree
                    break;
                case 3u:
                    usMC_HallEva_SectCenterAngle = 27306;
                    usMC_HallEva_SecStartAngle = 21845;
                    usMC_HallEva_AngleDownLim = 16748;
                    usMC_HallEva_AngleUpLim = 26942;
                    usMC_HallEva_AngleLimitforLowSpd = 32768;       //-32768;
                    usMC_HallEva_AngleLimitforNormalSpd = 37865;    //-27671;
                    break;
                case 4u:
                    usMC_HallEva_SectCenterAngle = 38229;           //-27307;
                    usMC_HallEva_SecStartAngle = 32768;             //-32768;
                    usMC_HallEva_AngleDownLim = 27671;
                    usMC_HallEva_AngleUpLim = 37865;                //-27671;
                    usMC_HallEva_AngleLimitforLowSpd = 43690;       //-21846;
                    usMC_HallEva_AngleLimitforNormalSpd = 48787;    //-16749;
                    break;
                case 5u:
                    usMC_HallEva_SectCenterAngle = 49151;           //-16385;
                    usMC_HallEva_SecStartAngle = 43690;             //-21846;
                    usMC_HallEva_AngleDownLim = 38593;              //-26943;
                    usMC_HallEva_AngleUpLim = 48787;                //-16749;
                    usMC_HallEva_AngleLimitforLowSpd = 54613;       //-10923;
                    usMC_HallEva_AngleLimitforNormalSpd = 59710;    //-5826;
                    break;
                case 6u:
                    usMC_HallEva_SectCenterAngle = 60074;           //-5462;
                    usMC_HallEva_SecStartAngle = 54613;             //-10923;
                    usMC_HallEva_AngleDownLim = 49516;              //-16020;
                    usMC_HallEva_AngleUpLim = 59710;                //-5826;
                    usMC_HallEva_AngleLimitforLowSpd = 65535;       //-1;
                    usMC_HallEva_AngleLimitforNormalSpd = 5097;
                    break;
                case 0u:
                    break;
                }
                //每次估算的偏差值 = 扇区起始角-估算角
                sMC_HallEva_DistanceLeading = usMC_HallEva_SecStartAngle - usMC_AngleEva;
            }     
            else     
            { 
                // CW
                switch ( ucMC_HallSector)   
                {
                case 1u:
                    usMC_HallEva_SectCenterAngle = 5461;  // 30 degree, sector down limit
                    usMC_HallEva_SecStartAngle = 10922;   // 60 degree sector up limit
                    usMC_HallEva_AngleDownLim = 5825;     // 32 degree 
                    usMC_HallEva_AngleUpLim = 16019;      // 87 degree
                    usMC_HallEva_AngleLimitforLowSpd = 0; // 0 degree    
                    usMC_HallEva_AngleLimitforNormalSpd = 60438;//-5098;
                    break;
                case 2u:
                    usMC_HallEva_SectCenterAngle = 0x3FFF;
                    usMC_HallEva_SecStartAngle = 21845;
                    usMC_HallEva_AngleDownLim = 16748;
                    usMC_HallEva_AngleUpLim = 26942;
                    usMC_HallEva_AngleLimitforLowSpd = 10922;
                    usMC_HallEva_AngleLimitforNormalSpd = 5825;
                    break;
                case 3u:
                    usMC_HallEva_SectCenterAngle = 27306;
                    usMC_HallEva_SecStartAngle = 32768;//-32768;
                    usMC_HallEva_AngleDownLim = 27671;
                    usMC_HallEva_AngleUpLim = 37865;//-27671;
                    usMC_HallEva_AngleLimitforLowSpd = 21845;
                    usMC_HallEva_AngleLimitforNormalSpd = 16748;
                    break;
                case 4u:
                    usMC_HallEva_SectCenterAngle = 38229;//-27307;
                    usMC_HallEva_SecStartAngle = 43690;//-21846;
                    usMC_HallEva_AngleDownLim = 38593;//-26943;
                    usMC_HallEva_AngleUpLim = 48787;//-16749;
                    usMC_HallEva_AngleLimitforLowSpd = 32768;//-32768;
                    usMC_HallEva_AngleLimitforNormalSpd = 27671;
                    break;
                case 5u:
                    usMC_HallEva_SectCenterAngle = 49151;//-16385;
                    usMC_HallEva_SecStartAngle = 54613;//-10923;
                    usMC_HallEva_AngleDownLim = 49516;//-16020;
                    usMC_HallEva_AngleUpLim = 59710;//-5826;
                    usMC_HallEva_AngleLimitforLowSpd = 43690;//-21846;
                    usMC_HallEva_AngleLimitforNormalSpd = 38593;//-26943;
                    break;
                case 6u:
                    usMC_HallEva_SectCenterAngle = 60074;//-5462;
                    usMC_HallEva_SecStartAngle = 65535;//-1;
                    usMC_HallEva_AngleDownLim = 60438;//-5098;
                    usMC_HallEva_AngleUpLim = 5097;
                    usMC_HallEva_AngleLimitforLowSpd = 54613;//-10923;
                    usMC_HallEva_AngleLimitforNormalSpd = 49516;//-16020;
                    if ( !ucMC_HallEva_Over180Flag )
                        ucMC_HallEva_EvaReadyFlag = 1;
                    break;
                case 0u:
                    break;
                }
                //每次估算的偏差值 = 估算角 - 扇区起始角
                sMC_HallEva_DistanceLeading = usMC_AngleEva - usMC_HallEva_SecStartAngle;
            }
        
            //loc_8003906
            //电角度周期小于1013*64 && 电角度转过一周
            if ( (usMC_HallEva_IntervalTime<= DEF_ANGLE_LIMIT_SPEED_FREQUENCY) && ucMC_HallEva_OneRoundFlag )     
            {
                //进入估算状态
                //周期小于 DEF_ANGLE_LIMIT_SPEED_FREQUENCY*64 说明频率大于 1/ DEF_ANGLE_LIMIT_SPEED_FREQUENCY*64 所以是高速
                //设置限制角 = 高速的限制角
                usMC_HallEva_AngleLimit = usMC_HallEva_AngleLimitforNormalSpd;
                
                
                /**以下是估算角大于最大启动角 或小于最小启动角的处理 */
                if ( ucMC_HallEva_Direction )    
                {   //CCW
                    if ( ucMC_HallSector == 1 )	
                    { 
                        // sector 1 evaluate    扇区1估算
                        if ( ucMC_HallEva_Over180Flag )	
                        {   // normal evaluate  普通估算
                            if ( usMC_AngleEva > usMC_HallEva_AngleUpLim )       
                            {   
                                //估算角超过最大启动角
                                usMC_AngleEva = usMC_HallEva_SecStartAngle; //从起始脚开始估算
                                sMC_HallEva_DistanceLeading = 0;            //估算偏差值清零
                                ucMC_HallEva_EvaReadyFlag = 0;              //就绪标记清零
                            }
                            //loc_8003AC0
                        }    
                        else 
                        {   // first evaluate 第一次估算？
                            if ( usMC_AngleEva < usMC_HallEva_AngleDownLim )  
                            {
                                //估算角超过最小启动角
                                usMC_AngleEva = usMC_HallEva_SecStartAngle;
                                sMC_HallEva_DistanceLeading = 0;
                                ucMC_HallEva_EvaReadyFlag = 0;
                            }
                        }
                    }     
                    else      
                    {   
                        //普通扇区估算
                        if ( (ucMC_HallSector == 6 )                   //第6个扇区 最后一个扇区
                            &&  (usMC_HallEva_IntervalTime > DEF_ANGLE_LIMIT_SPEED_FREQUENCY)     //电角度周期大于1013 是慢速
                            )
                        {
                            //与58行对应 可能有什么用？
                            usMC_HallEva_AngleLimit = 0xffff;           //-1 degree  
                        }
                        
                        //估算角度永远小于等于限制角度
                        if ( (usMC_AngleEva > usMC_HallEva_AngleUpLim)      //估算角度超过最大启动角
                            || (usMC_AngleEva < usMC_HallEva_AngleDownLim)  //估算角度超过最小启动角
                            )         
                        {
                            //估算角度超过上限或下限 估算错误 重新开始估算
                            // evaluate out limit
                            usMC_AngleEva = usMC_HallEva_SecStartAngle;
                            sMC_HallEva_DistanceLeading = 0;
                            ucMC_HallEva_EvaReadyFlag = 0;
                        }
                    }
                }     
                else 
                {//CW
                    // loc_80039E4
                    if ( ucMC_HallSector == 6 )  
                    {
                        if ( ucMC_HallEva_Over180Flag )   
                        {
                            if ( usMC_AngleEva < usMC_HallEva_AngleDownLim )    
                            {
                                // evaluate error
                                usMC_AngleEva = usMC_HallEva_SecStartAngle;
                                sMC_HallEva_DistanceLeading = 0;
                                ucMC_HallEva_EvaReadyFlag = 0;
                            }
                        }  
                        else 
                        {
                            if ( usMC_AngleEva > usMC_HallEva_AngleUpLim ) 
                            {
                                // evaluate error
                                usMC_AngleEva = usMC_HallEva_SecStartAngle;
                                sMC_HallEva_DistanceLeading = 0;
                                ucMC_HallEva_EvaReadyFlag = 0;
                            }
                        }
                    }
                    else      
                    {   // normal sector evaluate
                        if ( (ucMC_HallSector == 1)
                            && (usMC_HallEva_IntervalTime > DEF_ANGLE_LIMIT_SPEED_FREQUENCY) )
                        {
                            usMC_HallEva_AngleLimit = 0;            //0 degree  
                        }
                        if (  (usMC_AngleEva > usMC_HallEva_AngleUpLim)
                            || (usMC_AngleEva < usMC_HallEva_AngleDownLim))   
                        {
                            // evaluate error
                            usMC_AngleEva = usMC_HallEva_SecStartAngle;
                            sMC_HallEva_DistanceLeading = 0;
                            ucMC_HallEva_EvaReadyFlag = 0;
                        }
                    }
                }
            }    
            else      
            {
                // in first round evaluate state or slow movement state, use hall state directly
                // 在启动的第一个电角度周期 或 低速运行的状态下 直接使用霍尔状态
                usMC_AngleEva = usMC_HallEva_SecStartAngle;
                usMC_HallEva_AngleLimit = usMC_HallEva_AngleLimitforLowSpd;
                sMC_HallEva_DistanceLeading = 0;
                ucMC_HallEva_EvaReadyFlag = 0;
            }
        }    
        else    
        { 			
            // hall error
            ucMC_HallSectorErr = 1;
            //loc_800361E
            vBSP_DisableSVPWMOutput();
        }
    
        {
            int32_t accTime = 0;
            if ( usMC_HallEva_IntervalTime <= DEF_ANGLE_LIMIT_SPEED_FREQUENCY )   //电角度周期小于1013*64us
            {
                // speed enough evaluate  
                accTime = (usMC_HallEva_IntervalTime - usMC_HallEva_IntervalTimeLast);    //当前周期-上一次的周期 为正表示速度变慢 为负表示速度变快了
            } 
            else 
            {
                accTime = 0;
            }
        
            //计算角度插值 插入角
            //accTime上一个电角度的周期和这次电角度的周期的差 usMC_HallEva_IntervalTime时现在电角度的周期
            //现在电角度的周期 + 上一次电周期和这次电周期的周期差 = 估计下一个电角度的周期
            //sMC_HallEva_DistanceLeading = H200000F0_UW_HallEva_SecStartAngle_LAxis - ucMC_AngleEva;
            //sMC_HallEva_DistanceLeading + 0xFFFF / 估算的下一个电周期角度 + 0.5 
            usMC_HallEva_Interpolation = ( ((int32_t)sMC_HallEva_DistanceLeading) + 0xFFFF + ((usMC_HallEva_IntervalTime + accTime) >> 1)   )/ (usMC_HallEva_IntervalTime + accTime);
            //  (distance/(time+acc))+0.5
            ucMC_HallEva_Over180Flag = 0;
            usMC_HallEva_IntervalTimeLast = usMC_HallEva_IntervalTime;  
        }                          
    }
    
    /**
    超低速角度解决方案
    */
    if ( ucMC_HallEva_LowSpeedorEdged )
    { 
        // low speed, so use the center of sector 低速的时候不进行估算直接使用 中间角度
        usMC_AngleEva = usMC_HallEva_SectCenterAngle;
    }
    
    /**
    霍尔0位校准
    */
    // 估算角度 赋值外部全局变量
    usMC_AngleFb = usMC_AngleEva;
    
    // 角度加上偏置值
    if ( ucMC_HallEva_LowSpeedorEdged )  
    {
        usMC_AngleFb += 4732;//26 degree    4732
    }  
    else 
    {
        
        if ( ucMC_HallEva_Direction )  
        {   //CCW 逆时针旋转       
            usMC_AngleFb += (4732 + 1093);//32 degree
        }  
        else  
        {   //CW  顺时针旋转
            usMC_AngleFb += (4732 - 1093);//20 degree
        }
    }

    /**
    转速计算
    */
    {   
        //高低速的转折速度在 78 79 rpm左右 实测
         
        int16_t tmp = ((60/10)*16000)/usMC_HallEva_IntervalTime;
        if( tmp < 10 )
            tmp  = 0;
        if( !ucMC_HallEva_Direction )
            sMC_HallEstSpeed_RPM = -tmp;    //ccw
        else
            sMC_HallEstSpeed_RPM = tmp;     //cw
        
        
    }
    //
    
}
//角度估算结束
