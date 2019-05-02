/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_tempALte/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include "focL.h"
#include "mc_estHallAngleL.h"
#include "mc_estHallAngleR.h"
#include "rf2_4g.h"


/** @addtogroup STM32F10x_StdPeriph_tempALte
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DEF_MAX_TOROUE          (1000L)              //���ֵ��1024
#define DEF_LOW_MAX_SPEED       (900L)
#define DEF_HIGH_MAX_SPEED      (1800L)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

typedef enum { eSystemCalibration = -1, eSystemInit = 0, eSystemRun = 1, eSystemErr = 2} systemstate_t;

int16_t sFOC_PitchTorqueCmd = 0;
int16_t sMC_CurrentPhaseUL = 0;
int16_t sMC_CurrentPhaseVL = 0;
int16_t sMC_CurrentPhaseUR = 0;
int16_t sMC_CurrentPhaseVR = 0;

int32_t nMC_SpeedPIDInteger = 0;
int8_t  cMC_MotorState = 0;
int8_t  cMC_LastMotorState = 0;
uint8_t ucMC_SpeedLoopCnt = 0;    //
int32_t nMC_SpeedKp = 0x2700;       //0X1700
int32_t nMC_SpeedKi = 0x00FF;       //0X00FF
int32_t nMC_SpeedKd = 0x007F;       //0X007F
int32_t errTn_1 = 0;

int32_t nMC_SpeedCmd_RPM = 0;
int32_t nMC_AccelSpeed_RPMM = 0x017f;
int32_t nMc_SpeedRef_RPM = 0;
int32_t nMc_SpeedRefDisplay_RPM = 0;

int8_t  nMC_InitFinished = 0;

int32_t nMC_MotorEnableFlag = 0;

int8_t ucMC_CalibrationErr = 0;

int16_t	sFOC_QOutLFreeMax = 0;

systemstate_t ucSystemState = eSystemInit;



uint8_t ucPowButtonFristPush = 0;
uint32_t unPowButtonCnt = 0;


uint8_t ucPowButtonLongPushEvent = 0;
uint8_t ucPowButtonPowOffEvent = 0;
uint8_t ucPowButtonStartEvent = 0;

uint8_t ucSystemErrorCatchEvent = 0;
uint8_t ucSystemErrID = 0;

uint32_t unAutoPowerOffTime = 0;


_iq qVBUS_V = 0;
_iq qVBUS_A_Offset = 0;
_iq qVBUS_A = 0;

uint16_t uiBackPWMDuty = 0;
uint8_t ucOverLoadFlag = 0;

uint8_t ucMC_TempErr = 0;
uint8_t ucMC_OverCurrErr = 0;

float fVbusVolt = 0;
float fVbusCurr = 0;

/* Private function prototypes -----------------------------------------------*/
void BusVoltageProc(void);
void BusCurrentProc(void);
/* Private functions ---------------------------------------------------------*/

void vBSP_SystemClockConfig(void)
{
	RCC_DeInit(); 
	RCC_HSICmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY)== RESET);
	
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	
	FLASH_SetLatency(FLASH_Latency_2);   
	
	
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK2Config(RCC_HCLK_Div1); 
	RCC_PCLK1Config(RCC_HCLK_Div2);  
	
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);   //4*16 = 48MHZ
	RCC_PLLCmd(ENABLE);   
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) ; 
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
	while(RCC_GetSYSCLKSource() != 0x08){}
    
    SystemCoreClock = 64000000UL;
}

void vBSP_SystemClockOUTConfing(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    RCC_MCOConfig(RCC_MCO_SYSCLK);
}

int32_t vBSP_SVPWM_2ShuntCurrentReadingCalibration1(void)
{
    uint8_t i;
    uint32_t tempAL = 0;
    uint32_t tempBL = 0;
    uint32_t tempAR = 0;
    uint32_t tempBR = 0;
    int32_t _BusCurrentOffset = 0;
    for( i = 0 ; i < 32 ; i++ )
    {
        /* Clear the ADC1 JEOC pending flag */
        while(ucSystem1msFlag == 0){}
        ucSystem1msFlag = 0;
        tempAL += sMC_CurrentPhaseUL;
        tempBL += sMC_CurrentPhaseVL;
        tempAR += sMC_CurrentPhaseUR;
        tempBR += sMC_CurrentPhaseVR; 
            
        qVBUS_A_Offset +=  vBSP_GetVBusCurrent_A();//����ĸ�ߵ���
    }
    
    usMC_PhaseAOffsetL = tempAL >> 5;
    usMC_PhaseBOffsetL = tempBL >> 5;
    
    usMC_PhaseAOffsetR = tempAR >> 5;
    usMC_PhaseBOffsetR = tempBR >> 5;
    
    qVBUS_A_Offset = _IQdiv32(qVBUS_A_Offset);
    
    if( usMC_PhaseAOffsetL >= 0x900 || usMC_PhaseAOffsetL < 0x600)
        return -1;
    
    if( usMC_PhaseBOffsetL >= 0x900 || usMC_PhaseBOffsetL < 0x600)
        return -2; 

    if( usMC_PhaseAOffsetR >= 0x900 || usMC_PhaseAOffsetR < 0x600)
        return -1;
    
    if( usMC_PhaseBOffsetR >= 0x900 || usMC_PhaseBOffsetR < 0x600)
        return -2; 
    
    return 0;
}



void TorqueLoopCtrl(void)
{
    int16_t pitchAbs,QOutAbs;
    
    if ( sFOC_PitchTorqueCmd < 0 )
    {
        pitchAbs = -sFOC_PitchTorqueCmd;
    }
    else
    {
        pitchAbs = sFOC_PitchTorqueCmd;
    }
    
    if ( sFOC_QOutL < 0 )
    {
        QOutAbs = -sFOC_QOutL;
    }
    else
    {
        QOutAbs = sFOC_QOutL;
    }

    if ( pitchAbs <= QOutAbs )
    {
        sFOC_QOutL = sFOC_PitchTorqueCmd;
    }		
    else //if ( H200000BB_UB_OverLoad_VoltageLow_RAxis  == 0)		
    {
        if ( sFOC_QOutL > sFOC_PitchTorqueCmd )				
        {
            if ( sFOC_QOutL <= sFOC_PitchTorqueCmd + 2 )
                sFOC_QOutL = sFOC_PitchTorqueCmd;
            else
                sFOC_QOutL -= 2;
        }
        else if ( sFOC_QOutL < sFOC_PitchTorqueCmd )
        {
            if ( sFOC_QOutL + 2 >= sFOC_PitchTorqueCmd )			
                sFOC_QOutL = sFOC_PitchTorqueCmd;
            else 
                sFOC_QOutL += 2;  
        }
    }
}


void SpeedLoopCtrl(void)
{
    ucMC_SpeedLoopCnt++;
    if ( ucMC_SpeedLoopCnt > 16 * 5 )
    {
        ucMC_SpeedLoopCnt = 0;
        {

            int32_t errT = (nMc_SpeedRef_RPM>>16) - sMC_HallEstSpeed_RPML;
            
            errT <<= 16;      //IQ16;
            errT = errT / 60 / 200;
             
            
            nMC_SpeedPIDInteger+= errT * nMC_SpeedKi;

            if( nMC_SpeedPIDInteger > (DEF_MAX_TOROUE<<16))
            {
                nMC_SpeedPIDInteger = (DEF_MAX_TOROUE<<16);
            }
            else if( nMC_SpeedPIDInteger < (-DEF_MAX_TOROUE<<16) )
            {
                nMC_SpeedPIDInteger = (-DEF_MAX_TOROUE<<16);
            }
           
            //sFOC_PitchTorqueCmd = (((errT * nMC_SpeedKp + nMC_SpeedPIDInteger)>>16)*9 + sFOC_PitchTorqueCmd*1) / 10;
            
            sFOC_PitchTorqueCmd = (errT * nMC_SpeedKp + nMC_SpeedPIDInteger + nMC_SpeedKi * (errT - errTn_1))>>16;
            
            errTn_1 = errT;
            
            if (sFOC_PitchTorqueCmd > DEF_MAX_TOROUE)
                sFOC_PitchTorqueCmd = DEF_MAX_TOROUE;
            if (sFOC_PitchTorqueCmd < -DEF_MAX_TOROUE)
                sFOC_PitchTorqueCmd = -DEF_MAX_TOROUE;
        }
    }
}

void AutoPowerOffClear(void)
{
    unAutoPowerOffTime = 0;
}

void AutoPowerOffPoll(void)
{
    if( unAutoPowerOffTime < 100 * 15 *60 ) //
        unAutoPowerOffTime++;
    else
        ucPowButtonPowOffEvent = 1;             //���͹ػ��¼�
}

void AccelSpeedCtrl(void)
{
    int32_t cmdSpeed = (nMC_SpeedCmd_RPM<<16);
    
    if ( cmdSpeed > nMc_SpeedRef_RPM )
    {
        nMc_SpeedRef_RPM += nMC_AccelSpeed_RPMM; 
        if( nMc_SpeedRef_RPM > cmdSpeed)
            nMc_SpeedRef_RPM = cmdSpeed; 
    }
    else if ( cmdSpeed < nMc_SpeedRef_RPM )
    {
        nMc_SpeedRef_RPM -= nMC_AccelSpeed_RPMM;
        if( nMc_SpeedRef_RPM < cmdSpeed)
            nMc_SpeedRef_RPM = cmdSpeed; 
    }
    nMc_SpeedRefDisplay_RPM = nMc_SpeedRef_RPM>>16; 
}


void LedHandler(void)
{
    static uint8_t Led10msCnt = 0;
    static uint8_t LedErrSyteCnt = 0;
    
    if ( ucSystemErrID != 0)
    {
        if( ++Led10msCnt >= 20)
        {
            Led10msCnt = 0;
            if(++LedErrSyteCnt >= ucSystemErrID*2+2)
            {
                LedErrSyteCnt = 0;
            }
            
            if( LedErrSyteCnt >= ucSystemErrID*2)
            {
                vBSP_SetKeyLed(DISABLE);
            }
            else
            {
                if(LedErrSyteCnt & 0x01)
                    vBSP_SetKeyLed(ENABLE);
                else
                    vBSP_SetKeyLed(DISABLE);                
            }
        }
    }
    else
    {
        if(ucRfWorkState == eRFCalibration)
        {
            if( ++Led10msCnt >= 5)
            {
                Led10msCnt = 0;
                vBSP_CplKeyLed();
            }
        }
        else
        {
            if (ucRfConntState == eRFConnt )
            {
                vBSP_SetKeyLed(ENABLE);
            }
            else if(ucRfConntState == eRFDisconnt)
            {
                if( ++Led10msCnt >= 100)
                {
                    Led10msCnt = 0;
                    vBSP_CplKeyLed();
                }
            }
        }
    }
}


uint8_t MotionState = 0;
uint16_t DelayTime = 0;
void Motion(void)
{
#if 0
    if( sADValue < -60)
    {
        DelayTime = 0;
        AutoPowerOffClear();
        if( MotionState != 0 )
        {
            
            //nMC_SpeedPIDInteger = 0;
            nMc_SpeedRef_RPM = ((int32_t)sMC_HallEstSpeed_RPML)<<16;  
                        
            MotionState = 0;
        }
        
        cMC_MotorState = 1;
        {
            float acctmp;  

            acctmp = (( - 60 - sADValue)*(0x0fff-0x5ff)/(430-60) + 0x5ff);
            nMC_AccelSpeed_RPMM = acctmp; 
        }
        nMC_SpeedCmd_RPM = 0; 
        
/** ������� */     
        if( sMC_HallEstSpeed_RPML == 0 )
        {  
            if( nMC_SpeedPIDInteger > (10L<<16))
            {
                nMC_SpeedPIDInteger -= (10L<<16);
            }
            else if( nMC_SpeedPIDInteger < (-10L<<16) )
            {
                nMC_SpeedPIDInteger += (10L<<16);
            } 
            else
            {
                 nMC_SpeedPIDInteger = (0L<<16);
            }
        }

/*      ��������    */
         
        if( nMC_SpeedPIDInteger < ((-612)<<16))
        {
            nMC_SpeedPIDInteger = ((-612)<<16);
        }

        if( nMC_SpeedPIDInteger > (612<<16))
        {
            nMC_SpeedPIDInteger = (612<<16);
        }

/**     ��������    */
        if( sMC_HallEstSpeed_RPML < 100 && sMC_HallEstSpeed_RPML > -100)
        {
            if( ucDirMode )
            {            
                if( nMC_SpeedPIDInteger < ((-212)<<16))
                {
                    nMC_SpeedPIDInteger += (10L<<16);
                }
            }
            else
            {
                if( nMC_SpeedPIDInteger > (212<<16))
                {
                    nMC_SpeedPIDInteger -= (10L<<16);
                }                
            }         
        }


/*      ��������ŷ�ת     
        if( ucDirMode )
        {
            if( sMC_HallEstSpeed_RPML <= 0 )
            {  
                if( nMC_SpeedPIDInteger < (-10L<<16) )
                {
                    nMC_SpeedPIDInteger += (10L<<16);
                } 
                else
                {
                    nMC_SpeedPIDInteger = (0L<<16);
                }
            }
        }
        else
        {
            if( sMC_HallEstSpeed_RPML >= 0 )
            {  
                if( nMC_SpeedPIDInteger > (10L<<16))
                {
                    nMC_SpeedPIDInteger -= (10L<<16);
                }
                else
                {
                    nMC_SpeedPIDInteger = (0L<<16);
                }
            }   
        }
*/         

    }
#endif
    if( sADValue < -60)
    {
        cMC_MotorState = 2;
        if( sFOC_QOutL == 0 )
        {
            
        }
    }
    else if( sADValue > 60)
    {
        uiBackPWMDuty = 255;
        DelayTime = 0;
        AutoPowerOffClear();
        if( MotionState != 1 )
        {
            //nMC_SpeedPIDInteger = 0;
            nMc_SpeedRef_RPM = ((int32_t)sMC_HallEstSpeed_RPML)<<16;  
            MotionState = 1;
        }
        
        if( sFOC_QOutL < 80 && ucDirMode == 1 )
        {
            sFOC_QOutL+=2;
        }
        else if( sFOC_QOutL > -80 && ucDirMode == 0 )
        {
            sFOC_QOutL-=2;
        } 
        
        if( ucDirMode )
        {            
            if( nMC_SpeedPIDInteger < ((0)<<16))
            {
                nMC_SpeedPIDInteger += ((15)<<16);
            }
        }
        else
        {
            if( nMC_SpeedPIDInteger > (0<<16))
            {
                nMC_SpeedPIDInteger -= (15<<16);
            }                
        } 
        
        
        {
            float speedtmp;  
            
            if(ucDirMode)
                speedtmp = 1.0f;
            else
                speedtmp = -1.0f;
            
            if( ucSpeedMode == 0 )
            {
                speedtmp *= ((sADValue - 60)*(DEF_LOW_MAX_SPEED-100)/(430-60) + 100);
                nMC_AccelSpeed_RPMM = 0xff; 
            }
            else
            {
                speedtmp *= ((sADValue - 60)*(DEF_HIGH_MAX_SPEED-100)/(430-60) + 100);
                nMC_AccelSpeed_RPMM = 0x1ff; 
            }
            
            nMC_SpeedCmd_RPM = speedtmp;  
        }
        
        {
            int32_t absSpeedRef = nMC_SpeedCmd_RPM;
            int32_t absSpeedFdbk = sMC_HallEstSpeed_RPML;

            if(( absSpeedRef >= 0 && absSpeedFdbk >= 0)||
                ( absSpeedRef <= 0 && absSpeedFdbk <= 0))
            {
                if( absSpeedRef < 0)
                    absSpeedRef = -absSpeedRef;
                if( absSpeedFdbk < 0 )
                    absSpeedFdbk = -absSpeedFdbk; 
                if(absSpeedRef > absSpeedFdbk)
                    cMC_MotorState = 1;                    
            }
            else
            {
                cMC_MotorState = 1;     
            }
        }
    }
    else if( sADValue > -50 && sADValue < 50 )
    {

        nMC_AccelSpeed_RPMM = 0x07f;

        if( MotionState != 2 )
        {
            MotionState = 2;
            
        }
        nMc_SpeedRef_RPM = ((int32_t)sMC_HallEstSpeed_RPML)<<16;         
        nMC_SpeedCmd_RPM = 0;
        cMC_MotorState = 0;      

/**
        if( DelayTime <  250)
            DelayTime++;
        if( DelayTime >= 250)
        {
            nMc_SpeedRef_RPM = ((int32_t)sMC_HallEstSpeed_RPML)<<16;  
            cMC_MotorState = 0;

        }
        nMC_AccelSpeed_RPMM = 0x07f;
        AutoPowerOffClear();
        if( MotionState != 2 )
        {
            MotionState = 2;
            nMc_SpeedRef_RPM = ((int32_t)sMC_HallEstSpeed_RPML)<<16; 
        }   
        nMC_SpeedCmd_RPM = 0;
*/
    }

}



void SystemErrorCatch(void)
{
    if (ucComErrState == 0)
    {
        //0x01���Ƴ���R�� 0x02:�¶�65(����û��)  0x04:�¶�75(����û��) 0x08:��������R��
        //0x10���Ƴ���L�� 0x20:�¶�65(�ӻ�����)  0x40:�¶�75(�ӻ�����) 0x80:��������L��
         
        //����ƫ��У������ ����2
        if(ucMC_CalibrationErr)
        {
            ucComErrState = 0x08;       //����������
            ucSystemErrID = 4;
        }

        //��������
        if(ucMC_OverCurrErr)
        {
            ucComErrState = 0x01;       //�����Ƴ���            
            ucSystemErrID = 1;
        }

        //�������� ����3
        if(ucMC_HallSectorErrL)
        {
            ucComErrState = 0x01;       //�����Ƴ���            
            ucSystemErrID = 2;
        }

        //�¶ȴ���
        if(ucMC_TempErr)
        {            
            ucComErrState = 0x02;       //���¶ȴ���
            ucSystemErrID = 3;
        }

 
        if(ucComErrState != 0)
            ucSystemErrorCatchEvent = 1;
    }
}


#define DEF_POWBUTTON_POWON_LIMIT       100
#define DEF_POWBUTTON_POWOFF_LIMIT      2000
#define DEF_POWBUTTON_LONG_PUSH_LIMIT   3000


void PowBtnPoll(void)
{
    uint16_t	 i;

    if(unPowButtonCnt < 100000)
        unPowButtonCnt++;
    
    if( ucPowButtonFristPush == 0)
    {
        //��һ�ΰ���
        if( ucBSP_GetPowButtonState() == 1 )
        {
            if( unPowButtonCnt == DEF_POWBUTTON_LONG_PUSH_LIMIT )
                ucPowButtonLongPushEvent = 1;       //���ͳ����¼�
        }
        else
        {
            if (unPowButtonCnt < DEF_POWBUTTON_POWON_LIMIT)
            {
                //����ʱ��̫��
                ucPowButtonPowOffEvent = 1;         //���͹ػ��¼�
            }
            else 
            {
                unPowButtonCnt = 0;  
                ucPowButtonFristPush = 1; 
                ucPowButtonStartEvent = 1;          //���Ϳ����¼�          
            }
        }   
    }
    else
    {
        if( ucBSP_GetPowButtonState() == 1 )
        {
            if( unPowButtonCnt == DEF_POWBUTTON_POWOFF_LIMIT )
                ucPowButtonPowOffEvent = 1;         //���͹ػ��¼�
        }
        else
        {
            unPowButtonCnt = 0;
        } 
    }   
    return;			
}

void SystemEventHandler(void)
{
    if( ucPowButtonPowOffEvent )
    {        
        ucPowButtonPowOffEvent = 0;
        //�����ػ��¼�
        vBSP_SetKeyLed(DISABLE);        //�ر�LED
        vBSP_DisableSVPWMOutput();      //��SVPWM        

        while(ucBSP_GetPowButtonState() == 1)
        {
            //�ȴ���������
        }
        vBSP_SetPowerCrtl(DISABLE);     //��MOS��
        while(1){}  //��ѭ��ֱ���ػ�
    }

    if( ucPowButtonLongPushEvent )
    {
        ucPowButtonLongPushEvent = 0;
        //���������¼�
        ucSystemState = eSystemCalibration;  
        RF2_4G_StartCalibration();      //�������
    }
    
    if( ucSystemCalibrationFinishEvent )
    {            

        ucSystemCalibrationFinishEvent = 0;
        ucSystemState = eSystemRun;
    }
    
    if( ucSystemErrorCatchEvent )
    {
        ucSystemState = eSystemErr;
    }
    
    if( ucPowButtonStartEvent )
    {
        if( ucSystemState == eSystemInit)
        {
            ucSystemState = eSystemRun;
            ucRfWorkState = eRFWork;
            ucRfConntState = eRFDisconnt;  
        }
    }
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */
   
    vBSP_SystemClockConfig();
    nBSP_SysTickConfig(1000);           //1khz 1ms
    vBSP_LEDConfig();
    vBSP_SetPowerCrtl(ENABLE);
    
    vBSP_DelayMS(100);

    RF2_4G_Init();
  

    nBSP_USARTConfig(115200UL);
    nBSP_SVPWM2ShuntInit(32000UL);
//    //vBSP_DACConfig();  
    vBSP_KeyConfig();
   
    vBSP_HALLInit();

    vBSP_SetSVPWMDutyPhaseU_Pu(DEF_MOTOR_L,100);    //512����
    vBSP_SetSVPWMDutyPhaseV_Pu(DEF_MOTOR_L,100);    //512����
    vBSP_SetSVPWMDutyPhaseW_Pu(DEF_MOTOR_L,100);    //512����
    
    vBSP_SetSVPWMDutyPhaseU_Pu(DEF_MOTOR_R,100);    //512����
    vBSP_SetSVPWMDutyPhaseV_Pu(DEF_MOTOR_R,100);    //512����
    vBSP_SetSVPWMDutyPhaseW_Pu(DEF_MOTOR_R,100);    //512����
    
    vBSP_SetKeyLed(0);
    vBSP_SetKeyLed(1);
    
    TRAG_DBG("����PWM ��ȡ�˷�ƫ��\r\n");
    vBSP_EnableSVPWMOutput();
    if(vBSP_SVPWM_2ShuntCurrentReadingCalibration1() != 0)
    {
        ucMC_CalibrationErr = 1;
    }

    vBSP_DisableSVPWMOutput();    
    TRAG_DBG("�ر�PWM �˷�ƫ�û�ȡ���\r\n");
    TRAG_DBG("PhaseAOffset = 0x%04x\r\n",usMC_PhaseAOffsetL);
    TRAG_DBG("PhaseBOffset = 0x%04x\r\n",usMC_PhaseBOffsetL);
    
    /* Infinite loop */
    usMC_HallEva_IntervalTimeL = 16872;
    sMC_HallEstSpeed_RPML = 0;
    cMC_MotorState = 0;
    nMC_InitFinished = 1;
    while (1)
    {
        RF2_4G_Poll();
        
        //1ms��ʱ��Ƭ
        if(ucSystem1msFlag)
        {
            ucSystem1msFlag = 0;
            
            //��������
            PowBtnPoll();
            
            //ϵͳ�¼�����
            SystemEventHandler();
            
            if(ucSystemState == eSystemInit)
            {
                //��ʼ��״̬
                vBSP_SetKeyLed(ENABLE);
            }
            else if( ucSystemState == eSystemCalibration )
            {
 
            }
            else if( ucSystemState == eSystemRun )
            {
                //����״̬
                
                //�����̬����
                Motion();   
                SystemErrorCatch();
            }
            else if( ucSystemState == eSystemErr )
            {
                //����״̬
                
            }
            
            BusVoltageProc();
            BusCurrentProc();
        }
        
        //10ms��ʱ��Ƭ   
        if(ucSystem10msFlag)
        {
            ucSystem10msFlag = 0;
            RF2_4G_Poll_10ms();
            LedHandler();

            AutoPowerOffPoll();
        }
       //�������˵Ĵ���ʾ��������
       //serialPortHunterDebugLog();
    }
}

///����TIM1
void ADC1_2_IRQHandler1(void)
{
    if( ADC_GetITStatus(ADC1,ADC_IT_JEOC))
    {
        ADC_ClearITPendingBit(ADC1,ADC_IT_JEOC);  
    }
    
    if( ADC_GetITStatus(ADC2,ADC_IT_JEOC))
    {
        ADC_ClearITPendingBit(ADC2,ADC_IT_JEOC);    
        vMC_EST_HallAngleL();  
    }     
}

void ADC1_2_IRQHandler(void)
{
    if( ADC_GetITStatus(ADC1,ADC_IT_JEOC))
    {
        ADC_ClearITPendingBit(ADC1,ADC_IT_JEOC);  
    }
    
    if( ADC_GetITStatus(ADC2,ADC_IT_JEOC))
    {
        ADC_ClearITPendingBit(ADC2,ADC_IT_JEOC);    

        //��ȡ�����
        //sk-1s ADC����43.6906666666 ��������1A
        sMC_CurrentPhaseUL = sBSP_GetCurrentPhaseUL();    //a14   ����
        sMC_CurrentPhaseVL = sBSP_GetCurrentPhaseVL();    //a15   ����
        if(nMC_InitFinished)
        {

            sFOC_CurrUL = -(usMC_PhaseAOffsetL - sMC_CurrentPhaseUL) - (usMC_PhaseBOffsetL - sMC_CurrentPhaseVL);               //����
            sFOC_CurrVL = usMC_PhaseAOffsetL - sMC_CurrentPhaseUL;                                    //����

            
            vMC_EST_HallAngleL();      
            
            if( cMC_MotorState == 0 || ucOverLoadFlag == 1)
            {
                //nMC_SpeedPIDInteger = 0;  
                nMc_SpeedRef_RPM = ((int32_t)sMC_HallEstSpeed_RPML)<<16;  
                //sFOC_PitchTorqueCmd = 0; 
                //sFOC_QOutL = 0;            
            }
            else if( cMC_MotorState == 1 )
            {         
                if(ucOverLoadFlag == 0)
                {
                    AccelSpeedCtrl();
                    SpeedLoopCtrl();
                    TorqueLoopCtrl();                    
                }
            }  

            if ( ucMC_HallEva_LowSpeedorEdgedL )
            {
                //���ٲ�����ID
                vFOC_MotorLock();
            }		
            else		
            {			
                if (( sMC_CurrentPhaseUL < 3848 ) && ( sMC_CurrentPhaseVL < 3848 ))
                {
                    vFOC_Clark();  //Clark�任
                    vFOC_Park();   //Park�任
                    vFOC_DCurrentControl();    //Id����
                }
            }
            vFOC_IPark();
            if((cMC_MotorState != 2 || sFOC_QOutL != 0 ) && (cMC_MotorState != 0 || sFOC_QOutL != 0 ))
                vFOC_SVPWM();
            
            
            //Ϊ�˼�����SVPWM���ſ���SVPWMOUT
            //��ֹ�ڿ�����ʱ�����ϴε�ռ��������
#if 1
            {
                static int32_t dectime = 0;
                if( cMC_MotorState == 1)
                {
                    vBSP_BrakeDisable();
                    vBSP_EnableSVPWMOutput();
                    nMC_MotorEnableFlag = 1;
                    dectime = 0;
                }
                else if(cMC_MotorState == 0)
                {
                    vBSP_BrakeDisable();  
                    if (cMC_LastMotorState == 1)
                    {
                        if (sMC_HallEstSpeed_RPML > 500)
                            sFOC_QOutLFreeMax = sFOC_QOutL * 2 / 5;
                        else
                            sFOC_QOutLFreeMax = sFOC_QOutL / 3;
                        
                        if (sFOC_QOutLFreeMax < 0)
                            sFOC_QOutLFreeMax = -sFOC_QOutLFreeMax;
                    }
                    
                    if( dectime == 0)
                    {
#if 0                     
                        if(sFOC_QOutL < 0) {
                            sFOC_QOutL++;
                            uiBackPWMDuty = 255;
                            dectime = 20;//60
                        }
                        else if(sFOC_QOutL > 0) {
                            sFOC_QOutL--;
                            uiBackPWMDuty = 255;
                            dectime = 20;//60
                        }
                        else {    
                            if(uiBackPWMDuty < 512)
                                uiBackPWMDuty++;
                            TIM1->CCR1 = uiBackPWMDuty;
                            TIM1->CCR2 = uiBackPWMDuty;
                            TIM1->CCR3 = uiBackPWMDuty;
                            
                            if(TIM1->CCR1 >= 0x1ff)
                            {
                                nMC_MotorEnableFlag = 0;
                                vFOC_MotorLock(); 
                                vBSP_DisableSVPWMOutput();
                                sFOC_QOutL = 0;
                                sFOC_PitchTorqueCmd = 0;
                                nMC_SpeedPIDInteger = 0;
                            }
                            dectime = 50;//60
                        }

                        if( sFOC_QOutL > 500 )
                            sFOC_QOutL-=1;
                        else if( sFOC_QOutL < -500)
                            sFOC_QOutL+=1;
                        sFOC_PitchTorqueCmd = sFOC_QOutL;
                        nMC_SpeedPIDInteger = sFOC_PitchTorqueCmd << 16;
#endif                   
                   
                        if( uiBackPWMDuty <= 255 )
                        {

                            int16_t sAbsFOC_QOutL = sFOC_QOutL;
                          
                            uiBackPWMDuty = 255;
                            dectime = 50;//60
                            
                            if(sAbsFOC_QOutL < 0)
                                sAbsFOC_QOutL = -sAbsFOC_QOutL;   

                            if( sAbsFOC_QOutL <= sFOC_QOutLFreeMax)//&& usFOC_DOut == 0)
                            {  
                                nMC_MotorEnableFlag = 0;
                                vFOC_MotorLock(); 
                                vBSP_DisableSVPWMOutput();
                                sFOC_QOutL = 0;
                                sFOC_PitchTorqueCmd = 0;
                                nMC_SpeedPIDInteger = 0;
                            }
                            else
                            {
                                if(sFOC_QOutL < 0) {
                                    sFOC_QOutL++;
                                }
                                else if(sFOC_QOutL > 0) {
                                    sFOC_QOutL--;
                                }
                            
                                if( sFOC_QOutL > 500 )
                                    sFOC_QOutL-=1;
                                else if( sFOC_QOutL < -500)
                                    sFOC_QOutL+=1;
                                sFOC_PitchTorqueCmd = sFOC_QOutL;
                                nMC_SpeedPIDInteger = sFOC_PitchTorqueCmd << 16;
                            } 
                        }
                        else
                        {
                            dectime = 4;//60
                            uiBackPWMDuty--;                            
                            TIM1->CCR1 = 255;
                            TIM1->CCR2 = 255;
                            TIM1->CCR3 = 255;

                        }
                  
                    }
                    else 
                        dectime--;                         
                    
                }
                else if(cMC_MotorState == 2)
                {
                    vBSP_EnableSVPWMOutput();
                    vBSP_BrakeDisable();
                    if( dectime == 0)
                    { 
                        dectime = 4;//60
                        if(sFOC_QOutL < 0) {
                            sFOC_QOutL++;
                        }
                        else if(sFOC_QOutL > 0) {
                            sFOC_QOutL--;
                        }
                        else
                        { 
                            if(uiBackPWMDuty < 256)
                                uiBackPWMDuty = 256;                            
                            if(uiBackPWMDuty < 512)
                                uiBackPWMDuty++;
                            TIM1->CCR1 = 512 - uiBackPWMDuty;
                            TIM1->CCR2 = 512 - uiBackPWMDuty;
                            TIM1->CCR3 = 512 - uiBackPWMDuty;
                            nMC_MotorEnableFlag = 0;
                            vFOC_MotorLock(); 
                            //vBSP_BrakeEnable();
                        }
                    }
                    else 
                        dectime--;   
                
                    if( sFOC_QOutL > 500 )
                        sFOC_QOutL-=1;
                    else if( sFOC_QOutL < -500)
                        sFOC_QOutL+=1;
                    sFOC_PitchTorqueCmd = sFOC_QOutL;
                    nMC_SpeedPIDInteger = sFOC_PitchTorqueCmd << 16;
                }
            }
#endif 

            cMC_LastMotorState = cMC_MotorState;
        }
        //vBSP_SetDACValue(usMC_AngleFbL>>4);      //DAC����Ƕ�
        //vBSP_SetKeyLed(DISABLE);
    }
}


///����TIM8
void ADC3_IRQHandler(void)
{
    vBSP_SetKeyLed(1); 
    if( ADC_GetITStatus(ADC3,ADC_IT_JEOC))
    {
        ADC_ClearITPendingBit(ADC3,ADC_IT_JEOC);  

        sMC_CurrentPhaseUR = sBSP_GetCurrentPhaseUR();    //a14   ����
        sMC_CurrentPhaseVR = sBSP_GetCurrentPhaseVR();    //a15   ����
        if(nMC_InitFinished)
        {
            vMC_EST_HallAngleR();
        }    
    }
    vBSP_SetKeyLed(0);
}

void BusCurrentProc(void)
{
    qVBUS_A = vBSP_GetVBusCurrent_A() - qVBUS_A_Offset;  
    fVbusCurr = _IQtoF(qVBUS_A);
    //if (qVBUS_A < _IQ(10.0) && qVBUS_V > _IQ(21.3))
    if (qVBUS_V > _IQ(21.3))
    {
        ucOverLoadFlag = 0;
    }
    else
    {
        ucOverLoadFlag = 1;
		if ( sFOC_QOutL >= 0 )
			--sFOC_QOutL;// decrease output
		else
			++sFOC_QOutL;
    }
    
}


void BusVoltageProc(void) 
{
    qVBUS_V = vBSP_GetVBus_V();        
    fVbusVolt = _IQtoF(qVBUS_V);
    if( qVBUS_V < _IQ(21.3))
    {
        ucVdcBusState = 4;
    }
    else if( qVBUS_V < _IQ(23.5))
    {
        ucVdcBusState = 0;
    }
    else if( qVBUS_V < _IQ(24.27))
    {
        ucVdcBusState = 1;
    } 
    else if( qVBUS_V < _IQ(27.07))
    {
        ucVdcBusState = 2;
    }
    else
    {
        ucVdcBusState = 3;
    }
	return;
}




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
