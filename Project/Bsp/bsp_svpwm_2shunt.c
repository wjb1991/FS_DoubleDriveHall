#include "bsp_svpwm_2shunt.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define ADC_SampleTime_XCycles5 ADC_SampleTime_7Cycles5        
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t ADC_RegularConvertedValueTab[30];

uint16_t usMC_PhaseAOffsetL = 0;
uint16_t usMC_PhaseBOffsetL = 0;
uint16_t usMC_PhaseAOffsetR = 0;
uint16_t usMC_PhaseBOffsetR = 0;

/* Private function prototypes -----------------------------------------------*/
void vRCC_Configuration(void);
void vGPIO_Configuration(void);
void vNVIC_Configuration(void);
void vBSP_SVPWM_2ShuntCurrentReadingCalibration(void);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
int32_t nBSP_SVPWM2ShuntInit(int nFrequency)
{
    
    /* -----------------------------------------------------------------------
    TIM1 Configuration to:

    1/ Generate 3 complementary PWM signals with 3 different duty cycles:
    TIM1CLK is fixed to SystemCoreClock, the TIM1 Prescaler is equal to 0 so the 
    TIM1 counter clock used is SystemCoreClock.
    * SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices. For Low-Density Value line and Medium-Density 
    Value line devices, SystemCoreClock is set to 24 MHz.

    The objective is to generate PWM signal at 17.57 KHz:
    - TIM1_Period = (SystemCoreClock / 17570) - 1

    The Three Duty cycles are computed as the following description: 

    The channel 1 duty cycle is set to 50% so channel 1N is set to 50%.
    The channel 2 duty cycle is set to 25% so channel 2N is set to 75%.
    The channel 3 duty cycle is set to 12.5% so channel 3N is set to 87.5%.
    The Timer pulse is calculated as follows:
      - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100

    2/ Insert a dead time equal to 11/SystemCoreClock ns
    3/ Configure the break feature, active at High level, and using the automatic 
     output enable feature
    4/ Use the Locking parameters level1. 
    ----------------------------------------------------------------------- */

    ADC_InitTypeDef           ADC_InitStructure;
    DMA_InitTypeDef           DMA_InitStructure;
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    TIM_OCInitTypeDef         TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

    vRCC_Configuration();
    vGPIO_Configuration();
    
    TIM_DeInit(TIM1);
    TIM_DeInit(TIM8);
		
    /* 最终频率的确认 72000000/(TIM_Prescaler+1)/(TIM_ClockDivision+1)/(TIM_Period+TIM_DeadTime)*/
        
    /* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 3;                                        //设置预分频值
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseStructure.TIM_Period = 0x1FF;//(SystemCoreClock / nFrequency) - 1;  d//设置在自动重装载周期值,
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;                         //设置时钟分割:TDTS = Tck_tim，// 采样分频
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;                                //重复寄存器，用于自动更新pwm占空比

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
		
    /* Channel 1, 2 and 3 Configuration in PWM mode */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                                //设置为pwm1输出模式
                                                                                    /*：PWM模式1－ 
                                                                                    在向上计数时，一旦TIMx_CNT<TIMx_CCR1时通道1为有效电平，否则为无效电平；
                                                                                    在向下计数时，一旦TIMx_CNT>TIMx_CCR1时通道1为无效电平(OC1REF=0)，
                                                                                    否则为有效电平(OC1REF=1)。
                                                                                    PWM模式2－ 
                                                                                    在向上计数时，一旦TIMx_CNT<TIMx_CCR1时通道1为无效电平，否则为有效电平；
                                                                                    在向下计数时，一旦TIMx_CNT>TIMx_CCR1时通道1为有效电平，否则为无效电平。
                                                                                    */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                   //使能该通道输出//比较输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;                 //使能互补端输出
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;                       //设置输出极性                                                                                       
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;                      //设置互补端输出极性    
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;                    //刹车后的输出状态    
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;                    //刹车后的互补端输出状态
    //设置通道1捕获比较寄存器的脉冲值
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    //设置通道2捕获比较寄存器的脉冲值
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    //设置通道3捕获比较寄存器的脉冲值
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
		
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                   //使能该通道输出//比较输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;                 //使能互补端输出
    
    //设置通道4捕获比较寄存器的脉冲值
    //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;     //PWM2在过中点之前 PWM1在过中点之后
    TIM_OCInitStructure.TIM_Pulse = 0x1FE;//(SystemCoreClock / nFrequency) - 2;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);

    //第五步，死区和刹车功能配置，高级定时器才有的，通用定时器不用配置TIM_BDTRInitStructure，共七项配置
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;                    //运行模式下输出选择
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;                    //空闲模式下输出选择    
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;                          //锁定设置
    TIM_BDTRInitStructure.TIM_DeadTime = 0x40;                                      //死区时间设置
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;                            //刹车功能使能
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;               //刹车输入极性 
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;         //自动输出使能 刹车后松开刹车 是否自动开启PWM

    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
    TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);
		
    //第六步，使能端的打开
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);               //使能TIMx在CCR1上的预装载寄存器，这句的功能是让改变CCR之后马上有效 ,立刻改变占空比
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);               //使能TIMx在CCR2上的预装载寄存器，这句的功能是让改变CCR之后马上有效 ,立刻改变占空比
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);               //使能TIMx在CCR3上的预装载寄存器，这句的功能是让改变CCR之后马上有效 ,立刻改变占空比
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);               //使能TIMx在CCR3上的预装载寄存器，这句的功能是让改变CCR之后马上有效 ,立刻改变占空比
    //TIM_ARRPreloadConfig(TIM1,ENABLE);                            //使能TIMx在ARR上的预装载寄存器?//立刻改变频率
		
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);               //使能TIMx在CCR1上的预装载寄存器，这句的功能是让改变CCR之后马上有效 ,立刻改变占空比
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);               //使能TIMx在CCR2上的预装载寄存器，这句的功能是让改变CCR之后马上有效 ,立刻改变占空比
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);               //使能TIMx在CCR3上的预装载寄存器，这句的功能是让改变CCR之后马上有效 ,立刻改变占空比
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);               //使能TIMx在CCR3上的预装载寄存器，这句的功能是让改变CCR之后马上有效 ,立刻改变占空比
    //TIM_ARRPreloadConfig(TIM1,ENABLE);                            //使能TIMx在ARR上的预装载寄存器?//立刻改变频率

    // ADC配置
    /* DMA1 Channel1 Configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_RegularConvertedValueTab;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 30;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
    ADC_TempSensorVrefintCmd(ENABLE);  
    
    /* ADC1 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;                  //
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //ADC_ExternalTrigConv_None
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 5;                             //规则通道数量
    ADC_Init(ADC1, &ADC_InitStructure);
    
    
    /* ADC2 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;                  //
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //ADC_ExternalTrigConv_None
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 0;                             //规则通道数量
    ADC_Init(ADC2, &ADC_InitStructure);
    
    /* ADC3 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;                  //
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //ADC_ExternalTrigConv_None
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 0;                             //规则通道数量
    ADC_Init(ADC3, &ADC_InitStructure);
    
    
    //page157 温度传感器和VREFINT只能出现在主ADC1中
    //==============ADC1规则通道配置==============//
    /* ADC1 regular channel14 configuration */ 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor,  1, ADC_SampleTime_239Cycles5);    
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint,     2, ADC_SampleTime_239Cycles5);    	
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10,          3, ADC_SampleTime_239Cycles5);    	//PC0 母线电压
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0,           4, ADC_SampleTime_239Cycles5);     	//PA0 左轴母线
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,           5, ADC_SampleTime_239Cycles5);     	//PA4 右轴母线
    
    //==============ADC1注入通道配置==============//
    /* Set injected sequencer length */
    ADC_InjectedSequencerLengthConfig(ADC1, 0);                                                 //注入通道数量    
    /* ADC1 injected external trigger configuration */
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);                 //从ADC模块用软件触发
    
    //==============ADC2注入通道配置==============//
    /* Set injected sequencer length */
    ADC_InjectedSequencerLengthConfig(ADC2, 3);                                                 //注入通道数量    
    /* ADC1 injected channel Configuration */ 
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_2,          1, ADC_SampleTime_XCycles5);        //PA2
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_13,         2, ADC_SampleTime_XCycles5);        //PC3
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_1,          3, ADC_SampleTime_XCycles5);        //PA1
    /* ADC1 injected external trigger configuration */
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_None);                 //从ADC模块用软件触发
    
    //==============ADC3注入通道配置==============//
    /* Set injected sequencer length */
    ADC_InjectedSequencerLengthConfig(ADC3, 3);                                                 //注入通道数量    
    /* ADC1 injected channel Configuration */ 
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_12,         1, ADC_SampleTime_XCycles5);        //PC2
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_3,          2, ADC_SampleTime_XCycles5);        //PA3
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_11,         3, ADC_SampleTime_XCycles5);        //PC1 IN9绿
    /* ADC1 injected external trigger configuration */
    ADC_ExternalTrigInjectedConvConfig(ADC3, ADC_ExternalTrigInjecConv_T1_TRGO);                 //从ADC模块用软件触发
    
    
    /* Enable automatic injected conversion start after regular one */
    //ADC_AutoInjectedConvCmd(ADC1, ENABLE);                                                    //自动注入 软件触发也算注入了

    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* Enable ADC1 external trigger */ 
    ADC_ExternalTrigConvCmd(ADC1, DISABLE);
    
    /* Enable ADC1 external Injected trigger */ 
    ADC_ExternalTrigInjectedConvCmd(ADC1, DISABLE); 
    
    
    
    /* Enable ADC2 external trigger */ 
    ADC_ExternalTrigConvCmd(ADC2, DISABLE);
    
    /* Enable ADC2 external Injected trigger */ 
    ADC_ExternalTrigInjectedConvCmd(ADC2, ENABLE); 
    
    /* Enable JEOC interrupt */
    ADC_ITConfig(ADC2, ADC_IT_JEOC, ENABLE);
    
    
    
    /* Enable ADC3 external trigger */ 
    ADC_ExternalTrigConvCmd(ADC3, ENABLE);
    
    /* Enable ADC3 external Injected trigger */ 
    ADC_ExternalTrigInjectedConvCmd(ADC3, ENABLE); 
    
    /* Enable JEOC interrupt */
    ADC_ITConfig(ADC3, ADC_IT_JEOC, ENABLE);

    //==============开启ADC1 校正ADC==============//
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* Enable ADC1 reset calibration register */   
    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));

    //==============开启ADC2 校正ADC==============//
    /* Enable ADC2 */
    ADC_Cmd(ADC2, ENABLE);

    /* Enable ADC2 reset calibration register */   
    ADC_ResetCalibration(ADC2);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC2));

    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC2);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC2));
    
    
    //==============开启ADC3 校正ADC==============//
    /* Enable ADC3 */
    ADC_Cmd(ADC3, ENABLE);

    /* Enable ADC2 reset calibration register */   
    ADC_ResetCalibration(ADC3);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC3));

    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC3);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC3));
    

    //==============开启ADC1_2规则转换==============//
    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));
     
    //==============================================//
    //vBSP_DelayMS(10);
    //vBSP_SVPWM_2ShuntCurrentReadingCalibration1();

    /* Start ADC1 Software Conversion */ 
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); 
    
    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
		
    // Resynch to have the Update evend during Undeflow
    //TIM_GenerateEvent(TIM1, TIM_EventSource_Update);                //主动发生UEV事件,UG=1
    
    /* Main Output Enable */
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    TIM_CtrlPWMOutputs(TIM8, DISABLE);
		
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC4Ref);               //触发时间输出配置
    TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_OC4Ref);               //触发时间输出配置
		
    /* Clear Update Flag */ 
    //TIM_ClearFlag(TIM1, TIM_FLAG_Update);

    //TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);

    //TIM_ITConfig(TIM1, TIM_IT_CC4,DISABLE);
    ADC_ClearITPendingBit(ADC1,ADC_IT_JEOC);
    ADC_ClearITPendingBit(ADC3,ADC_IT_JEOC);
    
    vNVIC_Configuration();
    
    return 0;
}   

/*******************************************************************************
* Function Name  : SVPWM_3ShuntCurrentReadingCalibration
* Description    : Store zero current converted values for current reading 
                   network offset compensation in case of 3 shunt resistors 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


void vBSP_SVPWM_2ShuntCurrentReadingCalibration(void)
{
#if 0
    uint8_t i ;
    //==============ADC1注入中断关闭==============//
    ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);
    
    //==============ADC1注入通道配置==============//
    /* Set injected sequencer length */
    ADC_InjectedSequencerLengthConfig(ADC1, 1);                         //注入通道数量    
    /* ADC1 injected channel Configuration */ 
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_XCycles5);    //PB0 IN8蓝 
    /* ADC1 injected external trigger configuration */
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);       //先用软件触发读取偏置量 
  
     //==============ADC2注入通道配置==============//
    /* Set injected sequencer length */
    ADC_InjectedSequencerLengthConfig(ADC2, 1);                         //注入通道数量    
    /* ADC1 injected channel Configuration */ 
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_15, 1, ADC_SampleTime_XCycles5);    //PB1 IN9绿
    /* ADC1 injected external trigger configuration */
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_None);       //从ADC模块用软件触发  

    
    for( i = 0 ; i < 16 ; i++ )
    {
        /* Clear the ADC1 JEOC pending flag */
        ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);  
        ADC_SoftwareStartInjectedConvCmd(ADC1,ENABLE);

        while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC)) { } 
        vBSP_DelayMS(1);
        usMC_PhaseAOffsetL += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1));
        usMC_PhaseBOffsetL += (ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_1));
            
    }
    usMC_PhaseAOffsetL>>=4;
    usMC_PhaseBOffsetL>>=4;
    
    //==============ADC1注入通道配置==============//
    /* Set injected sequencer length */
    ADC_InjectedSequencerLengthConfig(ADC1, 1);                         //注入通道数量    
    /* ADC1 injected channel Configuration */ 
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_XCycles5);    //PB0 IN8蓝 
    /* ADC1 injected external trigger configuration */
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);//ADC_ExternalTrigInjecConv_T1_TRGO 
  
     //==============ADC2注入通道配置==============//
    /* Set injected sequencer length */
    ADC_InjectedSequencerLengthConfig(ADC2, 1);                         //注入通道数量    
    /* ADC1 injected channel Configuration */ 
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_15, 1, ADC_SampleTime_XCycles5);    //PB1 IN9绿
    /* ADC1 injected external trigger configuration */
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_None);       //从ADC模块用软件触发  
    
    
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);  
#endif
}


/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void vRCC_Configuration(void)
{
    //配置时钟定时器时钟 

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | 
                           RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO, ENABLE);
     
    //配置ADC时钟
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
    /* ADCCLK = PCLK2/2 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div2);
#else
    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
#endif
    /* Enable peripheral clocks ------------------------------------------------*/
    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* Enable ADC1, ADC2 and GPIOC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3 |
                         RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void vGPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 

    //配置ADC引脚
    /* Configure PC.01, PC.02 and PC.04 (ADC Channel11, Channel12 and Channel14)
    as analog input ----------------------------------------------------------*/
    //
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //GL BL PC4 ANIN14 PC5 AIN15
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);    
    
    //SVPWM引脚初始化
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                                 //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    /* GPIOA Configuration: Channel 1, 2 and 3 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* GPIOB Configuration: Channel 1N, 2N and 3N as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* GPIOB Configuration: BKIN pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
		
    /* GPIOA Configuration: BKIN pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
}

/**
  * @brief  Configures NVIC and Vector Table base location.
  * @param  None
  * @retval None
  */
void vNVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* Enable the Update Interrupt 
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);*/

    /* Enable the TIM1 BRK Interrupt 
    NVIC_InitStruct.NVIC_IRQChannel = TIM1_BRK_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = BRK_PRE_EMPTION_PRIORITY;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = BRK_SUB_PRIORITY;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;		//Ã»ÓÐ´ò¿ª½ô¼±Í£³µ¹¦ÄÜ
    NVIC_Init(&NVIC_InitStruct);*/ 

    /* Configure and enable ADC interrupt */
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
#else
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
#endif
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = ADC3_IRQn;
    NVIC_Init(&NVIC_InitStructure);
} 

_iq vBSP_GetVBus_V(void)
{
    _iq VBus_pu = _IQ(0.0);
    int i = 0;
    for( i = 0 ; i < 8 ; i ++)
    {
        VBus_pu += (ADC_RegularConvertedValueTab[i*5+2]&0xffff0000) >> 12;  
    }
    VBus_pu = _IQdiv8(VBus_pu);
    VBus_pu = _IQmpy(VBus_pu,_IQ(42.9));
    return (VBus_pu);
}


_iq vBSP_GetVBusCurrent_A(void)
{
    _iq VBus_pu = _IQ(0.0);
    int i = 0;
    // (3.3 - vCurrent) * ( 22/23) * 11 * 4096 / 3.3 = adval 
    // (3.3 - ((3.3 - c * r ) * ( 22/23 )) )* 11 * 4096 / 3.3 = adval 
    
    
    //3.3 / (0.002r / 2 * 1A * 22 / 23 * 11)
    // 0.006r = 313.63/3 = 104.54554
    
    //满量程是627a 但0点在1.57V 所以最大只有318A左右
    for( i = 0 ; i < 16 ; i ++)
    {
        VBus_pu += (ADC_RegularConvertedValueTab[i*2+1]&0x0000ffff)<<4;  
    }
    VBus_pu = _IQdiv16(VBus_pu);
    VBus_pu = _IQmpy(VBus_pu,_IQ(104.54554));
    return (VBus_pu);
}

int32_t vBSP_BrakeEnableL(void)
{
    TIM1->CCER &= 0xeee;//使能该通道输出//比较输出使能 
    return 0;
}

int32_t vBSP_BrakeDisableL(void)
{
    TIM1->CCER |= 0x111;//使能该通道输出//比较输出使能 
    return 0;
}

int32_t vBSP_BrakeEnableR(void)
{
    TIM8->CCER &= 0xeee;//使能该通道输出//比较输出使能 
    return 0;
}

int32_t vBSP_BrakeDisableR(void)
{
    TIM8->CCER |= 0x111;//使能该通道输出//比较输出使能 
    return 0;
}
