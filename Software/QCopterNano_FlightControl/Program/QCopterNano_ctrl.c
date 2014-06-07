/*====================================================================================================*/
/*====================================================================================================*/
#include "stm32f4_system.h"
#include "QCopterNano_ctrl.h"
#include "QCopterNano_transport.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define PWM1  TIM4->CCR4
#define PWM2  TIM4->CCR1
#define PWM3  TIM4->CCR2
#define PWM4  TIM4->CCR3
/*====================================================================================================*/
/*====================================================================================================*/
u16 BasicThr = 0;  // 0 ~ 10000 對應到 0.00% ~ 100.00%
/*=====================================================================================================*/
/*=====================================================================================================*
**函數 : Motor_Config
**功能 : 配置馬達
**輸入 : None
**輸出 : None
**使用 : Motor_Config();
**=====================================================================================================*/
/*=====================================================================================================*/
void Motor_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

  /* TIM4 PWM1 PB6 */ /* TIM4 PWM2 PB7 */ /* TIM4 PWM3 PB8 */ /* TIM4 PWM4 PB9 */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  TIM_DeInit(TIM4);

  /************************** PWM Output **************************************/
  /* TIM4 Time Base */
  TIM_TimeBaseStruct.TIM_Period = (u16)(MOTOR_PWM_RANGE-1);   // Period = 11.9us => Freq = 400Hz
  TIM_TimeBaseStruct.TIM_Prescaler = (u16)(21-1);             // fCK_PSC = APB1*2 = 42*2 = 84, fCK_PSC /21 = 4M ( 0.25us )
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;    // Count Up
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);

  /* TIM4 OC */
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;              // Set PWM1 Mode
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;  // Enable OC
  TIM_OCInitStruct.TIM_Pulse = MOTOR_PWM_MIN;                 // Set Pulse
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;      // when Count < PWM_MOTOR_MIN => High
  TIM_OC1Init(TIM4, &TIM_OCInitStruct);                       // Init TIM4 OC1
  TIM_OC2Init(TIM4, &TIM_OCInitStruct);                       // Init TIM4 OC2
  TIM_OC3Init(TIM4, &TIM_OCInitStruct);                       // Init TIM4 OC3
  TIM_OC4Init(TIM4, &TIM_OCInitStruct);                       // Init TIM4 OC4
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);           // Enable TIM4 OC1 Preload
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);           // Enable TIM4 OC2 Preload
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);           // Enable TIM4 OC3 Preload
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);           // Enable TIM4 OC4 Preload

  TIM4->CCR1 = MOTOR_PWM_MIN;
  TIM4->CCR2 = MOTOR_PWM_MIN;
  TIM4->CCR3 = MOTOR_PWM_MIN;
  TIM4->CCR4 = MOTOR_PWM_MIN;

  /* Enable */
  TIM_ARRPreloadConfig(TIM4, ENABLE);                         // Enable TIM4 ARR Preload
  TIM_Cmd(TIM4, ENABLE);                                      // Start TIM4
}
/*=====================================================================================================*/
/*=====================================================================================================*
**函數 : Ctrl_MotorPWM
**功能 : 馬達脈波控制
**輸入 : MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4
**輸出 : None
**使用 : Ctrl_MotorPWM(MOTOR[0], MOTOR[1], MOTOR[2], MOTOR[3]);
**=====================================================================================================*/
/*=====================================================================================================*/
void Ctrl_MotorPWM( s16 MOTOR_1, s16 MOTOR_2, s16 MOTOR_3, s16 MOTOR_4 )
{
  if(MOTOR_1>MOTOR_PWM_MAX)       MOTOR_1 = MOTOR_PWM_MAX;
  else if(MOTOR_1<MOTOR_PWM_MIN)  MOTOR_1 = MOTOR_PWM_MIN;
  if(MOTOR_2>MOTOR_PWM_MAX)       MOTOR_2 = MOTOR_PWM_MAX;
  else if(MOTOR_2<MOTOR_PWM_MIN)  MOTOR_2 = MOTOR_PWM_MIN;
  if(MOTOR_3>MOTOR_PWM_MAX)       MOTOR_3 = MOTOR_PWM_MAX;
  else if(MOTOR_3<MOTOR_PWM_MIN)  MOTOR_3 = MOTOR_PWM_MIN;
  if(MOTOR_4>MOTOR_PWM_MAX)       MOTOR_4 = MOTOR_PWM_MAX;
  else if(MOTOR_4<MOTOR_PWM_MIN)  MOTOR_4 = MOTOR_PWM_MIN;

  PWM1 = MOTOR_1;
  PWM2 = MOTOR_2;
  PWM3 = MOTOR_3;
  PWM4 = MOTOR_4;
}
/*=====================================================================================================*/
/*=====================================================================================================*
**函數 : Ctrl_MotorTHR
**功能 : 馬達油門控制
**輸入 : MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4
**輸出 : None
**使用 : Ctrl_MotorTHR(MOTOR[0], MOTOR[1], MOTOR[2], MOTOR[3]);
**=====================================================================================================*/
/*=====================================================================================================*/
void Ctrl_MotorTHR( s16 MOTOR_1, s16 MOTOR_2, s16 MOTOR_3, s16 MOTOR_4 )
{
  if(MOTOR_1>MOTOR_THR_MAX)             MOTOR_1 = MOTOR_THR_MAX;
  else if(MOTOR_1<(MOTOR_THR_MIN+500))  MOTOR_1 = MOTOR_THR_MIN;
  if(MOTOR_2>MOTOR_THR_MAX)             MOTOR_2 = MOTOR_THR_MAX;
  else if(MOTOR_2<(MOTOR_THR_MIN+500))  MOTOR_2 = MOTOR_THR_MIN;
  if(MOTOR_3>MOTOR_THR_MAX)             MOTOR_3 = MOTOR_THR_MAX;
  else if(MOTOR_3<(MOTOR_THR_MIN+500))  MOTOR_3 = MOTOR_THR_MIN;
  if(MOTOR_4>MOTOR_THR_MAX)             MOTOR_4 = MOTOR_THR_MAX;
  else if(MOTOR_4<(MOTOR_THR_MIN+500))  MOTOR_4 = MOTOR_THR_MIN;

  PWM1 = (u16)(MOTOR_1>>2);
  PWM2 = (u16)(MOTOR_2>>2);
  PWM3 = (u16)(MOTOR_3>>2);
  PWM4 = (u16)(MOTOR_4>>2);
}
/*=====================================================================================================*/
/*=====================================================================================================*
**函數 : CTRL_SERVO
**功能 : 伺服馬達控制
**輸入 : SevroA, SevroB
**輸出 : None
**使用 : CTRL_SERVO( SevroA, SevroB );
**=====================================================================================================*/
/*=====================================================================================================*/
//void CTRL_SERVO( void )
//{

//}
/*=====================================================================================================*/
/*=====================================================================================================*
**函數 : Ctrl_BasicThr
**功能 : 飛行器基本油門控制
**輸入 : None
**輸出 : None
**使用 : Ctrl_BasicThr();
**=====================================================================================================*/
/*=====================================================================================================*/
void Ctrl_BasicThr( void )
{
  s16 TempThr = 0;
  static s16 TempThrB = 0;

  // 粗調油門
  if(KEY_LP == 1) {
    TempThrB = (s16)EXP_THR_B;
    if(TempThrB>MOTOR_THR_MAX)
      TempThrB = MOTOR_THR_MAX;
    if(TempThrB<MOTOR_THR_MIN)
      TempThrB = MOTOR_THR_MIN;
  }

  // 細調油門
  TempThr = ((s16)TempThrB + (s16)EXP_THR_S);
  if(TempThr>MOTOR_THR_MAX)
    TempThr = MOTOR_THR_MAX;
  if(TempThr<MOTOR_THR_MIN)
    TempThr = MOTOR_THR_MIN;

  if(TempThr<MOTOR_THR_MIN)
    BasicThr = MOTOR_THR_MIN;
  else
    BasicThr = TempThr;
}
/*====================================================================================================*/
/*====================================================================================================*/
