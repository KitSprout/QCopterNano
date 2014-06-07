/*====================================================================================================*/
/*====================================================================================================*/
#include "stm32f4_system.h"
#include "stm32f4_usart.h"
#include "QCopterNano.h"
#include "QCopterNano_ctrl.h"
#include "QCopterNano_transport.h"
#include "module_rs232.h"
#include "module_nrf24l01.h"
#include "module_imu.h"
#include "module_mpu9250.h"
#include "algorithm_pid.h"
#include "algorithm_ahrs.h"
#include "algorithm_string.h"
/*====================================================================================================*/
/*====================================================================================================*/
FSM_MODE FSM_STATE = FSM_TXRX;
SEN_MODE SEN_STATE = SEN_CORR;
/*====================================================================================================*/
/*====================================================================================================*/
void System_Init( void )
{
  SystemInit();

  GPIO_Config();
  RS232_Config();
  Motor_Config();
  Sensor_Config();
  NRF24L01_Config();

  LED = 0;

  Sensor_Init();
  NRF24L01_Init(NRF_MODE_FTLR);

  PID_Init(&PID_Yaw);
  PID_Init(&PID_Roll);
  PID_Init(&PID_Pitch);

  PID_Pitch.Kp = +1.0f;
  PID_Pitch.Ki = +0.000f;
  PID_Pitch.Kd = +3.0f;

  PID_Roll.Kp  = +1.0f;
  PID_Roll.Ki  = +0.000f;
  PID_Roll.Kd  = +3.0f;

  PID_Yaw.Kp   = +0.0f;
  PID_Yaw.Ki   = +0.0f;
  PID_Yaw.Kd   = +0.0f;

  AHRS_Init(&NumQ, &AngE);

  /* Systick Setup */
  if(SysTick_Config((u32)(SystemCoreClock/(float)SampleRateFreg)))
    while(1);

  /* Wait Correct */
  while(SEN_STATE != SEN_ALG);
  LED = 1;
}
/*====================================================================================================*/
/*====================================================================================================*/
int main( void )
{
  u8 Status = 0;

  System_Init();
  Delay_100ms(5);

  /* QCopterNano FSM */
  while(1) {
//    LED = !LED;
    switch(FSM_STATE) {

      /************************** FSM TXRX ****************************************/
      case FSM_TXRX:
        // FSM_TXRX
        Transport_Send(TxBuf);
        do {
          Status = NRF_TxPacket(TxBuf);
        } while(Status == NRF_STA_MAX_RT);
        NRF_RX_Mode();
        Status = NRF_RxPacket(RxBuf);
        if(Status == NRF_STA_RX_DR) {
          Transport_Recv(RxBuf);
        }
        // FSM_TXRX End
        FSM_STATE = FSM_CTRL;
        break;

      /************************** FSM CTRL ****************************************/
      case FSM_CTRL:
        // FSM_CTRL
        Ctrl_BasicThr();
        // FSM_CTRL End
        FSM_STATE = FSM_UART;
        break;

      /************************** FSM UART ****************************************/
      case FSM_UART:
        // FSM_UART
        RS232_VisualScope(TxBuf+20);
        // FSM_UART End
        FSM_STATE = FSM_DATA;
        break;

      /************************** FSM DATA ****************************************/
      case FSM_DATA:
        // FSM_DATA
        
        // FSM_DATA End
        FSM_STATE = FSM_TXRX;
        break;

      /************************** FSM Err *****************************************/
      default:
        LED = 1;
        while(1) {
          LED = !LED;
          Delay_100ms(10);
        }
    }
  }
}
/*====================================================================================================*/
/*====================================================================================================*/
void GPIO_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* LED Clk Init *************************************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* LED PC13 */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  LED = 1;
}
/*====================================================================================================*/
/*====================================================================================================*/
