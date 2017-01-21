/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"

volatile uint32_t mscounter;
volatile bool toggleFlag = false;
extern bool updated;
extern uint8_t mode; //0 - normal, 1 - hours, 2 - minutes, 3 seconds
extern uint16_t ssTogle;
volatile uint16_t sample;
volatile uint8_t i;
volatile uint8_t i2;
volatile uint8_t * wavPtr;
volatile uint8_t * wavPtrBegin;
volatile uint8_t timerproc_counter;

extern uint8_t numChannels;
extern uint16_t sampleRate;
extern uint8_t bitsPerSample;

extern bool wcisniecie;

FIL     plik;
UINT bytesToRead,bytesRead;

extern volatile bool canRead;

//uint8_t buffer[2][512];
#ifdef SAMPLE_WIDTH_16
	extern int16_t buffer[2][SAMPLE_BUFFER_SIZE];
#else
	#ifdef STEREO
		extern uint16_t buffer[2][SAMPLE_BUFFER_SIZE];
	#else
		extern uint8_t buffer[2][SAMPLE_BUFFER_SIZE];
	#endif
#endif

void TIM1_UP_IRQHandler(void)
{

	static uint16_t buf_idx,lc,rc;
    static uint8_t lc8,rc8;

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)	{   //przeladowanie licznika
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);



			//GPIO_WriteBit(GPIOC, GPIO_Pin_15, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_15)));  	//LED1




			if(numChannels == 2 && bitsPerSample == 16){ //16 bit stereo
				lc = (uint16_t)(buffer[i][buf_idx] + 32768);

					buf_idx+=1;


					rc = (uint16_t)(buffer[i][buf_idx] + 32768);

					TIM4->CCR2 = lc>>6;
					TIM4->CCR1 = rc>>6;


			}

			if(numChannels == 2 && bitsPerSample == 8){ //8 bit stereo ok

								lc8 = (uint8_t) (buffer[i][buf_idx]>>8);

								rc8 = (uint8_t) (buffer[i][buf_idx] & 0x00FF);



								TIM4->CCR2 = lc8;
								TIM4->CCR1 = rc8;


			}

			if(numChannels == 1 && bitsPerSample == 16){
				lc = (uint16_t)(buffer[i][buf_idx] + 32768);
				TIM4->CCR2 = ((uint16_t)lc)>>6;
			}

			buf_idx+=1;




				if(buf_idx > SAMPLE_BUFFER_SIZE-1){
					buf_idx=0;
					canRead = true;
					i ^=0x01;
				}



	}
}





/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}





/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
