/**
 ******************************************************************************
 * @file    Project/STM32F10x_StdPeriph_Template/main.c
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

#include "main.h"
#include "waveplayer.h"

static FATFS g_sFatFs; //obiekt FATFs



bool next = false;
bool play = true;
bool last_state;
bool prev = false;

bool prevSubDir = false;
bool nextSubDir = false;

uint8_t numChange = 0;

volatile uint8_t volume;
volatile uint8_t motorSpeed;

void TIM2_IRQHandler() {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		NEC_TimerRanOut();
		//GPIO_ToggleBits(GPIOD, GPIO_Pin_7);ss

	}
}

void EXTI1_IRQHandler() {
	if (EXTI_GetITStatus(IR_EXTI_LINE) != RESET) {
		NEC_HandleEXTI();
	}
	EXTI_ClearITPendingBit(IR_EXTI_LINE);
}

void NEC_ReceiveInterrupt(NEC_FRAME f) {
	//GPIO_ToggleBits(GPIOD, GPIO_Pin_7);

	switch (f.Command) {
	case 64: //next
		next = true;
		//GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
		break;
	case 68: //prev

		prev = true;

		break;

	case 67: //play/stop

		play = !play;
		break;

	case 21: //volume up
		if (volume + 5 < 200)
			volume = volume + 5;
		//setResistance(1,volume);
		setResistance(3, volume);
		break;
	case 7: //volume down
		if (volume - 5 > 0)
			volume = volume - 5;
		//setResistance(1,volume);
		setResistance(3, volume);
		break;
	case 9: //EQ - light LED
		GPIO_WriteBit(GPIOB, GPIO_Pin_5,
				(BitAction) (1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5))); //D17
		break;
	case 12:  //number change
		numChange = 1;
		break;
	case 24:
		numChange = 2;
		break;
	case 94:
		numChange = 3;
		break;
	case 8:
		numChange = 4;
		break;
	case 28:
		numChange = 5;
		break;
	case 90:
		numChange = 6;
		break;
	case 66:
		numChange = 7;
		break;
	case 82:
		numChange = 8;
		break;
	case 74:
		numChange = 9;
		break;

	case 69:
		//CH-
		prevSubDir = true;
		break;
	case 71:
		//CH+
		nextSubDir = true;
		break;

	case 25:
		//FL-
		if(motorSpeed-2>0){
			motorSpeed-=2;
			PWM_SetDC(3,motorSpeed);
		}
		break;
	case 13:
		//FL+
		if(motorSpeed+2<100){
			motorSpeed+=2;
			PWM_SetDC(3,motorSpeed);
		}
		break;


	default:

		break;
	}

}

volatile int songNum;

int main(void) {

	FRESULT res;

	uint8_t j, k;
	uint8_t dNumber;
	char currentDirName[20];
	char fullCurrentDirName[25];

	if (SysTick_Config(720000ul)) { //10 ms
		while (1)
			;
	}


	RCC_Config();
	GPIO_Config();



	NVIC_Config();
	SPI_Config();

	last_state = play;
	NEC_Init();

	res = f_mount(&g_sFatFs, "0:0", 1);
	if (res != FR_OK)
		return -1;

	volume = 25;
	motorSpeed = 25;

	mcpInit();
	setResistance(1, volume);

	TIM3_Config(motorSpeed);


	/*
	PWM_SetDC(3,10);
	PWM_SetDC(3,20);
	PWM_SetDC(3,50);
	PWM_SetDC(3,80);
	*/

	//playWav("wav/sine_44k_16bit_mono.wav");

	dNumber = countDirs("/wav");

	k = getSubDirByNumber(0, "/wav", currentDirName);
	strcpy(fullCurrentDirName, "/wav/");
	strcat(fullCurrentDirName, currentDirName);

	j = countFilesInDirectory(fullCurrentDirName);



	for (songNum = 1; songNum <= j; songNum++) {

		playWavInDirectory(fullCurrentDirName, songNum);
		if (prev == true && songNum > 1) {
			prev = false;
			songNum -= 2;
		}

		if (nextSubDir) {
			if (++k < dNumber) {
				k = getSubDirByNumber(k, "/wav", currentDirName);
				strcpy(fullCurrentDirName, "/wav/");
				strcat(fullCurrentDirName, currentDirName);

			}
			numChange = 1;
			nextSubDir = false;
		}

		if (prevSubDir) {
			if (--k >= 0) {
				k = getSubDirByNumber(k, "/wav", currentDirName);
				strcpy(fullCurrentDirName, "/wav/");
				strcat(fullCurrentDirName, currentDirName);

			}
			numChange = 1;
			prevSubDir = false;
		}

		if (numChange != 0) {
			if (songNum <= j) {
				play = true;
				songNum = numChange;
			}
			numChange = 0;
		}

	}

	//playWav("wav/sine_22k_16bit_stereo.wav");
	//playWav("wav/sine_44k_16bit_mono.wav");
	//playWav("wav/sine_44k_8bit_stereo.wav");
	//playWav("wav/sine_44k_16bit_stereo.wav");

	/* Infinite loop */
	while (1) {
	}
}

void RCC_Config(void)
//konfigurowanie sygnalow taktujacych
{
	ErrorStatus HSEStartUpStatus;  //zmienna opisujaca rezultat uruchomienia HSE

	RCC_DeInit();	                                        //Reset ustawien RCC
	RCC_HSEConfig(RCC_HSE_ON);                             //Wlaczenie HSE
	HSEStartUpStatus = RCC_WaitForHSEStartUp();  //Odczekaj az HSE bedzie gotowy
	if (HSEStartUpStatus == SUCCESS) {
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);            //
		FLASH_SetLatency(FLASH_Latency_2); //ustaw zwloke dla pamieci Flash; zaleznie od taktowania rdzenia
										   //0:<24MHz; 1:24~48MHz; 2:>48MHz
		RCC_HCLKConfig(RCC_SYSCLK_Div1);                     //ustaw HCLK=SYSCLK
		RCC_PCLK2Config(RCC_HCLK_Div1);                      //ustaw PCLK2=HCLK
		RCC_PCLK1Config(RCC_HCLK_Div2);                     //ustaw PCLK1=HCLK/2
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); //ustaw PLLCLK = HSE*9 czyli 8MHz * 9 = 72 MHz
		RCC_PLLCmd(ENABLE);                                  //wlacz PLL
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
			;  //odczekaj na poprawne uruchomienie PLL
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //ustaw PLL jako zrodlo sygnalu zegarowego
		while (RCC_GetSYSCLKSource() != 0x08)
			;                //odczekaj az PLL bedzie sygnalem zegarowym systemu

		/*Tu nalezy umiescic kod zwiazany z konfiguracja sygnalow zegarowych potrzebnych w programie peryferiow*/
		RCC_APB2PeriphClockCmd(
				RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO
				| RCC_APB2Periph_GPIOC, ENABLE); //wlacz taktowanie portu GPIO B, C (LED, LCD)
														 //GPIO A i SPI inicjalizowane w sd_stm32.c
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //wlacz taktowanie licznika TIM4

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	} else {
	}
}

void NVIC_Config(void) {

	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	//Konfigurowanie kontrolera przerwan NVIC
#ifdef  VECT_TAB_RAM
	// Jezeli tablica wektorow w RAM, to ustaw jej adres na 0x20000000
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  // VECT_TAB_FLASH
	// W przeciwnym wypadku ustaw na 0x08000000
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	//przerwanie UP (przepelnienie) timera1
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void GPIO_Config(void) {
	//konfigurowanie portow GPIO
	GPIO_InitTypeDef GPIO_InitStructure;

	/*Tu nalezy umiescic kod zwiazany z konfiguracja poszczegolnych portow GPIO potrzebnych w programie*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11
			| GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PA4 - nSS/CS SPI1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//SPI1 - SCK, MISO, MOSI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//linia DETECT zlacza SD - nieu¿ywana
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*** SPI2 ******/
	//SPI1 - SCK, MISO, MOSI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PB12 - nSS/CS SPI2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//pwm pin timer 4 channel 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//pwm pin timer 4 channel 1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//pwm pin timer 3 channel 3
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void SPI_Config(void) {
	//konfigurowanie interfejsu SPI
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitTypeDef SPI_InitStructure2;

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//transmisja z wykorzystaniem jednej linii, transmisja jednokierunkowa
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;               //tryb pracy SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;       //8-bit ramka danych
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //stan sygnalu taktujacego przy braku transmisji - wysoki
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //aktywne zbocze sygnalu taktujacego - 2-gie zbocze
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //programowa obsluga linii NSS (CS)
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //prescaler szybkosci tansmisji  72MHz/4=18MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //pierwszy bit w danych najbardziej znaczacy
	SPI_InitStructure.SPI_CRCPolynomial = 7; //stopien wielomianu do obliczania sumy CRC
	SPI_Init(SPI1, &SPI_InitStructure);                      //inicjalizacja SPI

	SPI_Cmd(SPI1, ENABLE);  	// Wlacz SPI1

	/*******************************************/

	SPI_InitStructure2.SPI_Direction = SPI_Direction_1Line_Tx; //transmisja z wykorzystaniem jednej linii, transmisja jednokierunkowa
	SPI_InitStructure2.SPI_Mode = SPI_Mode_Master;              //tryb pracy SPI
	SPI_InitStructure2.SPI_DataSize = SPI_DataSize_16b;    //16-bit ramka danych
	SPI_InitStructure2.SPI_CPOL = SPI_CPOL_High; //stan sygnalu taktujacego przy braku transmisji - wysoki
	SPI_InitStructure2.SPI_CPHA = SPI_CPHA_2Edge; //aktywne zbocze sygnalu taktujacego - 2-gie zbocze
	SPI_InitStructure2.SPI_NSS = SPI_NSS_Soft; //programowa obsluga linii NSS (CS)
	SPI_InitStructure2.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; //prescaler szybkosci tansmisji  72MHz/8=9MHz
	SPI_InitStructure2.SPI_FirstBit = SPI_FirstBit_MSB; //pierwszy bit w danych najbardziej znaczacy
	SPI_InitStructure2.SPI_CRCPolynomial = 7; //stopien wielomianu do obliczania sumy CRC
	SPI_Init(SPI2, &SPI_InitStructure2);                     //inicjalizacja SPI

	SPI_Cmd(SPI2, ENABLE);  	// Wlacz SPI2
	/********************************************/

}

TIM3_Config(uint8_t initPulse){
	/********** TIM3 CH3 - PWM for motor control ****/
	TIM_OCInitTypeDef TIM_OCInitStructure;

		uint16_t PrescalerValue = 0;
		  /* Compute the prescaler value */
		  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;
		  /* Time base configuration */
		  TIM_TimeBaseStructure.TIM_Period = 1000;
		  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
		  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		  /* PWM1 Mode configuration: Channel3 */
		  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		    TIM_OCInitStructure.TIM_Pulse = initPulse*10;
		    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
		    TIM_Cmd(TIM3, ENABLE);
}

void TIM_Config(uint16_t sampleRate, uint8_t bits, uint8_t numChannels) {
	//Konfiguracja timerow
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructureT1;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructureT3;

	TIM_OCInitTypeDef TIM_OCInitStructure;

	//Konfiguracja licznika 1 - fsamp 44k lub 22k
	//Ustawienia taktowania i trybu pracy licznika 1

	switch (sampleRate) {
	case 22050:
		TIM_TimeBaseStructureT1.TIM_Prescaler = 72 - 1;
		break;
	case 44100:
		TIM_TimeBaseStructureT1.TIM_Prescaler = 36 - 1;
		break;
	default:
		break;
	}

	// TIM_TimeBaseStructureT1.TIM_Prescaler = 36-1;                //taktowanie licznka fclk = 72MHz/36 = 2MHz

	TIM_TimeBaseStructureT1.TIM_Period = 45; //45 mikrosekund , jenoostka mikrosekunda
	TIM_TimeBaseStructureT1.TIM_ClockDivision = TIM_CKD_DIV1; //dzielnik zegara dla ukladu generacji dead-time i filtra
	TIM_TimeBaseStructureT1.TIM_RepetitionCounter = 0;       //licznik powtorzen
	TIM_TimeBaseStructureT1.TIM_CounterMode = TIM_CounterMode_Up; //tryb pracy licznika

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructureT1);   //Inicjalizacja licznika

	// Konfiguracja TIM4
	// Ustawienia ukladu podstawy czasu
	TIM_TimeBaseStructure.TIM_Prescaler = 0;      //bez prescelera (prescaler=1)

	if (bits == 16) {
		TIM_TimeBaseStructure.TIM_Period = 1023ul; //konwersja z 16 na 10 bitów
	} else { // 8 bitów - pe³na 16 bitowa rozdzielczosc pwm
		TIM_TimeBaseStructure.TIM_Period = 255ul;
	}

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	//channel 2 PB7 - TIM4_CH2
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	if (bits == 16) {
		TIM_OCInitStructure.TIM_Pulse = 512ul;      //wypelnienie = 512/1023=50%
	} else {
		TIM_OCInitStructure.TIM_Pulse = 127ul;      //wypelnienie = 512/1023=50%
	}
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//channel2
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);

	if (numChannels == 2) {
		//channel1
		TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	}

	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);    //wlaczenie buforowania

	// Wlaczenie przerwan od licznikow
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //wlaczenie przerwania od przepelnienia


}

void PWM_SetDC(uint16_t channel,uint16_t dutycycle)
{
  if (channel == 1)
  {
    TIM3->CCR1 = dutycycle;
  }
  else if (channel == 2)
  {
    TIM3->CCR2 = dutycycle;
  }
  else if (channel == 3)
  {
    TIM3->CCR3 = dutycycle*10;
  }
  else
  {
    TIM3->CCR4 = dutycycle;
  }
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
