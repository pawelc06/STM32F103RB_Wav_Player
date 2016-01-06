#ifndef NEC_DECODE_H_
#define NEC_DECODE_H_

#include "stm32f10x_conf.h"
#include "ir_decode.h"


// This timing is in microseconds
#define T_AGC_PULSE			9000	// first "AGC" pulse
#define T_AGC_SPACE			4500	// space after "AGC" pulse

//#define	T_PULSE				560		// IR burst length
#define	T_PULSE				630		// IR burst length

//#define T_ZERO_SPACE		560		// space after IR burst during bit 0
#define T_ZERO_SPACE		508		// space after IR burst during bit 0

//#define T_ONE_SPACE			1690	// space after IR burst during bit 1
#define T_ONE_SPACE			1640	// space after IR burst during bit 1

#define T_TOLERANCE			0.25f	// 25% timing tolerance
#define EXTENDED			1		// extended:16 bit address no complement, not extended:8 bit address with complement

typedef struct
{
  uint16_t Address;    /*!< Address field */
  uint8_t Command;    /*!< Command field */

} NEC_FRAME;

extern void NEC_ReceiveInterrupt(NEC_FRAME f);

void NEC_Init(void);
void NEC_DeInit(void);
void NEC_TimingDecode(uint32_t th,uint32_t tl);
void NEC_PushBit(uint8_t bit);
uint32_t NEC_GetRawData();
void NEC_Reset();
void NEC_TimerRanOut();
void NEC_HandleEXTI();
uint16_t NEC_GetTime();
void NEC_StartTimer();
void NEC_StopTimer();

#endif
