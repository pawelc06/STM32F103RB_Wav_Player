/*
 * waveplayer.h
 *
 *  Created on: 18 lip 2014
 *      Author: Pawe³
 */

#ifndef WAVEPLAYER_H_
#define WAVEPLAYER_H_

#include "main.h"

//#define SAMPLING_FREQ 11025
//#define SAMPLING_FREQ 22050
//#define SAMPLING_FREQ 44100
//#define DMA_TIMER_VAL 1451 //for 22050
//#define DMA_TIMER_VAL (1000.0/SAMPLING_FREQ)/31.25E-06



void convertBufferTo10bit(int16_t * buffer);

void playWav(char *name);
void playWavFromMemory(uint8_t * memBuffer);
void playWavFromIntMemory(const uint8_t * memBuffer);
void playStereoSine12b();
void playStereoSaw8b();
void playStereoWav8b(uint8_t * name);

void stopTimers(void);
void startTimers(void);
int playWavInDirectory(char *dirName,uint8_t number);
uint8_t countFilesInDirectory(char *dirName);


uint32_t DualSine12bit[32];

//#define DAC_DHR12RD_Address      0x40007420

/* Init Structure definition */
DAC_InitTypeDef            DAC_InitStructure;
DMA_InitTypeDef            DMA_InitStructure;
TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
//uint32_t Idx1 = 0;

#endif /* WAVEPLAYER_H_ */


