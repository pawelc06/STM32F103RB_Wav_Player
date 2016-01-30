/*
 * waveplayer.c
 *
 *  Created on: 18 lip 2014
 *      Author: Paweł
 */

//#include "main.h"
#include "waveplayer.h"
#include "string.h"
//#include "clock.h"
#include "stdbool.h"

#include "./../FATFs_0.10/src/ff.h"
#include "./../FATFs_0.10/src/diskio.h"

#define STEREO 1
#define SAMPLE_WIDTH_16 1
//#define WAV_FROM_INT_MEMORY 1
//#define SAMPLE_BUFFER_SIZE 512


extern bool prev,next,play,last_state;
extern uint8_t numChange;

volatile uint8_t numChannels;

volatile uint8_t bitsPerSample;


int16_t buffer[2][SAMPLE_BUFFER_SIZE];


extern FIL plik;
volatile bool canRead;
extern uint8_t i;





/**
 * @brief  PrecConfiguration: configure PA4 and PA5 in analog,
 *                           enable DAC clock, enable DMA1 clock
 * @param  None
 * @retval None
 */


void convertBufferTo10bit(int16_t * buffer) {
	uint16_t i;
	for (i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
		buffer[i] = (buffer[i] + 32768) >> 6;
	}
}


void playStereoSine12b(){

	const uint16_t Sine12bit2[32] = {
	                      2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056,
	                      3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909,
	                      599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647};

	uint32_t Idx1 = 0;

	numChannels = 2;
	/* Preconfiguration before using DAC----------------------------------------*/


	  /* Fill Sine32bit table */
	  for (Idx1 = 0; Idx1 < 32; Idx1++)
	  {
	    DualSine12bit[Idx1] = (Sine12bit2[Idx1] << 16) + (Sine12bit2[Idx1]);
	  }




	  while (1)
	  {
	  }
}

void playStereoSaw8b(){

	uint16_t Saw8bit2[512];

	uint16_t Idx1 = 0;

	numChannels = 2;


	  /* Fill Sine32bit table */
	  for (Idx1 = 0; Idx1 < 510; Idx1++)
	  {
		  Saw8bit2[Idx1] = ((Idx1%256) << 8) + (Idx1%256);

	  }




	  while (1)
	  {
	  }
}



void playWav(char * name) {

	FRESULT fresult;

	UINT bytesToRead, bytesRead;

	NVIC_InitTypeDef nvicStructure;
	uint16_t sampleRate;


	fresult = f_open(&plik, name, FA_READ);

		bytesToRead = f_size(&plik);

		fresult = f_lseek(&plik, 22);

		f_read(&plik, &numChannels, 2, &bytesRead);
		f_read(&plik, &sampleRate, 2, &bytesRead);

		fresult = f_lseek(&plik, 34);

		f_read(&plik, &bitsPerSample, 2, &bytesRead);



		//fresult = f_lseek(&plik, 44);
		fresult = f_lseek(&plik, 0);


		fresult = f_read(&plik, &buffer[0][0], SAMPLE_BUFFER_SIZE*2, &bytesRead);
		//convertBufferTo10bit(&buffer[0][0]);

		fresult = f_read(&plik, &buffer[1][0], SAMPLE_BUFFER_SIZE*2, &bytesRead);
		//convertBufferTo10bit(&buffer[1][0]);



		  TIM_Config(sampleRate,bitsPerSample,numChannels);

		  i=0;

		  while(!play); //waiting for play

		  startTimers();


	while (1) {

			if(play != last_state){
				if (play == false){

					stopTimers();
				} else {
					startTimers();
				}
				last_state = play;
			}

			if ((canRead == true) && (play == true)) {


					//GPIO_WriteBit(GPIOC, GPIO_Pin_15, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_15)));  	//LED1
					f_read(&plik, &buffer[i ^ 0x01][0], SAMPLE_BUFFER_SIZE*2 ,	&bytesRead);
					//GPIO_WriteBit(GPIOC, GPIO_Pin_15, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_15)));  	//LED1

					//convertBufferTo10bit(&buffer[i ^ 0x01][0]);

					canRead = false;
					if ((next==true) || (bytesRead < SAMPLE_BUFFER_SIZE*2)){
						next = false;
						break;
					}

					if ((prev==true) || (numChange!=0) ){
					//if ((prev==true)  ){
						break;
					}





			}

		}


	stopTimers(); //end of song



	fresult = f_close(&plik);

}

void stopTimers(){
	// Wylaczenie timerow
		TIM_Cmd(TIM1, DISABLE);

		TIM_Cmd(TIM4, DISABLE);
}

void startTimers(){
	// Wylaczenie timerow
		TIM_Cmd(TIM1, ENABLE);

		TIM_Cmd(TIM4, ENABLE);
}

uint8_t countFilesInDirectory(char *dirName){
	FRESULT res;
	FILINFO fno;
	DIR dir;


		char *fn;   /* This function assumes non-Unicode configuration */
		int i;


		i=0;
	#if _USE_LFN
    static char lfn[_MAX_LFN + 1];   /* Buffer to store the LFN */
    fno.lfname = lfn;
    fno.lfsize = sizeof lfn;
#endif

    res = f_chdir(dirName);
    if(res != FR_OK)
    	return -10;

    res = f_opendir(&dir, ".");                       /* Open the directory */
    if (res == FR_OK) {


        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            //res = f_chdir("/wav/arka/");
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                //sprintf(&path[i], "/%s", fn);
                //res = scan_files(path);

                if (res != FR_OK) break;
            } else {                                       /* It is a file. */
            	i++;
            }
        }
        f_closedir(&dir);
    } else {
    	//error
    	return -1;
    }
    return i;
}

int playWavInDirectory(char *dirName,uint8_t number){
	FRESULT res;
	FILINFO fno;
	DIR dir;


		char *fn;   /* This function assumes non-Unicode configuration */
		int i;


		i=0;
	#if _USE_LFN
    static char lfn[_MAX_LFN + 1];   /* Buffer to store the LFN */
    fno.lfname = lfn;
    fno.lfsize = sizeof lfn;
#endif

    res = f_chdir(dirName);
    if(res != FR_OK)
    	return -10;

    res = f_opendir(&dir, ".");                       /* Open the directory */
    if (res == FR_OK) {


        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            //res = f_chdir("/wav/arka/");
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                //sprintf(&path[i], "/%s", fn);
                //res = scan_files(path);

                if (res != FR_OK) break;
            } else {                                       /* It is a file. */
            	i++;

            	if(i==number){
            		playWav(fn);
            		f_closedir(&dir);
            		return 0;
            	}


            }
        }
        f_closedir(&dir);
    } else {
    	//error
    	return -1;
    }
    return 0;
}
