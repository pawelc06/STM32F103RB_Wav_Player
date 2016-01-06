#ifndef __IR_DECODE_H
#define __IR_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x_conf.h"
#include <stdio.h>
#include "nec_decode.h"

#define IR_GPIO_PORT            GPIOA                    /*!< Port which IR output is connected */
#define IR_GPIO_PORT_CLK        RCC_APB2Periph_GPIOA      /*!< IR pin GPIO Clock Port */
#define IR_GPIO_PIN             GPIO_Pin_1               /*!< Pin which IR is connected */
#define IR_EXTI_LINE			EXTI_Line1
#define IR_NVIC_IRQn			EXTI1_IRQn
#define IR_EXTI_PORT_SOURCE		GPIO_PortSourceGPIOA
#define IR_EXTI_PIN_SOURCE		GPIO_PinSource1

#ifdef __cplusplus
}
#endif

#endif
