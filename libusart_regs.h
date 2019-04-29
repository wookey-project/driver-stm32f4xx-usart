#ifndef STM32F4XX_USART_REGS_H
#define STM32F4XX_USART_REGS_H

#include "libc/types.h"
#include "libc/regutils.h"
#include "generated/usart1.h"
#include "generated/usart2.h"
#include "generated/usart3.h"
#include "generated/uart4.h"
#include "generated/uart5.h"
#include "generated/usart6.h"



#define PERIPH_BASE                         ((uint32_t) 0x40000000)
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)

/* Add some aliases for UART4 and UART 5 */
#define USART4_BASE	UART4_BASE
#define USART5_BASE	UART5_BASE
/* id 0 targets USART 1 (usart_config memset support) */
#define USART0_BASE	USART1_BASE

#define _r_CORTEX_M_USART_SR(n)		REG_ADDR(USART ## n ## _BASE + 0x00)
#define _r_CORTEX_M_USART_DR(n)		REG_ADDR(USART ## n ## _BASE + 0x04)
#define _r_CORTEX_M_USART_BRR(n)	REG_ADDR(USART ## n ## _BASE + 0x08)
#define _r_CORTEX_M_USART_CR1(n)	REG_ADDR(USART ## n ## _BASE + 0x0c)
#define _r_CORTEX_M_USART_CR2(n)	REG_ADDR(USART ## n ## _BASE + 0x10)
#define _r_CORTEX_M_USART_CR3(n)	REG_ADDR(USART ## n ## _BASE + 0x14)
#define _r_CORTEX_M_USART_GTPR(n)	REG_ADDR(USART ## n ## _BASE + 0x18)

#define USART_GET_REGISTER(reg)\
static inline volatile uint32_t* r_CORTEX_M_USART_##reg (uint8_t n){\
	switch(n){\
		case 1:\
			return _r_CORTEX_M_USART_##reg(1);\
			break;\
		case 2:\
			return _r_CORTEX_M_USART_##reg(2);\
			break;\
		case 3:\
			return _r_CORTEX_M_USART_##reg(3);\
			break;\
		case 4:\
			return _r_CORTEX_M_USART_##reg(4);\
			break;\
		case 5:\
			return _r_CORTEX_M_USART_##reg(5);\
			break;\
		case 6:\
			return _r_CORTEX_M_USART_##reg(6);\
			break;\
		default:\
			return NULL;\
	}\
}\

USART_GET_REGISTER(SR)
    USART_GET_REGISTER(DR)
    USART_GET_REGISTER(BRR)
    USART_GET_REGISTER(CR1)
    USART_GET_REGISTER(CR2)
    USART_GET_REGISTER(CR3)
    USART_GET_REGISTER(GTPR)
#endif                          /* STM32F4XX_USART_REGS_H */
