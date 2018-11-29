#include "api/print.h"
#include "api/libusart.h"
#include "api/libusart_regs.h"
#include "api/syscall.h"
#include "kernel/exported/devices.h"
#include "kernel/exported/gpio.h"

#define PROD_CLOCK_APB1  42000000
#define PROD_CLOCK_APB2  84000000

device_t usart_dev;
int      usart_desc = 0;


// TODO: differenciate tx_port and rx_port that may differs
static const struct {
  char *   name;
  uint8_t  port;
  uint8_t  tx_pin;
  uint8_t  rx_pin;
  uint8_t  sc_port;
  uint8_t  sc_tx_pin;
  uint8_t  sc_ck_pin;
  uint8_t  irq;
  uint8_t  af;
} usarts[] = {
 { "",       0,    0,  0,   0,  0,  0, 53, 0x7 },
 { "usart1", GPIO_PB,  6,  7, GPIO_PA,  9,  8, 53, 0x7 },
 { "usart2", GPIO_PA,  2,  3, GPIO_PA,  2,  4, 54, 0x7 },
 { "usart3", GPIO_PB, 10, 11, GPIO_PB, 10, 12, 55, 0x7 },
 { "uart4",  GPIO_PA,  0,  1,   0,  0,  0, 68, 0x8 },
 { "uart5",  GPIO_PC, 12,  2,   0,  0,  0, 69, 0x8 },
 { "usart6", GPIO_PC,  6,  7, GPIO_PC,  6,  8, 89, 0x8 }
};

/**** USART basic Read / Write ****/
void usart_putc(uint8_t usart, char c)
{
    /* Wait for TX to be ready */
    while (!get_reg(r_CORTEX_M_USART_SR(usart), USART_SR_TXE))
        continue;
    *r_CORTEX_M_USART_DR(usart) = c;
}

/* Instantiate the putc for each USART */
#define USART_PUTC_CALLBACK(num)\
void usart##num##_putc(char c)\
{\
	usart_putc(num, c);\
}\

USART_PUTC_CALLBACK(1)
    USART_PUTC_CALLBACK(2)
    USART_PUTC_CALLBACK(3)
    USART_PUTC_CALLBACK(4)
    USART_PUTC_CALLBACK(5)
    USART_PUTC_CALLBACK(6)

void usart_write(uint8_t usart, char *msg, uint32_t len)
{
    while (len--) {
        usart_putc(usart, *msg);
        msg++;
    }
}

char usart_getc(uint8_t usart)
{
    while (!get_reg(r_CORTEX_M_USART_SR(usart), USART_SR_RXNE))
        continue;
    return *r_CORTEX_M_USART_DR(usart);
}

/* Instantiate the getc for each USART */
#define USART_GETC_CALLBACK(num)\
char usart##num##_getc(void)\
{\
	return usart_getc(num);\
}\

USART_GETC_CALLBACK(1)
USART_GETC_CALLBACK(2)
USART_GETC_CALLBACK(3)
USART_GETC_CALLBACK(4)
USART_GETC_CALLBACK(5)
USART_GETC_CALLBACK(6)

uint32_t usart_read(uint8_t usart, char *buf, uint32_t len)
{
    uint32_t start_len = len;
    while (len--) {
        *buf = usart_getc(usart);
        if (*buf == '\n')
            break;
        buf++;
    }
    return start_len - len;
}

/**** DMA ****/
void usart_disable_dma(uint8_t usart)
{
    clear_reg_bits(r_CORTEX_M_USART_CR3(usart), USART_CR3_DMAT_Msk);
}

void usart_enable_dma(uint8_t usart)
{
    set_reg_bits(r_CORTEX_M_USART_CR3(usart), USART_CR3_DMAT_Msk);
}

volatile uint32_t *usart_get_data_addr(uint8_t usart)
{
    return r_CORTEX_M_USART_DR(usart);
}

void usart_set_baudrate(usart_config_t * config)
{
    uint32_t divider = 0;
    uint16_t mantissa = 0;
    uint8_t fraction = 0;

    /* FIXME we should check CR1 in order to get the OVER8 configuration */

    /* Compute the divider using the baudrate and the APB bus clock 
     * (APB1 or APB2) depending on the considered USART */
    divider = usart_get_bus_clock(config) / config->baudrate;

    mantissa = (uint16_t) divider >> 4;
    fraction = (uint8_t) ((divider - mantissa * 16));
    write_reg_value(r_CORTEX_M_USART_BRR(config->usart),
                    (((mantissa & 0x0fff) << 4) | (0x0f & fraction)));
}


/* This is a template configuration for the USART in UART mode */


/**** IRQ Handlers ****/
#define USART_IRQHANDLER(num, type) \
/* Global variable holding the callback to USART num */\
cb_usart_irq_handler_t cb_usart##num##_irq_handler = NULL;\
/* Register the IRQ */\
void U##type##ART##num##_IRQHandler(uint8_t irq __attribute__((unused)),\
                                    uint32_t status,\
                                    uint32_t data)\
{\
	if(cb_usart##num##_irq_handler != NULL){\
		cb_usart##num##_irq_handler(status, data);\
	}\
}\

/* Instantiate the IRQs for the 6 USARTs
 * The weird second macro argument handles the fact that USART 4 and 5 are in
 * UARTs.
 */
USART_IRQHANDLER(1, S)
USART_IRQHANDLER(2, S)
USART_IRQHANDLER(3, S)
USART_IRQHANDLER(4,)
USART_IRQHANDLER(5,)
USART_IRQHANDLER(6, S)

/* Configure the handlers */
#define USART_CONFIG_CALLBACKS(num, type) \
		if (config->callback_irq_handler != NULL){\
			/* Enable dedicated IRQ and register the callback */\
			cb_usart##num##_irq_handler = config->callback_irq_handler;\
		}\
		if(config->callback_usart_getc_ptr != NULL){\
			*(config->callback_usart_getc_ptr) = usart##num##_getc;\
		}\
		if(config->callback_usart_putc_ptr != NULL){\
			*(config->callback_usart_putc_ptr) = usart##num##_putc;\
		}

static void usart_callbacks_init(usart_config_t * config)
{
    // INFO: activating IRQ for RX/TX is under the responsability of the upper
    // layer
    switch (config->usart) {
       case 1:
          USART_CONFIG_CALLBACKS(1, S)
          break;
       case 2:
          USART_CONFIG_CALLBACKS(2, S)
          break;
       case 3:
          USART_CONFIG_CALLBACKS(3, S)
          break;
       case 4:
          USART_CONFIG_CALLBACKS(4,)
          break;
       case 5:
          USART_CONFIG_CALLBACKS(5,)
          break;
       case 6:
          USART_CONFIG_CALLBACKS(6, S)
          break;
    default:
        printf("Wrong usart %d. You should use USART1 to USART6", config->usart);
    }

    return;
}

#define GET_USART_ADDR(n) USART##n##_BASE
#define GET_USART_IRQ_HANDLER(n, type) U##type##ART##n##_IRQHandler \

/*
 * usart_enable - Enable the specific USART
 */
void usart_enable(usart_config_t *config)
{
	set_reg_bits(r_CORTEX_M_USART_CR1(config->usart), USART_CR1_UE_Msk);

	return;
}


/*
 * usart_disable - Disable the specific USART
 */
void usart_disable(usart_config_t *config)
{
	clear_reg_bits(r_CORTEX_M_USART_CR1(config->usart), USART_CR1_UE_Msk);

	return;
}


/*
 * To be executed *after* INIT_DONE. the USART must be mapped. 
 */
uint8_t usart_init(usart_config_t *config)
{
    usart_disable(config);
    usart_set_baudrate(config);
	usart_enable(config);

    /* Control register 1 */
    if (config->set_mask & USART_SET_PARITY) {
      set_reg(r_CORTEX_M_USART_CR1(config->usart), config->parity,
              USART_CONFIG_PARITY);
    }

    if (config->set_mask & USART_SET_WORD_LENGTH) {
      set_reg(r_CORTEX_M_USART_CR1(config->usart), config->word_length,
              USART_CONFIG_WORD_LENGTH_BITS);
    }

    if (config->set_mask & USART_SET_OPTIONS_CR1) {
      set_reg(r_CORTEX_M_USART_CR1(config->usart), config->options_cr1,
              USART_CONFIG_OPTIONS_CR1);
    }

    /* Control register 2 */
    if (config->set_mask & USART_SET_STOP_BITS) {
      set_reg(r_CORTEX_M_USART_CR2(config->usart), config->stop_bits,
              USART_CONFIG_STOP_BITS);
    }

    if (config->set_mask & USART_SET_OPTIONS_CR2) {
      set_reg(r_CORTEX_M_USART_CR2(config->usart), config->options_cr2,
              USART_CONFIG_OPTIONS_CR2);
    }

    /* USART 4 and 5 have some configuration limitations: check them before continuing */
    if ((config->hw_flow_control & (USART_CR3_CTSIE_Msk | USART_CR3_CTSE_Msk |
                                    USART_CR3_RTSE_Msk | USART_CR3_SCEN_Msk |
                                    USART_CR3_NACK_Msk))
        && ((config->usart == 4) || (config->usart == 5))) {
        printf
            ("Usart%x config error: asking for a flag in CR3 unavailable for USART4 and USART5",
             config->usart);
        return 1;
    }

    if ((config->hw_flow_control & (USART_CR3_DMAT_Msk | USART_CR3_DMAR_Msk))
        && (config->usart == 5)) {
        printf
            ("Usart%x config error: asking for a flag in CR3 unavailable for USART5",
             config->usart);
        return 1;
    }

    /* Control register 3 */
    if (config->set_mask & USART_SET_HW_FLOW_CTRL) {
      set_reg(r_CORTEX_M_USART_CR3(config->usart), config->hw_flow_control,
              USART_CONFIG_HW_FLW_CTRL);
    }

    if ((config->guard_time_prescaler)
        && ((config->usart == 4) || (config->usart == 5))) {
        printf
            ("Usart%x config error: asking for guard time/prescaler in GTPR unavailable for USART4 and USART5",
             config->usart);
        return 1;
    }

    if (config->set_mask & USART_SET_GUARD_TIME_PS) {
	  /* Prescaler and guard time */
	  set_reg(r_CORTEX_M_USART_GTPR(config->usart), config->guard_time_prescaler, USART_CONFIG_GUARD_TIME_PRESCALER);
    }

	/* Clear necessary bits */
	clear_reg_bits(r_CORTEX_M_USART_SR(config->usart), USART_SR_TC_Msk);
	clear_reg_bits(r_CORTEX_M_USART_SR(config->usart), USART_SR_RXNE_Msk);
	clear_reg_bits(r_CORTEX_M_USART_SR(config->usart), USART_SR_CTS_Msk);
	clear_reg_bits(r_CORTEX_M_USART_SR(config->usart), USART_SR_LIN_Msk);


    /* Initialize callbacks */
    if (config->set_mask & USART_SET_CB_RCV_IRQ) {
      usart_callbacks_init(config);
    }

    return 0;
}

/**** usart early init ****/
uint8_t usart_early_init(usart_config_t *config)
{
    memset((void*)&usart_dev, 0, sizeof(device_t));
    uint8_t ret = 0;

    strncpy(usart_dev.name, usarts[config->usart].name, strlen(usarts[config->usart].name));

    switch (config->usart) {
        default:
        case 0:
            printf("no USART0 support! Leaving!\n");
            return 1;
            break;
        case 1:
            usart_dev.address = GET_USART_ADDR(1);
            usart_dev.irqs[0].handler = GET_USART_IRQ_HANDLER(1, S);
            break;
        case 2:
            usart_dev.address = GET_USART_ADDR(2);
            usart_dev.irqs[0].handler = GET_USART_IRQ_HANDLER(2, S);
            break;
        case 3:
            usart_dev.address = GET_USART_ADDR(3);
            usart_dev.irqs[0].handler = GET_USART_IRQ_HANDLER(3, S);
            break;
        case 4:
            usart_dev.address = GET_USART_ADDR(4);
            usart_dev.irqs[0].handler = GET_USART_IRQ_HANDLER(4,);
            break;
        case 5:
            usart_dev.address = GET_USART_ADDR(5);
            usart_dev.irqs[0].handler = GET_USART_IRQ_HANDLER(5,);
            break;
        case 6:
            usart_dev.address = GET_USART_ADDR(6);
            usart_dev.irqs[0].handler = GET_USART_IRQ_HANDLER(6, S);
            break;
    }

    usart_dev.irqs[0].posthook.status = 0x0000; /* SR register */
    usart_dev.irqs[0].posthook.data   = 0x0004; /* DR register */

    usart_dev.irqs[0].posthook.action[0].instr = IRQ_PH_READ;
    usart_dev.irqs[0].posthook.action[0].read.offset = 0x0000; /* SR register */

    usart_dev.irqs[0].posthook.action[1].instr = IRQ_PH_READ;
    usart_dev.irqs[0].posthook.action[1].read.offset = 0x0004; /* DR register */

    usart_dev.irqs[0].posthook.action[2].instr = IRQ_PH_WRITE;
    usart_dev.irqs[0].posthook.action[2].write.offset = 0x0000;
    usart_dev.irqs[0].posthook.action[2].write.value  = 0x00;
    usart_dev.irqs[0].posthook.action[2].write.mask   = 0x3 << 6; /* clear TC & Tx status */

    /* force mainthread for SMARTCARD mode only */
    if (config->mode == SMARTCARD) {
      usart_dev.irqs[0].mode = IRQ_ISR_FORCE_MAINTHREAD;
    } else {
      usart_dev.irqs[0].mode = IRQ_ISR_STANDARD;
    }

    usart_dev.size = 0x400;
    usart_dev.irq_num = 1;
    usart_dev.gpio_num = 2;
    usart_dev.map_mode = DEV_MAP_AUTO;
    usart_dev.irqs[0].irq = usarts[config->usart].irq;

    /*
     * GPIOs
     */

    usart_dev.gpios[0].mask =
        GPIO_MASK_SET_MODE | GPIO_MASK_SET_TYPE | GPIO_MASK_SET_SPEED |
        GPIO_MASK_SET_PUPD | GPIO_MASK_SET_AFR;

    switch (config->mode) {
      case UART:
        usart_dev.gpios[0].kref.port = usarts[config->usart].port;
        usart_dev.gpios[0].kref.pin = usarts[config->usart].tx_pin;
        break;
      case SMARTCARD:
        usart_dev.gpios[0].kref.port = usarts[config->usart].sc_port;
        usart_dev.gpios[0].kref.pin = usarts[config->usart].sc_tx_pin;
        break;
    default:
        printf("Wrong usart mode %d.", config->mode);
        return 1;
    }

    usart_dev.gpios[0].mode = GPIO_PIN_ALTERNATE_MODE;
    usart_dev.gpios[0].speed = GPIO_PIN_VERY_HIGH_SPEED;
    usart_dev.gpios[0].afr = usarts[config->usart].af;

    usart_dev.gpios[1].mask =
        GPIO_MASK_SET_MODE | GPIO_MASK_SET_TYPE | GPIO_MASK_SET_SPEED |
        GPIO_MASK_SET_PUPD | GPIO_MASK_SET_AFR;

    switch (config->mode) {
      case UART:
        usart_dev.gpios[1].kref.port = usarts[config->usart].port;
        usart_dev.gpios[1].kref.pin = usarts[config->usart].rx_pin;
        break;
      case SMARTCARD:
        usart_dev.gpios[1].kref.port = usarts[config->usart].sc_port;
        usart_dev.gpios[1].kref.pin = usarts[config->usart].sc_ck_pin;
        break;
    default:
        printf("Wrong usart mode %d.", config->mode);
        return 1;
    }

    usart_dev.gpios[1].afr = usarts[config->usart].af;
    usart_dev.gpios[1].mode = GPIO_PIN_ALTERNATE_MODE;
    usart_dev.gpios[1].speed = GPIO_PIN_VERY_HIGH_SPEED;

    switch (config->mode) {
    case UART:
        {
            usart_dev.gpios[0].type = GPIO_PIN_OTYPER_PP;
            usart_dev.gpios[0].pupd = GPIO_NOPULL;
            usart_dev.gpios[1].type = GPIO_PIN_OTYPER_PP;
            usart_dev.gpios[1].pupd = GPIO_NOPULL;
            break;
        }
    case SMARTCARD:
        {
            usart_dev.gpios[0].type = GPIO_PIN_OTYPER_OD;
            usart_dev.gpios[0].pupd = GPIO_PULLUP;
            usart_dev.gpios[1].type = GPIO_PIN_OTYPER_PP;
            usart_dev.gpios[1].pupd = GPIO_PULLUP;
            break;
        }
    default:
        printf("Wrong usart mode %d.", config->mode);
        return 1;
    }

    ret = sys_init(INIT_DEVACCESS, &usart_dev, &usart_desc);
    return ret;
}

/* Get the current clock value of the USART bus */
uint32_t usart_get_bus_clock(usart_config_t * config)
{
    switch (config->usart) {
    case 1:
    case 6:
        return PROD_CLOCK_APB2;
        break;
    case 2:
    case 3:
    case 4:
    case 5:
        return PROD_CLOCK_APB1;
        break;
    default:
        printf("Wrong usart %d. You should use USART1 to USART6", config->usart);
    }

    return 0;
}
