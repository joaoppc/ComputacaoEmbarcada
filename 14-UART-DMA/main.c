#include "asf.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/
/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO       PIOC
#define LED_PIN		    8
#define LED_PIN_MASK  (1<<LED_PIN)

/**
 * Botão
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		      11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

/**
 * XDMA
 */

/** XDMAC channel used in this example. */
#define XDMAC_TX_CH 0
#define XDMAC_RX_CH 1

/** XDMAC channel HW Interface number for usart ,refer to datasheet. */
#define USART_XDMAC_TX_CH_NUM  9
#define USART_XDMAC_RX_CH_NUM  10

/** XDMA peripheral interface*/
#define USART_XDMA_DEST_REG    &CONSOLE_UART->US_THR // Mem Two Peripheral
#define USART_XDMA_ORIG_REG    &CONSOLE_UART->US_RHR // Peripheral Two Mem

/** The buffer size for transfer  */
#define BUFFER_SIZE          100

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

/** XDMAC channel configuration. */
static xdmac_channel_config_t g_xdmac_tx_cfg;
static xdmac_channel_config_t g_xdmac_rx_cfg;

/** 
 * XDMA
 */
uint8_t  g_tx_buffer[]             = "This is message from USART master transferred by XDMAC test \n";
uint8_t  g_rx_buffer[BUFFER_SIZE]  = "0";
uint32_t g_buffer_size             = sizeof(g_tx_buffer);

/************************************************************************/
/* Flags                                                                */
/************************************************************************/

volatile uint8_t flag_led0  = 1;
volatile uint8_t flag_rx    = 0;

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void LED_init(int estado);
void TC1_init(void);
void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);
static void uart_xdmac_configure();
static void uart_xdmac_Tx(uint32_t *dest_address, uint32_t *orgin_address, uint32_t buffer_size);
static void uart_xdmac_Rx(uint32_t *peripheral_address, uint32_t *orgin_address, uint32_t buffer_size);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
 *  Handle Interrupcao botao 1
 */
static void Button1_Handler(uint32_t id, uint32_t mask)
{
  /* configura e inicializa DMA */
  uart_xdmac_Tx(USART_XDMA_DEST_REG, (uint32_t) g_tx_buffer, g_buffer_size);
  uart_xdmac_Rx(USART_XDMA_ORIG_REG, (uint32_t) g_rx_buffer, g_buffer_size);
  pin_toggle(LED_PIO, LED_PIN_MASK);
}

/**
 *  Interrupt handler for TC0 interrupt. 
 */
void TC1_Handler(void){
  volatile uint32_t ul_dummy;

  /****************************************************************
  * Devemos indicar ao TC que a interrupção foi satisfeita.
  ******************************************************************/
  ul_dummy = tc_get_status(TC0, 1);

  /* Avoid compiler warning */
  UNUSED(ul_dummy);

  /** Muda o estado do LED */
    if(flag_led0)
        pin_toggle(LED_PIO, LED_PIN_MASK);
}

/**
 * \brief XDMAC interrupt handler.
 */
void XDMAC_Handler(void)
{
  uint32_t dma_status_tx, dma_status_rx;
    
  dma_status_tx = xdmac_channel_get_interrupt_status(XDMAC, XDMAC_TX_CH);
  dma_status_rx = xdmac_channel_get_interrupt_status(XDMAC, XDMAC_RX_CH);
    
  UNUSED(dma_status_tx);
  UNUSED(dma_status_rx);
    
  if(dma_status_rx & (XDMAC_CIS_BIS | XDMAC_CIS_LIS)){
    flag_rx = 1;
  }
}

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/** 
 *  Toggle pin controlado pelo PIO (out)
 */
void pin_toggle(Pio *pio, uint32_t mask){
   if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
   else
    pio_set(pio,mask);
}

/**
 * @Brief Inicializa o pino do BUT
 */
void BUT_init(void){
    /* config. pino botao em modo de entrada */
    pmc_enable_periph_clk(BUT_PIO_ID);
    pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
    /* config. interrupcao em borda de descida no botao do kit */
    /* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
    pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
    pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
    
    /* habilita interrupçcão do PIO que controla o botao */
    /* e configura sua prioridade                        */
    NVIC_EnableIRQ(BUT_PIO_ID);
    NVIC_SetPriority(BUT_PIO_ID, 1);
};

/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};

/**
 * Configura TimerCounter (TC0) para gerar uma interrupcao no canal 0-(ID_TC1) 
 * a cada 250 ms (4Hz)
 */
void TC1_init(void){   
    uint32_t ul_div;
    uint32_t ul_tcclks;
    uint32_t ul_sysclk = sysclk_get_cpu_hz();
    
    uint32_t channel = 1;
    
    /* Configura o PMC */
    pmc_enable_periph_clk(ID_TC1);    

    /** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
    tc_find_mck_divisor(4, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
    tc_init(TC0, channel, ul_tcclks | TC_CMR_CPCTRG);
    tc_write_rc(TC0, channel, (ul_sysclk / ul_div) / 4);

    /* Configura e ativa interrupçcão no TC canal 0 */
    NVIC_EnableIRQ((IRQn_Type) ID_TC1);
    tc_enable_interrupt(TC0, channel, TC_IER_CPCS);

    /* Inicializa o canal 0 do TC */
    tc_start(TC0, channel);
}

/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

static void uart_xdmac_configure()
{
    uint32_t xdmaint;

    /* Initialize and enable DMA controller */
    pmc_enable_periph_clk(ID_XDMAC);

    xdmaint = ( XDMAC_CIE_BIE   |
                XDMAC_CIE_DIE   |
                XDMAC_CIE_FIE   |
                XDMAC_CIE_RBIE  |
                XDMAC_CIE_WBIE  |
                XDMAC_CIE_ROIE);

    xdmac_channel_enable_interrupt(XDMAC, XDMAC_TX_CH, xdmaint);
    xdmac_enable_interrupt(XDMAC, XDMAC_TX_CH);

    xdmac_channel_enable_interrupt(XDMAC, XDMAC_RX_CH, xdmaint);
    xdmac_enable_interrupt(XDMAC, XDMAC_RX_CH);

    /*Enable XDMAC interrupt */
    NVIC_ClearPendingIRQ(XDMAC_IRQn);
    NVIC_SetPriority( XDMAC_IRQn ,1);
    NVIC_EnableIRQ(XDMAC_IRQn);
}

/*
45.6.10.12 / pg. 1185
The DMA uses the trigger flags, TXRDY and RXRDY, to write or read into the USART. The DMA always writes in
the Transmit Holding register (US_THR) and it always reads in the Receive Holding register (US_RHR). The size
of the data written or read by the DMA in the USART is always a byte
*/
static void uart_xdmac_Tx(uint32_t *peripheral_address, uint32_t *orgin_address, uint32_t buffer_size)
{

    /* Initialize channel config for transmitter */
    g_xdmac_tx_cfg.mbr_ubc = buffer_size;
    g_xdmac_tx_cfg.mbr_sa  =  (uint32_t)orgin_address;
    g_xdmac_tx_cfg.mbr_da  =  (uint32_t)peripheral_address;
    
    g_xdmac_tx_cfg.mbr_cfg =  XDMAC_CC_TYPE_PER_TRAN |
                            XDMAC_CC_MBSIZE_SINGLE |
                            XDMAC_CC_DSYNC_MEM2PER |
                            XDMAC_CC_CSIZE_CHK_1 |
                            XDMAC_CC_DWIDTH_BYTE |
                            XDMAC_CC_SIF_AHB_IF0 |
                            XDMAC_CC_DIF_AHB_IF1 |
                            XDMAC_CC_SAM_INCREMENTED_AM |
                            XDMAC_CC_DAM_FIXED_AM |
                            XDMAC_CC_PERID(USART_XDMAC_TX_CH_NUM);

    g_xdmac_tx_cfg.mbr_bc  = 0;
    g_xdmac_tx_cfg.mbr_ds  = 0;
    g_xdmac_tx_cfg.mbr_sus = 0;
    g_xdmac_tx_cfg.mbr_dus = 0;

    xdmac_configure_transfer(XDMAC, XDMAC_TX_CH, &g_xdmac_tx_cfg);
    xdmac_channel_set_descriptor_control(XDMAC, XDMAC_TX_CH, 0);
    xdmac_channel_enable(XDMAC, XDMAC_TX_CH);

}

static void uart_xdmac_Rx(uint32_t *peripheral_address, uint32_t *orgin_address, uint32_t buffer_size)
{
    /* Initialize channel config for receiver */
    g_xdmac_rx_cfg.mbr_ubc = buffer_size;
    g_xdmac_rx_cfg.mbr_da = (uint32_t)orgin_address;
    g_xdmac_rx_cfg.mbr_sa = (uint32_t)peripheral_address;
    
    g_xdmac_rx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
                            XDMAC_CC_MBSIZE_SINGLE |
                            XDMAC_CC_DSYNC_PER2MEM |
                            XDMAC_CC_CSIZE_CHK_1 |
                            XDMAC_CC_DWIDTH_BYTE|
                            XDMAC_CC_SIF_AHB_IF1 |
                            XDMAC_CC_DIF_AHB_IF0 |
                            XDMAC_CC_SAM_FIXED_AM |
                            XDMAC_CC_DAM_INCREMENTED_AM |
                            XDMAC_CC_PERID(USART_XDMAC_RX_CH_NUM);

    g_xdmac_rx_cfg.mbr_bc = 0;
    g_xdmac_tx_cfg.mbr_ds =  0;
    g_xdmac_rx_cfg.mbr_sus = 0;
    g_xdmac_rx_cfg.mbr_dus =0;

    xdmac_configure_transfer(XDMAC, XDMAC_RX_CH, &g_xdmac_rx_cfg);
    xdmac_channel_set_descriptor_control(XDMAC, XDMAC_RX_CH, 0);
    xdmac_channel_enable(XDMAC, XDMAC_RX_CH);
}

/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
  /* Initialize the SAM system */
  sysclk_init();
    
  /* Disable the watchdog */
  WDT->WDT_MR = WDT_MR_WDDIS;
    
  board_init();

  /* Configura Leds */
  LED_init(0);
	
  /* Configura os botões */
  BUT_init();    
    
  /* Inicializa UART (console) */
  configure_console();
  printf(" Inicializando teste de DMA \n");
    
  /* Configura DMA */
  uart_xdmac_configure();
  
  uart_xdmac_Tx(USART_XDMA_DEST_REG, (uint32_t) g_tx_buffer, g_buffer_size);

  while (1) {
    /* Entra em modo sleep */
	  if(flag_rx){
      printf("RX: %s", g_rx_buffer);
      flag_rx = 0 ;
    }
  }  
}
