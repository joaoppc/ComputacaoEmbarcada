/**************************************************************************
* Rafael Corsi   - Insper 
* rafael.corsi@insper.edu.br        
*
* Computação Embarcada
*
* 08-PIO-ENTRADA
************************************************************************/


#include "asf.h"
#include "conf_clock.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/**
 * LEDs
 */
#define LED1_PIO_ID		ID_PIOA
#define LED2_PIO_ID		ID_PIOC
#define LED3_PIO_ID		ID_PIOB

#define LED1_PIO         PIOA
#define LED2_PIO         PIOC
#define LED3_PIO         PIOB

#define LED1_PIN			0
#define LED2_PIN			30
#define LED3_PIN			2

#define LED1_PIN_MASK	(1<<LED1_PIN) 
#define LED2_PIN_MASK	(1<<LED2_PIN)
#define LED3_PIN_MASK	(1<<LED3_PIN)



/**
 * Botão
 */ 
#define BUTTON1_PIO_ID		ID_PIOD
#define BUTTON2_PIO_ID		ID_PIOC
#define BUTTON3_PIO_ID		ID_PIOA
	
#define BUTTON1_PIO         PIOD
#define BUTTON2_PIO         PIOC
#define BUTTON3_PIO         PIOA
	
#define BUTTON1_PIN			28
#define BUTTON2_PIN			31
#define BUTTON3_PIN			19
	
#define BUTTON1_PIN_MASK	(1<<BUTTON1_PIN)
#define BUTTON2_PIN_MASK	(1<<BUTTON2_PIN)
#define BUTTON3_PIN_MASK	(1<<BUTTON3_PIN)

#define BUT_DEBOUNCING_VALUE  79


/************************************************************************/
/* Prototipação                                                        */
/************************************************************************/
void ledConfig();
void butConfig();

/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void ledConfig(){
	//Ativa o clock do PIOA, PIOB, PIOC
	PMC->PMC_PCER0=(LED1_PIO_ID);
	PMC->PMC_PCER0=(LED2_PIO_ID);
	PMC->PMC_PCER0=(LED3_PIO_ID);
	
	//define os pinos dos leds como pinos de saída
	LED1_PIO->PIO_OER=(LED1_PIN_MASK);
	LED2_PIO->PIO_OER=(LED2_PIN_MASK);
	LED3_PIO->PIO_OER=(LED3_PIN_MASK);
	
	//define o comando para os PIO's e não para outro periférico
	LED1_PIO->PIO_PER=(LED1_PIN_MASK);
	LED2_PIO->PIO_PER=(LED2_PIN_MASK);
	LED3_PIO->PIO_PER=(LED3_PIN_MASK);
	
	//define a saida em nível alto
	LED1_PIO->PIO_CODR=(LED1_PIN_MASK);
	LED2_PIO->PIO_CODR=(LED2_PIN_MASK);
	LED3_PIO->PIO_CODR=(LED3_PIN_MASK);
};

void butConfig(){
	
	//ativa o clock do PIOD
	PMC->PMC_PCER0=(BUTTON1_PIO_ID);
	
	//define o pino dos botões como entrada
	BUTTON1_PIO->PIO_ODR=(BUTTON1_PIN_MASK);
	BUTTON2_PIO->PIO_ODR=(BUTTON2_PIN_MASK);
	BUTTON3_PIO->PIO_ODR=(BUTTON3_PIN_MASK);
	
	//define o comando para os PIO's e não para outro periférico
	BUTTON1_PIO->PIO_PER=(BUTTON1_PIN_MASK);
	BUTTON2_PIO->PIO_PER=(BUTTON2_PIN_MASK);
	BUTTON3_PIO->PIO_PER=(BUTTON3_PIN_MASK);
	
	//ativa o Pull-up dos botões, ou seja conecta ao vcc e a um resistor
	BUTTON1_PIO->PIO_PUER=(BUTTON1_PIN_MASK);
	BUTTON2_PIO->PIO_PUER=(BUTTON2_PIN_MASK);
	BUTTON3_PIO->PIO_PUER=(BUTTON3_PIN_MASK);
	
	//ativa o debouncing
	BUTTON1_PIO->PIO_IFER=(BUTTON1_PIN_MASK);
	BUTTON2_PIO->PIO_IFER=(BUTTON2_PIN_MASK);
	BUTTON3_PIO->PIO_IFER=(BUTTON3_PIN_MASK);
	
	//ativa o clock periférico
	BUTTON1_PIO->PIO_IFSCER=(BUTTON1_PIN_MASK);
	BUTTON2_PIO->PIO_IFSCER=(BUTTON2_PIN_MASK);
	BUTTON3_PIO->PIO_IFSCER=(BUTTON3_PIN_MASK);
	
	//define a frequência do debouncing
	BUTTON1_PIO->PIO_SCDR=BUT_DEBOUNCING_VALUE;
	BUTTON2_PIO->PIO_SCDR=BUT_DEBOUNCING_VALUE;
	BUTTON3_PIO->PIO_SCDR=BUT_DEBOUNCING_VALUE;
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{

	/************************************************************************/
	/* Inicialização básica do uC                                           */
	/************************************************************************/
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	/************************************************************************/
	/* Inicializa perifericos                                               */
	/************************************************************************/
	// Configura LED em modo saída
	ledConfig();

	// Configura botao
	butConfig();
	
	
	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
		if(BUTTON1_PIO->PIO_PDSR & (BUTTON1_PIN_MASK)){
			LED1_PIO->PIO_CODR = LED1_PIN_MASK;
		}
			
		else{
			LED1_PIO->PIO_SODR = LED1_PIN_MASK;

		}
	  
	};
}


