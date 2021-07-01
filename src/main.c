/*
 * main implementation: use this 'C' sample to create your own application
 *
 */
#include "S32K144.h"

#if defined (__ghs__)
    #define __INTERRUPT_SVC  __interrupt
    #define __NO_RETURN _Pragma("ghs nowarning 111")
#elif defined (__ICCARM__)
    #define __INTERRUPT_SVC  __svc
    #define __NO_RETURN _Pragma("diag_suppress=Pe111")
#elif defined (__GNUC__)
    #define __INTERRUPT_SVC  __attribute__ ((interrupt ("SVC")))
    #define __NO_RETURN
#else
    #define __INTERRUPT_SVC
    #define __NO_RETURN
#endif

int counter, accumulator = 0, limit_value = 1000000;

extern uint32_t __VECTOR_RAM[];

void WDOG_disable (void)
{
	WDOG->CNT=0xD928C520; 	 // Unlock WDOG
	while(WDOG->CS & WDOG_CS_RCS_MASK);
	while(!(WDOG->CS & WDOG_CS_ULK_MASK));

	WDOG->TOVAL=0x0000FFFF;	 // Set maximum value for TOVAL
	WDOG->CS = 0x00002100;   // Disable WDOG
	                         // Unlock not allowed except POR
	while(!(WDOG->CS & WDOG_CS_RCS_MASK));
	while(WDOG->CS & WDOG_CS_ULK_MASK);
}

void SOSC_init(void)
{
  	SCG->SOSCCFG=0x00000034;  // RANGE=3; HIGH Frequnecy Range 8~40MHz for PLL input
                              // HGO=0; crsytal oscillator for low-gain operation
                              // EREFS=1; Use crystal oscillator

  	SCG->SOSCDIV=0x00000000;  // SOSCDIV2 & SOSCDIV1 =0; clock disabled

  	if(SCG->SOSCCSR & SCG_SOSCCSR_LK_MASK) SCG->SOSCCSR &= ~SCG_SOSCCSR_LK_MASK;

  	SCG->SOSCCSR=0x00830001;  // LK=1; LOCK SOSCCSR, protect accidentally write once.
  							  // SOSCCMRE=1
                              // SOSCCM=1
                              // SOSCEN=1; enalbe SOSC
  	while(!(SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK)); // Wait SOSC vaild
}

void SPLL_init(void)
{
	SCG->SPLLCFG = 0x00180100; // MULT=24
							   // PREDIV=1
							   // SOSC as PLL source (8MHZ)
							   // VCO_CLK=SPLL_SOURCE/(PREDIV+1)*(MULT+16) = 160MHz
							   // SPLL_CLK=(VCO_CLK)/2 = 80MHz

	SCG->SPLLDIV = 0x00000000; // SPLLDIV2 & SPLLDIV1 = 0; clock disabled

	if(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK) SCG->SPLLCSR &= ~SCG_SPLLCSR_LK_MASK;

	SCG->SPLLCSR = 0x00830001; //LK=1; LOCK SPLLCSR, protect accidentally write once.
							   //SPLLCMRE=1
							   //SPLLCM=1
							   //SPLLEN=1; enable SPLL
	while(!(SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK)); // Wait SPLL valid
}

void buttonISR(void)
{
	uint32_t buttonsPressed = PORTC->ISFR & ((1 << 13) | (1 << 12));

	if(buttonsPressed & (1 << 13)) {
		PORTC->PCR[13] |= PORT_PCR_ISF_MASK; // clear pending interrupt
	}

	if(buttonsPressed & (1 << 12)) {
		PORTC->PCR[12] |= PORT_PCR_ISF_MASK; // clear pending interrupt
	}

	if((PTC->PDIR & (1 << 13)) | (PTC->PDIR & (1 << 12))) {
		if( (~(PTD->PDOR)) & (1 << 0)) {
			PTD->PSOR |= (1 << 0);
			PTD->PCOR |= (1 << 15);
			PTD->PSOR |= (1 << 16);
		} else if( (~(PTD->PDOR)) & (1 << 15)) {
			PTD->PSOR |= (1 << 0);
			PTD->PSOR |= (1 << 15);
			PTD->PCOR |= (1 << 16);
		} else if((~(PTD->PDOR)) & (1 << 16)) {
			PTD->PSOR |= (1 << 0);
			PTD->PSOR |= (1 << 15);
			PTD->PSOR |= (1 << 16);
		} else {
			PTD->PCOR |= (1 << 0);
			PTD->PSOR |= (1 << 15);
			PTD->PSOR |= (1 << 16);
		}
	}

}

int main(void) {
    counter = 0;

    WDOG_disable();
	SOSC_init();               // external crystal osc (8MHz) init
	SPLL_init();               // Use SOSC as SPLL source and SPLL output is 80MHz

	// SPLL as system clock in RUN mode
	// CORE_CLK = SYS_CLK = 80MHz
	// BUS_CLK = 40MHz
	// FLASH_CLK = 26.67MHz
	SCG->RCCR = SCG_RCCR_SCS(6) + SCG_RCCR_DIVCORE(0) + SCG_RCCR_DIVBUS(1) + SCG_RCCR_DIVSLOW(2);
	while(!(SCG->SPLLCSR & SCG_SPLLCSR_SPLLSEL_MASK)); // Wait SPLL as system clock

    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; //enable clock for PORTD
    PCC->PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK; //enable clock for PORTC

    PORTD->PCR[0]  = PORT_PCR_MUX(1); //set PORTD PIN0  pin as GPIO
    PORTD->PCR[15] = PORT_PCR_MUX(1); //set PORTD PIN15 pin as GPIO
    PORTD->PCR[16] = PORT_PCR_MUX(1); //set PORTD PIN16 pin as GPIO

    // PORTD pin0. pin15 and pin16 is configured as output pin
    // pin0(blue LED). pin15(red LED). pin16(green LED)
    PTD->PDDR |= ((1 << 0) | (1 << 15) | (1 << 16));
    PTD->PSOR |= ((1 << 0) | (1 << 15) | (1 << 16)); //turn off all LED

    //set PORTC PIN12 pin as GPIO and rising edge interrupt
    PORTC->PCR[12] &= ~(PORT_PCR_IRQC_MASK | PORT_PCR_MUX_MASK);
    PORTC->PCR[12] |= PORT_PCR_IRQC(9) + PORT_PCR_MUX(1);

    //set PORTC PIN13 pin as GPIO and rising edge interrupt
    PORTC->PCR[13] &= ~(PORT_PCR_IRQC_MASK | PORT_PCR_MUX_MASK);
    PORTC->PCR[13] |= PORT_PCR_IRQC(9) + PORT_PCR_MUX(1);

	uint32_t * pVectorRam = (uint32_t *)__VECTOR_RAM;
	pVectorRam[PORTC_IRQn + 16] = (uint32_t)(buttonISR);

	//clear pending IRQ
	S32_NVIC->ICPR[PORTC_IRQn/32] |= (uint32_t)(1 << (PORTC_IRQn%32));
	//enable IRQ
	S32_NVIC->ISER[PORTC_IRQn/32] |= (uint32_t)(1 << (PORTC_IRQn%32));

    for (;;) {
        counter++;

        if (counter >= limit_value) {
            __asm volatile ("svc 0");
            counter = 0;
        }
    }
    /* to avoid the warning message for GHS and IAR: statement is unreachable*/
    __NO_RETURN
    return 0;
}

__INTERRUPT_SVC void SVC_Handler() {
    accumulator += counter;
}
