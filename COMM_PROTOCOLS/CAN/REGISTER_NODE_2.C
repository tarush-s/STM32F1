#include "stm32f1xx.h"

//protype functions
void can_send(void);
void can_init(void);
void ports(void);
void SysTick_Handler(void);
void dms(int ms);


//variable prototypes
volatile static int count=0;

int main()
{
	 ports();

	 SystemCoreClockUpdate();
   SysTick_Config(SystemCoreClock/1000);
  can_init();




	while(1)
	{

		can_send();
		GPIOC->BSRR |= 1<<(13);
	  dms(1000);


	}
}

void ports()
{
	//configure clocks for port a and c (alt func)
	 RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPCEN;
	//PA12 as can transmitter & PA11 is input by default
		GPIOA->CRH |= GPIO_CRH_MODE12_0 | GPIO_CRH_CNF12_1;
	  GPIOA->CRH &= ~GPIO_CRH_CNF12_0;
	//PC13 os output
		GPIOC->CRH |= GPIO_CRH_MODE13_1;
	  GPIOC->CRH &= ~(GPIO_CRH_CNF13_1 | GPIO_CRH_CNF13_0);
}
void can_init()
{
	//enable clock for can
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
	//enter intialization mode
	CAN1->MCR &= ~CAN_MCR_SLEEP;
		while(CAN1->MSR & CAN_MSR_SLAK);

	CAN1->MCR |= CAN_MCR_INRQ;
	//wait for ack

	while(!(CAN1->MSR & CAN_MSR_INAK));
	//enable no automatic transmission
	CAN1->MCR |= CAN_MCR_NART;
	//* 250Kbit/s , PRESCALER 2 , NO OF tq = 16 . seg1 = 13 , seg2 = 2 , SJW = 1tq***//
	CAN1->BTR = 0x1C0001;
	// SJW 1 by default
	CAN1->MCR&=~(1u<<0);         	 //enter normal mode

	while(!(CAN1->MSR & CAN_MSR_INAK));  //wait for ack

}

void can_send()
{
	// mailbox 0 as transmitter
	// standart id is 7
	CAN1->sTxMailBox[0].TIR = 7<<21;
	// set data frame
	CAN1->sTxMailBox[0].TIR &= ~(CAN_TI0R_IDE | CAN_TI0R_RTR);
	//send one byte of data
	CAN1->sTxMailBox[0].TDTR |= 1<<0;
  //send 10
	CAN1->sTxMailBox[0].TDLR = 10;
  // send the data and wait for complete transmission
  CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
  while(!(CAN1->TSR & CAN_TSR_RQCP0))	;
}
void SysTick_Handler()
{
	count++;
}

void dms(int ms)
{
	count=0;
	while(count<ms);
}
