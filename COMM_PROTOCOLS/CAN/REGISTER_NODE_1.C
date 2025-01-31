#include "stm32f10x.h"

//protype functions
void can_init(void);
void filters(void);
void ports(void);
void SysTick_Handler(void);
void dms(int ms);
void SysTick_Handler(void);
void CAN1_RX0_IRQHandler(void); 

//variable prototypes 
volatile static int count=0;

int main()
{
	 SystemCoreClockUpdate();
   SysTick_Config(SystemCoreClock/1000);
	 can_init();
	 ports();
	 filters();
	
	
	while(1)
	{
		int x=0;
		if (CAN1->RF0R & 3) //check if message pending in fifo0
		{
			x=(unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR);  //get data from mailbox
			CAN1->RF0R |= 0x20;// release fifo0 mailbox
		}
	  if(x)
		{
		 dms(100);
		 GPIOC->BSRR = (1<<13); //set pin13 high
		 dms(100);
		 GPIOC->BSRR = (1<<29);  // set pin13 low
		}
	
  
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
	CAN1->MCR |= CAN_MCR_INRQ;			
	//wait for ack
	while(!(CAN1->MSR & CAN_MSR_INAK));
	//enable no automatic transmission 
	CAN1->MCR |= CAN_MCR_NART;
	//*** 250Kbit/s , PRESCALER 2 , NO OF tq = 16 . seg1 = 13 , seg2 = 2 , SJW = 1tq***//
	CAN1->BTR = 0x1C0001;
	// SJW 1 by default
	CAN1->MCR&=~(1u<<0);         	 //enter normal mode
	while(!(CAN1->MSR & CAN_MSR_INAK));  //wait for ack	
}

void filters()
{
	//init mode for filters 
	CAN1->FMR |= 1; 
  //
	CAN1->FM1R &=~(CAN_FM1R_FBM0);
	CAN1->FS1R|=CAN_FS1R_FSC0;
	CAN1->FFA1R &=~(CAN_FFA1R_FFA0);
	CAN1->FA1R |= CAN_FA1R_FACT0;
	CAN1->sFilterRegister[0].FR2=0xFFFFFFFF;
	CAN1->sFilterRegister[0].FR1=0x1;
	//
	//filter mode active
	CAN1->FMR &= ~1u;       
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


