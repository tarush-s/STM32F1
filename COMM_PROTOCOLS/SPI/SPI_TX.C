#include "stm32f10x.h"
volatile static myTicks=0;

//prototype start
void spi(void);
void ports(void);
void SysTick_Handler(void);
void delayMs(uint16_t ms);
void send(char tx_char);
void CS_Disable (void);
void CS_Enable (void);
void blink (void);
// prototype end

int main()
{
  SysTick_Config(SystemCoreClock / 1000);
	spi();
	ports();
	
  

	while(1)
	{
		

		if(GPIOA -> IDR & 0x00000001) // IDR -> INPUT DATA REGISTER | CHECKING STATUS OF A0
		{ 
			CS_Enable();
			send('A');
			CS_Disable();
			delayMs(10);
		}
		else
		{
			CS_Enable();
			send('P');
			CS_Disable();
			delayMs(10);
		}

   
	}


}

void blink ()
{
	int t=0;
	while (t<2)
	{
		GPIOC -> ODR |= 0x2000;
		delayMs(100);
		GPIOC ->  ODR &= ~0x2000;
		delayMs(100);
		t++;
	}
}

void ports(void)
{
	RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;  // Enable GPIOA clock
	RCC->APB2ENR |=  RCC_APB2ENR_IOPCEN;
	GPIOC -> CRH &= 0xFF0FFFFF; // RESET PIN 13
	GPIOC -> CRH |= 0x00300000; // PIN 13 | OUTPUT MODE | MAX SPEED = 50Hz
	
	//spi ports
	GPIOA->CRL |=(1<<21);
	GPIOA->CRL &=~(1<<20);
	GPIOA->CRL |=(1<<22) | (1<<23);
	
	GPIOA->CRL |=(1<<29);
	GPIOA->CRL &=~(1<<28);
	GPIOA->CRL |=(1<<30) | (1U<<31);
	
	
	
	
	
	GPIOA -> CRL &= 0xFFFFFFF0; // RESET PORT A TO 0
	GPIOA -> CRL |= 0x8; // INPUT MODE | PUSH-PULL| PIN A0

}
void spi()
{
	RCC->APB2ENR |= (1<<12);  // Enable SPI1 CLock
//*******Setup SPI peripherals*****
	SPI1->CR1 |= (1<<15) | (1<<14) | (1<<9);
SPI1->CR1 |= 0x4; // Master Mode
SPI1->CR1 |= 0x31; // fclk / 265
SPI1->CR2 |= 0x4;
SPI1->CR1 |= 0x40; // Enabling SPI SPI periph



 }

void send(char tx_char)
{
  
        {
		SPI1->DR = tx_char;
		while(SPI1->SR & 0x80);

	}
  
}
void SysTick_Handler(void)
{
	myTicks++;
}

void delayMs(uint16_t ms)
{
	myTicks = 0;
	while(myTicks<ms);
}
void CS_Enable (void)
{
	GPIOA->BSRR |= (1<<4)<<16;
}

void CS_Disable (void)
{
	GPIOA->BSRR |= (1<<4);
}
