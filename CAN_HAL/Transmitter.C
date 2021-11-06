#include "main.h"

CAN_HandleTypeDef hcan;					//struct containing CAN init settings
CAN_FilterTypeDef sFilterConfig;			//struct containing filter settings
CAN_RxHeaderTypeDef RxMessage;	 //struct for recieved data frame
CAN_TxHeaderTypeDef TxMessage;   // struct for transmitted dataframe
uint8_t rxData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };	//array for recieved data (8 bytes)
uint8_t txData[8] = { 10, 0, 0, 0, 0, 0, 0, 0 };
uint32_t usedmailbox;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);

int main(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN_Init();

	TxMessage.IDE = CAN_ID_STD;				//standard identifier format (11bit)
	TxMessage.StdId = 0x211;									//identifier value
	TxMessage.RTR = CAN_RTR_DATA;//indicates frame mode (data frame or remote frame)
	TxMessage.DLC = 8;									//data length (8 bytes)
	TxMessage.TransmitGlobalTime = DISABLE;	//time of transmission is not transmitted along with the data

	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//filter bank consists of 2 32bit values (mask and ID)
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//filter set to mask and ID mode
	sFilterConfig.FilterBank = 0;				//filter bank number 0 selected
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;//assign filter bank to FIFO 0
	sFilterConfig.FilterIdHigh = 0x446 << 5;//STD ID value is 7, here shifted by 5 because 11 bits starting from the left are for STD ID (FilterIdHigh is 16bit)
	sFilterConfig.FilterIdLow = 0;										//LSB
	sFilterConfig.FilterMaskIdHigh = 0x446 << 5;//0b111 shifted by 5 for the same reason, first 11 bits are for Identifier
	sFilterConfig.FilterMaskIdLow = 0;								//LSB
	sFilterConfig.FilterActivation = ENABLE;				//activate filter

	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);	//commits filter settings
	HAL_CAN_Start(&hcan);								//start the CAN periph

	while (1) {

		if(GPIOA -> IDR & 0x00000001) // IDR -> INPUT DATA REGISTER | CHECKING STATUS OF A0
		{
		HAL_CAN_AddTxMessage(&hcan, &TxMessage, txData, &usedmailbox);//send data
		HAL_Delay(500);
		}

		if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0))//checks if the number of messages in FIFO 0 is non zero
				{
			HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxMessage, rxData);//stores the data frame in RxMessage struct, stores data in rsData array
		}
		if (rxData[0] == 10)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);		//LED ON
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	//LED OFF
			HAL_Delay(500);
			rxData[0] = 0;							//reset data
		}

	}

}

void SystemClock_Config(void)					//sets system clock to 32mhz HSE
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_CAN_Init(void)	//configured for 250Kbits/s , sample point 87.5%
{
	__HAL_RCC_CAN1_CLK_ENABLE();
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 8;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}

}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	GPIOC->CRH |= ((1<<31) | (1<<30)); //output at 50MGz
	GPIOC->CRH &= ~ ((1<<29) | (1<<28)); // general purpose output

	GPIOA->CRL &= 0xFFFFFFF0; // RESET PORT A TO 0
	GPIOA->CRL |= 0x8; // INPUT MODE | PUSH-PULL| PIN A0

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}
