/* USER CODE BEGIN Header */

/* VERSION PROBADA 2022 09 30
 * VERSION SOLO PARA TARJETA AC
 * CORREGIDA FUNCION EMERGENCIA, NO ACTIVABA EN FUNCIONES MEMORIAS
 */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * 23-FEB-2021 - 9:00 PM - PRUEBA EN TECNIPOLO - EDUARDO DANIEL DON LEONARDO
  * 2-MZO-2021 - 10:20 AM - Se entra a función CHEK con Solo M0 + RST presionadas al inicio
  * El cambio se hace en MCU_Init()
  * ----------------------------------------------------------------------------
  * 4-MZO-2021 - 7:52 PM - CAMBIO PARA CONTAR INTERRUPCIONES
  * SE hacen cambios en EXTI y TIM
  * ----------------------------------------------------------------------------
  * 5-MZO-2021 - 10:15 PM - SE AGREGAN Y CORRIGNE MENSAJES TX-UART
  * SE verifico todo el funcionamiento, incluso con interrupciones
  * Tambien se hace calibracion y se verifica todo funciona ok
  * ----------------------------------------------------------------------------
  * VERSION PROBADA 2022 09 30
  * SOLO PARA TARJETA AC
  * CORREGIDA FUNCION EMERGENCIA, NO ACTIVABA EN FUNCIONES MEMORIAS
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATE_SBY		0		//STAND BY
#define STATE_SS		1		//SUBE SILLA
#define STATE_BS		2		//BAJA SILLA
#define STATE_SE		3		//SUBE ESPALDAR
#define STATE_BE		4		//BAJA EPALDAR
#define STATE_ESC		5		//ESCUPIDERA
#define STATE_LA		6		//LAMPARA
#define STATE_M0		7		//MEMORIA M0
#define STATE_M1		8		//MEMORIA M1
#define STATE_M2		9		//MEMORIA M2
#define STATE_M3		10		//MEMORIA M3
#define STATE_ME		99		//EMERGENCIA

#define TIMER_UPDATE	1		//PERMITE ACTUALIZAR TIEMPOS DEPENDIENDO DEL STATE
#define TIMER_NO_CHANGE	0		//NO PERMITE ACTUALIZAR TIEMPOS

#define ESC_POS_HIGH	1		//LA SILLA ESTA EN POSICION DE ESCUPIR
#define ESC_POS_LOW		0		//LA ESCUPIDERA ESTA EN CUALQUIER POSICION

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int STATE;						//ALMACENA LOS ESTADOS
int STATE_TIMER;				//TIMER CAMBIA O NO? // (1)=TIMER_UPDATE. (0)=TIMER_NO_CHANGE
int ESC_POS;					//POICION ESCUPIDERA // (1)=ESC_POS_HIGH. (0)=ESC_POS_LOW

int FLAG_STOP_MOTORS;			//MOTORES DETENIDOS POR TECLAS?
int FLAG_INTERRUPT;				//MOTORES DETENIDOS POR MICROSUICHES? //(0)=NO INT. (1)=INT (2)=VARIAS INT

int32_t Contador_Silla;			//POSICION DE SILLA
int32_t Contador_Espaldar;		//POSICION DEL ESPALDAR

int32_t Contador_Rebotes;		//PARA EVITAR REBOTES EN MICROSUICHES

//====================================================VARIABLES DE EEPROM ALMACENADAS EN LA RAM===========================//
int32_t TIME_S;
int32_t TIME_E;
int32_t TIME_S_M1;
int32_t TIME_E_M1;
int32_t TIME_S_M2;
int32_t TIME_E_M2;
int32_t TIME_E_temp;
int32_t MAX_TIME_SS;
int32_t MAX_TIME_SE;
int32_t MIN_TIME_SS = 0;
int32_t MIN_TIME_SE = 0;
int32_t FLAG_CALIBRA = 0;
int32_t CONT_EEPROM = 0;

//====================================================VARIABLES ALMACENADAS EN LA EEPROM==================================//
uint32_t FLASH_TIME_S  	    = 0x08080000 + 32*0;
uint32_t FLASH_TIME_E 	    = 0x08080000 + 32*1;
uint32_t FLASH_TIME_S_M1    = 0x08080000 + 32*2;
uint32_t FLASH_TIME_E_M1    = 0x08080000 + 32*3;
uint32_t FLASH_TIME_S_M2    = 0x08080000 + 32*4;
uint32_t FLASH_TIME_E_M2    = 0x08080000 + 32*5;
uint32_t FLASH_MAX_TIME_SS  = 0x08080000 + 32*6;
uint32_t FLASH_MAX_TIME_SE  = 0x08080000 + 32*7;
uint32_t FLASH_MIN_TIME_SS  = 0x08080000 + 32*8;
uint32_t FLASH_MIN_TIME_SE  = 0x08080000 + 32*9;
uint32_t FLASH_FLAG_CALIBRA = 0x08080000 + 32*10;
uint32_t FLASH_CONT_EEPROM	= 0x08080000 + 32*11;


//char buffer[30] = "";				//BUFFER PARA UART_Tx
uint8_t uart_buf_len;	// ancho del buffer de UART
uint8_t uart_buf[80];	// buffer del UART

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

//Funciones de Estado
void FUN_ME();			//Funcion Emergencia
void FUN_SBY();			//Funcion Standby
void FUN_SS();			//Funcion Sube Silla
void FUN_BS();			//Funcion Baja Silla
void FUN_SE();			//Funcion Sube Espaldar
void FUN_BE();			//Funcion Baja Espaldar
void FUN_LA();			//Funcion Lampara
void FUN_ESC();			//Funcion Escupidera
void FUN_M0();			//Funcion Memoria M0
void FUN_M1();			//Funcion Memoria M1
void FUN_M2();			//Funcion Memoria M2
void FUN_M3();			//Funcion Memoria M3

void BEEP();			//Beep suena en pruebas
void BEEPx(int x);		//Beep suena siempre
void MCU_Init();		//Inicializar MCU
void All_Off();			//Apagar motores
void FUN_CALIBRA();		//Calibrcion
void FUN_CHECK();		//Chequeo de entradas

int stop_teclas();		//Detiene motores al precionar teclas en M0, M1, M2, M3, ESC, CALIBRA
int stop_tiempos();		//Detiene motores al exceder timer, base de tiempo 1ms
int stop_microsuiches();//Detiene motores al activar microsuiches

void Send_UART_TX1();	//Enviar datos por UART

void Tx_Tiempos_EEprom();	// Transmite los tiempos TIME_XXX de EEPROM - 21.84 ms


int32_t readFromEEPROM (uint32_t address);					//Leer valor de EEPROM
void writeToEEPROM (uint32_t address, int32_t value);		//Escribir valor en EEPROM
void getEEPROM();											//Obtener variables de EEPROM y guardarlas en RAM
void updatePosition();										//Actualizar posicion de silla en EEPROM


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //====================================================INICIALIZAR MCU==================================//
  MCU_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //====================================================EVALUAR ESTADOS==================================//
		switch(STATE){
		  case STATE_SBY:
				FUN_SBY();	//STANDBY
				break;
		  case STATE_SS:
				FUN_SS();	//SUBE SILLA
				break;
		  case STATE_BS:
				FUN_BS();	//BAJA SILLA
				break;
		  case STATE_SE:
				FUN_SE();	//SUBE ESPALDAR
				break;
		  case STATE_BE:
				FUN_BE();	//BAJA ESPALDAR
				break;
		  case STATE_LA:
				FUN_LA();	//LAMPARA
				break;
		  case STATE_ESC:
			  FUN_ESC();	//ESCUPIDERA
				break;
		  case STATE_M0:
				FUN_M0();	//MEMORIA 0
				break;
		  case STATE_M1:
				FUN_M1();	//MEMORIA 1
				break;
		  case STATE_M2:
				FUN_M2();	//MEMORIA 2
				break;
		  case STATE_M3:
				FUN_M3();	//MEMORIA 3
				break;
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZZ_Pin|OUT_MB_DT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT_ESC_Pin|OUT_LA_Pin|OUT_BE_Pin|OUT_SE_Pin
                          |OUT_BS_Pin|OUT_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN_SS_Pin IN_BS_Pin IN_SE_Pin IN_BE_Pin
                           IN_M0_Pin IN_M1_Pin IN_M2_Pin AC_DC_Pin */
  GPIO_InitStruct.Pin = IN_SS_Pin|IN_BS_Pin|IN_SE_Pin|IN_BE_Pin
                          |IN_M0_Pin|IN_M1_Pin|IN_M2_Pin|AC_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_M3_Pin IN_ESC_Pin IN_LA_Pin IN_CALIBRA_Pin */
  GPIO_InitStruct.Pin = IN_M3_Pin|IN_ESC_Pin|IN_LA_Pin|IN_CALIBRA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_ME_Pin IN_MSS_Pin IN_MBS_Pin IN_MBE_Pin
                           IN_MSE_Pin */
  GPIO_InitStruct.Pin = IN_ME_Pin|IN_MSS_Pin|IN_MBS_Pin|IN_MBE_Pin
                          |IN_MSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZ_Pin OUT_MB_DT_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin|OUT_MB_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_ESC_Pin OUT_LA_Pin OUT_BE_Pin OUT_SE_Pin
                           OUT_BS_Pin OUT_SS_Pin */
  GPIO_InitStruct.Pin = OUT_ESC_Pin|OUT_LA_Pin|OUT_BE_Pin|OUT_SE_Pin
                          |OUT_BS_Pin|OUT_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

//======================*** INTERRUPCIONES ***====================//

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){

	/* FUNCION DE INTERRUPCIONES EN MICROSUICHES
	 * AL ACTIVARSE UN MICROSUICE, LOS MOTORES SE APAGAN
	 * LA VARIABLE FLAG_INTERRUP SE PONE EN 1 --> stop_microsuiches() == HAL_ERROR
	 * SE ACTUALIZAN POSICION SEGUN MICROSUICHE ACTIVADO
	 * */

	// FLAG_INTERRUPT = 2 --> SE ACABA DE DESACTIVAR UN MICROSUICHE
	if((GPIO_Pin == OUT_SS_Pin  &&  HAL_GPIO_ReadPin(OUT_SS_GPIO_Port, OUT_SS_Pin) == GPIO_PIN_RESET)
	|| (GPIO_Pin == OUT_BS_Pin  &&  HAL_GPIO_ReadPin(OUT_BS_GPIO_Port, OUT_BS_Pin) == GPIO_PIN_RESET)
	|| (GPIO_Pin == OUT_SE_Pin  &&  HAL_GPIO_ReadPin(OUT_SE_GPIO_Port, OUT_SE_Pin) == GPIO_PIN_RESET)
	|| (GPIO_Pin == OUT_BE_Pin  &&  HAL_GPIO_ReadPin(OUT_BE_GPIO_Port, OUT_BE_Pin) == GPIO_PIN_RESET)){

		FLAG_INTERRUPT = 2;		//SE PONE LA BANDERA EN 2 PARA EVITAR REBOTES (TRUCO)
								//LA BANDERA VUELVE A SER CERO; LUEGO DE UNOS CUANTOS MILISEGUNDOS
	}


	// FLAG_INTERRUPT = 1 --> SE ACABA DE ACTIVAR UN MICROSUICHE
	if((GPIO_Pin == OUT_SS_Pin  &&  HAL_GPIO_ReadPin(OUT_SS_GPIO_Port, OUT_SS_Pin) == GPIO_PIN_SET)
	|| (GPIO_Pin == OUT_BS_Pin  &&  HAL_GPIO_ReadPin(OUT_BS_GPIO_Port, OUT_BS_Pin) == GPIO_PIN_SET)
	|| (GPIO_Pin == OUT_SE_Pin  &&  HAL_GPIO_ReadPin(OUT_SE_GPIO_Port, OUT_SE_Pin) == GPIO_PIN_SET)
	|| (GPIO_Pin == OUT_BE_Pin  &&  HAL_GPIO_ReadPin(OUT_BE_GPIO_Port, OUT_BE_Pin) == GPIO_PIN_SET)){

		if(FLAG_INTERRUPT != 2){ 					//TRUCO PARA EVITAR REBOTES
			//ACTIVAR BANDERA DE INTERRUPCION
			FLAG_INTERRUPT = 1;

			//APAGAR TODOS LOS MOTORES
			HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(OUT_SS_GPIO_Port, OUT_SS_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(OUT_BE_GPIO_Port, OUT_BE_Pin, GPIO_PIN_RESET);
		}
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	/* BASE DE TIEMPO 1ms
	 *
	 * VARIABLES:
	 *   TIMER_UPDATE		PERMITE ACTUALIZAR POSICION
	 * 	 TIMER_NO_CHANGE	NO PERMITE CTUALIZAR POSICION
	 *
	 * CADA VEZ QUE PASA 1ms DEPENDIENDO DEL ESTADO SE SUMARA O RESTARA A LA POSICION DE LA SILLA/ESPALDAR
	 */

	if(htim -> Instance == TIM2){

		//Actualizar Base de tiempo
		if(STATE_TIMER == TIMER_UPDATE){

			//Si esta subiendo incrementa el contador
			if(STATE == STATE_SS){
				Contador_Silla++;
			}

			//Si esta bajando decrementa el contador
			if(STATE == STATE_BS){
				Contador_Silla--;
			}

			//Si esta subiendo incrementa el contador
			if(STATE == STATE_SE){
				Contador_Espaldar++;
			}

			//Si esta bajando decrementa el contador
			if(STATE == STATE_BE){
				Contador_Espaldar--;
			}
		}


		//TRUCO PARA EVITAR REBOTES; EL CONTADOR DE REBOTES SE PONDRA EN UNO 50ms DESPUES DE HABERSE DESACTUVADO EL MICROSUICHE
		if(FLAG_INTERRUPT == 2){
			Contador_Rebotes++;

			if(Contador_Rebotes == 50){
				Contador_Rebotes = 0;
				FLAG_INTERRUPT = 0;
			}
		}

	}
}



//====================*** DIAGRAMA DE ESTADOS ***============================/

void  FUN_SBY(){
	//Apagar motores
	All_Off();
	int time_delay = 50;

	//Emergencia
	if(HAL_GPIO_ReadPin(IN_ME_GPIO_Port, IN_ME_Pin)==GPIO_PIN_SET){
		FUN_ME();
	}

	//Sube Silla
	else if (HAL_GPIO_ReadPin(IN_SS_GPIO_Port, IN_SS_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_SS_GPIO_Port, IN_SS_Pin)==GPIO_PIN_RESET){
			  STATE = STATE_SS;
		}
	}

	//Baja Silla
	else if (HAL_GPIO_ReadPin(IN_BS_GPIO_Port, IN_BS_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_BS_GPIO_Port, IN_BS_Pin)==GPIO_PIN_RESET){
			STATE = STATE_BS;
		}
	}

	//Sube Espaldar
	else if (HAL_GPIO_ReadPin(IN_SE_GPIO_Port, IN_SE_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_SE_GPIO_Port, IN_SE_Pin)==GPIO_PIN_RESET){
			STATE = STATE_SE;
		}
	}

	//Baja Espaldar
	else if (HAL_GPIO_ReadPin(IN_BE_GPIO_Port, IN_BE_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_BE_GPIO_Port, IN_BE_Pin)==GPIO_PIN_RESET){
			STATE = STATE_BE;
		}
	}

	//Lampara
	else if (HAL_GPIO_ReadPin(IN_LA_GPIO_Port, IN_LA_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_LA_GPIO_Port, IN_LA_Pin)==GPIO_PIN_RESET){
			STATE = STATE_LA;
		}
	}

	//Escupidera
	else if (HAL_GPIO_ReadPin(IN_ESC_GPIO_Port, IN_ESC_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_ESC_GPIO_Port, IN_ESC_Pin)==GPIO_PIN_RESET){
			STATE = STATE_ESC;
		}
	}

	//Memoria M0
	else if (HAL_GPIO_ReadPin(IN_M0_GPIO_Port, IN_M0_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_M0_GPIO_Port, IN_M0_Pin)==GPIO_PIN_RESET){
			STATE = STATE_M0;
		}
	}

	//Memoria M1
	else if (HAL_GPIO_ReadPin(IN_M1_GPIO_Port, IN_M1_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_M1_GPIO_Port, IN_M1_Pin)==GPIO_PIN_RESET){
			STATE = STATE_M1;
		}
	}

	//Memoria M2
	else if (HAL_GPIO_ReadPin(IN_M2_GPIO_Port, IN_M2_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_M2_GPIO_Port, IN_M2_Pin)==GPIO_PIN_RESET){
			STATE = STATE_M2;
		}
	}

	//Memoria M3
	else if (HAL_GPIO_ReadPin(IN_M3_GPIO_Port, IN_M3_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_M3_GPIO_Port, IN_M3_Pin)==GPIO_PIN_RESET){
			STATE = STATE_M3;
		}
	}
}

//========================== EMERGENCIA ================================
void FUN_ME(){
	//Encender Buzzer en señal de alerta
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
	STATE_TIMER = TIMER_UPDATE;
	STATE = STATE_SS;

	//Subir Silla
	FLAG_INTERRUPT = 0;
	while((HAL_GPIO_ReadPin(IN_ME_GPIO_Port, IN_ME_Pin)==GPIO_PIN_SET)
			&&(HAL_GPIO_ReadPin(IN_MSS_GPIO_Port, IN_MSS_Pin)==GPIO_PIN_RESET)
			&&(stop_microsuiches() == HAL_OK)
			&&(stop_tiempos() == HAL_OK)){
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUT_SS_GPIO_Port, OUT_SS_Pin, GPIO_PIN_SET);
	}

	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
	updatePosition();
}
//========================== SUBE SILLA ================================
void FUN_SS(){
	ESC_POS = ESC_POS_LOW;
	BEEP();

	//SUBE SILLA
	FLAG_INTERRUPT = 0;
	STATE_TIMER = TIMER_UPDATE;
	while((HAL_GPIO_ReadPin(IN_SS_GPIO_Port, IN_SS_Pin)==GPIO_PIN_RESET)
	  && (stop_microsuiches() == HAL_OK)
	  /*&& (stop_tiempos() == HAL_OK)*/
	  && (STATE != STATE_ME)){
		stop_tiempos();
		HAL_GPIO_WritePin(OUT_SS_GPIO_Port, OUT_SS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	updatePosition();
	BEEP();
}
//========================== BAJA SILLA ================================
void FUN_BS(){
	ESC_POS = ESC_POS_LOW;
	BEEP();

	//BAJA SILLA
	FLAG_INTERRUPT = 0;
	STATE_TIMER = TIMER_UPDATE;
	while((HAL_GPIO_ReadPin(IN_BS_GPIO_Port, IN_BS_Pin)==GPIO_PIN_RESET)
	  && (stop_microsuiches() == HAL_OK)
	  /*&& (stop_tiempos() == HAL_OK)*/
	  && (STATE != STATE_ME)){
		stop_tiempos();
		HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	updatePosition();
	BEEP();
}
//========================== SUBE ESPALDAR ================================
void FUN_SE(){
	ESC_POS = ESC_POS_LOW;
	BEEP();

	//SUBE ESPALDAR
	FLAG_INTERRUPT = 0;
	STATE_TIMER = TIMER_UPDATE;
	while((HAL_GPIO_ReadPin(IN_SE_GPIO_Port, IN_SE_Pin)==GPIO_PIN_RESET)
	  && (stop_microsuiches() == HAL_OK)
	  /*&& (stop_tiempos() == HAL_OK)*/
	  && (STATE != STATE_ME)){
		stop_tiempos();
		HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	updatePosition();
	BEEP();
}
//========================== BAJA ESPALDAR ================================
void FUN_BE(){
	ESC_POS = ESC_POS_LOW;
	BEEP();

	//BAJA ESPALDAR
	FLAG_INTERRUPT = 0;
	STATE_TIMER = TIMER_UPDATE;
	while((HAL_GPIO_ReadPin(IN_BE_GPIO_Port, IN_BE_Pin)==GPIO_PIN_RESET)
	  && (stop_microsuiches() == HAL_OK)
	  /*&& (stop_tiempos() == HAL_OK)*/
	  && (STATE != STATE_ME)){
		stop_tiempos();
		HAL_GPIO_WritePin(OUT_BE_GPIO_Port, OUT_BE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	updatePosition();
	BEEP();
}

//========================== LAMPARA ================================
void FUN_LA(){
	ESC_POS = ESC_POS_LOW;
	BEEP();

	//LAMPARA
	HAL_GPIO_TogglePin(OUT_LA_GPIO_Port, OUT_LA_Pin);
	Tx_Tiempos_EEprom();	// Transmite los tiempos TIME_XXX de EEPROM - 21.84 ms
	while(HAL_GPIO_ReadPin(IN_LA_GPIO_Port, IN_LA_Pin)==GPIO_PIN_RESET);

	STATE = STATE_SBY;
	BEEP();
}

//========================== ESCUPIDERA ================================
void FUN_ESC(){
	BEEP();

	//Apagar lampara
	HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_RESET);

	//SECUENCIA DE CON ESPALDAR SUBIENDO
	if(ESC_POS == ESC_POS_LOW){

		//Guarda posicion actual en variable temporal
		TIME_E_temp = TIME_E;

		//Subir Espaldar hasta el microcuiche
		STATE_TIMER = TIMER_UPDATE;
		STATE = STATE_SE;
		FLAG_INTERRUPT = 0;
		while(stop_microsuiches() == HAL_OK
		  && stop_teclas() == HAL_OK
		  /*&& stop_tiempos() == HAL_OK*/
		  && (STATE != STATE_ME)){
			HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
		}

		Contador_Espaldar = MAX_TIME_SE;
		updatePosition();


		//Encender escupidera durante 5seg
			HAL_GPIO_WritePin(OUT_ESC_GPIO_Port, OUT_ESC_Pin, GPIO_PIN_SET);
			HAL_Delay(5000);
			HAL_GPIO_WritePin(OUT_ESC_GPIO_Port, OUT_ESC_Pin, GPIO_PIN_RESET);


		//Escupidera con espaldar arriba
		ESC_POS = ESC_POS_HIGH;
	}

	//SECUENCIA CON ESPALDAR BAJANDO
	else if (ESC_POS == ESC_POS_HIGH){

		//Bajar Espaldar hasta llegar a la posicion en la que inicialmente estaba
		STATE_TIMER = TIMER_UPDATE;
		STATE = STATE_BE;
		FLAG_INTERRUPT = 0;
		while(Contador_Espaldar > TIME_E_temp
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
			HAL_GPIO_WritePin(OUT_BE_GPIO_Port, OUT_BE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
		}

		updatePosition();

		//Encender Lampara
		HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_SET);

		//Escupidera con espaldar abajo
		ESC_POS = ESC_POS_LOW;
	}

	BEEP();
}

//========================== M0 ================================
void FUN_M0(){
	ESC_POS = ESC_POS_LOW;
	BEEP();

	//Apagar Lampara
	HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_RESET);


	//Subir Espaldar hasta el microcuiche
	STATE_TIMER = TIMER_UPDATE;
	STATE = STATE_SE;
	FLAG_INTERRUPT = 0;
	while(stop_microsuiches() == HAL_OK
	  && stop_teclas() == HAL_OK
	  && (STATE != STATE_ME)){
		HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	if(FLAG_STOP_MOTORS != 2){
		Contador_Espaldar = MAX_TIME_SE;
	}
	updatePosition();
	BEEP();


	//Bajar Silla hasta el microsuiche
	STATE_TIMER = TIMER_UPDATE;
	FLAG_INTERRUPT = 0;
	STATE = STATE_BS;
	while(stop_microsuiches() == HAL_OK
	  && stop_teclas() == HAL_OK
	  && (STATE != STATE_ME)){
		HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}
	if(FLAG_STOP_MOTORS != 2){
		Contador_Silla = 0;
	}
	updatePosition();
	BEEP();

	FLAG_STOP_MOTORS = 0;
}
//========================== M1 ================================
void FUN_M1(){
	ESC_POS = ESC_POS_LOW;
	BEEP();

	//Apagar Lampara
	HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_RESET);

//------ CHEK SI GUARDA POSICION EN MEMORIA M1 ------------
	//Si el boton se presiona durante mas de 1 segunos, guardar posicion actual en Memoria M1
	HAL_Delay(1000);
	if (HAL_GPIO_ReadPin(IN_M1_GPIO_Port, IN_M1_Pin)==GPIO_PIN_RESET){

		TIME_S_M1 = TIME_S;
		TIME_E_M1 = TIME_E;

		writeToEEPROM (FLASH_TIME_S_M1, TIME_S_M1);
		writeToEEPROM (FLASH_TIME_E_M1, TIME_E_M1);

		BEEPx(100);
		BEEPx(100);
		BEEPx(100);
	}

//------ VA A PPSICION M1 ----------------------------------
	//Si el boton se preiona durante menos de 1 segundos, ir a posicion de Memoria M1
	else{

//----- ESCENARIO ESPECIAL ESPALDAR PRIMERO
		if((TIME_S_M1 < Contador_Silla)&&(TIME_E_M1 > Contador_Espaldar)){
			STATE_TIMER = TIMER_UPDATE;
			STATE = STATE_SE;
			FLAG_INTERRUPT = 0;
			while(TIME_E_M1 > Contador_Espaldar
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
				HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
			}

			updatePosition();


			STATE_TIMER = TIMER_UPDATE;
			STATE = STATE_BS;
			FLAG_INTERRUPT = 0;
			while(TIME_S_M1 < Contador_Silla
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
				HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_SET);
				//HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
			}

			if(FLAG_STOP_MOTORS != 2){
				Contador_Silla = TIME_S_M1;
				Contador_Espaldar = TIME_E_M1;
			}

			updatePosition();
		}


//----- SECUENCIA DE SUBIR/BAJAR SILLA
		STATE_TIMER = TIMER_UPDATE;
		if(TIME_S_M1 > Contador_Silla){
			STATE = STATE_SS;
			FLAG_INTERRUPT = 0;
			while(TIME_S_M1 > Contador_Silla
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
				HAL_GPIO_WritePin(OUT_SS_GPIO_Port, OUT_SS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
			}
		}
		else{
			STATE = STATE_BS;
			FLAG_INTERRUPT = 0;
			while(TIME_S_M1 < Contador_Silla
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
				HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_SET);
				//HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
			}
		}

		updatePosition();


//----- SECUENCIA DE SUBIR/BAJAR ESPALDAR
		STATE_TIMER = TIMER_UPDATE;
		if(TIME_E_M1 > Contador_Espaldar){
			STATE = STATE_SE;
			FLAG_INTERRUPT = 0;
			while(TIME_E_M1 > Contador_Espaldar
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
				HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
			}
		}
		else{
			STATE = STATE_BE;
			FLAG_INTERRUPT = 0;
			while(TIME_E_M1 < Contador_Espaldar
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
				HAL_GPIO_WritePin(OUT_BE_GPIO_Port, OUT_BE_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
			}
		}

		if(FLAG_STOP_MOTORS != 2){
			Contador_Silla = TIME_S_M1;
			Contador_Espaldar = TIME_E_M1;
		}

		updatePosition();

		//Encender Lampara
		HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_SET);

	}

	FLAG_STOP_MOTORS = 0;
	BEEP();
}

//========================== M2 ================================
void FUN_M2(){
	ESC_POS = ESC_POS_LOW;
	BEEP();

	//Apagar Lampara
	HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_RESET);

//------ CHEK SI GUARDA POSICION EN MEMORIA M2 ------------
	//Si el boton se presiona durante mas de 1 segunos, guardar posicion actual en Memoria M2
	HAL_Delay(1000);
	if (HAL_GPIO_ReadPin(IN_M2_GPIO_Port, IN_M2_Pin)==GPIO_PIN_RESET){
		TIME_S_M2 = TIME_S;
		TIME_E_M2 = TIME_E;

		writeToEEPROM (FLASH_TIME_S_M2, TIME_S_M2);
		writeToEEPROM (FLASH_TIME_E_M2, TIME_E_M2);

		BEEPx(100);
		BEEPx(100);
		BEEPx(100);
	}

//------ VA A PPSICION M2 ----------------------------------
	//Si el boton se preiona durante menos de 1 segundos, ir a posicion de Memoria M2
	else{

//----- ESCENARIO ESPECIAL ESPALDAR PRIMERO
		if((TIME_S_M2 < Contador_Silla)&&(TIME_E_M2 > Contador_Espaldar)){
			STATE_TIMER = TIMER_UPDATE;
			STATE = STATE_SE;
			FLAG_INTERRUPT = 0;
			while(TIME_E_M2 > Contador_Espaldar
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
				HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
			}

			updatePosition();

			STATE_TIMER = TIMER_UPDATE;
			STATE = STATE_BS;
			FLAG_INTERRUPT = 0;
			while(TIME_S_M2 < Contador_Silla
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
				HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_SET);
				//HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
			}

			updatePosition();

			if(FLAG_STOP_MOTORS != 2){
				Contador_Silla = TIME_S_M2;
				Contador_Espaldar = TIME_E_M2;
			}

			updatePosition();
		}


//----- SECUENCIA DE SUBIR/BAJAR SILLA
		STATE_TIMER = TIMER_UPDATE;
		if(TIME_S_M2 > Contador_Silla){
			STATE = STATE_SS;
			FLAG_INTERRUPT = 0;
			while(TIME_S_M2 > Contador_Silla
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
				HAL_GPIO_WritePin(OUT_SS_GPIO_Port, OUT_SS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
			}
		}
		else{
			STATE = STATE_BS;
			FLAG_INTERRUPT = 0;
			while(TIME_S_M2 < Contador_Silla
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
				HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_SET);
				//HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
			}
		}

		updatePosition();


//----- SECUENCIA DE SUBIR/BAJAR ESPALDAR
		STATE_TIMER = TIMER_UPDATE;
		if(TIME_E_M2 > Contador_Espaldar){
			STATE = STATE_SE;
			FLAG_INTERRUPT = 0;
			while(TIME_E_M2 > Contador_Espaldar
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
				HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
			}
		}
		else{
			STATE = STATE_BE;
			FLAG_INTERRUPT = 0;
			while(TIME_E_M2 < Contador_Espaldar
			  && stop_microsuiches() == HAL_OK
			  && stop_teclas() == HAL_OK
			  && stop_tiempos() == HAL_OK
			  && STATE != STATE_ME){
				HAL_GPIO_WritePin(OUT_BE_GPIO_Port, OUT_BE_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
			}
		}

		if(FLAG_STOP_MOTORS != 2){
			Contador_Silla = TIME_S_M2;
			Contador_Espaldar = TIME_E_M2;
		}

		updatePosition();

		//Encender Lampara
		HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_SET);

	}

	FLAG_STOP_MOTORS = 0;
	BEEP();
}

//========================== M3 ================================
void FUN_M3(){
	ESC_POS = ESC_POS_LOW;
	BEEP();

	//Apagar Lampara
	HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_RESET);

	//TREN DEL EMBUR: LA SILLA SE QUEDA IGUAL, EL ESPALDAR HASTA ABAJO
	STATE_TIMER = TIMER_UPDATE;
	STATE = STATE_BE;
	FLAG_INTERRUPT = 0;
	while(stop_microsuiches() == HAL_OK
			&& stop_teclas() == HAL_OK
			&& (HAL_GPIO_ReadPin(IN_MBE_GPIO_Port, IN_MBE_Pin)==GPIO_PIN_RESET)){
		HAL_GPIO_WritePin(OUT_BE_GPIO_Port, OUT_BE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	if(FLAG_STOP_MOTORS != 2){
		Contador_Espaldar = 0;
	}
	updatePosition();


	FLAG_STOP_MOTORS = 0;
	BEEP();
}

//=========================*** FUNCIONES ***===============================/

// Beep con Buzzer
void BEEP(){
/*
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
*/
}

// Beep X ms con Buzzer
void BEEPx(int x){
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
	HAL_Delay(x);
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
	HAL_Delay(x);
}

// Inicializa MCU.
// Pone datos iniciales, Lee datos TIME_XXX de EEPROM -> y los pone en RAM
// Detecta si Tacla CAL=CALIBRACION o M0=FUN_CHEK al principio
// Genera 3 Beep = Inicio MCU
void MCU_Init(){
	//Apagar todos los motores
	All_Off();

	//Base de tiempo 1ms
	HAL_TIM_Base_Start_IT(&htim2);

	//Inicializamos variables
	STATE = STATE_SBY;
	STATE_TIMER = TIMER_NO_CHANGE;
	ESC_POS = ESC_POS_LOW;


	Contador_Rebotes = 0;

	//Obtener memorias de la EEPROM
	getEEPROM();

	//Actualizar posicion actual
	Contador_Silla = TIME_S;
	Contador_Espaldar = TIME_E;

	  //---- PONE MENSAJE "- - - TECNIPOLO SAS - - -"
	  //---- PONE MENSAJE " VERSION 2 - 2021-FEB-16"
		uart_buf_len = sprintf(uart_buf, "\n\n- - - TECNIPOLO SAS - - -\r\n VERSION 2 - 2021-MZO-05 \r\n\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 5);

	//FUNCION DE CALIBRACION
	if (HAL_GPIO_ReadPin(IN_CALIBRA_GPIO_Port, IN_CALIBRA_Pin)==GPIO_PIN_RESET){
		FUN_CALIBRA();
	}

	//FUNCION PARA CHEQUEO DE TECLADO Y MICROSWITCHES
	else if (HAL_GPIO_ReadPin(IN_M0_GPIO_Port, IN_M0_Pin)==GPIO_PIN_RESET){
		FUN_CHECK();
	}

	//TRES BEEP QUE INDICAN ENCENDIDO OPERACION NORMAL
	else{
		BEEPx(100);BEEPx(100);BEEPx(100);
	}

}

// Apaga todos los Motores o salidas OUT_XX
void All_Off(){
	STATE = STATE_SBY;
	FLAG_STOP_MOTORS = 0;

	HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT_SS_GPIO_Port, OUT_SS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT_BE_GPIO_Port, OUT_BE_Pin, GPIO_PIN_RESET);


	//HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(OUT_ESC_GPIO_Port, OUT_ESC_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
}

//=============================== CALIBRACION =============================
// Hace funcion CALIBRACION
//1-M0, 2-SS, 3-BS, 4-BE, 5-SE
void FUN_CALIBRA(){
	//char uart_buf[20] = "";


	long int TIME_SS;	//Tiempo sube silla
	long int TIME_BS;	//Tiempo baja silla
	long int TIME_SE;	//Tiempo baja espaldar
	long int TIME_BE;	//Tiempo sube espaldar

	//Suena 5 beep que indican inicio de calibracion
	BEEPx(100);BEEPx(100);BEEPx(100);BEEPx(100);BEEPx(100);

//----------------* IR A POSICIÓN M0 *----------/
	FUN_M0();

	//Esperamos 1seg por seguridad
	HAL_Delay(1000);


//-----------CALIBRACION MOTOR SILLA -----------

//---- Sube silla hasta el microsuiche y guarda el tiempo
	STATE_TIMER = TIMER_UPDATE;
	STATE = STATE_SS;
	FLAG_INTERRUPT = 0;
	Contador_Silla = 0;
	while(HAL_GPIO_ReadPin(IN_MSS_GPIO_Port, IN_MSS_Pin)!=GPIO_PIN_SET
	  && stop_teclas() == HAL_OK
	  && FLAG_INTERRUPT == 0
	  && STATE != STATE_ME){
		HAL_GPIO_WritePin(OUT_SS_GPIO_Port, OUT_SS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}	All_Off();

	STATE_TIMER = TIMER_NO_CHANGE;
	TIME_SS = Contador_Silla;
	HAL_Delay(500);

	//sprintf(buffer, "\nTIME_SS = %ld\n", TIME_SS);
	uart_buf_len = sprintf(uart_buf, "\nTIME_SS = %ld\n", TIME_SS);
	//HAL_UART_Transmit_IT(&huart1, (uint8_t *)buffer, sizeof(buffer));
	//uart_buf_len = sprintf(uart_buf, "STBY - TIME_S = %ld\r\nSTBY - TIME_E = %ld\r\n", TIME_S, TIME_E);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 10);
	HAL_Delay(100);


//----- Baja silla hasta el microsuiche y guarda el tiempo
	STATE = STATE_SS; //SS para que sume y no reste
	STATE_TIMER = TIMER_UPDATE;
	FLAG_INTERRUPT = 0;
	Contador_Silla = 0;
	while(HAL_GPIO_ReadPin(IN_MBS_GPIO_Port, IN_MBS_Pin)!=GPIO_PIN_SET
	  && stop_teclas() == HAL_OK
	  && FLAG_INTERRUPT == 0
	  && STATE != STATE_ME){
		HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_SET);
	}	All_Off();

	STATE_TIMER = TIMER_NO_CHANGE;
	TIME_BS = Contador_Silla;
	HAL_Delay(500);

	//sprintf(buffer, "\nTIME_BS = %ld\n", TIME_BS);
	uart_buf_len = sprintf(uart_buf, "\nTIME_BS = %ld\n", TIME_BS);
	//HAL_UART_Transmit_IT(&huart1, (uint8_t *)buffer, sizeof(buffer));
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 10);
	HAL_Delay(100);

//----------- CALIBRACION MOTOR ESPALDAR ----------------

//----- Baja espaldar hasta el microsuiche y guarda el tiempo
	STATE_TIMER = TIMER_UPDATE;
	STATE = STATE_SE;
	FLAG_INTERRUPT = 0;
	Contador_Espaldar = 0;
	while(HAL_GPIO_ReadPin(IN_MBE_GPIO_Port, IN_MBE_Pin)!=GPIO_PIN_SET
	  && stop_teclas() == HAL_OK
	  && FLAG_INTERRUPT == 0
	  && STATE != STATE_ME){
		HAL_GPIO_WritePin(OUT_BE_GPIO_Port, OUT_BE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}	All_Off();

	STATE_TIMER = TIMER_NO_CHANGE;
	TIME_BE = Contador_Espaldar;
	HAL_Delay(500);

	//sprintf(buffer, "\nTIME_BE = %ld\n", TIME_BE);
	uart_buf_len = sprintf(uart_buf, "\nTIME_BE = %ld\n", TIME_BE);
	//HAL_UART_Transmit_IT(&huart1, (uint8_t *)buffer, sizeof(buffer));
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 10);

	HAL_Delay(100);

//----- Sube espaldar hasta el microsuiche y guarda el tiempo
	STATE_TIMER = TIMER_UPDATE;
	STATE = STATE_SE;
	FLAG_INTERRUPT = 0;
	Contador_Espaldar = 0;
	while(HAL_GPIO_ReadPin(IN_MSE_GPIO_Port, IN_MSE_Pin)!=GPIO_PIN_SET
	  && stop_teclas() == HAL_OK
	  && FLAG_INTERRUPT == 0
	  && STATE != STATE_ME){
		HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}	All_Off();

	STATE_TIMER = TIMER_NO_CHANGE;
	TIME_SE = Contador_Espaldar;
	HAL_Delay(500);

	//sprintf(buffer, "\nTIME_SE = %ld\n", TIME_SE);
	uart_buf_len = sprintf(uart_buf, "\nTIME_SE = %ld\n", TIME_SE);
	//HAL_UART_Transmit_IT(&huart1, (uint8_t *)buffer, sizeof(buffer));
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 10);

	HAL_Delay(100);

// --------------- EVALUA TIEMPOS MAXIMOS -----------------

//----- Maximo tiempo de Sube Silla igual al mayor valor medido
	if(TIME_SS > TIME_BS){
		MAX_TIME_SS = TIME_SS;
	}else{
		MAX_TIME_SS = TIME_BS;
	}   MIN_TIME_SS = 0;

	//sprintf(buffer, "\nMAX_TIME_SS = %ld\n", MAX_TIME_SS);
	uart_buf_len = sprintf(uart_buf, "\nMAX_TIME_SS = %ld\n", MAX_TIME_SS);
	//HAL_UART_Transmit_IT(&huart1, (uint8_t *)buffer, sizeof(buffer));
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 10);

	HAL_Delay(100);


//----- Maximo tiempo de Sube Espaldar igual al mayor valor medido
	if(TIME_SE > TIME_BE){
		MAX_TIME_SE = TIME_SE;
	}else{
		MAX_TIME_SE = TIME_BE;
	}	MIN_TIME_SE = 0;

	//sprintf(buffer, "\nMAX_TIME_SE = %ld\n", MAX_TIME_SE);
	uart_buf_len = sprintf(uart_buf, "\nMAX_TIME_SE = %ld\n", MAX_TIME_SE);
	//HAL_UART_Transmit_IT(&huart1, (uint8_t *)buffer, sizeof(buffer));
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 10);

	HAL_Delay(100);

//----- SE ESCRIBEN LOS DATOS OBTENIDOS EN MEMORIA -----------

	writeToEEPROM (FLASH_MAX_TIME_SS, MAX_TIME_SS);
	writeToEEPROM (FLASH_MAX_TIME_SE, MAX_TIME_SE);
	writeToEEPROM (FLASH_MIN_TIME_SE, MIN_TIME_SE);
	writeToEEPROM (FLASH_MIN_TIME_SE, MIN_TIME_SE);

	//Se actualiza posición de silla y espaldar
	Contador_Silla = MIN_TIME_SS;
	Contador_Espaldar = MAX_TIME_SE;
	updatePosition();

	FLAG_CALIBRA = 1;	//Bandera que indica calibración exitosa
	writeToEEPROM (FLASH_FLAG_CALIBRA, FLAG_CALIBRA);
}

// Chekea las TECLAS Y MICROSWITCHES + CAL + AC/DC
// Al presionar una tecla o actua un microswitche O tope, o tecla CAL o AC/DC, genera Beep hasta que se suelta
void FUN_CHECK(){

	BEEPx(100);	BEEPx(100);

	//TECLADO: SS, BS, SE, BE, M0, M1, M2, M3, ESC, LA
	while(1){
		while (HAL_GPIO_ReadPin(IN_SS_GPIO_Port, IN_SS_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_BS_GPIO_Port, IN_BS_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_SE_GPIO_Port, IN_SE_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_BE_GPIO_Port, IN_BE_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_M0_GPIO_Port, IN_M0_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_M1_GPIO_Port, IN_M1_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_M2_GPIO_Port, IN_M2_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_M3_GPIO_Port, IN_M3_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_ESC_GPIO_Port, IN_ESC_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_LA_GPIO_Port, IN_LA_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}


		//MICROSUICHES: MSS, MBS, MSE, MBE, ME
		while (HAL_GPIO_ReadPin(IN_MSS_GPIO_Port, IN_MSS_Pin)==GPIO_PIN_SET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_MBS_GPIO_Port, IN_MBS_Pin)==GPIO_PIN_SET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_MSE_GPIO_Port, IN_MSE_Pin)==GPIO_PIN_SET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_MBE_GPIO_Port, IN_MBE_Pin)==GPIO_PIN_SET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_ME_GPIO_Port, IN_ME_Pin)==GPIO_PIN_SET){
			BEEPx(100);
		}

		//BOTON CALIBRA
		while (HAL_GPIO_ReadPin(IN_CALIBRA_GPIO_Port, IN_CALIBRA_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}

		//PUENTE AC_DC: AC=1 || DC=0
		while (HAL_GPIO_ReadPin(AC_DC_GPIO_Port, AC_DC_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
	}
}

// Al presionar una tecla pone FLAG_STOP_MOTORS = 2 y retorna HAL_ERROR
//---OJO--- no entiendo la otra logica
int stop_teclas(){

	if(FLAG_STOP_MOTORS == 0){	//M0 = Activacion de tecla... M1 = Loop de funcion
		STATE_TIMER = TIMER_NO_CHANGE;
		HAL_Delay(1000);
		STATE_TIMER = TIMER_UPDATE;
		FLAG_STOP_MOTORS = 1;
	}

	else if(FLAG_STOP_MOTORS == 2){
		return HAL_ERROR;
	}




	if(HAL_GPIO_ReadPin(IN_SS_GPIO_Port, IN_SS_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_BS_GPIO_Port, IN_BS_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_SE_GPIO_Port, IN_SE_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_BE_GPIO_Port, IN_BE_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_M0_GPIO_Port, IN_M0_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_M1_GPIO_Port, IN_M1_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_M2_GPIO_Port, IN_M2_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_M3_GPIO_Port, IN_M3_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_ESC_GPIO_Port, IN_ESC_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_LA_GPIO_Port, IN_LA_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else {
		return HAL_OK;
	}
}

// Si Contador_Silla/Espladar >=MAX o <=MIN Tope, -> ponga el valor del Tope MAX/MIN_TIME_XX en Contador_Silla/Espaldar
int stop_tiempos(){


	if((Contador_Silla>=MAX_TIME_SS)&&(STATE == STATE_SS)){
		BEEP();
		Contador_Silla=MAX_TIME_SS;
		return HAL_ERROR;
	}

	else if((Contador_Silla<=MIN_TIME_SS)&&(STATE == STATE_BS)){
		BEEP();
		Contador_Silla=0;
		return HAL_ERROR;
	}

	else if((Contador_Espaldar>=MAX_TIME_SE)&&(STATE == STATE_SE)){
		BEEP();
		Contador_Espaldar=MAX_TIME_SE;
		return HAL_ERROR;
	}

	else if((Contador_Espaldar<=MIN_TIME_SE)&&(STATE == STATE_BE)){
		BEEP();
		Contador_Espaldar=0;
		return HAL_ERROR;
	}

	else{
		return HAL_OK;
	}
}

// Si hay un microswitche activado (stop), ponga el valor MAX/MIN_TIME_XX o tope -> en el Contador_Silla/Espaldar
// Apaga la bandera de interrupcion FLAG_INTERRUPT = 0; y retorna HAL_ERROR
//---OJO--- que pasa si hay MSW EMErgencia?????
int stop_microsuiches(){

	if(HAL_GPIO_ReadPin(IN_ME_GPIO_Port, IN_ME_Pin)==GPIO_PIN_SET){
		STATE = STATE_ME;
	}

	if(HAL_GPIO_ReadPin(IN_MSS_GPIO_Port, IN_MSS_Pin)==GPIO_PIN_SET
		&& STATE == STATE_SS){
		Contador_Silla = MAX_TIME_SS;
		FLAG_INTERRUPT = 0;
		return HAL_ERROR;
	}

	if(HAL_GPIO_ReadPin(IN_MBS_GPIO_Port, IN_MBS_Pin)==GPIO_PIN_SET
		&& STATE == STATE_BS){
		Contador_Silla = 0;
		FLAG_INTERRUPT = 0;
		return HAL_ERROR;
	}

	if(HAL_GPIO_ReadPin(IN_MSE_GPIO_Port, IN_MSE_Pin)==GPIO_PIN_SET
			&& STATE == STATE_SE){
		Contador_Espaldar = MAX_TIME_SE;
		FLAG_INTERRUPT = 0;
		return HAL_ERROR;
	}

	if(HAL_GPIO_ReadPin(IN_MBE_GPIO_Port, IN_MBE_Pin)==GPIO_PIN_SET
			&& STATE == STATE_BE){
		Contador_Espaldar = 0;
		FLAG_INTERRUPT = 0;
		return HAL_ERROR;
	}

	if(FLAG_INTERRUPT == 1){
		FLAG_INTERRUPT = 0;
		return HAL_ERROR;
	}


	return HAL_OK;
}



//================= FUNCIONES DE EEPROM =========================
// Lee 1 dato de 32 bits en la direccion address de EEPROM
int32_t readFromEEPROM (uint32_t address){
   return (*(__IO uint32_t *)address);
}

// Escribe 1 dato de 32 bits en la (direccion address, el dato value) en EEPROM
void writeToEEPROM (uint32_t address, int32_t value){
  HAL_StatusTypeDef flash_ok = HAL_ERROR;

  while (flash_ok != HAL_OK){
    flash_ok = HAL_FLASHEx_DATAEEPROM_Unlock();
  }	flash_ok = HAL_ERROR;

  while (flash_ok != HAL_OK)  {
    flash_ok = HAL_FLASHEx_DATAEEPROM_Erase (address);
  }	flash_ok = HAL_ERROR;

  while (flash_ok != HAL_OK)  {
    flash_ok = HAL_FLASHEx_DATAEEPROM_Program (FLASH_TYPEPROGRAMDATA_WORD, address, value);
  }	flash_ok = HAL_ERROR;

  while (flash_ok != HAL_OK){
    flash_ok = HAL_FLASHEx_DATAEEPROM_Lock ();
  }
}

// Lee los datos TIME_X de 32 bits de EEPROM -> y los pone en RAM
void getEEPROM(){
	TIME_S = readFromEEPROM (FLASH_TIME_S);
	TIME_E = readFromEEPROM (FLASH_TIME_E);

	TIME_S_M1 = readFromEEPROM (FLASH_TIME_S_M1);
	TIME_E_M1 = readFromEEPROM (FLASH_TIME_E_M1);
	TIME_S_M2 = readFromEEPROM (FLASH_TIME_S_M2);
	TIME_E_M2 = readFromEEPROM (FLASH_TIME_E_M2);

	MAX_TIME_SS = readFromEEPROM (FLASH_MAX_TIME_SS);
	MAX_TIME_SE = readFromEEPROM (FLASH_MAX_TIME_SE);
	MIN_TIME_SS = readFromEEPROM (FLASH_MIN_TIME_SS);
	MIN_TIME_SE = readFromEEPROM (FLASH_MIN_TIME_SE);

	//HAL_Delay(500); //---OJO--- ESTO NO DEBE IR. C/ESCRITURA = 6.8 MS
}

// Escribe los datos Contador_Silla/Espaldar en TIME_S/E y estos en EEPROM
void updatePosition(){
	All_Off();

	STATE_TIMER = TIMER_NO_CHANGE;
	TIME_S = Contador_Silla;
	TIME_E = Contador_Espaldar;

	writeToEEPROM (FLASH_TIME_S, TIME_S);
	writeToEEPROM (FLASH_TIME_E, TIME_E);
	HAL_Delay(500);		//---OJO--- ESTO NO DEBE IR. C/ESCRITURA = 6.8 MS

	Send_UART_TX1();
}


/* FUNCIONES DE UART */

void Send_UART_TX1(){

	//char buffer[50] = "";
	//uint32_t uart_buf_len;

	//uart_buf_len = sprintf(buffer, "\nTIME_S = %ld\nTIME_E = %ld\n\n", TIME_S, TIME_E);
	uart_buf_len = sprintf(uart_buf, "\nTIME_S = %ld\nTIME_E = %ld\n\n", TIME_S, TIME_E);
	//HAL_UART_Transmit(&huart1, (uint8_t *)buffer, uart_buf_len, 5);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 10);

	HAL_Delay(10);		//---OJO--- ESTO NO DEBE PONERSE - BORRAR O COMENTAR

}

// Transmite los tiempos TIME_XXX de EEPROM - 21.84 ms
// Lee en 5.2us dato 32 bits = 5.2 x 2 = 10.4 us // y son 5 lecturas 10.4 x 5 = 52 us.. No apreciable
// Tramsmite cada linea o dato en 1.82ms x 2 = 3.64 ms
// y son 6 tipos de datos 6 x 3.64 ms = 21.84 toma esta funcion
void Tx_Tiempos_EEprom(){
uint32_t aux1;		// auxiliar 1
uint32_t aux2;		// auxiliar 2

	uart_buf_len = sprintf(uart_buf, "\n\n = TIEMPOS EEPROM =\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 5);

	aux1 = readFromEEPROM (FLASH_TIME_S);
	aux2 = readFromEEPROM (FLASH_TIME_E);

	uart_buf_len = sprintf(uart_buf, "TIME_S     = %ld\nTIME_E     = %ld\n", aux1, aux2);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 5);

	aux1 = readFromEEPROM (FLASH_TIME_S_M1);
	aux2 = readFromEEPROM (FLASH_TIME_E_M1);

	uart_buf_len = sprintf(uart_buf, "TIME_S_M1  = %ld\nTIME_E_M1  = %ld\n", aux1, aux2);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 5);

	aux1 = readFromEEPROM (FLASH_TIME_S_M2);
	aux2 = readFromEEPROM (FLASH_TIME_E_M2);

	uart_buf_len = sprintf(uart_buf, "TIME_S_M2  = %ld\nTIME_E_M2  = %ld\n", aux1, aux2);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 5);

	aux1 = readFromEEPROM (FLASH_MAX_TIME_SS);
	aux2 = readFromEEPROM (FLASH_MAX_TIME_SE);

	uart_buf_len = sprintf(uart_buf, "MAX_TIME_S = %ld\nMAX_TIME_E = %ld\n", aux1, aux2);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 5);

	aux1 = readFromEEPROM (FLASH_MIN_TIME_SS);
	aux2 = readFromEEPROM (FLASH_MIN_TIME_SE);

	uart_buf_len = sprintf(uart_buf, "MIN_TIME_S = %ld\nMIN_TIME_E = %ld\n", aux1, aux2);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len , 5);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
