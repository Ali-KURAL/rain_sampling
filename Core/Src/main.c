/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FutureContracts.h"
#include "EventGenerator.h"
#include "MotorDriver.h"
#include "ValveDriver.h"
#include "RTU/Slave/ModbusRtu_Slave.h"
#include "Core/ModbusApp.h"
#include "StateMachine.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Configuration

#define TIMEOUT_FOR_COVER_CLOSING 			15000
#define TIMEOUT_FOR_SAMPLING_BOX_FILLING 	20000
#define MODBUS_SLAVE_DEV_ID   				7

// Modbus Register Configuration
#define RAINING_STATUS_REGISTER 			10
#define RAIN_BOX_FULL_STATUS_REGISTER 		11
#define RAIN_BOX_EMPTY_STATUS_REGISTER		12
#define SAMPLE_BOX_FULL_STATUS_REGISTER		13

#define DISCHARGING_VALVE_STATUS_REGISTER  	14
#define SAMPLE_BOX_VALVE_STATUS_REGISTER	15


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_HandleTypeDef heth;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

FutureContract_Handle_t dustBoxCoverTimeoutHandle = -1;
FutureContract_Handle_t rainBoxCoverTimeoutHandle = -1;
FutureContract_Handle_t samplingBoxFillTimeoutHandle = -1;

ModbusRegister_Handle_t rainingRegHandle = { -1 ,0 };
ModbusRegister_Handle_t rainBoxFullRegHandle = { -1 ,0 };
ModbusRegister_Handle_t rainBoxEmptyRegHandle = { -1 ,0 };
ModbusRegister_Handle_t sampleBoxFullRegHandle = { -1 ,0 };

ModbusRegister_Handle_t dischargeValveRegHandle = { -1 ,0 };
ModbusRegister_Handle_t sampleBoxValveRegHandle = { -1 ,0 };

ModbusRegister_Handle_t userLed1RegHandle = { -1 ,0 };
ModbusRegister_Handle_t userLed2RegHandle = { -1 ,0 };
ModbusRegister_Handle_t userLed3RegHandle = { -1 ,0 };
// Timer Callbacks
void onDustBoxCoverClosingTimeout(){
	MotorDriver_Stop();
}

void onRainBoxCoverClosingTimeout(){
	MotorDriver_Stop();
}

void onSamplingBoxFillingTimeout(){
	ValveDriver_CloseSamplingBoxValve();
	ModbusSlave_SetRegisterValue( &sampleBoxValveRegHandle, 0 );
}

void onBme280ReadPeriodTimeout(){

}

/* Actuator dispatchers */
void RainChangeDispatcher( SystemCommand action,const SystemState* state){
	switch( action ){
		case TURN_COVER_MOTOR_TO_RAIN_BOX:
			// set up timer
			rainBoxCoverTimeoutHandle = FutureContracts_Register( TIMEOUT_FOR_COVER_CLOSING, 1, onRainBoxCoverClosingTimeout );
			MotorDriver_CloseRainBox();
			break;
		case TURN_COVER_MOTOR_TO_DUST_BOX:
			// set up timer
			dustBoxCoverTimeoutHandle = FutureContracts_Register( TIMEOUT_FOR_COVER_CLOSING, 1, onDustBoxCoverClosingTimeout );
			MotorDriver_CloseDustBox();
			break;
		case CLOSE_DISCHARGING_VALVE:
			ValveDriver_CloseDischarcingValve();
			ModbusSlave_SetRegisterValue( &dischargeValveRegHandle, 0 );
			break;
		case CLOSE_SAMPLING_BOX_VALVE:
			ValveDriver_CloseSamplingBoxValve();
			ModbusSlave_SetRegisterValue( &sampleBoxValveRegHandle, 0 );
			break;
		default:
			break;
	}
}

void valveDispatcher(SystemCommand action, const SystemState* state ){
	switch( action ){
	case OPEN_DISCHARGING_VALVE:
		ValveDriver_OpenDischarcingValve();
		ModbusSlave_SetRegisterValue( &dischargeValveRegHandle, 1 );
		break;
	case CLOSE_DISCHARGING_VALVE:
		ValveDriver_CloseDischarcingValve();
		ModbusSlave_SetRegisterValue( &dischargeValveRegHandle, 0 );
		break;
	case OPEN_SAMPLING_BOX_VALVE:
		samplingBoxFillTimeoutHandle = FutureContracts_Register( TIMEOUT_FOR_SAMPLING_BOX_FILLING, 1, onSamplingBoxFillingTimeout );
		ValveDriver_OpenSamplingBoxValve();
		ModbusSlave_SetRegisterValue( &sampleBoxValveRegHandle, 1 );
		break;
	case CLOSE_SAMPLING_BOX_VALVE:
		ValveDriver_CloseSamplingBoxValve();
		ModbusSlave_SetRegisterValue( &sampleBoxValveRegHandle, 0 );
		break;
	default:
		break;
	}
}

void rainBoxCoverChangeActuator( SystemCommand action, const SystemState* state ){
	switch(action){
		case STOP_COVER_MOTOR:
			FutureContracts_Unregister(&rainBoxCoverTimeoutHandle);
			MotorDriver_onRainBoxClosed();
			break;
		default:
			break;
	}
}

void dustBoxCoverChangeActuator( SystemCommand action, const SystemState* state ){
	switch(action){
		case STOP_COVER_MOTOR:
			FutureContracts_Unregister(&dustBoxCoverTimeoutHandle);
			MotorDriver_onDustBoxClosed();
			break;
		default:
			break;
	}
}

/* Sensor state changes */
void onRainSensorUpdate( uint8_t newState ){
	if( newState == GPIO_PIN_SET ){
		StateMachine_Act( RAIN_STARTED, RainChangeDispatcher );
		ModbusSlave_SetRegisterValue( &rainingRegHandle, 1 );
	}else{
		StateMachine_Act( RAIN_FINISHED, RainChangeDispatcher );
		ModbusSlave_SetRegisterValue( &rainingRegHandle, 0 );
	}
}

void onRainBoxTopSensorChange( uint8_t newState ){
	if( newState == GPIO_PIN_SET ){
		StateMachine_Act( RAIN_BOX_FILLED, valveDispatcher );
		ModbusSlave_SetRegisterValue( &rainBoxFullRegHandle, 1 );
	}else{
		StateMachine_Act( RAIN_BOX_STARTED_DRAINING, valveDispatcher );
		ModbusSlave_SetRegisterValue( &rainBoxFullRegHandle, 0 );
	}
}

void onRainBoxBottomSensorChange( uint8_t newState ){
	if( newState == GPIO_PIN_SET ){
		StateMachine_Act( RAIN_BOX_STARTED_FILLING, valveDispatcher );
		ModbusSlave_SetRegisterValue( &rainBoxEmptyRegHandle, 0 );
	}else{
		StateMachine_Act( RAIN_BOX_EMPTIED, valveDispatcher );
		ModbusSlave_SetRegisterValue( &rainBoxEmptyRegHandle, 1 );
	}
}

void onSamplingBoxTopSensorChange( uint8_t newState ){
	if( newState == GPIO_PIN_SET ){
		StateMachine_Act( SAMPLING_BOX_FILLED, valveDispatcher );
		ModbusSlave_SetRegisterValue( &sampleBoxFullRegHandle, 1 );
	}else{
		StateMachine_Act( SAMPLING_BOX_STARTED_DRAINING, valveDispatcher );
		ModbusSlave_SetRegisterValue( &sampleBoxFullRegHandle, 0 );
	}
}

void onRainBoxCoverSensorChange( uint8_t newState ){
	if( newState == GPIO_PIN_SET ){
		StateMachine_Act( RAIN_BOX_COVER_CLOSED, rainBoxCoverChangeActuator );
	}else{
		StateMachine_Act( RAIN_BOX_COVER_OPENED, rainBoxCoverChangeActuator );
	}
}

void onDustBoxCoverSensorChange( uint8_t newState ){
	if( newState == GPIO_PIN_SET ){
		StateMachine_Act( DUST_BOX_COVER_CLOSED, dustBoxCoverChangeActuator );
	}
	else{
		StateMachine_Act( DUST_BOX_COVER_OPENED, dustBoxCoverChangeActuator );
	}
}


// Modbus Register Callbacks
void led1RegOnWrite(uint16_t value){
	HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,(GPIO_PinState)value);
}

void led2RegOnWrite(uint16_t value){
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,(GPIO_PinState)value);
}

void led3RegOnWrite(uint16_t value){
	HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,(GPIO_PinState)value);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if( huart->Instance == USART3 ){
		ModbusRtu_Slave_onReceive();
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if( huart->Instance == USART3 ){
		HAL_GPIO_TogglePin( LD2_GPIO_Port, LD2_Pin );
	}
}


void ModbusRtu_InitCallback(){
	//HAL_UART_Receive_IT( &huart3, &data, 1);
}

void ModbusRtu_TransmitCallback( uint8_t* buffer, uint16_t lengthOfBuffer){
	HAL_UART_Transmit( &huart3, buffer, lengthOfBuffer, 200 );
}

void ModbusRtu_ReadRequestCallback( uint8_t* buffer, uint16_t length ){
	HAL_UART_Receive_IT(&huart3, buffer, length );
}

ModbusRtu_Config_t    _rtuConfig;
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
  StateMachine_Init();
  FutureContracts_Init();
  EventGenerator_AddInput( 100, 10, GPIO_PIN_RESET, USER_Btn_GPIO_Port, USER_Btn_Pin, onRainSensorUpdate );
  EventGenerator_AddInput( 100, 10, GPIO_PIN_RESET, GPIOD, GPIO_PIN_0, onDustBoxCoverSensorChange );
  EventGenerator_AddInput( 100, 10, GPIO_PIN_RESET, GPIOD, GPIO_PIN_1, onRainBoxCoverSensorChange );

  EventGenerator_AddInput( 100, 10, GPIO_PIN_RESET, GPIOF, GPIO_PIN_8, onRainBoxTopSensorChange );
  EventGenerator_AddInput( 100, 10, GPIO_PIN_RESET, GPIOF, GPIO_PIN_7, onRainBoxBottomSensorChange );
  EventGenerator_AddInput( 100, 10, GPIO_PIN_RESET, GPIOF, GPIO_PIN_9, onSamplingBoxTopSensorChange );
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_ETH_Init();
  MX_USART3_UART_Init();
  //MX_USB_OTG_FS_PCD_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  _rtuConfig.init = ModbusRtu_InitCallback;
  _rtuConfig.send = ModbusRtu_TransmitCallback;
  _rtuConfig.requestRead = ModbusRtu_ReadRequestCallback;
  _rtuConfig.id = MODBUS_SLAVE_DEV_ID;

  ModbusRtu_Slave_Init( &_rtuConfig );
  // demo
  ModbusSlave_CreateHoldingRegister( 10, 0x3441 );
  ModbusSlave_CreateHoldingRegister( 11, 0x1209 );

  rainingRegHandle = ModbusSlave_CreateInputStatus( RAINING_STATUS_REGISTER, 0 );
  rainBoxFullRegHandle = ModbusSlave_CreateInputStatus( RAIN_BOX_FULL_STATUS_REGISTER, 0 );
  rainBoxEmptyRegHandle = ModbusSlave_CreateInputStatus( RAIN_BOX_EMPTY_STATUS_REGISTER, 1 );
  sampleBoxFullRegHandle = ModbusSlave_CreateInputStatus( SAMPLE_BOX_FULL_STATUS_REGISTER, 0 );

  dischargeValveRegHandle = ModbusSlave_CreateInputStatus( DISCHARGING_VALVE_STATUS_REGISTER, 0 );
  sampleBoxValveRegHandle = ModbusSlave_CreateInputStatus( SAMPLE_BOX_VALVE_STATUS_REGISTER, 0 );

  userLed1RegHandle = ModbusSlave_CreateCoilStatus(10, 0 );
  userLed2RegHandle = ModbusSlave_CreateCoilStatus(11, 0 );
  userLed3RegHandle = ModbusSlave_CreateCoilStatus(12, 0 );

  ModbusSlave_AddOnWriteCallback(&userLed1RegHandle, led1RegOnWrite);
  ModbusSlave_AddOnWriteCallback(&userLed2RegHandle, led2RegOnWrite);
  ModbusSlave_AddOnWriteCallback(&userLed3RegHandle, led3RegOnWrite);


  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */
    
  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE6 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE12 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  const int pool_period = 50;
  for(;;)
  {
    osDelay(pool_period);
    EventGenerator_ReadInputs(pool_period);
    FutureContracts_Work(pool_period);
    HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
