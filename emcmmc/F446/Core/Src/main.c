/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/*Command line to load program over CAN
:~~/GliderWinchCommons/embed/svn_discoveryf4/PC/sensor/CANldr/trunk$ ./CANldr 127.0.0.1 32123 B0A00000 ~/GliderWinchItems/BMS/bmsadbms1818/build/bms1818.xbin; echo $?
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "morse.h"
#include "DTW_counter.h"
#include <math.h>

#include "adcparams.h"
#include "SerialTaskSend.h"
#include "SerialTaskReceive.h"
#include "CanTask.h"
#include "can_iface.h"
#include "canfilter_setup.h"
#include "getserialbuf.h"
#include "yprintf.h"
#include "ADCTask.h"
#include "MailboxTask.h"
#include "CanCommTask.h"
#include "CoolingTask.h"
#include "rtcregs.h"
#include "EMCLTask.h"
#include "RyTask.h"
#include "emcl_idx_v_struct.h"
#include "iir_f1.h"
#include "StringChgrTask.h"
#include "stringchgr_items.h"
#include "GatewayTask.h"
#include "DMOCTask.h"
#include "ContactorTask.h"

#include "FreeRTOS.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t timectr = 0;
struct CAN_CTLBLOCK* pctl0; // Pointer to CAN1 control block
struct CAN_CTLBLOCK* pctl1; // Pointer to CAN2 control block

uint32_t debugTX1b;
uint32_t debugTX1b_prev;

uint32_t debugTX1c;
uint32_t debugTX1c_prev;

uint32_t debug03;
uint32_t debug03_prev;

//extern osThreadId SerialTaskHandle;
//extern osThreadId CanTxTaskHandle;
//extern osThreadId CanRxTaskHandle;
//extern osThreadId SerialTaskReceiveHandle;

uint16_t m_trap = 450; // Trap codes for MX Init() and Error Handler

uint8_t canflag;
uint8_t canflag1;
//uint8_t canflag2;

int8_t rtcregs_ret; // RTC register init return

//const uint32_t i_am_canid = I_AM_CANID;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
  /* 
  Timer usage
                         1    2   3   4   auto  auto
  TIM1 charger control  PWM   x   x   x  comp1 comp2
                        PA8               PA6   PA11 
  TIM2 FAN PWM TACH     PWM   x  IC   x    
                        PA5     PB10
  TIM7  System          x     

  TIM15 SPI/ADC/BMS     OC    x
  
  */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
uint16_t errorcode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM9_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

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
  BaseType_t ret;    // Used for returns from function calls
  osThreadId Thrdret;  // Return from thread create
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  DTW_counter_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_TIM5_Init();
  MX_TIM13_Init();
  MX_TIM9_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  /* CAN ID for this node is passed in to make from command line. */
//  i_am_canid = I_AM_CANID;

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  #define NUMCIRBCB1  16 // Size of circular buffer of BCB for usart3
  ret = xSerialTaskSendAdd(&HUARTMON, NUMCIRBCB1, 1); // dma
  if (ret < 0) morse_trap(111); // Panic LED flashing
  
  yprintf_init();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  //morse_trap(333);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  //while(1==1);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 448);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

/* Relay/output handler. */
  //ry_init(); // Task does this
  TaskHandle_t rett = xRyTaskCreate(osPriorityNormal);
  if (rett == NULL) morse_trap(109);

  Thrdret = xADCTaskCreate(osPriorityNormal+2); // (arg) = priority
  if (Thrdret == NULL) morse_trap(117);

/* Create serial task (priority) */
  Thrdret = xSerialTaskSendCreate(osPriorityNormal); // Create task and set Task priority
  if (Thrdret == NULL) morse_trap(112);

  /* Add bcb circular buffer to SerialTaskSend for usart3 -- PC monitor */
  #define NUMCIRBCB3  16 // Size of circular buffer of BCB for usart3
  ret = xSerialTaskSendAdd(&HUARTMON, NUMCIRBCB3, 1); // dma
  if (ret < 0) morse_trap(14); // Panic LED flashing

  /* Setup TX linked list for CAN  */
   // CAN1 (CAN_HandleTypeDef *phcan, uint8_t canidx, uint16_t numtx, uint16_t numrx);
  pctl0 = can_iface_init(&hcan1, 0, 64, 32);       
  if (pctl0 == NULL) morse_trap(118); // Panic LED flashing
  if (pctl0->ret < 0) morse_trap(119);

#ifdef CONFIGCAN2
  pctl1 = can_iface_init(&hcan2, 1,32, 64);
  if (pctl1 == NULL) morse_trap(8); // Panic LED flashing
  if (pctl1->ret < 0) morse_trap(88);  
#endif

  /* Setup CAN hardware filters to default to accept all ids. */
  HAL_StatusTypeDef Cret;
  Cret = canfilter_setup_first(1, &hcan1, 15); // CAN1
  if (Cret == HAL_ERROR) morse_trap(219);

#ifdef CONFIGCAN2
  Cret = canfilter_setup_first(2, &hcan2, 15); // CAN2
  if (Cret == HAL_ERROR) morse_trap(217);
#endif  

/* Create CanTxTask - CAN driver TX interface. */
  // CanTask priority, Number of msgs in queue
  QueueHandle_t QHret = xCanTxTaskCreate(osPriorityNormal+1, 48); 
  if (QHret == NULL) morse_trap(120); // Panic LED flashing

  /* definition and creation of CanRxTask - CAN driver RX interface. */
  /* The MailboxTask takes care of the CANRx                         */
//  Qidret = xCanRxTaskCreate(1, 32); // CanTask priority, Number of msgs in queue
//  if (Qidret < 0) morse_trap(6); // Panic LED flashing

/* Create MailboxTask */
  xMailboxTaskCreate(osPriorityNormal+1); // (arg) = priority

/* Create Mailbox control block w 'take' pointer for each CAN module. */
  struct MAILBOXCANNUM* pmbxret;
  // (CAN1 control block pointer, size of circular buffer)
  pmbxret = MailboxTask_add_CANlist(pctl0, 32);
  if (pmbxret == NULL) morse_trap(215);

#ifdef CONFIGCAN2
    pmbxret = MailboxTask_add_CANlist(pctl1, 32);
    if (pmbxret == NULL) morse_trap(215);
#endif    

/* Create GatewayTask */
  xGatewayTaskCreate(osPriorityNormal); // (arg) = priority  

/* Create serial receiving task. */
  ret = xSerialTaskReceiveCreate(osPriorityNormal);
  if (ret != pdPASS) morse_trap(113);

/* Create EMCLTask */
  Thrdret = xEMCLTaskCreate(osPriorityNormal);
  if (Thrdret == NULL) morse_trap(108);

  /* Initialize hard-code params for functions. */
  emcl_idx_v_struct_hardcode_params(&emclfunction.lc);

/* Create CoolingTask */
  Thrdret = xCoolingTaskCreate(osPriorityNormal);
  if (Thrdret == NULL) morse_trap(110);

/* Create String Charging Task. */  
  Thrdret = xStringChgrTaskCreate(osPriorityNormal);
  if (Thrdret == NULL) morse_trap(114);

/* Create DMOC task. */
  Thrdret = xDMOCTaskCreate(osPriorityNormal);
  if (Thrdret == NULL) morse_trap(116);

/* Create Contactor Task. */
  Thrdret = xContactorTaskCreate(osPriorityNormal);
  if (Thrdret == NULL) morse_trap(122);

/* Create CAN communication task handler. */
  rett = xCanCommCreate(osPriorityNormal+2);
  if (rett == NULL) morse_trap(121);

  /* Select interrupts for CAN1 */
  HAL_CAN_ActivateNotification(&hcan1, \
    CAN_IT_TX_MAILBOX_EMPTY     |  \
    CAN_IT_RX_FIFO0_MSG_PENDING |  \
    CAN_IT_RX_FIFO1_MSG_PENDING    );

#ifdef CONFIGCAN2
/* Select interrupts for CAN2 */
  HAL_CAN_ActivateNotification(&hcan2, \
    CAN_IT_TX_MAILBOX_EMPTY     |  \
    CAN_IT_RX_FIFO0_MSG_PENDING |  \
    CAN_IT_RX_FIFO1_MSG_PENDING    );
#endif  

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 12;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 8;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 10;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = 11;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 10;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  errorcode = 991; // Error_Handler morse_trap code
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  errorcode = 992; // Error_Handler morse_trap code
     /* *** Note: the following htim1.Init.Period setting is
            overwritten in chgr_items.c initialization
            using the parameter tim1_arr_init from the 
            parameter file.
     *** */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 18000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  errorcode = 993; // Error_Handler morse_trap code
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 18000;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  errorcode = 994; // Error_Handler morse_trap code

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 18000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 9000;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 9000;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim9, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 18000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 9000;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream1 on DMA2_Stream1 */
  hdma_memtomem_dma2_stream1.Instance = DMA2_Stream1;
  hdma_memtomem_dma2_stream1.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma2_stream1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream1.Init.Priority = DMA_PRIORITY_LOW;
  hdma_memtomem_dma2_stream1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream1.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RTC_WAKE_FETGATE_GPIO_Port, RTC_WAKE_FETGATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M_RESET_GPIO_Port, M_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED5_GRN_Pin|LED6_RED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : RTC_WAKE_FETGATE_Pin */
  GPIO_InitStruct.Pin = RTC_WAKE_FETGATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RTC_WAKE_FETGATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JP9_THERM_Pin JP8_THERM_Pin JP10_THERM_Pin */
  GPIO_InitStruct.Pin = JP9_THERM_Pin|JP8_THERM_Pin|JP10_THERM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : M_RESET_Pin */
  GPIO_InitStruct.Pin = M_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED5_GRN_Pin LED6_RED_Pin */
  GPIO_InitStruct.Pin = LED5_GRN_Pin|LED6_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Keyswsense_Pin */
  GPIO_InitStruct.Pin = Keyswsense_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Keyswsense_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ACzeroxing_Pin */
  GPIO_InitStruct.Pin = ACzeroxing_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ACzeroxing_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : JP14_IN_Pin */
  GPIO_InitStruct.Pin = JP14_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JP14_IN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
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
  
  struct SERIALSENDTASKBCB* pbuf1 = getserialbuf(&HUARTMON,144);
  if (pbuf1 == NULL) morse_trap(115);
  struct SERIALSENDTASKBCB* pbuf2 = getserialbuf(&HUARTMON,128);
  if (pbuf2 == NULL) morse_trap(125);
struct SERIALSENDTASKBCB* pbuf3 = getserialbuf(&HUARTMON,64);
  if (pbuf3 == NULL) morse_trap(125);
//struct SERIALSENDTASKBCB* pbuf4 = getserialbuf(&HUARTMON,64);
//if (pbuf4 == NULL) morse_trap(125);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // RED ON
osDelay(20);
  yprintf(&pbuf1,"\n\n\rPROGRAM STARTS\n\r");
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // RED OFF
  
//    qret=xQueueSendToBack(RyTaskReadReqQHandle, &rytest[j].preq, portMAX_DELAY);
//    if (qret == errQUEUE_FULL) morse_trap(123);  
//uint32_t ctr =0;
  /* Infinite loop */
  for(;;)
  {   

#if 0    
  extern uint32_t dbgcool1;
    yprintf(&pbuf1,"%5d\n\r", dbgcool1);
#endif

#if 1 // Green LED winking
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // GRN ON
        osDelay(3);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // GRN OFF
      osDelay(250-3);
#else
  static uint32_t yyctr;
  yprintf(&pbuf2,"%5d ",yyctr++);
  osDelay(1001);
#endif

#if 1
  static uint32_t sctr;
//  extern uint8_t do_bms_status_flag;
  extern uint8_t polltim_flag;
  struct STRINGCHGRFUNCTION* ps= &emclfunction.lc.lcstring;
  struct ELCONSTUFF* pe = &ps->elconstuff;    
  extern uint16_t pccmd;
//  if (do_bms_status_flag != 0)
  if (polltim_flag != 0)    
  {
    yprintf(&pbuf1,"%5d, statusmax %02x pccmd %04X ",sctr++,ps->statusmax,pccmd);
    yprintf(&pbuf2,"ststate %d discovery_ctr %d\n\r",ps->stsstate);
//    do_bms_status_flag = 0;
    polltim_flag = 0;
  }

extern uint8_t dbg_do_elcon_poll_flag;
extern uint8_t dbg_do_elcon_poll_status_elcon;
extern int32_t dbg_do_elcon_poll_toctr_elcon_rcv;
  if (dbg_do_elcon_poll_flag > 0)
  {
    yprintf(&pbuf1,"elcon_poll_flag %d  status_elcon %02X toctr_elcon_rcv %d",
      dbg_do_elcon_poll_flag,dbg_do_elcon_poll_status_elcon,dbg_do_elcon_poll_toctr_elcon_rcv);
    dbg_do_elcon_poll_flag = 0;
extern int32_t dbg_amps;
    yprintf(&pbuf3," ichgr_setvolts %d ichgr_maxamps %d dbg_amps %d\n\r",pe->ichgr_setvolts,pe->ichgr_maxamps,dbg_amps);
    yprintf(&pbuf2," chgr_rate_idx %d",ps->chgr_rate_idx);

    /* BMS status: CELLTOOHI for discovered nodes */
    char ln[12];
    char* pln = &ln[0];
    *pln++ = '|';
    for (int n = 0; n < ps->bmsnum; n++)
    { 
      if ((ps->pbmstbl[n]->batt & BSTATUS_CELLTOOHI) != 0)
        *pln = 'X';
      else
        *pln = '.';
      pln +=1;
    }
    *pln++ = '|'; *pln = 0;
extern int32_t toctr_relax;
    yprintf(&pbuf1," TOOHI %s statusmax %02X toctr_relax %d bmsnum %d\n\r",ln,ps->statusmax,ps->toctr_relax,ps->bmsnum);
  }

#endif   

#if 1 // BMS Discovery table data
    struct STRINGCHGRFUNCTION* ps2= &emclfunction.lc.lcstring;
extern uint8_t discovery_end_flag; // Signal main for printf'ing
  if (discovery_end_flag != 0)
  {
    discovery_end_flag = 0;
    yprintf(&pbuf1,"\n\rDiscovery END: Sorted Table----------------\n\r");
    yprintf(&pbuf2,"           v_max i_max i_bal\n\r");
    for (int n = 0; n < ps2->bmsnum; n++)
    {
      yprintf(&pbuf3,"%d %08X %5d %5d %5d\n\r", (n+1),ps->pbmstbl[n]->id,ps->pbmstbl[n]->v_max,ps->pbmstbl[n]->i_max, ps->pbmstbl[n]->i_bal );
    }
  yprintf(&pbuf1,"MAX: %12.1f %4.1f %5.1f\n\r\n\r",ps->chgr_maxvolts,ps->chgr_maxamps,ps->chgr_balamps);
  }

#endif  

#if 0
static uint32_t yyctr;
  static uint8_t bmsnum_prev;
  float fmsum;
  float totalv;
  yprintf(&pbuf1,"bmsnum %d\n\r",emclfunction.lc.lcstring.bmsnum);
  /* Output table when there is a change in size,
     OR, every 10 cycles. */
  if ((bmsnum_prev != emclfunction.lc.lcstring.bmsnum) || 
      ((yyctr % 10) == 0)      )
  { 
    bmsnum_prev = emclfunction.lc.lcstring.bmsnum;
    totalv = 0;
    for (int j = 0; j < emclfunction.lc.lcstring.bmsnum; j++)
    {
      yprintf(&pbuf2,"%2d %08X",j,emclfunction.lc.lcstring.pbmstbl[j]->id);

      // Sum cell readings for =>module<=
      uint32_t msum = 0;
      for (int k = 0; k < 18; k++)
      {
        msum += emclfunction.lc.lcstring.pbmstbl[j]->cell[k];
      }
      fmsum = msum;

      // Display
      if (emclfunction.lc.lcstring.pbmstbl[j]->toctr_cell > 0)
      { // Timeout counter for cell readings has not expired
        yprintf(&pbuf1," %6.3f",fmsum*0.0001f);
        for (int k = 0; k < 18; k++)
        { // Print individual cell readings (100uV)
          yprintf(&pbuf2," %4d",emclfunction.lc.lcstring.pbmstbl[j]->cell[k]);
        }
      }
      else
      { // Timeout expired, so readings are "stale"
        yprintf(&pbuf1," %6.3f",fmsum*0.0001f);
        for (int k = 0; k < 18; k++)
        { // Print individual cell readings (100uV)
          yprintf(&pbuf2,"_stale");
        }  
      }
      // Check gateway_items summation
      yprintf(&pbuf2,"\n\r      vsum %7.3f\n\r",emclfunction.lc.lcstring.pbmstbl[j]->vsum*0.0001f);
      totalv += fmsum; // Sum total battery string volts
    }
    yprintf(&pbuf1,"     total %7.3f\n\r",totalv*0.0001f);

  }
#endif  




#if 0    
  static uint32_t yctr;      
//  yprintf(&pbuf2,"%5d", yctr++);
  struct MOTORRAMP* pmr; //
   yprintf(&pbuf2,"%5d",yctr++);
  for (int k = 0; k < 4; k++)
// int k = 2; // Motor index (0-3) to be monitored
  {
    pmr = &emclfunction.lc.lccool.motorramp[k]; // Working struct
    yprintf(&pbuf1,":%2d %2d %4.1f %3d",pmr->state, pmr->target, 
      pmr->frampaccum,pmr->irampaccum);
  }
  yprintf(&pbuf2,"\n\r");
#endif

#if 0 // pwm testing
static struct RYREQ_Q ryreq_q1;
static struct RYREQ_Q* pryreqpssb;
static uint8_t ryreqinit;
static uint8_t ryrequpdn;
if (ryreqinit == 0)
{
  #define PWMSTEPUP 2 // If zero no remains at initial
  #define PWMSTEPDN 1 // 
  #define PWMMIN 5// Initial and slewing low limit
  #define PWMMAX 70 // Slewing high limit
  ryreqinit = 1;
  ryreq_q1.idx = 10; 
  ryreq_q1.pwm = PWMMIN; // Initial PWM
  ryreq_q1.cancel  = 0;
  pryreqpssb = &ryreq_q1;

}
if (ryrequpdn == 0)
{ // Here, Increase PWM
  ryreq_q1.pwm += PWMSTEPUP;
  if (ryreq_q1.pwm >= PWMMAX)
  {
    ryrequpdn = 1;
    ryreq_q1.pwm = PWMMAX;
  }
}
else
{ // Here Decrease PWM
  ryreq_q1.pwm -= PWMSTEPDN;

  if ((int8_t)ryreq_q1.pwm <= PWMMIN)
  {
    ryrequpdn = 0;
  }
}
xQueueSendToBack(RyTaskReadReqQHandle,&pryreqpssb,10000);
yprintf(&pbuf1,"PWM: %4d\n\r",ryreq_q1.pwm);

#endif      

#if 0
static uint32_t ctr;      
if (ctr == 0)
{
  yprintf(&pbuf1,"ADC1IDX_THERMISTOR1   0 PC0 IN10   JP9  Thermistor\n\r");
  yprintf(&pbuf1,"ADC1IDX_THERMISTOR2   1 PC1 IN11   JP8  Thermistor\n\r");
  yprintf(&pbuf1,"ADC1IDX_THERMISTOR3   2 PC2 IN12   JP10 Thermistor\n\r");
  yprintf(&pbuf1,"ADC1IDX_THERMISTOR4   3 PC3 IN13   JP11 Thermistor\n\r");
  yprintf(&pbuf1,"ADC1IDX_DIVIDEDSPARE  4 PC4 IN14   JP17 Spare: 10k|10k divider\n\r");
  yprintf(&pbuf1,"ADC1IDX_PRESS_SENSE   5 PC5 IN15   JP24 Pressure sensor\n\r");
  yprintf(&pbuf1,"ADC1IDX_12V_POWR      6 PA7 IN7    12v Power supply\n\r");
  yprintf(&pbuf1,"ADC1IDX_BATTLEAK_P    7 PB0 IN8    Battery string plus leakage\n\r");
  yprintf(&pbuf1,"ADC1IDX_BATTLEAK_M    8 PB1 IN9    Battery string minus leakage\n\r");
  yprintf(&pbuf1,"ADC1IDX_12V_POWR_DUP  6 PA7 IN7    12v Power supply (duplicate)\n\r");
  yprintf(&pbuf1,"ADC1IDX_INTERNALVBAT 10 IN18       VBAT\n\r");
  yprintf(&pbuf1,"ADC1IDX_INTERNALVREF 11 IN17       VREF\n\r");
  for(int j = 0; j < ADC1DIRECTMAX; j++)
    yprintf(&pbuf2,"        %2d",j);
  yprintf(&pbuf1,"\n\r");
}
ctr += 1;
if (ctr >= 32) ctr = 0;
{
 #if 1 // List calibrated & filtered.
  for(int j = 0; j < ADC1DIRECTMAX; j++)
  {
    yprintf(&pbuf2," %9.3f",adc1.abs[j].filt);
  }
  yprintf(&pbuf1,"\n\r");
 #endif

extern struct CANRCVBUF cooltest; 
extern uint8_t cooltest1;
  if (cooltest1 != 0)
  {
    cooltest1 = 0;
    yprintf(&pbuf1,"\n\r 0x%08X",cooltest.id);
  }

 #if 0 // List ADC sum (use for calibration)
 extern uint32_t dbg_adcsum[ADC1DIRECTMAX];
 static struct FILTERIIRF1 iiradcsum[ADC1DIRECTMAX]; // iir_f1 (float) filter
 static float fsumsum;
 static uint8_t dbgflag;
 if (dbgflag == 0)
 {
  // OTO initilization
  for(int j = 0; j < ADC1DIRECTMAX; j++)
  {
    iiradcsum[j].skipctr  = 4;
    iiradcsum[j].coef     = 0.95f;
    iiradcsum[j].onemcoef = 1 - iiradcsum[j].coef;  
  }
  dbgflag = 1;
 }
  for(int j = 0; j < ADC1DIRECTMAX; j++)
  {
    float ftmp = dbg_adcsum[j];
    fsumsum = iir_f1_f(&iiradcsum[j], ftmp);
    yprintf(&pbuf2," %9.1f",fsumsum);
//    yprintf(&pbuf2," %9d",dbg_adcsum[j]);
  }
  yprintf(&pbuf1,"\n\r");
 #endif
}
#endif

#if 0 // 60 Hz output ac measurement
float fnum;
float facc;
float fsmq;
float frecip;
float filtoffset;
struct ADC2NUM* pnum = get_adc2num();
/* Setup pbuf1 while pbuf2 sends */
if (pnum == NULL)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // GRN ON 
}
else
{
  fnum = pnum->ctr;
  facc = pnum->acc;
  fsmq = pnum->smq;
  frecip = 1.0/fnum;
  facc = facc * frecip;
  fsmq = sqrtf(fsmq * frecip) * adc1.lc.adc2offset_scale;
  filtoffset = iir_f1_f(&adc1.lc.iir_adc2offset, facc);
  yprintf(&pbuf2,"\n\r%2d %8.2f  %8.2f %8.3f",ctr, facc,filtoffset,fsmq);
  ctr += 1; if (ctr >= 60) ctr = 0;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // GRN OFF
}
/* Setup pbuf2 while pbuf1 sends */
pnum = get_adc2num();
if (pnum == NULL)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // GRN ON 
}
else
{
  fnum = pnum->ctr;
  facc = pnum->acc;
  fsmq = pnum->smq;
  frecip = 1.0/fnum;
  facc = facc * frecip;
  fsmq = sqrtf(fsmq * frecip) * adc1.lc.adc2offset_scale;
  filtoffset = iir_f1_f(&adc1.lc.iir_adc2offset, facc);
  yprintf(&pbuf2,"\n\r%2d %8.2f  %8.2f %8.3f",ctr, facc,filtoffset,fsmq);
  ctr += 1; if (ctr >= 60) ctr = 0;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // GRN OFF
}
osDelay(20); // Do not hog all the time.
#endif


#if 0 // Display (large) buffer of raw ADC2 readings
#ifdef MEMTOMEMCOPY // Uncomment in ADCTask.h
#define DEBUGSZ (ADC2SEQNUM*32)
extern uint32_t debugadcsumidx;
extern uint16_t debugadcsum[];
extern uint16_t debugexti[];
      if (debugadcsumidx >= DEBUGSZ)
      {
        ctr = 0;
        for (int j = 0; j < DEBUGSZ; j++)
        {
          yprintf(&pbuf1,"\n\r%5d %9d",ctr++,debugadcsum[j], debugexti[j]);
          if (debugexti[j] != 0)
            yprintf(&pbuf2," %6d",debugexti[j]);
        }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // GRN OFF
extern uint32_t debugdmamm2;
extern uint32_t debugdmakk2;
yprintf(&pbuf1,"\n\rm-m: %d inline: %d\n\r",debugdmamm2,debugdmakk2);
        osDelay(1000*6);
        debugadcsumidx = 0;
        ctr = 0;
      }
      else
      {
//          yprintf(&pbuf1,"\n\r%5d %9d",ctr++,debugadcsumidx);
      }
#endif
#endif


#if 0 // Display time between EXTI interrupts (60 Hz)
extern uint32_t exti15dtw_diff;
extern uint32_t  exti15dtw_flag;
extern uint32_t  exti15dtw_flag_prev;

extern uint32_t exti15dtw_accum_flag;
extern int32_t  exti15dtw_accum_diff;
if (exti15dtw_accum_flag != 0)
{
      yprintf(&pbuf1,"\n\r%9d %8d %8d",ctr++,exti15dtw_diff,exti15dtw_diff/180);
        exti15dtw_flag_prev = exti15dtw_flag;
}        
//extern uint32_t exti15dtw_irqctr;
      //yprintf(&pbuf2," %8d",exti15dtw_irqctr);

extern uint32_t exti15dtw_reg; 
extern uint32_t debugadc2dma_pdma2;
extern uint32_t adc2dma_cnt;
//        yprintf(&pbuf2," Reg: 0x%08X %7d %7d",adc2dma_cnt,debugadc2dma_pdma2,exti15dtw_reg);
//extern exti15dtw_reg1;
//      yprintf(&pbuf1," %d",exti15dtw_reg1);
#endif

#if 0
extern uint32_t exti15dtw_accum_flag;
extern int32_t  exti15dtw_accum_diff;
if (exti15dtw_accum_flag != 0)
{
  exti15dtw_accum_flag = 0;
  float faccum = (exti15dtw_accum_diff/240);
  faccum = (faccum /180.0f);
  yprintf(&pbuf1," %7d %10.1f %10.4f%% %10.3f",(exti15dtw_accum_diff/240),faccum,100.0f*((1E6f/60.0f)/faccum - 1.0f),(1E6f/faccum));
}
#endif

#if 0
extern struct ADC2NUMALL adc2numall;
float fsmq = adc2numall.diff.smq;
  yprintf(&pbuf1,"\n\r%3d %7d %9d %12.0f",ctr++,adc2numall.diff.ctr,adc2numall.diff.acc,fsmq);
float fctr = adc2numall.diff.ctr;
float facc = adc2numall.diff.acc;
  yprintf(&pbuf2," %8.2f %8.2f",(facc/fctr)  - 1884.5f,sqrtf(fsmq/fctr));
extern uint32_t exti15dtw_diff;
  yprintf(&pbuf1," %d",exti15dtw_diff);
//extern uint32_t debugadct2;
//extern uint32_t debugadct4;
//  yprintf(&pbuf2," %d %d",debugadct2/512,debugadct4/512);

extern uint32_t debug_adc2cnvrsn_ctr;
extern uint32_t debug_dmaidx;
  yprintf(&pbuf2," %6d %6d",debug_adc2cnvrsn_ctr,debug_dmaidx);

#endif

#if 0      

extern uint8_t debug_pdma_flag;
if (debug_pdma_flag)
{
  extern uint32_t debugdma, adc2dma_cnt;
  yprintf(&pbuf1,"\n\rD %d %d",debugdma, adc2dma_cnt);
  yprintf(&pbuf2,"\n\rB %d, %d",sumsqbundle.pdma,sumsqbundle.pdma_end);
  yprintf(&pbuf1,"\n\rB %d, %d",sumsqbundle.adc2ctr,sumsqbundle.adcaccum);
  double fer = sumsqbundle.sumsq;
  yprintf(&pbuf2," %14.0f",fer);
  yprintf(&pbuf1,"\n\rERROR HANG\n\r");
  while(1==1);
}
#endif

#if 0 
float fav = 0;
float favq = 0;
float f10;
float f21;
float f22;
float fsq;
if (debugnum_flag != 0)  
{
  for (int k=3; k < DEBUGNUMSIZE; k++)
  {
    f10 = (1.0/(float)debugnum[k].ctr);
    f21 = debugnum[k].acc;
    f22 = debugnum[k].smq;
    fsq = sqrtf(f22*f10) * .009810806f;
    fav += f21;
    favq += fsq;
    
    yprintf(&pbuf1,"\n\r%3d %7d %12.2f %12.3f",ctr++,debugnum[k].ctr,f21*f10,fsq);
//    yprintf(&pbuf1,"\n\r%3d %7d %6d %4d",ctr++,debugnum[k].ctr,debugnum[k].acc,debugnum[k].idx);
  }
  yprintf(&pbuf2,"\n\n\rAVE:         %12.3f %12.4f",(f10*fav)/(ctr),(favq/ctr));
  
  debugnum_flag = 0;
  ctr = 0;

uint32_t adc2getsyscycle(void);
yprintf(&pbuf1,"\n\r syscycle_per_adcconversion: %d",adc2getsyscycle());

  yprintf(&pbuf2,"\n\r\n");
  osDelay(2000);
}

  }
}
#endif





  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
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
  morse_trap(2222);
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  morse_trap(3339);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
