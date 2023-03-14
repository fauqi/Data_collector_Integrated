/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"CANbus.h"
#include "stdio.h"
#include <stdbool.h>
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <rs485-bus.h>
#include <rs485-bus/slave.h>
#include <dma_streamer.h>
//#include "serial_com.h"
#include <report.h>
#include "mode.h"
#include"ws2812.h"
#include"FLASH_PAGE_F1.h"
//#include"FLASH_SECTOR_H7.h"
//#include"backup.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS485_SLAVE_ID_1 1
#define RS485_SLAVE_ID_2 2
#define BUF_UART_RX_SZ 1024


#define EVENT_REQ_REPORT 0x01
#define EVENT_START_CHARGING 0x02
#define EVENT_STOP_CHARGING 0x03
#define EVENT_LOCK 0x04
#define EVENT_UNLOCK 0x05

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_FilterTypeDef sFilterConfig2;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;

uint32_t data[]={0x11111111,0x22222222};
uint32_t data2[]={0x00000000,0x00000000};

struct {
 	unsigned int pc;
 	unsigned int led;
 	unsigned int RS485;
 	unsigned int backup;
 } tick;

  slot slot1,slot2;
  bool report_finish=0;
  unsigned int current_millis=0;
  unsigned int can_tx_tick=0;

  char b[15]="\r\n";
  char koma[15]=",";
  char header[15]="$fauqi";
  char buff[99];
  char cVbatt[16];
  char cAbatt[16];
  char cbatt_SOC[16];
  char cbatt_temp[16];
  char cVcharger[16];
  char cAcharger[16];

  FDCAN_RxHeaderTypeDef RxHeader2;
  FDCAN_TxHeaderTypeDef TxHeader2;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch2;
DMA_HandleTypeDef hdma_tim1_ch3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

RS485_Bus_SlaveHandler rs485_hslave;

STRM_handlerTypeDef hdma;
static uint8_t UartBufferRx[BUF_UART_RX_SZ];
static uint8_t eventRespSuccess = 0;
static uint8_t eventRespError = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MX_RS485_Slave_Init(void);

static int UartRead(uint8_t *buffer, uint16_t bufferLen);
static int UartWrite(uint8_t *buffer, uint16_t bufferLen);

static void onEventReqReport(RS484_BUS_Id_t,
                             uint8_t *buffer, uint16_t bufferLen,
                             RS485_Bus_EventResponse_t *resp);
static void onEventStartCharging(RS484_BUS_Id_t,
                                 uint8_t *buffer, uint16_t bufferLen,
                                 RS485_Bus_EventResponse_t *resp);
static void onEventStopCharging(RS484_BUS_Id_t,
                                uint8_t *buffer, uint16_t bufferLen,
                                RS485_Bus_EventResponse_t *resp);
static void onEventLock(RS484_BUS_Id_t,
                                uint8_t *buffer, uint16_t bufferLen,
                                RS485_Bus_EventResponse_t *resp);
static void onEventUnlock(RS484_BUS_Id_t,
                                uint8_t *buffer, uint16_t bufferLen,
                                RS485_Bus_EventResponse_t *resp);
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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_RS485_Slave_Init();
  CAN_config();
  slot1.notif=led_standby;
  slot2.notif=led_standby;
  slot1.state = standby;
  slot2.state=standby;

  slot1.wake_trig=1;
  slot2.wake_trig=1;
  slot1.last_lock_state=1;
  slot2.last_lock_state=1;
  slot1.charger_start_DIS=1;
  slot2.charger_start_DIS=1;
  slot1.unlock_command=1;
  slot2.unlock_command=1;
  slot1.batt_state=0x01;
  slot2.batt_state=0x01;

  slot1.datasentflag=0;
  slot2.datasentflag=1;

  fault_led(&slot1);
  fault_led(&slot2);

  HAL_GPIO_WritePin(TX_En_GPIO_Port, TX_En_Pin, GPIO_PIN_RESET);
  HAL_UART_Receive_DMA(&huart1, UartBufferRx,BUF_UART_RX_SZ);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /*
	   * 											Data Collector work flow
	   * mula-mula kondisi slot adalah stanby, pada kondisi stanby ini triger wake akan bernilai HIGH dan aktuator akan open,pada kondisi konektor belum dipasangkan pada baterai sehingga CAN baterai belum terbaca
	   * pada kondisi stanby ini, jika terjadi komunikasi error pada charger maka sistem akan mengirimkan informasi berupa CAN communication Error, sedangkan jika baterai yg error tidak terjadi apa-apa karena dianggap baterai masih belum ditancapkan
	   * pada kondisi standby saat terjadi hardware failure maka akan masuk mode fault
	   *
	   * saat konektor ditancapkan pada baterai dan CAN dari baterai terdeteksi oleh data collector, maka triger wakeup baterai akan low dan mode berubah menjadi swap
	   * pada kondisi mode swap, jika CAN baterai kembali tidak terdeteksi maka mode akan kembali menjadi stanby dan triger wakeup akan kembali HIGH,
	   * pada kondisi mode swap jika terjadi error pada charger maka akan mengirimkan notif CAN error state dan sistem tidak bisa dijalankan(lock/start charging)
	   * pada kondisi mode swap jika diperintah start charging maka perintah tidak akan dilaksanakan, melainkan aktuator harus dipastikan dalam keadaan mengunci dan tidak ada error(fault) pada baterai maupun charger
	   * seteleah kondisi terpenuhi dan perintah start charging diterima, maka mode akan menjadi charging kemudian batre dan charger akan diset untuk charge
	   * pada mode swap, saat diberikan perintah lock maka aktuator akan lock, saat unlock maka akan unlock
	   * pada kondisi mode swap saat terjadi hardware failure maka akan masuk mode fault
	   *
	   * pada mode charging, saat CAN baik dari batre maupun charger tidak terbaca selama 5 detik maka akan masuk mode fault, namun jika CAN nya kembali maka akan tetap lanjut charging
	   * pada mode charging, saat perintah unlock diterima namun kondisi charger masih dalam charging state maka unlock dihiraukan,
	   * pada mode charging, ketika perintah stop charging diberikan maka charger dan bms akan diset ke state open,
	   * pada mode charging, ketika baterai sudah full maka led akan masuk mode stanby
	   * pada mode charging, ketika perintah unlock diterima dan charging sudah di stop maka aktuator akan dibuka dan led akan berubah menjadi mode ready_pick hingga CAN terputus,setelah itu masuk mode stanby
	   * pada mode charging, saat terjadi hardware failure maka akan masuk mode fault
	   *
	   * saat kondisi slot statusnya Fault: jika status sebelumnya adalah standby dan errornya adalah CAN communication error pada charger, maka hanya mengirimkan error tersebut dan saat CAN kembali maka mode akan otomatis ke standby
	   * 									  jika status sebelumnya adalah swap dan CAN baterai hilang maka mode akan dikembalikan ke standby
	   * 									  jika status sebelumnya adalah charging, saat jika baterai saja yang CAN nya hilang maka akan terindikasi adanya pencurian
	   * 									  jika baterai dan charger yang hilang maka yang diindikasi error adalah komunikasinya
	   *	saat kondisi hardware failure, maka charger dan bms akan diopen
	   */

//slot1.notif = led_swap;
//slot2.notif = led_swap;


	  switch(slot1.notif)
	  {
	  case led_standby:
		  standby_led(&slot1);
		  break;
	  case led_swap:
		  swap_led(&slot1);
		  break;
	  case led_charging:
		   charging_led(&slot1);
		   break;
	  case led_fault:
		  fault_led(&slot1);
		  break;
	  case led_ready_pick:
		  ready_pick_led(&slot1);
		  break;
	  default:
		  standby_led(&slot1);
		  break;
	  }

	  switch(slot2.notif)
	  {
	  case led_standby:
		  standby_led(&slot2);
		  break;
	  case led_swap:
		  swap_led(&slot2);
		  break;
	  case led_charging:
		   charging_led(&slot2);
		   break;
	  case led_fault:
		  fault_led(&slot2);
		  break;
	  case led_ready_pick:
		  ready_pick_led(&slot2);
		  break;
	  default:
		  standby_led(&slot2);
		  break;
	  }

	  RS485_Bus_Slave_Process(&rs485_hslave);
	  if(slot1.batt_protocol1==1&&slot1.batt_protocol2==1)slot1.isBattery=1;
	  else slot1.isBattery=0;
	  if(slot2.batt_protocol1==1&& slot2.batt_protocol2==1)slot2.isBattery=1;
	  else slot2.isBattery=0;

	  switch(slot1.state)
		  {
		  case standby:
			  slot1.notif=led_standby;
			  standby_mode(&slot1);
			  break;
		  case charging:
			  if(slot1.SOC_batt<100&&slot1.charge_state==1)
			  slot1.notif=led_charging;
			  charging_mode(&slot1);
			  break;
		  case fault:
			  slot1.notif=led_fault;
			  fault_mode(&slot1);
			  break;
		  case swap:
			  slot1.notif=led_swap;
			  swap_mode(&slot1);
			  break;
		  default:
			  slot1.notif=standby;
			  slot1.state = standby;
			  break;
		  }

	  switch(slot2.state)
		  {
		  case standby:
			  slot2.notif=led_standby;
			  standby_mode(&slot2);
			  break;
		  case charging:
			  if(slot2.SOC_batt<100&&slot2.charge_state==1)
			  slot2.notif=led_charging;
			  charging_mode(&slot2);
			  break;
		  case fault:
			  slot2.notif=led_fault;
			  fault_mode(&slot2);
			  break;
		  case swap:
			  slot2.notif=led_swap;
			  swap_mode(&slot2);
			  break;
		  default:
			  slot2.notif=standby;
			  slot2.state = standby;
			  break;
		  }


		  if(slot1.batt_discharge_overcurrent==1||slot1.batt_charge_overCurrent==1||slot1.batt_short_circuit==1||slot1.batt_discharge_overtempreature==1||slot1.batt_discharge_undertempreature==1|| slot1.batt_charge_overtempreature==1||slot1.batt_charge_undertempreature==1||slot1.batt_under_voltage==1||slot1.batt_over_voltage==1||slot1.batt_over_discharge_capacity==1||slot1.batt_unbalance==1|| slot1.batt_system_failure==1|| slot1.charger_hardware_error==1||slot1.charger_temp==1||slot1.charger_input_voltage==1||slot1.charger_work_condition==1||slot1.charger_communication_error==1)
		  {
			  slot1.hardware_failure=1;
		  }
		  else slot1.hardware_failure=0;

		  if(slot2.batt_discharge_overcurrent==1||slot2.batt_charge_overCurrent==1||slot2.batt_short_circuit==1||slot2.batt_discharge_overtempreature==1||slot2.batt_discharge_undertempreature==1|| slot2.batt_charge_overtempreature==1||slot2.batt_charge_undertempreature==1||slot2.batt_under_voltage==1||slot2.batt_over_voltage==1||slot2.batt_over_discharge_capacity==1||slot2.batt_unbalance==1|| slot2.batt_system_failure==1|| slot2.charger_hardware_error==1||slot2.charger_temp==1||slot2.charger_input_voltage==1||slot2.charger_work_condition==1||slot2.charger_communication_error==1)
		  {
			slot2.hardware_failure=1;

		  }
		  else slot2.hardware_failure=0;

		  if(HAL_GetTick()-can_tx_tick>1000)
		  {
//			  counter_todel=counter_todel+1.0;
//			  Vbatt=Vbatt-(0.001*counter_todel);
//			  ftoa(Vbatt,cVbatt,2);
//			  ftoa(Abatt,cAbatt,1);
//			  ftoa(SOC_batt,cbatt_SOC,2);
//			  ftoa(Vcharger,cVcharger,2);
//			  ftoa(Acharger,cAcharger,2);
//			  intToStr(temp_batt, cbatt_temp,2);
//			  sprintf(buff,"$fauqi,%s,%s,%s,%s,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\r\n",cVbatt,cAbatt,cbatt_SOC,cbatt_temp,cVcharger,cAcharger,charger_start_DIS,charger_hardware_error,charger_temp,charger_input_voltage,charger_work_condition,charger_communication_error,batt_discharge_overcurrent,batt_charge_overCurrent,batt_short_circuit,batt_discharge_overtempreature,batt_discharge_undertempreature,batt_charge_overtempreature,batt_charge_undertempreature,batt_under_voltage,batt_over_voltage,batt_over_discharge_capacity,batt_unbalance,batt_system_failure,batt_charge_state,batt_discharge_state,batt_sleep_state);
//			  strcpy(Tx_Data,buff);
			  CAN_TX();

	//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
	//		  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Data, strlen(Tx_Data), 100);
	//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
			  can_tx_tick=HAL_GetTick();

		  }
		  if(HAL_GetTick()-current_millis>5000)
		  {
			  rutin_cek_can();
			  current_millis=HAL_GetTick();
		  }

		  if(slot1.current_lock_state!=slot1.last_lock_state)
		  {
			  if(slot1.current_lock_state==0)
			  {
				  slot1.trig1_state=0;
				  slot1.trig2_state=1;
			  }
			  else
			  {
				  slot1.trig1_state=1;
				  slot1.trig2_state=0;
			  }
			  slot1.current_millis_trig=HAL_GetTick();
			  slot1.last_lock_state=slot1.current_lock_state;
		  }


		  if(HAL_GetTick()-slot1.current_millis_trig>4000)
		  {
			  slot1.trig1_state=0;
			  slot1.trig2_state=0;
		  }
		  if(HAL_GetTick()-slot2.current_millis_trig>4000)
		  {
			  slot2.trig1_state=0;
			  slot2.trig2_state=0;
		  }
		  if(slot1.wake_trig)HAL_GPIO_WritePin(GNDBMSWAKEUP1_GPIO_Port, GNDBMSWAKEUP1_Pin, GPIO_PIN_SET);
		  else HAL_GPIO_WritePin(GNDBMSWAKEUP1_GPIO_Port, GNDBMSWAKEUP1_Pin, GPIO_PIN_RESET);
		  if(slot2.wake_trig)HAL_GPIO_WritePin(GNDBMSWAKEUP2_GPIO_Port, GNDBMSWAKEUP2_Pin, GPIO_PIN_SET);
		  else HAL_GPIO_WritePin(GNDBMSWAKEUP2_GPIO_Port, GNDBMSWAKEUP2_Pin, GPIO_PIN_RESET);
		  send_led();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 16;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 7;
  hfdcan2.Init.NominalTimeSeg2 = 8;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 79;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG2_AKTUATOR2_Pin|TRIG1_AKTUATOR2_Pin|TRIG2_AKTUATOR1_Pin|TRIG1_AKTUATOR1_Pin
                          |TRIG_ALARM_Pin|GNDBMSWAKEUP2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GNDBMSWAKEUP1_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TX_En_GPIO_Port, TX_En_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRIG2_AKTUATOR2_Pin TRIG1_AKTUATOR2_Pin TRIG2_AKTUATOR1_Pin TRIG1_AKTUATOR1_Pin
                           TRIG_ALARM_Pin GNDBMSWAKEUP2_Pin */
  GPIO_InitStruct.Pin = TRIG2_AKTUATOR2_Pin|TRIG1_AKTUATOR2_Pin|TRIG2_AKTUATOR1_Pin|TRIG1_AKTUATOR1_Pin
                          |TRIG_ALARM_Pin|GNDBMSWAKEUP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GNDBMSWAKEUP1_Pin PB7 */
  GPIO_InitStruct.Pin = GNDBMSWAKEUP1_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TX_En_Pin */
  GPIO_InitStruct.Pin = TX_En_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TX_En_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void MX_RS485_Slave_Init(void)
{
  HAL_UART_Receive_DMA(&huart1, UartBufferRx,BUF_UART_RX_SZ);
  rs485_hslave.read = UartRead;
  rs485_hslave.write = UartWrite;
  RS485_Bus_Slave_Init(&rs485_hslave);
  RS485_Bus_Slave_AddId(&rs485_hslave, RS485_SLAVE_ID_1);
  RS485_Bus_Slave_AddId(&rs485_hslave, RS485_SLAVE_ID_2);

  RS485_Bus_Slave_On(&rs485_hslave, EVENT_REQ_REPORT, onEventReqReport);
  RS485_Bus_Slave_On(&rs485_hslave, EVENT_START_CHARGING, onEventStartCharging);
  RS485_Bus_Slave_On(&rs485_hslave, EVENT_STOP_CHARGING, onEventStopCharging);
  RS485_Bus_Slave_On(&rs485_hslave, EVENT_LOCK, onEventLock);
  RS485_Bus_Slave_On(&rs485_hslave, EVENT_UNLOCK, onEventUnlock);

  hdma.huart = &huart1;
  STRM_Init(&hdma, 0, 0, &UartBufferRx[0],BUF_UART_RX_SZ);

}

static int UartRead(uint8_t *buffer, uint16_t bufferLen)
{

 return (int) STRM_Read(&hdma, buffer, bufferLen, 10);
}

static int UartWrite(uint8_t *buffer, uint16_t bufferLen)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_UART_Transmit(&huart1, buffer, bufferLen, 100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  return bufferLen;
}


static void onEventReqReport(RS484_BUS_Id_t id, uint8_t *buffer, uint16_t bufferLen, RS485_Bus_EventResponse_t *resp)
{
  static int soc = 0;
  soc++;

  Report_ResetFrame(&tmpReportFrame);
  if(id==RS485_SLAVE_ID_1)
  {
  tmpReportFrame.charger.status = slot1.batt_charge_state;
  tmpReportFrame.bms.SOC = slot1.SOC_batt;
  tmpReportFrame.bms.voltage =slot1.Vbatt;
  tmpReportFrame.bms.current=slot1.Abatt;
  tmpReportFrame.bms.SOH=slot1.SOH_batt;
  tmpReportFrame.bms.temperature=slot1.temp_batt;
  tmpReportFrame.bms.id=slot1.batt_id;

  slot1.CAN_error_state=0;
  }
  else if(id==RS485_SLAVE_ID_2)
  {
  tmpReportFrame.charger.status = slot2.batt_charge_state;
  tmpReportFrame.bms.SOC = slot2.SOC_batt;
  tmpReportFrame.bms.voltage =slot2.Vbatt;
  tmpReportFrame.bms.current=slot2.Abatt;
  tmpReportFrame.bms.SOH=slot2.SOH_batt;
  tmpReportFrame.bms.temperature=slot2.temp_batt;
  tmpReportFrame.bms.id=slot2.batt_id;
  slot2.CAN_error_state=0;
  }

  resp->data = (uint8_t*) &tmpReportFrame;
  resp->dataLen = sizeof(ReportFrame_t);
  report_finish=1;

  return;
}

static void onEventStartCharging(RS484_BUS_Id_t id, uint8_t *buffer, uint16_t bufferLen, RS485_Bus_EventResponse_t *resp)
{
  // TODO start charing

	if(id==RS485_SLAVE_ID_1) 	  slot1.charge_command=1;
	else if(id==RS485_SLAVE_ID_2) slot2.charge_command=1;
	resp->data = (uint8_t*) &eventRespSuccess;
	resp->dataLen = sizeof(eventRespSuccess);
	return;
}

static void onEventStopCharging(RS484_BUS_Id_t id, uint8_t *buffer, uint16_t bufferLen, RS485_Bus_EventResponse_t *resp)
{
  // TODO stop charing
	if(id==RS485_SLAVE_ID_1)slot1.stop_charge_command=1;
	else if(id==RS485_SLAVE_ID_2)slot2.stop_charge_command=1;
	resp->data = (uint8_t*) &eventRespSuccess;
	resp->dataLen = sizeof(eventRespSuccess);
	return;
}

static void onEventLock(RS484_BUS_Id_t id, uint8_t *buffer, uint16_t bufferLen, RS485_Bus_EventResponse_t *resp)
{
  // TODO lock aktuator
	if(id==RS485_SLAVE_ID_1) slot1.lock_command=1;
	if(id==RS485_SLAVE_ID_2) slot2.lock_command=1;
	resp->data = (uint8_t*) &eventRespSuccess;
	resp->dataLen = sizeof(eventRespSuccess);
	return;

}

static void onEventUnlock(RS484_BUS_Id_t id, uint8_t *buffer, uint16_t bufferLen, RS485_Bus_EventResponse_t *resp)
{
  // TODO unlock aktuator
	if(id==RS485_SLAVE_ID_1)slot1.unlock_command=1;
	else if(id==RS485_SLAVE_ID_2)slot2.unlock_command=1;
	resp->data = (uint8_t*) &eventRespSuccess;
	resp->dataLen = sizeof(eventRespSuccess);
	return;
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
