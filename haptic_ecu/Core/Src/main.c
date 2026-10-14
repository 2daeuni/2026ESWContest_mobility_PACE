/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Haptic ECU
  *
  * Raspberry Pi Safety ECU
  *        |
  *        | CAN 0x110 HAPTIC_COMMAND
  *        v
  * STM32F446RE Haptic ECU
  *        |
  *        | USB HID Feature Report
  *        v
  * SIMAGIC P2000 HCB -> P-HPR
  *
  * STM32F446RE
  *        |
  *        | CAN 0x200 HAPTIC_STATUS
  *        v
  * Raspberry Pi Safety ECU
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usbh_hid.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t risk_level;
  uint8_t vibration_command;
  uint8_t intensity;
  uint8_t frequency;
  uint8_t alive_counter;
} HapticCommand_t;
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* =========================================================
 * P2000
 * ========================================================= */
#define P2000_CHANNEL              0U
/* =========================================================
 * CAN Message ID
 * ========================================================= */
#define HAPTIC_COMMAND_ID          0x110U
#define HAPTIC_STATUS_ID           0x200U
/* =========================================================
 * CAN Timing
 * ========================================================= */
#define HAPTIC_STATUS_PERIOD_MS    10U
#define HAPTIC_COMMAND_TIMEOUT_MS  100U
/* =========================================================
 * Fault Code
 * ========================================================= */
#define FAULT_NO_FAULT             0U
#define FAULT_ACTUATOR             1U
#define FAULT_DRIVER               2U
#define FAULT_INVALID_COMMAND      3U
#define FAULT_COMMUNICATION        4U
/* ECU Status */
#define ECU_STATUS_ERROR           0U
#define ECU_STATUS_NORMAL          1U
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
extern USBH_HandleTypeDef hUsbHostFS;
/* =========================================================
 * P2000
 * ========================================================= */
static uint8_t p2000_report[49] = {0};
static volatile uint8_t p2000_send_pending = 0U;
static uint8_t p2000_current_intensity = 0U;
static uint8_t p2000_current_frequency = 0U;
static uint8_t p2000_current_status    = 0U;

/* =========================================================
 * CAN RX
 * ========================================================= */
static CAN_RxHeaderTypeDef can_rx_header;
static uint8_t can_rx_data[8];

/* ISR -> Main 전달용 */
static volatile HapticCommand_t pending_command;
static volatile uint8_t command_pending = 0U;

/* =========================================================
 * 현재 Haptic ECU 상태
 * ========================================================= */
static uint8_t current_risk_level = 0U;
static volatile uint8_t fault_code = FAULT_NO_FAULT;
static volatile uint8_t command_timeout = 1U;
static uint8_t ecu_status = ECU_STATUS_NORMAL;

/* 마지막 정상 HAPTIC_COMMAND 수신 시간 */
static volatile uint32_t last_command_tick = 0U;
/* 한 번이라도 정상 명령을 받았는지 */
static volatile uint8_t command_received_once = 0U;

/* =========================================================
 * CAN STATUS
 * ========================================================= */
static uint32_t last_status_tick = 0U;
static uint8_t status_alive_counter = 0U;

/* Timeout 시 Stop 명령을 계속 생성하지 않도록 */
static uint8_t timeout_stop_done = 0U;
/* HAPTIC_COMMAND Alive Counter 감시 */
static volatile uint8_t previous_command_alive = 0U;
static volatile uint8_t alive_received_once = 0U;
/* P2000 USB 연결 상태 감시 */
static uint8_t p2000_usb_ready = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void P2000_SetHaptic(uint8_t intensity,
                     uint8_t frequency);
void P2000_StopHaptic(void);
static void CAN_Filter_Config(void);
static uint8_t HapticCommand_IsValid(
    const HapticCommand_t *cmd,
    const uint8_t *raw_data);
static void HapticCommand_Process(void);
static void HapticTimeout_Process(void);
static void HapticStatus_Send(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* =========================================================
 * printf -> USART2
 * ========================================================= */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2,
                    (uint8_t *)ptr,
                    len,
                    HAL_MAX_DELAY);
  return len;
}

/* =========================================================
 * P2000 Haptic ON
 *
 * intensity : 1 ~ 100 %
 * frequency : 10 ~ 50 Hz
 *
 * 실제 USB 전송은 main while()에서 처리
 * ========================================================= */
void P2000_SetHaptic(uint8_t intensity,
                     uint8_t frequency)
{
  memset(p2000_report,
         0,
         sizeof(p2000_report));
  /*
   * P2000 Feature Report
   *
   * Byte 0 : 0xF1
   * Byte 1 : 0xEC
   * Byte 2 : Channel
   * Byte 3 : Enable
   * Byte 4 : Frequency
   * Byte 5 : Intensity
   */
  p2000_report[0] = 0xF1U;
  p2000_report[1] = 0xECU;
  p2000_report[2] = P2000_CHANNEL;
  p2000_report[3] = 0x01U;
  p2000_report[4] = frequency;
  p2000_report[5] = intensity;
  /*
   * 아직 USB 전송이 완료된 것은 아니므로
   * 실제 적용 상태는 USBH_OK 이후 갱신한다.
   */
  p2000_send_pending = 1U;
}

/* =========================================================
 * P2000 Haptic OFF
 * ========================================================= */
void P2000_StopHaptic(void)
{
  memset(p2000_report,
         0,
         sizeof(p2000_report));
  p2000_report[0] = 0xF1U;
  p2000_report[1] = 0xECU;
  p2000_report[2] = P2000_CHANNEL;
  p2000_report[3] = 0x00U;
  p2000_report[4] = 0x00U;
  p2000_report[5] = 0x00U;
  p2000_send_pending = 1U;
}

/* =========================================================
 * CAN Filter
 *
 * HAPTIC_COMMAND
 * ID = 0x110
 *
 * 만 수신
 * ========================================================= */
static void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef filter;
  memset(&filter, 0, sizeof(filter));
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  /*
   * Standard CAN ID는 Filter register에서
   * 5 bit left shift
   */
  filter.FilterIdHigh =
      (uint16_t)(HAPTIC_COMMAND_ID << 5U);
  filter.FilterIdLow = 0x0000U;
  /*
   * 0x7FF -> Standard ID 11bit 전체 비교
   */
  filter.FilterMaskIdHigh =
      (uint16_t)(0x7FFU << 5U);
  filter.FilterMaskIdLow = 0x0000U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;
  if (HAL_CAN_ConfigFilter(&hcan1,
                           &filter) != HAL_OK)
  {
    Error_Handler();
  }
}

/* =========================================================
 * HAPTIC_COMMAND 유효성 검사
 *
 * Byte 0 : Risk Level
 * Byte 1 : Vibration Command
 * Byte 2 : Intensity
 * Byte 3 : Frequency
 * Byte 4 : Reserved = 0
 * Byte 5 : Reserved = 0
 * Byte 6 : Reserved = 0
 * Byte 7 : Alive Counter
 * ========================================================= */
static uint8_t HapticCommand_IsValid(
    const HapticCommand_t *cmd,
    const uint8_t *raw_data)
{
  /* Risk Level : 0 ~ 3 */
  if (cmd->risk_level > 3U)
  {
    return 0U;
  }
  /* Vibration Command : 0 or 1 */
  if (cmd->vibration_command > 1U)
  {
    return 0U;
  }
  /* Reserved bytes */
  if ((raw_data[4] != 0x00U) ||
      (raw_data[5] != 0x00U) ||
      (raw_data[6] != 0x00U))
  {
    return 0U;
  }
  /*
   * OFF
   *
   * Command   = 0
   * Intensity = 0
   * Frequency = 0
   */
  if (cmd->vibration_command == 0U)
  {
    if ((cmd->intensity != 0U) ||
        (cmd->frequency != 0U))
    {
      return 0U;
    }
    return 1U;
  }
  /*
   * ON
   *
   * Intensity : 1 ~ 100 %
   * Frequency : 10 ~ 50 Hz
   */
  if ((cmd->intensity < 1U) ||
      (cmd->intensity > 100U))
  {
    return 0U;
  }
  if ((cmd->frequency < 10U) ||
      (cmd->frequency > 50U))
  {
    return 0U;
  }
  return 1U;
}

/* =========================================================
 * CAN으로 받은 HAPTIC_COMMAND 처리
 * ========================================================= */
static void HapticCommand_Process(void)
{
  HapticCommand_t cmd;
  if (command_pending == 0U)
  {
    return;
  }
  /*
   * ISR이 쓰는 데이터를 짧게 보호해서 복사
   */
  __disable_irq();
  cmd.risk_level =
      pending_command.risk_level;
  cmd.vibration_command =
      pending_command.vibration_command;
  cmd.intensity =
      pending_command.intensity;
  cmd.frequency =
      pending_command.frequency;
  cmd.alive_counter =
      pending_command.alive_counter;
  command_pending = 0U;
  __enable_irq();

  current_risk_level = cmd.risk_level;

  /*
   * OFF 명령
   */
  if (cmd.vibration_command == 0U)
  {
    P2000_StopHaptic();
    printf("[CAN RX] 0x110 "
           "Risk=%u OFF Alive=%u\r\n",
           cmd.risk_level,
           cmd.alive_counter);
  }
  /*
   * ON 명령
   */
  else
  {
    P2000_SetHaptic(cmd.intensity,
                    cmd.frequency);
    printf("[CAN RX] 0x110 "
           "Risk=%u ON "
           "Intensity=%u%% "
           "Frequency=%uHz "
           "Alive=%u\r\n",
           cmd.risk_level,
           cmd.intensity,
           cmd.frequency,
           cmd.alive_counter);
  }
}

/* =========================================================
 * HAPTIC_COMMAND Timeout
 *
 * 100 ms 이상 명령이 없으면 Fail-Safe OFF
 * ========================================================= */
static void HapticTimeout_Process(void)
{
  uint32_t now = HAL_GetTick();
  /*
   * 아직 정상 명령을 한 번도 받지 않은 경우
   * 안전상 OFF 상태 유지
   */
  if (command_received_once == 0U)
  {
    command_timeout = 1U;
    return;
  }

  if ((now - last_command_tick) >=
      HAPTIC_COMMAND_TIMEOUT_MS)
  {
    command_timeout = 1U;
    current_risk_level = 0U;
    /*
     * Timeout 발생 시 딱 한 번만
     * P2000 OFF 전송
     */
    if (timeout_stop_done == 0U)
    {
      P2000_StopHaptic();
      timeout_stop_done = 1U;
      printf("[CAN] HAPTIC_COMMAND TIMEOUT -> Haptic OFF\r\n");
    }
  }
}

/* =========================================================
 * HAPTIC_STATUS
 *
 * CAN ID : 0x200
 * DLC    : 8
 *
 * Byte 0 : Current Risk Level
 * Byte 1 : Vibration Status
 * Byte 2 : Applied Intensity
 * Byte 3 : Applied Frequency
 * Byte 4 : Fault Code
 * Byte 5 : Command Timeout
 * Byte 6 : ECU Status
 * Byte 7 : Alive Counter
 * ========================================================= */
static void HapticStatus_Send(void)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];
  uint32_t tx_mailbox;

  tx_header.StdId = HAPTIC_STATUS_ID;
  tx_header.ExtId = 0U;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8U;
  tx_header.TransmitGlobalTime = DISABLE;

  tx_data[0] = current_risk_level;
  tx_data[1] = p2000_current_status;
  tx_data[2] = p2000_current_intensity;
  tx_data[3] = p2000_current_frequency;
  tx_data[4] = fault_code;
  tx_data[5] = command_timeout;
  tx_data[6] = ecu_status;
  tx_data[7] = status_alive_counter;

  /*
   * Mailbox가 있을 때만 비동기 송신
   */
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
  {
    if (HAL_CAN_AddTxMessage(&hcan1,
                             &tx_header,
                             tx_data,
                             &tx_mailbox) == HAL_OK)
    {
      /*
       * STATUS Alive Counter는 STM32가 독립 생성
       * 255 -> 0 자동 rollover
       */
      status_alive_counter++;
    }
  }
}

/* =========================================================
 * CAN RX FIFO0 Interrupt Callback
 * ========================================================= */
void HAL_CAN_RxFifo0MsgPendingCallback(
    CAN_HandleTypeDef *hcan)
{
  HapticCommand_t cmd;

  if (hcan->Instance != CAN1)
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan,
                           CAN_RX_FIFO0,
                           &can_rx_header,
                           can_rx_data) != HAL_OK)
  {
    return;
  }

  /*
   * Standard ID / Data Frame / DLC 검사
   */
  if ((can_rx_header.IDE != CAN_ID_STD) ||
      (can_rx_header.RTR != CAN_RTR_DATA) ||
      (can_rx_header.StdId != HAPTIC_COMMAND_ID) ||
      (can_rx_header.DLC != 8U))
  {
    return;
  }

  /*
   * 0x110 Parsing
   */
  cmd.risk_level =
      can_rx_data[0];
  cmd.vibration_command =
      can_rx_data[1];
  cmd.intensity =
      can_rx_data[2];
  cmd.frequency =
      can_rx_data[3];
  cmd.alive_counter =
      can_rx_data[7];

  /*
   * 유효하지 않은 명령
   */
  if (HapticCommand_IsValid(&cmd,
                            can_rx_data) == 0U)
  {
    /*
     * ISR에서는 USB 제어를 직접 하지 않음.
     *
     * OFF 명령을 main loop로 전달.
     */
    pending_command.risk_level =
        0U;
    pending_command.vibration_command =
        0U;
    pending_command.intensity =
        0U;
    pending_command.frequency =
        0U;
    pending_command.alive_counter =
        cmd.alive_counter;
    command_pending = 1U;
    fault_code =
        FAULT_INVALID_COMMAND;
    /*
     * Invalid Command는 정상 명령으로 인정하지 않음.
     * 따라서 Timeout 기준 시간은 갱신하지 않는다.
     */
    return;
  }

  /*
   * Alive Counter 검사
   * 첫 정상 프레임은 수용하고, 이후 동일 값 반복 시 통신 이상 처리.
   * 프레임 유실로 값이 건너뛰는 경우는 즉시 Fault 처리하지 않음.
   */
  if ((alive_received_once == 1U) &&
      (cmd.alive_counter == previous_command_alive))
  {
  /* Haptic OFF */
  pending_command.risk_level = 0U;
  pending_command.vibration_command = 0U;
  pending_command.intensity = 0U;
  pending_command.frequency = 0U;
  pending_command.alive_counter = cmd.alive_counter;
  command_pending = 1U;
  fault_code = FAULT_COMMUNICATION;
    /* 정상 명령 수신 시간은 갱신하지 않음 */
    return;
  }
  /* Alive Counter 검사까지 통과한 정상 명령 */
  last_command_tick = HAL_GetTick();
  command_received_once = 1U;
  command_timeout = 0U;
  timeout_stop_done = 0U;
  previous_command_alive = cmd.alive_counter;
  alive_received_once = 1U;
  /*
   * 정상 명령
   */
  pending_command.risk_level =
      cmd.risk_level;
  pending_command.vibration_command =
      cmd.vibration_command;
  pending_command.intensity =
      cmd.intensity;
  pending_command.frequency =
      cmd.frequency;
  pending_command.alive_counter =
      cmd.alive_counter;
  command_pending = 1U;

  /* 정상 명령이 들어오면 Command 관련 Fault 해제 */
  if ((fault_code == FAULT_INVALID_COMMAND) ||
      (fault_code == FAULT_COMMUNICATION))
  {
    fault_code = FAULT_NO_FAULT;
  }
}

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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();
  MX_CAN1_Init();

  /* USER CODE BEGIN 2 */
  printf("\r\n");
  printf("========================================\r\n");
  printf(" STM32 HAPTIC ECU\r\n");
  printf("========================================\r\n");
  printf("CAN Baud        : 500 kbps\r\n");
  printf("HAPTIC_COMMAND  : 0x110\r\n");
  printf("HAPTIC_STATUS   : 0x200\r\n");
  printf("STATUS Period   : 10 ms\r\n");
  printf("Command Timeout : 100 ms\r\n");
  printf("========================================\r\n");

  /*
   * CAN Filter
   */
  CAN_Filter_Config();

  /*
   * CAN Start
   */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * RX FIFO0 Message Pending Interrupt
   */
  if (HAL_CAN_ActivateNotification(
          &hcan1,
          CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * 시작 시 Haptic OFF
   */
  P2000_StopHaptic();
  last_status_tick = HAL_GetTick();

  printf("[CAN] CAN1 started\r\n");
  printf("[CAN] Waiting for HAPTIC_COMMAND 0x110...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    /* =========================================================
     * 0. P2000 USB 연결 상태 감시
     * ========================================================= */
    if (hUsbHostFS.gState == HOST_CLASS)
    {
      if (p2000_usb_ready == 0U)
      {
        p2000_usb_ready = 1U;
        ecu_status = ECU_STATUS_NORMAL;
        if (fault_code == FAULT_DRIVER)
        {
          fault_code = FAULT_NO_FAULT;
        }
        printf("[P2000] USB READY\r\n");
      }
    }
    else
    {
      if (p2000_usb_ready == 1U)
      {
        printf("[P2000] USB DISCONNECTED\r\n");
      }
      p2000_usb_ready = 0U;
      /* P2000 사용 불가 시 STATUS를 안전하게 OFF로 보고 */
      p2000_current_status = 0U;
      p2000_current_intensity = 0U;
      p2000_current_frequency = 0U;
      ecu_status = ECU_STATUS_ERROR;
      fault_code = FAULT_DRIVER;
    }

    /* =========================================================
     * 1. CAN 명령 처리
     * ========================================================= */
    HapticCommand_Process();

    /* =========================================================
     * 2. 100 ms CAN Command Timeout 감시
     * ========================================================= */
    HapticTimeout_Process();

    /* =========================================================
     * 3. P2000 Feature Report 전송
     * ========================================================= */
    if ((p2000_send_pending == 1U) &&
        (hUsbHostFS.gState == HOST_CLASS))
    {
      USBH_StatusTypeDef usb_status;

      usb_status =
          USBH_HID_SetReport(
              &hUsbHostFS,
              0x03U,     /* Feature Report */
              0xF1U,     /* Report ID */
              p2000_report,
              sizeof(p2000_report));

      /*
       * USB 전송 성공
       */
      if (usb_status == USBH_OK)
      {
        p2000_send_pending = 0U;

        /*
         * 여기서부터 STATUS의 값은
         * "CAN으로 받은 Target 값"이 아니라
         * P2000에 전송 완료된 Applied 제어값
         */
        if (p2000_report[3] == 0x00U)
        {
          p2000_current_status = 0U;
          p2000_current_intensity = 0U;
          p2000_current_frequency = 0U;

          printf("[P2000] Applied OFF\r\n");
        }
        else
        {
          p2000_current_status = 1U;
          p2000_current_frequency =
              p2000_report[4];
          p2000_current_intensity =
              p2000_report[5];

          printf("[P2000] Applied ON "
                 "Intensity=%u%% "
                 "Frequency=%uHz\r\n",
                 p2000_current_intensity,
                 p2000_current_frequency);
        }
      }

      /*
       * Feature Report 미지원
       */
      else if (usb_status ==
               USBH_NOT_SUPPORTED)
      {
        p2000_send_pending = 0U;
        fault_code =
            FAULT_DRIVER;
        ecu_status =
            ECU_STATUS_ERROR;
        printf("[P2000] Feature Report NOT SUPPORTED\r\n");
      }

      /*
       * USBH_BUSY
       *
       * 아무것도 하지 않음.
       * 다음 while loop에서 재시도.
       */
    }

    /* =========================================================
     * 4. HAPTIC_STATUS 0x200
     *
     * 10 ms Period
     * ========================================================= */
    if ((HAL_GetTick() - last_status_tick) >=
        HAPTIC_STATUS_PERIOD_MS)
    {
      last_status_tick +=
          HAPTIC_STATUS_PERIOD_MS;
      HapticStatus_Send();
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(
      PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState =
      RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState =
      RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource =
      RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP =
      RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(
          &RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK |
      RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 |
      RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource =
      RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider =
      RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider =
      RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider =
      RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(
          &RCC_ClkInitStruct,
          FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  */
static void MX_CAN1_Init(void)
{
  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
  /* USER CODE END CAN1_Init 1 */

  /*
   * APB1 = 42 MHz
   *
   * Prescaler = 6
   * BS1       = 11 TQ
   * BS2       = 2 TQ
   * SJW       = 1 TQ
   *
   * 42 MHz / 6 / (1 + 11 + 2)
   * = 500 kbit/s
   */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode =
      CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth =
      CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 =
      CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 =
      CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode =
      DISABLE;
  hcan1.Init.AutoBusOff =
      ENABLE;
  hcan1.Init.AutoWakeUp =
      DISABLE;
  hcan1.Init.AutoRetransmission =
      ENABLE;
  hcan1.Init.ReceiveFifoLocked =
      DISABLE;
  hcan1.Init.TransmitFifoPriority =
      DISABLE;

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN CAN1_Init 2 */
  /* USER CODE END CAN1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */

  huart2.Instance =
      USART2;
  huart2.Init.BaudRate =
      115200;
  huart2.Init.WordLength =
      UART_WORDLENGTH_8B;
  huart2.Init.StopBits =
      UART_STOPBITS_1;
  huart2.Init.Parity =
      UART_PARITY_NONE;
  huart2.Init.Mode =
      UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl =
      UART_HWCONTROL_NONE;
  huart2.Init.OverSampling =
      UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(
      LD2_GPIO_Port,
      LD2_Pin,
      GPIO_PIN_RESET);

  GPIO_InitStruct.Pin =
      B1_Pin;
  GPIO_InitStruct.Mode =
      GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull =
      GPIO_NOPULL;
  HAL_GPIO_Init(
      B1_GPIO_Port,
      &GPIO_InitStruct);

  GPIO_InitStruct.Pin =
      LD2_Pin;
  GPIO_InitStruct.Mode =
      GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull =
      GPIO_NOPULL;
  GPIO_InitStruct.Speed =
      GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(
      LD2_GPIO_Port,
      &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file,
                   uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif
