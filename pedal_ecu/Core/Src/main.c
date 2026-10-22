/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Pedal ECU
  ******************************************************************************
  * CAN ID : 0x100
  * DLC    : 8
  * Cycle  : 10 ms
  *
  * Byte 0   : Accelerator Pedal (%)
  * Byte 1   : Brake Pedal (%)
  * Byte 2~3 : Accelerator Rate (int16_t, 0.1 %/s, Little Endian)
  * Byte 4~5 : Brake Rate       (int16_t, 0.1 %/s, Little Endian)
  * Byte 6   : Sensor Status
  * Byte 7   : Alive Counter
  *
  * Sensor Status
  * 0x00 : Normal
  * 0x01 : Accelerator ADC Range Error
  * 0x02 : Brake ADC Range Error
  * 0x04 : Accelerator Sensor Disconnect
  * 0x08 : Brake Sensor Disconnect
  * 0x10 : Accelerator Calibration Error
  * 0x20 : Brake Calibration Error
  * 0x40 : Reserved
  * 0x80 : Reserved
  *
 * Sensor disconnect policy:
 *   PB0/PB1 Detect lines are NOT used.
 *   PA0/PA1 internal pull-up/pull-down are NOT used.
 *   Percentage-based disconnect detection is disabled
 *   because the floating value overlaps the normal pedal range.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdint.h>

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Private define ------------------------------------------------------------*/

/* ADC mapping */

#define BRAKE_ADC_CHANNEL               ADC_CHANNEL_0   /* PA0 / A0 */
#define ACCEL_ADC_CHANNEL               ADC_CHANNEL_1   /* PA1 / A1 */

/* Calibration values */

#define ACCEL_RELEASED_ADC              3842U
#define ACCEL_FULL_ADC                   451U
#define BRAKE_RELEASED_ADC              3705U
#define BRAKE_FULL_ADC                   404U
#define CALIBRATION_MIN_SPAN            2500U

/* ============================================================
 * ADC FILTER
 * Average 16 ADC samples per channel.
 * Ignore changes within the deadband to reduce small fluctuations.
 * ============================================================ */

#define ADC_AVERAGE_SAMPLES               16U
#define ADC_DEADBAND                      15U

/* ============================================================
 * PEDAL PERCENT STABILIZATION
 * Clamp 0-2% to 0% and 98-100% to 100%.
 * Ignore changes below 2 percentage points in the middle range.
 * ============================================================ */

#define PEDAL_ZERO_DEADZONE_PERCENT        2U
#define PEDAL_FULL_DEADZONE_PERCENT       98U
#define PEDAL_PERCENT_HYSTERESIS            2U

/* ============================================================
 * ADC Range Error
 * Internal pull-up/pull-down resistors are not used.
 * Check for readings outside the configured valid ADC range.
 * ============================================================ */

#define ACCEL_ADC_MIN_VALID              300U
#define ACCEL_ADC_MAX_VALID             4000U
#define BRAKE_ADC_MIN_VALID              250U
#define BRAKE_ADC_MAX_VALID             3900U

/* 10 ms x 10 = 100 ms */
#define ADC_RANGE_ERROR_COUNT_LIMIT       10U

/* Clear the error after 20 consecutive valid samples (200 ms). */

#define ADC_RANGE_RECOVERY_COUNT_LIMIT    20U

/* CAN */

#define PEDAL_CAN_ID                    0x100U
#define PEDAL_CAN_DLC                      8U
#define PEDAL_CAN_PERIOD_MS               10U

/* Live Expressions debug update */

#define DEBUG_UPDATE_PERIOD_MS            200U

/* Global debug/state variables ---------------------------------------------*/

/* Final ADC values after filtering */

volatile uint16_t accel_adc = 0U;
volatile uint16_t brake_adc = 0U;

/* ADC averages before applying the deadband */

volatile uint16_t accel_adc_average = 0U;
volatile uint16_t brake_adc_average = 0U;

static uint8_t adc_filter_initialized = 0U;
volatile uint16_t accel_tenth_percent = 0U;
volatile uint16_t brake_tenth_percent = 0U;
volatile uint8_t accel_percent = 0U;
volatile uint8_t brake_percent = 0U;

static uint8_t accel_percent_stable = 0U;
static uint8_t brake_percent_stable = 0U;
static uint8_t percent_filter_initialized = 0U;
volatile uint8_t acc_disconnect_active = 0U;
volatile uint8_t brake_disconnect_active = 0U;
volatile int16_t accel_rate_raw = 0;
volatile int16_t brake_rate_raw = 0;
volatile uint8_t sensor_status = 0x00U;
volatile uint8_t alive_counter = 0U;

/* ADC Range Error debounce / recovery */

volatile uint8_t acc_adc_range_error_count = 0U;
volatile uint8_t brake_adc_range_error_count = 0U;
volatile uint8_t acc_adc_range_recovery_count = 0U;
volatile uint8_t brake_adc_range_recovery_count = 0U;
volatile uint8_t acc_adc_range_error_active = 0U;
volatile uint8_t brake_adc_range_error_active = 0U;
volatile uint32_t can_tx_ok_count = 0U;
volatile uint32_t can_tx_error_count = 0U;
volatile uint32_t can_busoff_count = 0U;
volatile uint32_t can_last_error = 0U;
volatile uint32_t can_esr = 0U;
volatile uint8_t can_busoff_active = 0U;

/* Slow variables for Live Expressions */

volatile uint16_t debug_accel_adc = 0U;
volatile uint16_t debug_brake_adc = 0U;
volatile uint16_t debug_accel_adc_average = 0U;
volatile uint16_t debug_brake_adc_average = 0U;
volatile uint8_t debug_accel_percent = 0U;
volatile uint8_t debug_brake_percent = 0U;
volatile int16_t debug_accel_rate_raw = 0;
volatile int16_t debug_brake_rate_raw = 0;
volatile uint8_t debug_sensor_status = 0x00U;
volatile uint8_t debug_alive_counter = 0U;
volatile uint8_t debug_acc_adc_range_error = 0U;
volatile uint8_t debug_brake_adc_range_error = 0U;

static uint32_t last_pedal_tick = 0U;
static uint32_t last_debug_tick = 0U;
static uint16_t prev_accel_tenth_percent = 0U;
static uint16_t prev_brake_tenth_percent = 0U;
static uint8_t first_pedal_cycle = 1U;
static CAN_TxHeaderTypeDef pedal_tx_header;
static uint8_t pedal_tx_data[8];
static uint32_t pedal_tx_mailbox;

/* Function prototypes -------------------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM3_Init(void);
static void CAN_Start(void);
static void Read_Pedal_ADC(void);
static uint16_t Apply_ADC_Deadband(uint16_t previous_value,
                                   uint16_t new_value,
                                   uint16_t deadband);

static uint8_t Apply_Percent_Stabilization(uint8_t previous_percent,
                                           uint8_t new_percent);

static uint16_t ADC_To_TenthPercent(uint16_t adc_value,
                                    uint16_t released_adc,
                                    uint16_t full_adc);

static int16_t Saturate_Int16(int32_t value);
static void Calculate_Pedal_Data(void);
static void Update_ADC_Range_Diagnostic(void);
static void Update_Sensor_Status(void);
static void Send_Pedal_CAN(void);
static void Pedal_ECU_10ms_Task(void);
static void Update_Debug_Variables(void);

/* User functions ------------------------------------------------------------*/

static void CAN_Start(void)
{
    pedal_tx_header.StdId = PEDAL_CAN_ID;
    pedal_tx_header.ExtId = 0U;
    pedal_tx_header.IDE = CAN_ID_STD;
    pedal_tx_header.RTR = CAN_RTR_DATA;
    pedal_tx_header.DLC = PEDAL_CAN_DLC;
    pedal_tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(
            &hcan1,
            CAN_IT_BUSOFF |
            CAN_IT_ERROR_WARNING |
            CAN_IT_ERROR_PASSIVE |
            CAN_IT_LAST_ERROR_CODE) != HAL_OK)
    {
        Error_Handler();
    }
}

static void Read_Pedal_ADC(void)
{
    uint32_t brake_sum = 0U;
    uint32_t accel_sum = 0U;
    uint16_t sample;
    uint16_t valid_sample_count = 0U;

    /* Measure and average 16 samples each for brake and accelerator
 * whenever the 10 ms task runs. */

    for (sample = 0U; sample < ADC_AVERAGE_SAMPLES; sample++)
    {
        if (HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            continue;
        }

        /* Rank 1 : Brake / PA0 / ADC1_IN0 */
        if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc1);
            continue;
        }
        brake_sum += HAL_ADC_GetValue(&hadc1);

        /* Rank 2 : Accelerator / PA1 / ADC1_IN1 */
        if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc1);
            continue;
        }
        accel_sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        valid_sample_count++;
    }

    /* Keep previous ADC values when no valid samples are acquired. */

    if (valid_sample_count == 0U)
    {
        return;
    }

    brake_adc_average =
        (uint16_t)(brake_sum / valid_sample_count);
    accel_adc_average =
        (uint16_t)(accel_sum / valid_sample_count);

    /* Use the first ADC averages directly. */

    if (adc_filter_initialized == 0U)
    {
        brake_adc = brake_adc_average;
        accel_adc = accel_adc_average;
        adc_filter_initialized = 1U;
    }
    else
    {
        /* Keep the previous value if the average differs by at most
 * the configured deadband, suppressing small fluctuations. */

        brake_adc =
            Apply_ADC_Deadband(
                brake_adc,
                brake_adc_average,
                ADC_DEADBAND
            );
        accel_adc =
            Apply_ADC_Deadband(
                accel_adc,
                accel_adc_average,
                ADC_DEADBAND
            );
    }
}

static uint16_t Apply_ADC_Deadband(uint16_t previous_value,
                                   uint16_t new_value,
                                   uint16_t deadband)
{
    uint16_t diff;

    if (new_value >= previous_value)
    {
        diff = new_value - previous_value;
    }
    else
    {
        diff = previous_value - new_value;
    }

    /* Ignore changes within the deadband. */

    if (diff <= deadband)
    {
        return previous_value;
    }

    /* Apply changes exceeding the deadband. */

    return new_value;
}

static uint8_t Apply_Percent_Stabilization(uint8_t previous_percent,
                                           uint8_t new_percent)
{
    uint8_t diff;

    /* Force 0% near the released position. */

    if (new_percent <= PEDAL_ZERO_DEADZONE_PERCENT)
    {
        return 0U;
    }

    /* Force 100% near full pedal travel. */

    if (new_percent >= PEDAL_FULL_DEADZONE_PERCENT)
    {
        return 100U;
    }

    if (new_percent >= previous_percent)
    {
        diff = new_percent - previous_percent;
    }
    else
    {
        diff = previous_percent - new_percent;
    }

    /* Ignore one-percentage-point changes as noise. */

    if (diff < PEDAL_PERCENT_HYSTERESIS)
    {
        return previous_percent;
    }

    return new_percent;
}

static uint16_t ADC_To_TenthPercent(uint16_t adc_value,
                                    uint16_t released_adc,
                                    uint16_t full_adc)
{
    uint32_t result;

    if (released_adc <= full_adc)
    {
        return 0U;
    }

    if (adc_value >= released_adc)
    {
        return 0U;
    }

    if (adc_value <= full_adc)
    {
        return 1000U;
    }

    result = ((uint32_t)(released_adc - adc_value) * 1000U) /
             (uint32_t)(released_adc - full_adc);

    if (result > 1000U)
    {
        result = 1000U;
    }

    return (uint16_t)result;
}

static int16_t Saturate_Int16(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }

    if (value < -32768)
    {
        return -32768;
    }

    return (int16_t)value;
}

static void Calculate_Pedal_Data(void)
{
    int32_t acc_rate_temp;
    int32_t brake_rate_temp;

    accel_tenth_percent = ADC_To_TenthPercent(accel_adc,
                                               ACCEL_RELEASED_ADC,
                                               ACCEL_FULL_ADC);
    brake_tenth_percent = ADC_To_TenthPercent(brake_adc,
                                               BRAKE_RELEASED_ADC,
                                               BRAKE_FULL_ADC);

    /* --------------------------------------------------------
     * CAN Byte 0, 1 : 0~100%
     * -------------------------------------------------------- */
    {
        uint8_t accel_percent_new;
        uint8_t brake_percent_new;

        accel_percent_new =
            (uint8_t)((accel_tenth_percent + 5U) / 10U);
        brake_percent_new =
            (uint8_t)((brake_tenth_percent + 5U) / 10U);

        if (accel_percent_new > 100U)
        {
            accel_percent_new = 100U;
        }

        if (brake_percent_new > 100U)
        {
            brake_percent_new = 100U;
        }

        if (percent_filter_initialized == 0U)
        {
            accel_percent_stable =
                Apply_Percent_Stabilization(
                    accel_percent_new,
                    accel_percent_new
                );
            brake_percent_stable =
                Apply_Percent_Stabilization(
                    brake_percent_new,
                    brake_percent_new
                );
            percent_filter_initialized = 1U;
        }
        else
        {
            accel_percent_stable =
                Apply_Percent_Stabilization(
                    accel_percent_stable,
                    accel_percent_new
                );
            brake_percent_stable =
                Apply_Percent_Stabilization(
                    brake_percent_stable,
                    brake_percent_new
                );
        }

        accel_percent = accel_percent_stable;
        brake_percent = brake_percent_stable;
    }

    /* First sample: rate = 0 */

    if (first_pedal_cycle != 0U)
    {
        accel_rate_raw = 0;
        brake_rate_raw = 0;
        prev_accel_tenth_percent = accel_tenth_percent;
        prev_brake_tenth_percent = brake_tenth_percent;
        first_pedal_cycle = 0U;
        return;
    }

    /*
     * Sample time = 10ms = 0.01s
     * Internal position unit = 0.1%
     * delta * 100 -> 0.1 %/s
     */

    acc_rate_temp = ((int32_t)accel_tenth_percent -
                     (int32_t)prev_accel_tenth_percent) * 100;
    brake_rate_temp = ((int32_t)brake_tenth_percent -
                       (int32_t)prev_brake_tenth_percent) * 100;
    accel_rate_raw = Saturate_Int16(acc_rate_temp);
    brake_rate_raw = Saturate_Int16(brake_rate_temp);
    prev_accel_tenth_percent = accel_tenth_percent;
    prev_brake_tenth_percent = brake_tenth_percent;
}

/**
 * @brief Check the ADC valid range every 10 ms.
 * 100 ms of invalid readings: set error.
 * 200 ms of valid readings: clear error.
 */

static void Update_ADC_Range_Diagnostic(void)
{
    uint8_t acc_invalid;
    uint8_t brake_invalid;

    acc_invalid =
        ((accel_adc < ACCEL_ADC_MIN_VALID) ||
         (accel_adc > ACCEL_ADC_MAX_VALID))
        ? 1U : 0U;
    brake_invalid =
        ((brake_adc < BRAKE_ADC_MIN_VALID) ||
         (brake_adc > BRAKE_ADC_MAX_VALID))
        ? 1U : 0U;

    /* --------------------------------------------------------
     * Accelerator
     * -------------------------------------------------------- */

    if (acc_invalid != 0U)
    {
        acc_adc_range_recovery_count = 0U;

        if (acc_adc_range_error_count < ADC_RANGE_ERROR_COUNT_LIMIT)
        {
            acc_adc_range_error_count++;
        }

        if (acc_adc_range_error_count >= ADC_RANGE_ERROR_COUNT_LIMIT)
        {
            acc_adc_range_error_active = 1U;
        }
    }
    else
    {
        acc_adc_range_error_count = 0U;

        if (acc_adc_range_error_active != 0U)
        {
            if (acc_adc_range_recovery_count <
                ADC_RANGE_RECOVERY_COUNT_LIMIT)
            {
                acc_adc_range_recovery_count++;
            }

            if (acc_adc_range_recovery_count >=
                ADC_RANGE_RECOVERY_COUNT_LIMIT)
            {
                acc_adc_range_error_active = 0U;
                acc_adc_range_recovery_count = 0U;
            }
        }
        else
        {
            acc_adc_range_recovery_count = 0U;
        }
    }

    /* --------------------------------------------------------
     * Brake
     * -------------------------------------------------------- */

    if (brake_invalid != 0U)
    {
        brake_adc_range_recovery_count = 0U;

        if (brake_adc_range_error_count < ADC_RANGE_ERROR_COUNT_LIMIT)
        {
            brake_adc_range_error_count++;
        }

        if (brake_adc_range_error_count >= ADC_RANGE_ERROR_COUNT_LIMIT)
        {
            brake_adc_range_error_active = 1U;
        }
    }
    else
    {
        brake_adc_range_error_count = 0U;

        if (brake_adc_range_error_active != 0U)
        {
            if (brake_adc_range_recovery_count <
                ADC_RANGE_RECOVERY_COUNT_LIMIT)
            {
                brake_adc_range_recovery_count++;
            }

            if (brake_adc_range_recovery_count >=
                ADC_RANGE_RECOVERY_COUNT_LIMIT)
            {
                brake_adc_range_error_active = 0U;
                brake_adc_range_recovery_count = 0U;
            }
        }
        else
        {
            brake_adc_range_recovery_count = 0U;
        }
    }
}

static void Update_Sensor_Status(void)
{
    uint8_t status = 0x00U;

    /* ADC Range Error */

    if (acc_adc_range_error_active != 0U)
    {
        status |= 0x01U;
    }

    if (brake_adc_range_error_active != 0U)
    {
        status |= 0x02U;
    }

    /* --------------------------------------------------------
 * Sensor Disconnect
 * Percentage-based disconnect detection is disabled in this version.
 * -------------------------------------------------------- */

    if (acc_disconnect_active != 0U)
    {
        status |= 0x04U;
    }

    if (brake_disconnect_active != 0U)
    {
        status |= 0x08U;
    }

    /* Calibration Error */

    if ((ACCEL_RELEASED_ADC <= ACCEL_FULL_ADC) ||
        ((ACCEL_RELEASED_ADC - ACCEL_FULL_ADC) < CALIBRATION_MIN_SPAN) ||
        (ACCEL_RELEASED_ADC > 4095U) ||
        (ACCEL_FULL_ADC > 4095U))
    {
        status |= 0x10U;
    }

    if ((BRAKE_RELEASED_ADC <= BRAKE_FULL_ADC) ||
        ((BRAKE_RELEASED_ADC - BRAKE_FULL_ADC) < CALIBRATION_MIN_SPAN) ||
        (BRAKE_RELEASED_ADC > 4095U) ||
        (BRAKE_FULL_ADC > 4095U))
    {
        status |= 0x20U;
    }

    /* No error = 0x00; no separate normal-state bit is used. */

    sensor_status = status;
}

static void Send_Pedal_CAN(void)
{
    uint16_t accel_rate_u16 = (uint16_t)accel_rate_raw;
    uint16_t brake_rate_u16 = (uint16_t)brake_rate_raw;

    pedal_tx_data[0] = accel_percent;
    pedal_tx_data[1] = brake_percent;

    /* Little Endian */

    pedal_tx_data[2] = (uint8_t)(accel_rate_u16 & 0xFFU);
    pedal_tx_data[3] = (uint8_t)((accel_rate_u16 >> 8) & 0xFFU);
    pedal_tx_data[4] = (uint8_t)(brake_rate_u16 & 0xFFU);
    pedal_tx_data[5] = (uint8_t)((brake_rate_u16 >> 8) & 0xFFU);
    pedal_tx_data[6] = sensor_status;
    pedal_tx_data[7] = alive_counter;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
    {
        if (HAL_CAN_AddTxMessage(&hcan1,
                                 &pedal_tx_header,
                                 pedal_tx_data,
                                 &pedal_tx_mailbox) == HAL_OK)
        {
            can_tx_ok_count++;
            alive_counter++;
        }
        else
        {
            can_tx_error_count++;
        }
    }
    else
    {
        can_tx_error_count++;
    }
}

static void Pedal_ECU_10ms_Task(void)
{
    Read_Pedal_ADC();
    Calculate_Pedal_Data();
    Update_ADC_Range_Diagnostic();
    Update_Sensor_Status();
    Send_Pedal_CAN();
}

static void Update_Debug_Variables(void)
{
    debug_accel_adc = accel_adc;
    debug_brake_adc = brake_adc;
    debug_accel_adc_average = accel_adc_average;
    debug_brake_adc_average = brake_adc_average;
    debug_accel_percent = accel_percent;
    debug_brake_percent = brake_percent;
    debug_accel_rate_raw = accel_rate_raw;
    debug_brake_rate_raw = brake_rate_raw;
    debug_sensor_status = sensor_status;
    debug_alive_counter = alive_counter;
    debug_acc_adc_range_error = acc_adc_range_error_active;
    debug_brake_adc_range_error = brake_adc_range_error_active;
    can_esr = hcan1.Instance->ESR;

    if ((can_esr & CAN_ESR_BOFF) != 0U)
    {
        if (can_busoff_active == 0U)
        {
            can_busoff_count++;
            can_busoff_active = 1U;
        }
    }
    else
    {
        can_busoff_active = 0U;
    }
}

/* Main ----------------------------------------------------------------------*/

int main(void)
{
    uint32_t now;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();

    /* PA0/PA1 are analog inputs without internal pull-up/pull-down. */

    MX_CAN1_Init();
    MX_TIM3_Init();
    CAN_Start();
    last_pedal_tick = HAL_GetTick();
    last_debug_tick = HAL_GetTick();

    while (1)
    {
        now = HAL_GetTick();

        if ((uint32_t)(now - last_pedal_tick) >= PEDAL_CAN_PERIOD_MS)
        {
            last_pedal_tick += PEDAL_CAN_PERIOD_MS;
            Pedal_ECU_10ms_Task();
        }

        if ((uint32_t)(now - last_debug_tick) >= DEBUG_UPDATE_PERIOD_MS)
        {
            last_debug_tick = now;
            Update_Debug_Variables();
        }
    }
}

/* System Clock --------------------------------------------------------------*/

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ADC1 ----------------------------------------------------------------------*/

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Rank 1 : Brake / PA0 / ADC1_IN0 */
    sConfig.Channel = BRAKE_ADC_CHANNEL;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Rank 2 : Accelerator / PA1 / ADC1_IN1 */
    sConfig.Channel = ACCEL_ADC_CHANNEL;
    sConfig.Rank = 2;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* CAN1 : 500 kbps -----------------------------------------------------------*/

static void MX_CAN1_Init(void)
{
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 6;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* TIM3 ----------------------------------------------------------------------*/

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
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

    HAL_TIM_MspPostInit(&htim3);
}

/* USART2 --------------------------------------------------------------------*/

static void MX_USART2_UART_Init(void)
{
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
}

/* GPIO ----------------------------------------------------------------------*/

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA,
                      LD2_Pin | GPIO_PIN_8 | GPIO_PIN_9,
                      GPIO_PIN_RESET);

    /* User button */

    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /* LD2 / PA8 / PA9 */

    GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA0 / PA1: ADC analog inputs; no internal pull-up/pull-down. */

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB0 / PB1 detect function is not used. */
}

/* CAN Error Callback --------------------------------------------------------*/

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        can_last_error = HAL_CAN_GetError(hcan);
    }
}

/* Error Handler -------------------------------------------------------------*/

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}

#endif
