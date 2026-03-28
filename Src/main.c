/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 二轮自平衡小车主程序
  *                   主控: STM32F412
  *                   传感器: MPU6886 (I2C1: PB8=SCL, PB9=SDA)
  *                   电机PWM: TIM1_CH1(PA8=pwmA), TIM1_CH2(PA9=pwmB)
  *                   电机方向: PB0(AIN1), PB1(AIN2), PC11(BIN1), PC10(BIN2)
  *                   编码器A: TIM3(PA6=E1A_CH1, PA7=E2A_CH2)
  *                   编码器B: TIM4(PB6=E2B_CH1, PB7=E1B_CH2)
  *                   蓝牙: USART2_RX(PA3=blue_rx)
  *                   控制周期: TIM2 产生 5ms 中断
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
#include "rtt_log.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6886.h"
#include "bluetooth.h"
#include "pid.h"
#include "balance.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 主循环状态打印周期（ms）
#define STATUS_PRINT_PERIOD_MS   1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 系统状态标志
static volatile uint8_t g_control_flag = 0;   // 由TIM2中断置1，主循环检测
static volatile uint8_t g_mpu_ok       = 0;   // MPU6886初始化结果

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*--------- 中断回调 ---------*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        g_control_flag = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        BT_ProcessRxByte(*BT_GetRxBytePtr());
        HAL_StatusTypeDef ret = HAL_UART_Receive_IT(
                                    &huart2, BT_GetRxBytePtr(), 1);
        if (ret != HAL_OK) {
            RTT_LOG_ERROR("[IT] UART2重启接收失败 HAL_ERR=%d", (int)ret);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint32_t err = HAL_UART_GetError(huart);
        RTT_LOG_ERROR("[IT] UART2错误 ERR=0x%08lX", err);
        HAL_UART_Receive_IT(&huart2, BT_GetRxBytePtr(), 1);
    }
}

/*--------- App_Init ---------*/

static void App_Init(void)
{
    RTT_LOG_INFO("[Init] ---- 开始应用初始化 ----");

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    RTT_LOG_INFO("[Init] FRAM_CS(PB12) 已拉高");

    Motor_Init();
    Encoder_Init();

    g_mpu_ok = MPU6886_Init();
    if (!g_mpu_ok) {
        RTT_LOG_ERROR("[Init] MPU6886 初始化失败！");
        RTT_LOG_ERROR("[Init] 检查: 1.PB8(SCL)/PB9(SDA)接线");
        RTT_LOG_ERROR("[Init]       2.MPU6886供电(3.3V)");
        RTT_LOG_ERROR("[Init]       3.AD0电平(GND=0x68, VCC=0x69)");
        RTT_LOG_WARN ("[Init] 平衡功能禁用，电机保持停止");
    } else {
        RTT_LOG_INFO("[Init] MPU6886 初始化成功");
    }

    BT_Init();
    Balance_Init();

    HAL_StatusTypeDef ret = HAL_TIM_Base_Start_IT(&htim2);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[Init] TIM2中断启动失败 HAL_ERR=%d", (int)ret);
        RTT_LOG_ERROR("[Init] 请在CubeMX中开启 TIM2 NVIC 中断");
    } else {
        RTT_LOG_INFO("[Init] TIM2 5ms控制周期已启动");
    }

    RTT_LOG_INFO("[Init] ---- 初始化完成 ----");
    RTT_LOG_INFO("[Init] MPU=%s | BT=UART2(PA3) | CTRL=TIM2(5ms)",
                 g_mpu_ok ? "OK" : "FAIL");
}

/*--------- App_PrintStatus ---------*/

static void App_PrintStatus(void)
{
    MPU6886_Data   *imu = MPU6886_GetData();
    BT_ControlData *bt  = BT_GetControlData();

    RTT_LOG_DEBUG("---- [%lu ms] ----", HAL_GetTick());

    if (g_mpu_ok) {
        RTT_LOG_DEBUG("[IMU ] pitch=%.2f roll=%.2f gyroY=%.1f temp=%.1f",
                      imu->pitch, imu->roll,
                      imu->gyroY, imu->temp);
    } else {
        RTT_LOG_WARN("[IMU ] 未就绪");
    }

    RTT_LOG_DEBUG("[ENC ] A=%d(%.2fr/s) B=%d(%.2fr/s) avg=%.3fm/s",
                  Encoder_GetCountA(), Encoder_GetSpeedA_rps(),
                  Encoder_GetCountB(), Encoder_GetSpeedB_rps(),
                  Encoder_GetSpeedAvg_ms());

    RTT_LOG_DEBUG("[BT  ] cmd=%c spd=%.2f turn=%.2f",
                  (bt->command != BT_CMD_NONE) ? (char)bt->command : '-',
                  bt->speed_target,
                  bt->turn_target);
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    // RTT 最先初始化，确保后续所有日志可以输出
    SEGGER_RTT_Init();
    RTT_LOG_INFO("========================================");
    RTT_LOG_INFO("  二轮自平衡小车 启动");
    RTT_LOG_INFO("  STM32F412 Balance Car v1.0");
    RTT_LOG_INFO("========================================");
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

    // 应用层初始化
    App_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    uint32_t last_print_tick = 0;

    RTT_LOG_INFO("[Main] 进入主循环，等待控制周期中断...");

    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        /*----------------------------------------------------
         * 5ms 控制任务（由 TIM2 中断触发）
         *----------------------------------------------------*/
        if (g_control_flag)
        {
            g_control_flag = 0;

            if (g_mpu_ok)
            {
                // 执行平衡控制主循环
                Balance_Update();
            }
            else
            {
                // MPU未就绪时停止电机
                Motor_Stop();
            }
        }

        /*----------------------------------------------------
         * 状态打印（每 STATUS_PRINT_PERIOD_MS ms 一次）
         *----------------------------------------------------*/
        uint32_t now = HAL_GetTick();
        if (now - last_print_tick >= STATUS_PRINT_PERIOD_MS)
        {
            last_print_tick = now;
            App_PrintStatus();
        }

    } /* while(1) */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    RTT_LOG_ERROR("[SYS] Error_Handler 触发！系统挂起");
    while (1) { }
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
    RTT_LOG_ERROR("[ASSERT] 断言失败: file=%s line=%lu", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
