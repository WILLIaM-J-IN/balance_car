/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Two-wheel self-balancing car
  *
  *  Pin map:
  *    Motor PWM : TIM1_CH1(PA8=pwmA)  TIM1_CH2(PA9=pwmB)
  *    Motor DIR : PB0(AIN1) PB1(AIN2) PC11(BIN1) PC10(BIN2)
  *    Encoder A : TIM3  PA6=E1A  PA7=E2A
  *    Encoder B : TIM4  PB6=E2B  PB7=E1B
  *    IMU       : I2C1  PB8=SCL  PB9=SDA
  *    Bluetooth : UART2 PA3=RX (HC-05, 9600 baud)
  *    FRAM_CS   : PB12
  *    Buzzer    : PB4  (high=on)
  *    Control   : TIM2 5ms (Period=999 Prescaler=79)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
#include "move.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define STATUS_PERIOD_MS  1000u
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static volatile uint8_t g_control_flag = 0;
static volatile uint8_t g_mpu_ok       = 0;
static volatile uint8_t data_received  = 0;
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */

/*------------------------------------------------------------
 *  TIM2 5ms control flag
 *------------------------------------------------------------*/















void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
        g_control_flag = 1;
}

/*------------------------------------------------------------
 *  UART2 RX → Bluetooth frame parser ($CMD#)
 *------------------------------------------------------------*/
/*------------------------------------------------------------
 *  UART2 RX 环形缓冲 (中断只塞字节,解析放主循环)
 *------------------------------------------------------------*/
#define BT_RING_SIZE 128
static volatile uint8_t  bt_ring[BT_RING_SIZE];
static volatile uint16_t bt_ring_head = 0;
static volatile uint16_t bt_ring_tail = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        data_received = 1;

        /* 把字节塞进环形缓冲,中断里就这一件事 */
        uint16_t next = (bt_ring_head + 1) % BT_RING_SIZE;
        if (next != bt_ring_tail) {
            bt_ring[bt_ring_head] = *BT_GetRxBytePtr();
            bt_ring_head = next;
        }
        /* 缓冲满则丢弃,正常情况下主循环消费很快不会满 */

        /* 立即重启接收 */
        HAL_UART_Receive_IT(&huart2, BT_GetRxBytePtr(), 1);
    }
}

/*------------------------------------------------------------
 *  消费环形缓冲 (主循环每次都调,解析放在这里)
 *------------------------------------------------------------*/
static void BT_ProcessRing(void)
{
    while (bt_ring_tail != bt_ring_head) {
        uint8_t byte = bt_ring[bt_ring_tail];
        bt_ring_tail = (bt_ring_tail + 1) % BT_RING_SIZE;
        BT_ParseByte(byte);    /* 在主循环上下文,可以安全打日志 */
    }
}

/*------------------------------------------------------------
 *  UART2 error → clear flags, reset parser
 *------------------------------------------------------------*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        __HAL_UART_CLEAR_NEFLAG(&huart2);
        __HAL_UART_CLEAR_FEFLAG(&huart2);
        BT_ResetParser();
        HAL_UART_AbortReceive(&huart2);
        HAL_UART_Receive_IT(&huart2, BT_GetRxBytePtr(), 1);
    }
}

/*------------------------------------------------------------
 *  App init
 *------------------------------------------------------------*/
static void App_Init(void)
{
    RTT_LOG_INFO("[Init] ---- App Init Start ----");

    /* Buzzer off */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    /* FRAM_CS high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    /* Motors */
    Motor_Init();

    /* Encoders */
    Encoder_Init();

    /* MPU6886 (gyro calibration ~1s, keep car still!) */
    RTT_LOG_INFO("[Init] MPU6886 init, keep car still for 1s...");
    g_mpu_ok = MPU6886_Init();
    if (!g_mpu_ok) {
        RTT_LOG_ERROR("[Init] MPU6886 FAILED!");
        RTT_LOG_ERROR("[Init]   1. PB8(SCL) PB9(SDA) wiring");
        RTT_LOG_ERROR("[Init]   2. 3.3V power");
        RTT_LOG_ERROR("[Init]   3. AD0 = GND");
        /* Buzz error pattern */
        for (int i = 0; i < 5; i++) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_Delay(100);
        }
    } else {
        RTT_LOG_INFO("[Init] MPU6886 OK");
    }

    /* Bluetooth */
    BT_Init();

    /* Balance controller */
    Balance_Init();

    /* TIM2 5ms interrupt */
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        RTT_LOG_ERROR("[Init] TIM2 FAILED! Enable TIM2 NVIC in CubeMX");
    } else {
        RTT_LOG_INFO("[Init] TIM2 5ms OK");
    }

    /* Single beep = boot done */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    RTT_LOG_INFO("[Init] ---- Init Done ----");
    RTT_LOG_INFO("[Init] MPU=%s BT=UART2(9600) CTRL=TIM2(5ms)",
                 g_mpu_ok ? "OK" : "FAIL");
}

/*------------------------------------------------------------
 *  Status print + BT status send
 *------------------------------------------------------------*/
static void App_PrintStatus(void)
{
    MPU6886_Data   *imu = MPU6886_GetData();
    BT_ControlData *bt  = BT_GetControlData();

    /* Integer conversion for RTT */
    int p_i = (int)imu->pitch;
    int p_f = (int)(imu->pitch * 100.0f) % 100;
    if (p_f < 0) p_f = -p_f;

    int r_i = (int)imu->roll;
    int r_f = (int)(imu->roll * 100.0f) % 100;
    if (r_f < 0) r_f = -r_f;

    int spd = (int)(Encoder_GetSpeedAvg_ms() * 100.0f);
    int s_f = spd % 100;
    if (s_f < 0) s_f = -s_f;

    RTT_LOG_DEBUG("[Status] pitch=%d.%02d roll=%d.%02d "
                  "spd=%d.%02d bt=%c",
                  p_i, p_f, r_i, r_f,
                  spd/100, s_f,
                  (bt->command != BT_CMD_NONE) ? (char)bt->command : '-');

    /* Send status over Bluetooth */
    char msg[64];
    snprintf(msg, sizeof(msg),
             "STATUS:pitch=%d,spd=%d,encA=%d,encB=%d\r\n",
             p_i, spd/100,
             Encoder_GetCountA(),
             Encoder_GetCountB());
    BT_SendString(msg);
}


static void Pitch_Test(void)
{
    RTT_LOG_INFO("[PitchTest] Start, keep car still...");

    uint32_t tick = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        if (now - tick >= 200)
        {
            tick = now;
            MPU6886_Update();

            float pitch = MPU6886_GetPitch();

            int p_i = (int)pitch;
            int p_f = (int)(pitch * 100.0f) % 100;
            if (p_f < 0) p_f = -p_f;

            SEGGER_RTT_printf(0, "[%lu] pitch=%d.%02d\r\n",
                              now, p_i, p_f);
        }
    }
}




/**
 * 死区测试 - 手动观察版
 * 从 0 开始每隔 500ms 增加 5,直到你看到电机转动
 * 分别测试 A正/A反/B正/B反 四个方向
 */
static void Motor_DeadzoneTest_Manual(void)
{
    RTT_LOG_INFO("[DZTest] ==== Manual Deadzone Test ====");
    RTT_LOG_INFO("[DZTest] Watch the wheels, note the PWM when they START to spin");
    HAL_Delay(2000);

    const int16_t step = 5;
    const int16_t max_test = 200;   /* 一般死区不会超过 200 */
    const uint32_t interval = 500;  /* 每个 PWM 持续 500ms */

    /* 测试顺序: A正 -> A反 -> B正 -> B反 */
    const char *labels[4] = {"A_FWD", "A_BKW", "B_FWD", "B_BKW"};
    int8_t signs[4]       = { +1,      -1,      +1,      -1   };
    uint8_t channels[4]   = { 0,        0,       1,       1   }; /* 0=A, 1=B */

    for (int t = 0; t < 4; t++)
    {
        RTT_LOG_INFO("[DZTest] ---- Testing %s ----", labels[t]);
        HAL_Delay(1500);

        for (int16_t pwm = 0; pwm <= max_test; pwm += step)
        {
            int16_t v = pwm * signs[t];

            /* 注意:这里要绕过 apply_deadzone,直接调底层
             * 否则你测的就不是真实死区了。
             * 临时方案:把 apply_deadzone 里的 dz 设为 0 来测,
             * 或者写一个绕过死区的接口。见下方说明。 */
            if (channels[t] == 0) {
                Motor_SetSpeed(v, 0);
            } else {
                Motor_SetSpeed(0, v);
            }

            RTT_LOG_INFO("[DZTest] %s PWM = %d", labels[t], pwm);
            HAL_Delay(interval);
        }

        Motor_Stop();
        RTT_LOG_INFO("[DZTest] %s done, stopping for 2s", labels[t]);
        HAL_Delay(2000);
    }

    RTT_LOG_INFO("[DZTest] ==== All Done ====");
    Motor_Stop();
}





static void BT_DebugMonitor(void)
{
    BT_ControlData *bt = BT_GetControlData();

    if (bt->updated) {
        bt->updated = 0;     /* 消费标志 */

        const char *cmd_name = "?";
        switch (bt->command) {
            case BT_CMD_FORWARD:  cmd_name = "FORWARD";  break;
            case BT_CMD_BACKWARD: cmd_name = "BACKWARD"; break;
            case BT_CMD_LEFT:     cmd_name = "LEFT";     break;
            case BT_CMD_RIGHT:    cmd_name = "RIGHT";    break;
            case BT_CMD_STOP:     cmd_name = "STOP";     break;
            case BT_CMD_VELOCITY: cmd_name = "VELOCITY"; break;
            default: break;
        }

        int spd_cm = (int)(bt->speed_target * 100);   /* cm/s */
        int trn_x  = (int)(bt->turn_target  * 100);   /* ×100 */

        RTT_LOG_INFO("\x1B[1;32m[BT-MON] cmd=%s spd=%d cm/s turn=%d\x1B[0m",
                     cmd_name, spd_cm, trn_x);
    }
}









static void Encoder_VerifyTest(void)
{
    RTT_LOG_INFO("[EncTest] ==== Encoder Verify ====");
    Encoder_ResetDistance();

    uint32_t last_print = 0;
    uint32_t last_update = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* 每 5ms 更新一次编码器(模拟平衡环周期) */
        if (now - last_update >= 5) {
            last_update = now;
            Encoder_Update();
        }

        /* 每 200ms 打印 */
        if (now - last_print >= 200) {
            last_print = now;

            int distA_mm = (int)(Encoder_GetDistA_m() * 1000);   /* 毫米 */
            int distB_mm = (int)(Encoder_GetDistB_m() * 1000);
            int spdA     = (int)(Encoder_GetSpeedA_rps() * 100); /* 0.01 r/s */
            int spdB     = (int)(Encoder_GetSpeedB_rps() * 100);

            RTT_LOG_INFO("DistA=%d mm DistB=%d mm | SpdA=%d SpdB=%d (0.01r/s)",
                         distA_mm, distB_mm, spdA, spdB);
        }
    }
}








void Wait_Until_Reached(void) {
    while (1) {
        float err = Balance_GetPositionError();
        // 如果误差绝对值小于阈值，说明已到达
        if (err < POS_REACHED_THRESH && err > -POS_REACHED_THRESH) {
            break;
        }
        rt_thread_mdelay(20); // 每20ms检查一次，释放CPU
    }
}









/* USER CODE END 0 */

/*============================================================
 *  Main
 *============================================================*/
int main(void)
{
    HAL_Init();

    /* USER CODE BEGIN SysInit */
    SEGGER_RTT_Init();
    RTT_LOG_INFO("========================================");
    RTT_LOG_INFO("  balance_car start");
    RTT_LOG_INFO("  STM32F412 Balance Car v1.0");
    RTT_LOG_INFO("========================================");
    /* USER CODE END SysInit */

    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM12_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_SPI2_Init();
    Move_Init();

    /* USER CODE BEGIN 2 */
    App_Init();

    uint32_t last_status_tick = 0;
    uint32_t last_bt_tick     = 0;
//    /* ---- 主循环用到的状态变量 ---- */
//    uint32_t t_balance   = HAL_GetTick();
//    uint32_t t_move      = HAL_GetTick();
//    uint32_t script_timer = HAL_GetTick();
//    uint8_t  script_step  = 0;

        uint32_t boot_tick        = HAL_GetTick(); // <--- 声明 boot_tick
        uint8_t  pos_init_done    = 0;             // <--- 声明 pos_init_done




//        while (1)
//        {
//            /*--- 5ms 平衡环 ---*/
//            if (g_control_flag) {
//                g_control_flag = 0;
//                if (g_mpu_ok) Balance_Update();
//                else          Motor_Stop();
//            }
//
//            /*--- 蓝牙: 消费缓冲 + 监控 + 异步 ACK ---*/
//                BT_ProcessRing();      /* ← 新增,放在最前面 */
//                BT_DebugMonitor();
//                BT_FlushAck();
//
//            uint32_t now = HAL_GetTick();
//
//            /*--- 1s 状态打印 ---*/
//            if (now - last_status_tick >= STATUS_PERIOD_MS) {
//                last_status_tick = now;
//                if (g_mpu_ok) App_PrintStatus();
//            }
//
//            /*--- 1s 蓝牙心跳 ---*/
//            if (now - last_bt_tick >= 1000) {
//                last_bt_tick = now;
//                if (!data_received) RTT_LOG_WARN("[BT] No data in last 1s");
//                data_received = 0;
//            }
//        }

        uint8_t  hold_inited = 0;
        while (1)
               {
                   /*--- 5ms 平衡环 ---*/
                   if (g_control_flag) {
                       g_control_flag = 0;
                       if (g_mpu_ok) Balance_Update();
                       else          Motor_Stop();
                   }
                   /*--- 蓝牙: 消费缓冲 + 监控 + 异步 ACK ---*/
                                   BT_ProcessRing();      /* ← 新增,放在最前面 */
                                   BT_DebugMonitor();
                                   BT_FlushAck();
                   uint32_t now = HAL_GetTick();

                   /*--- 开机 5 秒后启用"自动位置环"模式 ---*/
                   if (!hold_inited && g_mpu_ok && now > 5000) {
                       hold_inited = 1;
                       Balance_EnableAutoHold();   /* ← 改成这个 */
                   }

                   /* ... status / bt 心跳 ... */
               }





//pid 3.0的平衡测试代码

//        uint8_t  hold_inited = 0;
//
//        while (1)
//        {
//            /*--- 5ms 平衡环 ---*/
//            if (g_control_flag) {
//                g_control_flag = 0;
//                if (g_mpu_ok) Balance_Update();
//                else          Motor_Stop();
//            }
//
//            uint32_t now = HAL_GetTick();
//
//            /*--- 开机 5 秒后启用"自动位置环"模式 ---*/
//            if (!hold_inited && g_mpu_ok && now > 5000) {
//                hold_inited = 1;
//                Balance_EnableAutoHold();   /* ← 改成这个 */
//            }
//
//            /* ... status / bt 心跳 ... */
//        }













    RTT_LOG_INFO("[Main] Enter main loop");
    /* USER CODE END 2 */


    while (1)
    {
        /* USER CODE BEGIN 3 */

        /*--- 5ms balance control ---*/
        if (g_control_flag)
        {
            g_control_flag = 0;

            if (g_mpu_ok) {
                Balance_Update();
            } else {
                Motor_Stop();
            }
        }

        uint32_t now = HAL_GetTick();

        /*--- Status print every 1s ---*/
        if (now - last_status_tick >= STATUS_PERIOD_MS)
        {
            last_status_tick = now;
            if (g_mpu_ok) {
                App_PrintStatus();
            }
        }

        /*--- BT watchdog every 1s ---*/
        if (now - last_bt_tick >= 1000)
        {
            last_bt_tick = now;
            if (!data_received) {
                RTT_LOG_WARN("[BT] No data in last 1s");
            }
            data_received = 0;
        }

        /* USER CODE END 3 */
    }
}

/*============================================================
 *  System Clock Config (HSI 16MHz, no PLL)
 *============================================================*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    RTT_LOG_ERROR("[SYS] Error_Handler! Halted");
    while (1) { }
}




#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    RTT_LOG_ERROR("[ASSERT] file=%s line=%lu", file, line);
}
#endif

