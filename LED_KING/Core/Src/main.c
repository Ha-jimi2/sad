/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint8_t running;
    uint8_t dir;
    uint16_t step_ms;
    uint8_t led;
} RunState;

typedef struct
{
    uint8_t raw;
    uint8_t stable;
    uint32_t t_change;
} KeyState;

#define PKT_LEN  6
typedef struct
{
    uint8_t buf[PKT_LEN];
    uint8_t cnt;
} PktState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_N        3
#define STEP_DEFAULT 500

#define RB_SIZE 32

#define PKT_HEAD 0xAA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static RunState g_run = {1, 0, STEP_DEFAULT, 1};

static uint32_t g_led_tick = 0;

static KeyState g_key_dir = {1, 1, 0};
static KeyState g_key_pause = {1, 1, 0};

static uint8_t g_uart_ch = 0;

static volatile uint8_t g_rb[RB_SIZE];
static volatile uint8_t g_rb_head = 0;
static volatile uint8_t g_rb_tail = 0;

static PktState g_pkt = {{0}, 0};

static char g_tx_buf[64];
static volatile uint8_t g_tx_busy = 0;
static volatile uint8_t g_tx_pending = 0;

static RunState g_ui_last;
static uint8_t  g_ui_valid = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Led_Show(uint8_t idx)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (idx == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (idx == 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (idx == 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t Led_Next(uint8_t cur, uint8_t dir)
{
    if (dir == 0)
        return (cur >= LED_N) ? 1 : (uint8_t)(cur + 1);
    return (cur <= 1) ? LED_N : (uint8_t)(cur - 1);
}

static uint8_t Key_Edge(KeyState* k, GPIO_TypeDef* port, uint16_t pin)
{
    uint8_t raw = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 0 : 1;

    if (raw != k->raw)
    {
        k->raw = raw;
        k->t_change = HAL_GetTick();
        return 0;
    }

    if (raw != k->stable && (HAL_GetTick() - k->t_change) >= 20)
    {
        k->stable = raw;
        return (uint8_t)(k->stable == 0);
    }

    return 0;
}

static void Uart_RequestFeedback(void)
{
    g_tx_pending = 1;
}

static void Rb_Put(uint8_t b)
{
    uint8_t next = (uint8_t)((g_rb_head + 1) & (RB_SIZE - 1));
    if (next == g_rb_tail) return;
    g_rb[g_rb_head] = b;
    g_rb_head = next;
}

static uint8_t Rb_Get(uint8_t* b)
{
    if (g_rb_tail == g_rb_head) return 0;
    *b = g_rb[g_rb_tail];
    g_rb_tail = (uint8_t)((g_rb_tail + 1) & (RB_SIZE - 1));
    return 1;
}

static uint8_t Pkt_Checksum(const uint8_t* p)
{
    uint16_t sum = 0;
    uint8_t i;
    for (i = 0; i < 5; i++) sum += p[i];
    return (uint8_t)(sum & 0xFF);
}

static void Proto_Apply(const uint8_t* p)
{
    uint8_t run = p[1];
    uint8_t dir = p[2];
    uint16_t ms = (uint16_t)((uint16_t)p[3] | ((uint16_t)p[4] << 8));

    if (run > 1 || dir > 1) return;
    if (ms < 100 || ms > 2000) return;


    g_run.running = run;
    g_run.dir = dir;
    g_run.step_ms = ms;
    g_led_tick = HAL_GetTick();

    Uart_RequestFeedback();
}

static void Proto_Poll(void)
{
    uint8_t b;

    while (Rb_Get(&b))
    {
        if (g_pkt.cnt == 0)
        {
            if (b == PKT_HEAD) g_pkt.buf[g_pkt.cnt++] = b;
            continue;
        }
        g_pkt.buf[g_pkt.cnt++] = b;

        if (g_pkt.cnt == PKT_LEN)
        {
            g_pkt.cnt = 0;
            if (g_pkt.buf[5] == Pkt_Checksum(g_pkt.buf))
                Proto_Apply(g_pkt.buf);
        }
    }
}

static void Feedback_Poll(void)
{
    int n;

    if (!g_tx_pending || g_tx_busy) return;
    g_tx_pending = 0;

    n = snprintf(g_tx_buf, sizeof g_tx_buf,
                 "STATE=%s,DIR=%s,STEP=%u,LED=%u\r\n",
                 g_run.running ? "RUN" : "PAUSE",
                 g_run.dir ? "REV" : "FWD",
                 (unsigned)g_run.step_ms,
                 (unsigned)g_run.led);

    if (n <= 0 || n >= (int)sizeof g_tx_buf) return;

    g_tx_busy = 1;
    if (HAL_UART_Transmit_IT(&huart1, (const uint8_t*)g_tx_buf, (uint16_t)n) != HAL_OK)
        g_tx_busy = 0;
}

static void Ui_Refresh(void)
{
    char buf[8];
    uint8_t need_all;

    if (g_ui_valid &&
        g_ui_last.running == g_run.running &&
        g_ui_last.dir     == g_run.dir     &&
        g_ui_last.step_ms == g_run.step_ms &&
        g_ui_last.led     == g_run.led)
    {
        return;
    }

    need_all = (uint8_t)(!g_ui_valid ||
                         g_ui_last.running != g_run.running ||
                         g_ui_last.dir     != g_run.dir     ||
                         g_ui_last.step_ms != g_run.step_ms);

    g_ui_last  = g_run;
    g_ui_valid = 1;

    if (need_all)
    {
        OLED_ShowString(1, 1, "STATE:");
        OLED_ShowString(1, 7, g_run.running ? "RUN  " : "PAUSE");

        OLED_ShowString(2, 1, "DIR:  ");
        OLED_ShowString(2, 7, g_run.dir ? "REV" : "FWD");

        snprintf(buf, sizeof buf, "%4ums", (unsigned)g_run.step_ms);
        OLED_ShowString(3, 1, "STEP: ");
        OLED_ShowString(3, 7, buf);
    }

    snprintf(buf, sizeof buf, "%u", (unsigned)g_run.led);
    OLED_ShowString(4, 1, "LED:  ");
    OLED_ShowString(4, 7, buf);
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

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    g_led_tick = HAL_GetTick();
    g_key_dir.t_change = HAL_GetTick();
    g_key_pause.t_change = HAL_GetTick();

    Led_Show(g_run.led);

    HAL_UART_Receive_IT(&huart1, &g_uart_ch, 1);
    Uart_RequestFeedback();

    OLED_Init();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (Key_Edge(&g_key_dir, KEY_DIR_GPIO_Port, KEY_DIR_Pin))
        {
            g_run.dir ^= 1;
            Uart_RequestFeedback();
        }

        if (Key_Edge(&g_key_pause, KEY_PAUSE_GPIO_Port, KEY_PAUSE_Pin))
        {
            g_run.running ^= 1;
            g_led_tick = HAL_GetTick();
            Uart_RequestFeedback();
        }

        if (g_run.running)
        {
            if (HAL_GetTick() - g_led_tick >= g_run.step_ms)
            {
                g_led_tick = HAL_GetTick();
                g_run.led = Led_Next(g_run.led, g_run.dir);
                Led_Show(g_run.led);
            }
        }
        else
        {
            g_led_tick = HAL_GetTick();
        }

        Proto_Poll();
        Feedback_Poll();
        Ui_Refresh();
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

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        Rb_Put(g_uart_ch);
        HAL_UART_Receive_IT(&huart1, &g_uart_ch, 1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        g_tx_busy = 0;
    }
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
