/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ELE201 Traffic Light – PART 2 (Timer + EXTI)
  ******************************************************************************
  */
 /* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { ST_RED = 0, ST_RED_YEL, ST_GREEN, ST_GRN_YEL_PED } tl_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- Pin mapping (external LEDs) --- */
#define LED_RED_Port    GPIOB
#define LED_RED_Pin     GPIO_PIN_8
#define LED_YEL_Port    GPIOB
#define LED_YEL_Pin     GPIO_PIN_9
#define LED_GRN_Port    GPIOB
#define LED_GRN_Pin     GPIO_PIN_10

/* Button (pedestrian) */
#define BTN_Port        GPIOD
#define BTN_Pin         GPIO_PIN_3     // EXTI3

/* Button polarity: 1 = active-high (press -> logic 1), 0 = active-low (press -> logic 0) */
#ifndef BTN_ACTIVE_HIGH
#define BTN_ACTIVE_HIGH 1
#endif
#define BTN_ACTIVE_HIGH 0
/* LED polarity: 1 = active-high (SET turns LED on), 0 = active-low (RESET turns LED on) */
#ifndef LED_ACTIVE_HIGH
#define LED_ACTIVE_HIGH 1
#endif
#define LED_ACTIVE_HIGH 0
/* Optional boot LED probe (ms per LED, set 0 to skip) */
#ifndef LED_BOOT_PROBE_MS
#define LED_BOOT_PROBE_MS 600
#endif

/* Durations in 100 ms ticks (TIM3 base) */
#define T_RED_TICKS     200   // 20 s
#define T_REDY_TICKS     50   // 5 s
#define T_GRN_TICKS     100   // 10 s
#define T_PED_GY_TICKS   50   // 5 s (GREEN+YELLOW after press)

/* Debounce */
#define BTN_DEBOUNCE_MS  50
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ )
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT];
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT];
#elif defined ( __CC_ARM )
__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT];
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT];
#elif defined ( __GNUC__ )
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection")));
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart3;
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/* FSM + timing */
static volatile tl_state_t g_state        = ST_RED;
static volatile uint32_t   g_tick100ms    = 0;
static volatile uint32_t   g_target_ticks = 0;

/* Button handling */
static volatile bool       g_ped_request  = false; // latched while GREEN, served by TIM3 ISR
static volatile bool       g_ped_active   = false; // true during GREEN+YELLOW 5 s phase
static volatile bool       g_red_extended = false; // allow +10 s once per red
static volatile uint32_t   g_last_btn_ms  = 0;     // debounce (ms)
static volatile uint32_t   g_state_start_ms = 0;   // ms timestamp of state entry
/* USER CODE END PV */

/* Prototypes ---------------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static inline void set_lights(bool r, bool y, bool g);
static void enter_state(tl_state_t s);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static inline GPIO_PinState led_level(bool on)
{
  if (LED_ACTIVE_HIGH) {
    return on ? GPIO_PIN_SET : GPIO_PIN_RESET;
  } else {
    return on ? GPIO_PIN_RESET : GPIO_PIN_SET; // active-low
  }
}

static inline void set_lights(bool r, bool y, bool g)
{
  HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, led_level(r));
  HAL_GPIO_WritePin(LED_YEL_Port, LED_YEL_Pin, led_level(y));
  HAL_GPIO_WritePin(LED_GRN_Port, LED_GRN_Pin, led_level(g));
}

static void enter_state(tl_state_t s)
{
  g_state = s;
  g_tick100ms = 0;
  g_state_start_ms = HAL_GetTick();
  /* Default: LD2 OFF (used as debug indicator for pedestrian overlap) */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  switch (s) {
    case ST_RED:
      set_lights(true,  false, false);
      g_target_ticks  = T_RED_TICKS;
      g_red_extended  = false;           // new red cycle: allow one +10s
      break;

    case ST_RED_YEL:
      set_lights(true,  true,  false);
      g_target_ticks = T_REDY_TICKS;
      break;

    case ST_GREEN:
      set_lights(false, false, true );
      g_target_ticks = T_GRN_TICKS;
      break;

    case ST_GRN_YEL_PED:                 // pedestrian overlap
      set_lights(false, true,  true );   // GREEN + YELLOW
      g_target_ticks = T_PED_GY_TICKS;   // nominal 5 s (tick-based, but ms enforces below)
      g_ped_active   = true;             // ignore button during this window
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // debug: LD2 ON during overlap
      break;
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
    /* Ensure LEDs OFF respecting polarity */
    set_lights(false, false, false);
  /* USER CODE BEGIN 2 */
  enter_state(ST_RED);                       // start in RED
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_TIM_Base_Start_IT(&htim3);             // 100 ms periodic interrupt
  /* USER CODE END 2 */

    /* Optional boot-time LED mapping/polarity probe (R -> Y -> G) */
  #if (LED_BOOT_PROBE_MS > 0)
    set_lights(true, false, false);
    HAL_Delay(LED_BOOT_PROBE_MS);
    set_lights(false, true, false);
    HAL_Delay(LED_BOOT_PROBE_MS);
    set_lights(false, false, true);
    HAL_Delay(LED_BOOT_PROBE_MS);
    set_lights(false, false, false);
  #endif
  while (1) {
    // Non-blocking app; all timing handled in interrupts
  }
}

/* ===== SystemClock & Peripherals (as generated by CubeMX) ===== */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK; // 96 MHz
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;           // TIM3CLK = 96 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) { Error_Handler(); }
}

/* TIM3 = 100 ms base: PSC=959, ARR=9999 */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler         = 959;   // 96 MHz / (959+1) = 100 kHz
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 9999;  // 100 kHz / (9999+1) = 10 Hz -> 100 ms
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/* Other MX_* initializers left as generated */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* Start external LEDs OFF (level set later via set_lights in main()) */

  /* PB8/PB9/PB10 as outputs (traffic LEDs) */
  GPIO_InitStruct.Pin   = LED_RED_Pin | LED_YEL_Pin | LED_GRN_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* On-board LD2 (PB7) as output for debug */
  GPIO_InitStruct.Pin   = LD2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* PD3 button as EXTI with configurable edge and pull */
  GPIO_InitStruct.Pin  = BTN_Pin;
  GPIO_InitStruct.Mode = BTN_ACTIVE_HIGH ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = BTN_ACTIVE_HIGH ? GPIO_PULLDOWN : GPIO_PULLUP;
  HAL_GPIO_Init(BTN_Port, &GPIO_InitStruct);

  /* NVIC for EXTI3 */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* Stubs — keep as generated in your project */
static void MX_ETH_Init(void){ /* ... generated ... */ }
static void MX_USART3_UART_Init(void){ /* ... generated ... */ }
static void MX_USB_OTG_FS_PCD_Init(void){ /* ... generated ... */ }

/* USER CODE BEGIN 4 */
/* TIM3 ISR — 100 ms heartbeat */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    /* Serve pedestrian request immediately when GREEN, then exit ISR */
    if (g_state == ST_GREEN && g_ped_request) {
      g_ped_request = false;
      enter_state(ST_GRN_YEL_PED);       // GREEN + YELLOW for 5 s
      return;                            // crucial: do not continue this tick
    }

    /* Normal tick accounting */
    g_tick100ms++;

    switch (g_state)
    {
      case ST_RED:
        if (g_tick100ms >= g_target_ticks) enter_state(ST_RED_YEL);
        break;

      case ST_RED_YEL:
        if (g_tick100ms >= g_target_ticks) enter_state(ST_GREEN);
        break;

      case ST_GREEN:
        if (g_tick100ms >= g_target_ticks) {
          /* Boundary guard: if a request arrived right at the end of GREEN */
          if (g_ped_request) { g_ped_request = false; enter_state(ST_GRN_YEL_PED); }
          else               { enter_state(ST_RED); }
        }
        break;

      case ST_GRN_YEL_PED:
        // Enforce exact 5s overlap using ms timing (independent of TIM3 tick)
        if ((HAL_GetTick() - g_state_start_ms) >= 5000u) {
          enter_state(ST_RED);           // after 5 s overlap, go RED
          g_ped_active = false;          // pedestrian phase finished
        }
        break;
    }
  }
}

/* PD3 EXTI — only latch; FSM runs in TIM3 ISR */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BTN_Pin)
  {
    uint32_t now = HAL_GetTick();
    if ((now - g_last_btn_ms) < BTN_DEBOUNCE_MS) return;   // debounce
    g_last_btn_ms = now;

    if (g_ped_active) return;            // ignore during G+Y window

    if (g_state == ST_GREEN) {
      g_ped_request = true;              // will be served in TIM3 ISR
    }
    else if (g_state == ST_RED && !g_red_extended) {
      g_target_ticks += 100;             // +10 s (100 * 100 ms)
      g_red_extended  = true;            // only once per red cycle
    }
  }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(LED_RED_Port, LED_RED_Pin);
    HAL_Delay(100);
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){ (void)file; (void)line; }
#endif
