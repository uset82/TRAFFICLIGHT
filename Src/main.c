/*
 * ELE201 – Traffic Light PART 2 (TIM3 1 ms + EXTI, no HAL_Delay)
 *
 * External LEDs (GPIOB): PB8=RED, PB9=YELLOW, PB10=GREEN (active-high wiring)
 * Pedestrian button (GPIOD): PD3 → EXTI3 (button to 3V3, pulldown to GND)
 *
 * Spec:
 *  - Normal: RED 20 s → RED+YELLOW 5 s → GREEN 10 s → repeat
 *  - Press during GREEN: GREEN+YELLOW for 5 s, then only RED
 *  - Press during RED: add +10 s to the current RED time
 *
 * Design:
 *  - TIM3 fires every 1 ms. A simple FSM advances using an elapsed_ms counter.
 *  - EXTI3 only latches requests (debounced) and never blocks.
 *  - Pedestrian request is served before GREEN timeout in the TIM3 ISR.
 */
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* --- Pin mapping --- */
#define LED_PORT         GPIOB
#define LED_RED_Pin      GPIO_PIN_8
#define LED_YEL_Pin      GPIO_PIN_9
#define LED_GRN_Pin      GPIO_PIN_10

#define BTN_PORT         GPIOD
#define BTN_Pin          GPIO_PIN_3     // EXTI3 – button to 3.3V with pulldown to GND

/* --- Durations (ms) --- */
#define T_RED_MS         20000U  // 20 s
#define T_REDY_MS         5000U  // 5 s (red + yellow)
#define T_GRN_MS         10000U  // 10 s
#define T_PED_GY_MS       5000U  // 5 s (green + yellow on press)
#define T_RED_EXT_MS     10000U  // +10 s per press during red
#define BTN_DEBOUNCE_MS     50U

/* --- Peripherals --- */
TIM_HandleTypeDef htim3;

/* --- FSM --- */
typedef enum {
  ST_RED = 0,
  ST_RED_YEL,
  ST_GREEN,
  ST_PED_GREEN_YEL   // GREEN + YELLOW 5 s → then RED
} tl_state_t;

static volatile tl_state_t state = ST_RED;
static volatile uint32_t   elapsed_ms    = 0U;  // elapsed ms in current state
static volatile uint32_t   target_ms     = 0U;  // target duration for current state
static volatile uint32_t   red_extend_ms = 0U;  // accumulated extension while in RED
static volatile bool       req_ped       = false; // latched pedestrian request (when GREEN)
static volatile uint32_t   last_btn_ms   = 0U;  // debounce timestamp

/* ============ Prototipos ============ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void enter_state(tl_state_t s);
static inline void set_lights(bool r, bool y, bool g);

/* ===================== MAIN ===================== */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM3_Init();

  /* TIM3: 1 ms period (ISR every 1 ms) */
  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_TIM_Base_Start_IT(&htim3);

  enter_state(ST_RED);

  while (1) { __WFI(); }
}

/* ================== Callbacks ================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM3) return;

  /* Serve pedestrian request before GREEN timeout. */
  if (state == ST_GREEN && req_ped) {
    req_ped = false;
    enter_state(ST_PED_GREEN_YEL);
    return;  // do not also process GREEN timeout this tick
  }

  /* Advance by 1 ms */
  elapsed_ms++;

  switch (state)
  {
    case ST_RED:
      if (elapsed_ms >= (target_ms + red_extend_ms)) {
        enter_state(ST_RED_YEL);
      }
      break;

    case ST_RED_YEL:
      if (elapsed_ms >= target_ms) {
        enter_state(ST_GREEN);
      }
      break;

    case ST_GREEN:
      if (elapsed_ms >= target_ms) {
        enter_state(ST_RED);
      }
      break;

    case ST_PED_GREEN_YEL:
      if (elapsed_ms >= target_ms) {
        enter_state(ST_RED);
      }
      break;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != BTN_Pin) return;

  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - last_btn_ms) < BTN_DEBOUNCE_MS) return;  // debounce
  last_btn_ms = now;

  if (state == ST_GREEN) {
    /* Latch; the transition occurs in the TIM3 ISR */
    req_ped = true;
  } else if (state == ST_RED) {
    /* Add +10 s while staying in RED (accumulates) */
    if (red_extend_ms <= (UINT32_MAX - T_RED_EXT_MS))
      red_extend_ms += T_RED_EXT_MS;
  }
  /* Other states: ignored by spec */
}

/* ================== FSM helpers ================== */
static void enter_state(tl_state_t s)
{
  state = s;
  elapsed_ms = 0U;

  switch (s) {
    case ST_RED:
      set_lights(true, false, false);
      target_ms     = T_RED_MS;    // 20 s base
      red_extend_ms = 0U;          // will be extended by button presses in RED
      break;

    case ST_RED_YEL:
      set_lights(true, true, false);
      target_ms = T_REDY_MS;       // 5 s
      break;

    case ST_GREEN:
      set_lights(false, false, true);
      target_ms = T_GRN_MS;        // 10 s
      break;

    case ST_PED_GREEN_YEL:
      set_lights(false, true, true); // GREEN + YELLOW
      target_ms = T_PED_GY_MS;       // 5 s
      break;
  }
}

static inline void set_lights(bool r, bool y, bool g)
{
  HAL_GPIO_WritePin(LED_PORT, LED_RED_Pin, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_YEL_Pin, y ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_GRN_Pin, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ================== Initialization ================== */
static void MX_TIM3_Init(void)
{
  /* Configure TIM3 to 1 kHz (1 ms):
     - With APB1 prescaler != 1, the TIM3 kernel clock is 2*PCLK1 (STM32 timer x2 rule).
     - We fix ARR=999 so (ARR+1)=1000 counts → 1 ms at 1 MHz timer tick.
     - Compute PSC from the actual TIM3 clock so that TIM3/(PSC+1)=1 MHz. */
  RCC_ClkInitTypeDef clk;
  uint32_t flashLatency;
  HAL_RCC_GetClockConfig(&clk, &flashLatency);

  uint32_t pclk1   = HAL_RCC_GetPCLK1Freq();
  uint32_t timclk  = pclk1;
  if (clk.APB1CLKDivider != RCC_HCLK_DIV1) timclk *= 2U; // timer clock doubles when APB1 prescaler != 1

  uint32_t arr = 999U;                        // (ARR+1)=1000 counts → 1 ms
  uint32_t psc = (timclk / (1000U * (arr + 1U)));
  if (psc > 0U) psc -= 1U;                    // PSC = timclk/(1000*(ARR+1)) - 1

  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler         = psc;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = arr;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* External LEDs initially OFF */
  HAL_GPIO_WritePin(LED_PORT, LED_RED_Pin|LED_YEL_Pin|LED_GRN_Pin, GPIO_PIN_RESET);

  /* PB8/PB9/PB10 as outputs */
  GPIO_InitStruct.Pin   = LED_RED_Pin|LED_YEL_Pin|LED_GRN_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  /* PD3 as EXTI Rising; use pulldown to keep line low when unpressed */
  GPIO_InitStruct.Pin  = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_PORT, &GPIO_InitStruct);

  /* NVIC EXTI3 */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* ================== System clock (96 MHz) ================== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 4;
  RCC_OscInitStruct.PLL.PLLN       = 96;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ       = 4;
  RCC_OscInitStruct.PLL.PLLR       = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;   // 96 MHz
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;             // TIM3CLK = 2*PCLK1
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) { Error_Handler(); }
}

/* ================== Error handler ================== */
void Error_Handler(void)
{
  __disable_irq();
  HAL_GPIO_WritePin(LED_PORT, LED_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_PORT, LED_YEL_Pin|LED_GRN_Pin, GPIO_PIN_RESET);
  while (1) { __NOP(); }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){ (void)file; (void)line; }
#endif
