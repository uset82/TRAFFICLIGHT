# ELE201 – Traffic Light (PART 1)  
**Detail Description – Changes from `maintemplate.c` to the final `main.c`**

This document explains, in order, **what was done to make the project work** using a **Timer + Interrupt** and a **finite state machine (FSM)**. 
---

## 0) Goal & constraints

- **Goal:** Implement a traffic light sequence with 3 LEDs (RED, YELLOW, GREEN) following:
  - RED 20 s → RED+YELLOW 5 s → GREEN 10 s → repeat forever.
- **Constraints/Style:** As covered in class: **no `HAL_Delay()` for timing**; use **Timer (TIM3) with interrupt every 100 ms** and drive a **state machine** inside the **ISR**.
- **Pins used (class-friendly and grouped):**
  - `PB8 → LED_RED`  
  - `PB9 → LED_YELLOW`  
  - `PB10 → LED_GREEN`  
  - `PD3 → BUTTON (EXTI Rising, pull-down)` → prepared for PART 2; **not used** in PART 1.

---

## 1) What CubeMX already generated (baseline: `maintemplate.c`)

After configuring GPIO and TIM3 in STM32CubeMX, the **template** typically contains:
- `#include "main.h"` / drivers includes, peripheral handles (`htim2`, `htim3`, etc.).
- `SystemClock_Config()` set to run the MCU (here: PLL 96 MHz, APB1 = HCLK/2 → TIM3CLK = 96 MHz).
- `MX_GPIO_Init()` sets the selected pins to Output/Input (PB8, PB9, PB10 outputs; PD3 EXTI).
- `MX_TIM3_Init()` configures TIM3 **(PSC=959, ARR=9999)** → **10 Hz = 100 ms period**.
- `main()` calls `HAL_Init()`, `SystemClock_Config()`, `MX_*_Init()`, then an empty `while(1)`.

**What is still missing at this point:** the **application logic** (FSM, tick counting) and **starting the timer interrupt** + **LED control**.

---

## 2) High‑level changes introduced

| Area | Template (`maintemplate.c`) | Final `main.c` | Why |
|---|---|---|---|
| **Mapping** | GPIOB outputs exist, but no app labels | `#define LED_RED/YEL/GRN` on PB8/PB9/PB10; `PD3` as `BTN_Pin` | Clear, self‑documenting pin mapping used by the application code |
| **FSM Types** | Not present | `typedef enum { ST_RED, ST_RED_YEL, ST_GREEN }` | Models the 3 phases cleanly |
| **Timing Base** | TIM3 configured but unused | Use TIM3 @ 100 ms as **system tick** | Non‑blocking timing as covered in class |
| **Durations** | Not present | `#define T_RED_TICKS 200`, `T_REDY_TICKS 50`, `T_GRN_TICKS 100` | 100 ms × ticks → 20 s / 5 s / 10 s |
| **LED Control** | Not present | `set_lights(r,y,g)` helper | Single point to set LEDs atomically |
| **State Entry** | Not present | `enter_state(s)` sets LEDs + resets tick | Keeps ISR small and readable |
| **ISR** | Weak callback exists but empty | `HAL_TIM_PeriodElapsedCallback()` → increments `g_tick100ms` & advances FSM | Periodic, non‑blocking application logic |
| **main() runtime** | Empty `while(1)` | `enter_state(ST_RED)` + `HAL_TIM_Base_Start_IT(&htim3)` | Starts in RED and enables periodic ISR |
| **Button PD3** | Configured by CubeMX | EXTI Rising + Pull-Down; callback stub only | Prepared for PART 2 (not used now) |

---

## 3) Exact code blocks its added/edited

### 3.1 Preprocessor & user includes
```c
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */
```

### 3.2 Pin mapping & timing (USER PD)
```c
/* USER CODE BEGIN PD */
#define LED_RED_Port    GPIOB
#define LED_RED_Pin     GPIO_PIN_8
#define LED_YEL_Port    GPIOB
#define LED_YEL_Pin     GPIO_PIN_9
#define LED_GRN_Port    GPIOB
#define LED_GRN_Pin     GPIO_PIN_10
#define BTN_Port        GPIOD
#define BTN_Pin         GPIO_PIN_3   // EXTI3 (not used in PART 1)

#define T_RED_TICKS     200   // 20 s at 100 ms/tick
#define T_REDY_TICKS     50   // 5 s
#define T_GRN_TICKS     100   // 10 s
/* USER CODE END PD */
```

### 3.3 FSM types & state variables (USER 0)
```c
/* USER CODE BEGIN 0 */
typedef enum { ST_RED = 0, ST_RED_YEL, ST_GREEN } tl_state_t;

static volatile tl_state_t g_state = ST_RED;
static volatile uint32_t   g_tick100ms = 0;   // increments every 100 ms

static inline void set_lights(bool r, bool y, bool g){
  HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_YEL_Port, LED_YEL_Pin, y ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GRN_Port, LED_GRN_Pin, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void enter_state(tl_state_t s){
  g_state = s;
  g_tick100ms = 0;
  switch(s){
    case ST_RED:     set_lights(true,  false, false); break;
    case ST_RED_YEL: set_lights(true,  true,  false); break;
    case ST_GREEN:   set_lights(false, false, true ); break;
  }
}
/* USER CODE END 0 */
```

### 3.4 `main()` – start in RED + start TIM3 interrupt
```c
/* USER CODE BEGIN 2 */
enter_state(ST_RED);                 // a) only RED at boot
HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(TIM3_IRQn);
HAL_TIM_Base_Start_IT(&htim3);       // periodic ISR every 100 ms
/* USER CODE END 2 */
```

### 3.5 Timer ISR – core application logic (USER 4)
```c
/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    g_tick100ms++;
    switch (g_state) {
      case ST_RED:
        if (g_tick100ms >= T_RED_TICKS)   enter_state(ST_RED_YEL);
        break;
      case ST_RED_YEL:
        if (g_tick100ms >= T_REDY_TICKS)  enter_state(ST_GREEN);
        break;
      case ST_GREEN:
        if (g_tick100ms >= T_GRN_TICKS)   enter_state(ST_RED);
        break;
    }
  }
}
/* USER CODE END 4 */
```

---

## 4) Timer math (why PSC=959, ARR=9999)

From the CubeMX clock tree: **TIM3CLK = 96 MHz** (APB1 divider = 2 → timer clock doubled to 96 MHz).  
The design targets an **update event every 100 ms** = **10 Hz**.

The configuration sets:
- **Prescaler = 959** → counter clock = 96 MHz / (959 + 1) = **100 kHz**
- **ARR = 9999** → update = 100 kHz / (9999 + 1) = **10 Hz**

Thus **`HAL_TIM_PeriodElapsedCallback()` fires every 100 ms.**

---

## 5) GPIO configuration that must match the mapping

In `MX_GPIO_Init()` ensure:
- **PB8, PB9, PB10** are **`GPIO_MODE_OUTPUT_PP`**, no pull, low speed.
- **PD3** is **`GPIO_MODE_IT_RISING`** with **`GPIO_PULLDOWN`** (for PART 2).
- Initial output level for PB8/9/10 is **RESET** (off). Then `enter_state(ST_RED)` turns **PB8** on.

---

## 6) Bring‑up checklist (if LEDs do not light)

1. **Smoke test** (optional): right after `MX_GPIO_Init()`
   ```c
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_SET);
   HAL_Delay(1000);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);
   ```
   If they don’t turn on → wiring issue (wrong header, no GND, LED reversed, no resistor).

2. **Header pins:** PB8/PB9/PB10 are on the Arduino/Zio header (D15/D14/D13 on many Nucleo layouts).

3. **Common GND:** Board GND must be tied to the protoboard GND.

4. **LED orientation:** Anode (long leg) toward the MCU pin through a **220–330 Ω** resistor; cathode to **GND**.

5. **Timer running:** Confirm `HAL_TIM_Base_Start_IT(&htim3)` is called and TIM3 NVIC is enabled.

---

## 7) Rationale for this architecture (as covered in class)

- **Non‑blocking**: No `HAL_Delay()`. The CPU remains free to perform other tasks (e.g., handle the push button in PART 2).  
- **Deterministic**: `TIM3` generates a precise **100 ms time base**.  
- **Maintainable**: The FSM isolates *what happens* in each phase from *how time passes*.  
- **Extensible**: Adding the **button rule‑set** (PART 2) only requires additional logic in the ISR or a separate event handler.

---

## 8) Minimal reproducible steps summary

1. **CubeMX**: set PB8/PB9/PB10 as outputs; PD3 as EXTI (pull‑down); TIM3 PSC=959, ARR=9999.  
2. **Generate code**; open in the IDE.  
3. **Add** (in USER sections): mapping `#define`s, FSM typedef & variables, `set_lights()`, `enter_state()`, the ISR, and in `main()` call `enter_state(ST_RED)` + `HAL_TIM_Base_Start_IT(&htim3)`.  
4. **Wire** LEDs to PB8/9/10 via 220–330 Ω to GND; PD3 prepared with 10 kΩ pull‑down (for PART 2).  
5. **Flash and test**: observe RED 20 s → RED+YEL 5 s → GREEN 10 s → repeat.

---

### Appendix A — Alternative with `HAL_GetTick()`
The same FSM can be driven by `HAL_GetTick()` (1 ms SysTick) instead of a hardware timer. This works, but the TIM3 approach scales better when adding more periodic tasks.
