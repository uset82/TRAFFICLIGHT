# Traffic Light – PART 2

**Difficulties, Root Causes, Fixes, and Suggestions**

## 0) Summary (what finally works)

* Normal cycle: **RED 20 s → RED+YELLOW 5 s → GREEN 10 s → repeat**.
* If button is pressed during **GREEN** → **GREEN+YELLOW for 5 s**, then **RED**.
* If button is pressed during **RED** → **+10 s** added (accumulable).
* No `HAL_Delay()`; timing uses a **1 kHz TIM3 ISR (1 ms)** in a **non‑blocking** FSM.
* TIM3 prescaler is **computed from APB1** to guarantee 1 ms periods; durations are accurate.

---

## 1) What blocked us (symptoms)

* Pressing during GREEN made **YELLOW blink ~0.5 s** and jump straight to RED (should be 5 s).
* The external **RED LED looked “dead”** while YELLOW/GREEN worked.
* Early on, the button worked only when forced with a wire to 3.3 V.
* We hit a compile warning:
  `implicit declaration of function '__HAL_RCC_VOLTAGESCALING_CONFIG' …`

---

## 2) Root causes

1. **Wrong time base (tick 10× faster than expected)**

   * Our first designs measured durations in **timer ticks** (e.g., “50 ticks = 5 s”).
   * But TIM3’s real period was mis-configured (APB prescaler effects): our “100 ms” tick was closer to **10 ms**, so 50 ticks ≈ 0.5 s → exactly the symptom we saw.

2. **Missing explicit state for the pedestrian overlap**

   * We went directly **GREEN → RED** on a press, instead of inserting a dedicated **GREEN+YELLOW (5 s)** state.
   * Even after adding the state, we forgot a **`return` inside the timer ISR** right after switching to that state, so the same interrupt tick also advanced to RED.

3. **LED polarity mismatch (active-low vs active-high)**

   * Our breadboard wiring for the external RED was **3.3 V → resistor → LED → MCU pin**, i.e., **active-low** (pin LOW turns LED **on**).
   * The code initially drove LEDs as **active-high**, so the red never lit.

4. **EXTI configuration & hardware pull-down**

   * The button line PD3 is wired to **3.3 V on press** with a **10 kΩ pull-down to GND**.
   * Cube initially had PD3 with `NoPull` (OK with external 10 k) but any mismatch here or missing NVIC/handlers made the button flaky.

5. **NVIC/handlers not aligned**

   * If `TIM3_IRQHandler` or `EXTI3_IRQHandler` aren’t present (or not calling HAL handlers), interrupts won’t fire.

6. **Wrong macro name**

   * The correct macro is **`__HAL_PWR_VOLTAGESCALING_CONFIG`** (not `__HAL_RCC_…`). That fixed the build warning.

---

## 3) What we changed (the fixes)

### Timing & FSM

* Switched to a **millisecond‑based FSM driven by TIM3 @ 1 kHz**.
  The ISR increments `elapsed_ms`, and the **actual durations** compare `elapsed_ms >= target_ms` per state.
  TIM3 PSC/ARR are computed from the real timer clock so 1 ms is precise.

* Added an explicit state **`ST_PED_GREEN_YEL`** (GREEN+YELLOW).
  In `HAL_TIM_PeriodElapsedCallback`, when `state==ST_GREEN && ped_request==true`:

  * We **enter** `ST_PED_GREEN_YEL` and **`return` immediately** from the ISR to avoid also hitting GREEN’s timeout path in the same tick.

### Button & debounce

* EXTI on **PD3**, **Rising edge**, **`GPIO_PULLDOWN`** (or `NoPull` if the external 10 kΩ is guaranteed).
* Software **debounce 50 ms** with `HAL_GetTick()` difference.

### LED polarity

* We kept external LEDs **active-high** in the final wiring; if needed, the code can be trivially patched with per-LED polarity flags (one line change) to support active-low wiring.

### NVIC & handlers

* Ensured:

  ```c
  // stm32f7xx_it.c
  extern TIM_HandleTypeDef htim3;
  void TIM3_IRQHandler(void)  { HAL_TIM_IRQHandler(&htim3); }
  void EXTI3_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3); }
  ```
* Enabled **TIM3 global** and **EXTI3** interrupts in Cube.

### Build macro

* Replaced `__HAL_RCC_VOLTAGESCALING_CONFIG` with **`__HAL_PWR_VOLTAGESCALING_CONFIG`** in `SystemClock_Config()`.

---

## 4) Why we couldn’t accomplish it at first

* We assumed our timer tick was exactly 100 ms and wrote the FSM in **abstract ticks**.
  Because of the APB timer clock doubling rule (TIMCLK = 2×PCLK when prescaler ≠ 1), our tick time was **wrong by a factor ~10**, making the **5 s overlap appear as ~0.5 s**.
* We also didn’t create a **dedicated overlap state** initially, so a press during GREEN jumped to RED.
* LED **polarity confusion** made the RED “dead,” hiding part of the behavior during tests.

---

## 5) How we verified the final behavior

* **Unit-style checks** with stopwatch:

  * GREEN press → **GREEN+YELLOW ~5 s** → RED.
  * Multiple presses in RED → +10 s per press (try 1×, 2× = +20 s).
* **Button mashing** → no re-entries or jitter (debounce OK).
* **Power-on LED self-test** (optional) to confirm each LED can light regardless of the FSM.

---

## 6) Suggestions for the teacher / lab handout

1. **Make the timer-clock caveat explicit**

   * A one-page note explaining APB prescalers and **TIMxCLK doubling** would save a lot of time.
   * Provide a short “**compute PSC/ARR**” snippet using `HAL_RCC_GetPCLKxFreq()`.

2. **Starter template with required IRQ handlers**

   * Ship a minimal `stm32f7xx_it.c` including `TIM3_IRQHandler` and `EXTI3_IRQHandler` already wired to HAL.

3. **Explicit wiring diagrams for active-high vs active-low**

   * Show both options (MCU→resistor→LED→GND **vs** 3V3→resistor→LED→MCU) and how that flips the logic.

4. **“Do not use HAL_Delay in ISRs” reminder**

   * Encourage **non-blocking FSMs** using `HAL_GetTick()` or a stable time base.

5. **Checklist before flashing**

   * Pin map (PB8/9/10 out, PD3 EXTI Rising).

   * Pull config (external 10 k→GND means `NoPull` or `PullDown` is OK).

   * NVIC enabled for TIM3 and EXTI3.

   * Quick LED self-test code snippet.

6. **Provide the .ioc or screenshots** of CubeMX pages (GPIO, NVIC, Clock) matching the board.

---

## 7) Quick troubleshooting guide (if something breaks later)

* **Yellow overlap < 5 s** → Check TIM3 is **1 kHz** (ARR=999, PSC computed) and there’s a **`return`** right after switching to `ST_PED_GREEN_YEL` in the ISR.
* **Red never lights** → Check **polarity/wiring** (active-low vs active-high), LED orientation, PB8 set as **Output**.
* **Button ignored** → Confirm EXTI3 enabled, handler present, PD3 PullDown or external 10 k to GND.
* **Times all off** → Revisit `SystemClock_Config()`, verify `__HAL_PWR_VOLTAGESCALING_CONFIG`, and that TIM3 is started with interrupts.

---

**Bottom line:** the project failed initially because our **time base was wrong** and we **skipped a dedicated overlap state**. We fixed it by moving to **ms-accurate timing with `HAL_GetTick()`**, adding **`ST_PED_GREEN_YEL`** with a **hard 5 s**, enforcing a **`return`** in the ISR after the state switch, and by resolving **LED polarity** and **EXTI** details.