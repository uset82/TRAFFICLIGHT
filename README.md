ELE201 – Traffic Light 

[![CI](https://github.com/uset82/TRAFFICLIGHT/actions/workflows/ci.yml/badge.svg)](https://github.com/uset82/TRAFFICLIGHT/actions/workflows/ci.yml)

Overview
This project implements the microcontroller part of ELE201 – Assignment 2 on an STM32F767ZI (Nucleo‑F767ZI) using STM32Cube HAL. It fulfills the Part 2 requirements without using HAL_Delay by combining a timer interrupt (TIM3) and an external interrupt (EXTI) for a push button.

Hardware
- Board: Nucleo‑F767ZI
- External LEDs (active‑high wiring):
  - PB8  → RED (series resistor to 3V3, cathode to GND)
  - PB9  → YELLOW
  - PB10 → GREEN
- Pedestrian button:
  - PD3 (EXTI3), button to 3V3, pulldown to GND (external 10 kΩ or internal pulldown enabled in code)

## Part 1 – Traffic Light (no button)

- **Behavior**:
  - On start: only RED is on
  - Sequence: RED 20 s → RED + YELLOW 5 s → GREEN 10 s → repeat indefinitely
- **Wiring**:
  - LEDs use PB8 (RED), PB9 (YELLOW), PB10 (GREEN); active‑high. Use series resistors to 3V3; LED cathodes to GND. Matches Figure‑1 in the assignment.
  - The PD3 button is configured in code but unused in Part 1.
- **Implementation (file `part1.c`)**:
  - Finite state machine with states: `ST_RED`, `ST_RED_YEL`, `ST_GREEN`.
  - Time base: `TIM3` generates an interrupt every 100 ms (10 Hz). With a 96 MHz timer clock, prescaler = 959 and period = 9999 yield 100 ms per update.
  - Tick counter: `g_tick100ms` increments in `HAL_TIM_PeriodElapsedCallback`. `enter_state(...)` switches LEDs via `set_lights(...)` and resets the counter.
  - LED control: `set_lights(red, yellow, green)` writes PB8/PB9/PB10 through HAL GPIO.
- **Durations and mapping to the assignment**:
  - `T_RED_TICKS = 200` → 200 × 0.1 s = 20 s (RED)
  - `T_REDY_TICKS = 50` → 50 × 0.1 s = 5 s (RED + YELLOW)
  - `T_GRN_TICKS = 100` → 100 × 0.1 s = 10 s (GREEN)
- **Where to look in the code**:
  - Pin defines and durations: top of `part1.c`
  - State entry and LED setting: `enter_state`
  - Timer ISR with transitions: `HAL_TIM_PeriodElapsedCallback` (checks `TIM3` and advances the FSM)
  - GPIO setup for PB8/PB9/PB10: `MX_GPIO_Init`


  -----------------------------------------------------------------------------------------------------

## Part 2 – Traffic Light  


Functional Requirements (Part 2)
- Normal cycle repeats:
  1) RED 20 s
  2) RED + YELLOW 5 s
  3) GREEN 10 s
- Button pressed while GREEN:
  - Show GREEN + YELLOW for 5 s, then only RED.
- Button pressed while RED:
  - Add +10 s to the current RED duration (accumulates if pressed multiple times).

Design Summary
- No HAL_Delay: All timing is done with a 1 kHz TIM3 interrupt (1 ms resolution).
- Simple FSM with four states: RED → RED_YELLOW → GREEN → PED_GREEN_YELLOW → RED.
- Debounced EXTI (PD3) only latches events; the FSM consumes them in the timer ISR.
- Pedestrian request is served before the GREEN timeout, so the 5 s overlap cannot be pre‑empted.

Code Structure (Src/main.c)
- TIM3 1 kHz configuration (ARR=999, PSC computed from real APB1 timer clock).
- EXTI3 debounce and latching (50 ms) for the button on PD3.
- FSM logic in TIM3 ISR using elapsed_ms and target_ms per state.
- External LEDs are controlled by set_lights(red, yellow, green).

Build / Flash (PlatformIO)
1) Install PlatformIO (VS Code extension or CLI).
2) From the project root, run: pio run
3) Flash the board: pio run -t upload

Files of Interest
- Src/main.c: Application entry, TIM3/EXTI setup, and FSM.
- Src/stm32f7xx_it.c: Default IRQ handlers forwarding to HAL.
- Inc/main.h: HAL pin names for on‑board LEDs and user button (if used).

Notes / Tips
- If external LEDs behave inverted, check wiring polarity; code assumes active‑high: driving PB8/PB9/PB10 high turns LEDs on.
- If button has no external pulldown, the code enables internal pulldown to keep PD3 stable when unpressed.

How it works
1) TIM3 generates an interrupt every 1 ms. A state‑local counter (elapsed_ms) increments inside the ISR.
2) The FSM compares elapsed_ms with target_ms to decide when to transition.
3) When the button EXTI fires during GREEN, we only set a flag; the TIM3 ISR sees the flag and immediately transitions to the pedestrian state (GREEN+YELLOW) and holds it for 5 s.
4) When the button EXTI fires during RED, we add 10 000 ms to the current red extension counter; the TIM3 ISR uses that to extend the red state before moving on.


