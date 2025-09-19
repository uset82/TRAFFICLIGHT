# STM32 Traffic Light Controller (ELE201)

A pedagogical traffic-light controller for the STM32 Nucleo‑F767ZI using the STM32Cube HAL and PlatformIO. It demonstrates timers, external interrupts (EXTI), GPIO control, and a clean finite state machine (FSM) architecture.

Two variants are included:
- `part1.c`: Basic timed cycle (no button)
- `Src/main.c`: Part 2 – adds a pedestrian push‑button with EXTI and debouncing
- `maintemplate.c`: The original STM32CubeMX template (baseline project scaffold)


## Hardware at a glance

- Board: Nucleo‑F767ZI (STM32F767ZI)
- External LEDs (traffic lights):
  - RED  -> PB8
  - YELLOW -> PB9
  - GREEN -> PB10
- Pedestrian push button: PD3 (EXTI3)
- On‑board LED used as debug: LD2 (PB7)

Polarity (very important):
- LEDs are configured by default as active‑low (pin drives LOW to turn LED on). This suits a common‑anode wiring where LED anodes connect to +3.3V via resistors and the MCU pin sinks current.
- The button is configured by default as active‑low with an internal pull‑up. Press connects PD3 to GND.

You can change these with two macros in `Src/main.c`:
- `#define LED_ACTIVE_HIGH 0`  // set to 1 if your LEDs turn on when pin is HIGH
- `#define BTN_ACTIVE_HIGH 0`  // set to 1 if your button pulls the pin HIGH when pressed

When you change the button polarity, the edge and pull in `MX_GPIO_Init()` automatically adapt (rising+pulldown for active‑high, falling+pullup for active‑low).

Typical wiring (defaults, active‑low shown):
- PB8/PB9/PB10 → LED cathodes; LED anodes → 330–1kΩ → 3.3V
- PD3 → button → GND (internal pull‑up enabled)


## Build and flash

This project uses PlatformIO with the STM32Cube framework.

- Board in `platformio.ini`: `nucleo_f767zi`
- Serial monitor speed: `115200` (no prints by default)

From VS Code (recommended):
- Run the task “PlatformIO: Build & Upload”, or use the PlatformIO toolbar.

From a PowerShell terminal:
```powershell
# Build
C:\Users\carlo\.platformio\penv\Scripts\platformio.exe run

# Flash the board
C:\Users\carlo\.platformio\penv\Scripts\platformio.exe run --target upload
```

If you have `pio` on PATH, you can use:
```powershell
pio run
pio run --target upload
```


## What the program does (Part 2)

It implements a traffic light with realistic timing and a pedestrian button:

Traffic‑light states (FSM):
1. RED (20 s)
2. RED+YELLOW (5 s)
3. GREEN (10 s)
4. GREEN+YELLOW with pedestrian overlap (5 s) – only when the button is pressed during GREEN

Pedestrian behavior:
- If you press the button during GREEN, the system immediately enters “GREEN+YELLOW overlap” for 5 seconds, then it goes to RED.
- If you press during RED, the current RED can be extended by +10 seconds once per red cycle.
- Button presses are debounced (50 ms).

Timing backbone:
- TIM3 is configured to fire an interrupt every 100 ms.
- A simple counter accumulates “ticks” to determine when to transition between states.
- The 5 s pedestrian overlap window is enforced with millisecond timing (`HAL_GetTick`) to make it exact even if a tick is skipped.

Debug aid:
- The on‑board LED LD2 turns ON while the system is in the pedestrian overlap (GREEN+YELLOW) to visualize that special window.
- Optional boot‑time LED probe (`LED_BOOT_PROBE_MS`) briefly cycles R→Y→G to verify wiring and polarity.

## Assignment mapping and acceptance criteria

This repository matches the ELE201 assignment parts as follows.

Part 1 (file: `part1.c`)
- a) On startup, only RED is ON.
- b) RED stays ON for 20 seconds.
- c) Then RED+YELLOW stays ON for 5 seconds.
- d) Finally, only GREEN stays ON for 10 seconds.
- e) Repeat forever.

How it’s implemented: a timer‑driven FSM in the TIM3 interrupt switches states based on 100 ms tick counters. LEDs are driven via `set_lights(r,y,g)`.

Part 2 (file: `Src/main.c`)
- a) A push button acts as a pedestrian request on PD3 (EXTI3).
  - i) If GREEN is ON when pressed: immediately enter GREEN+YELLOW for 5 seconds, then go to RED.
  - ii) If RED is ON when pressed: extend the current RED by +10 seconds (once per red cycle).

How it’s implemented: the EXTI callback latches requests and applies debounce; the TIM3 ISR executes the FSM and serves the request, ensuring the exact 5 s overlap window using millisecond timing.

State diagram (Part 2):

RED --20s--> RED+YEL --5s--> GREEN --10s--> RED
  ^                               |
  |                               | (button during GREEN)
  |                    GREEN+YEL (5s)
  +-------------------------------+

Button during RED: extend RED by +10 s (one time per red cycle).


## Code walkthrough

Main files:
- `Src/main.c`  — Final Part 2 with button + EXTI and debouncing
- `part1.c`     — Simpler Part 1 (no button), same timer‑driven FSM pattern
- `Inc/main.h`  — CubeMX‑generated pins (LD1/LD2, etc.) and peripherals
- `platformio.ini` — PlatformIO configuration (board, framework, flags)

Key ideas in `Src/main.c`:

1) FSM states and durations
- States: `ST_RED`, `ST_RED_YEL`, `ST_GREEN`, `ST_GRN_YEL_PED`
- Durations are in 100 ms ticks:
  - `T_RED_TICKS = 200` (20 s)
  - `T_REDY_TICKS = 50` (5 s)
  - `T_GRN_TICKS = 100` (10 s)
  - `T_PED_GY_TICKS = 50` (5 s nominal; actual 5 s enforced in ms)

2) LED control with polarity handling
```c
static inline GPIO_PinState led_level(bool on) {
  if (LED_ACTIVE_HIGH) return on ? GPIO_PIN_SET : GPIO_PIN_RESET;
  else                  return on ? GPIO_PIN_RESET : GPIO_PIN_SET; // active‑low
}
static inline void set_lights(bool r, bool y, bool g) {
  HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, led_level(r));
  HAL_GPIO_WritePin(LED_YEL_Port, LED_YEL_Pin, led_level(y));
  HAL_GPIO_WritePin(LED_GRN_Port, LED_GRN_Pin, led_level(g));
}
```
This lets you swap wiring without touching the rest of the logic.

3) enter_state: one place to set outputs and next timeout
```c
static void enter_state(tl_state_t s) {
  g_state = s;
  g_tick100ms = 0;
  g_state_start_ms = HAL_GetTick();
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  switch (s) {
    case ST_RED:          set_lights(true,  false, false); g_target_ticks=T_RED_TICKS;  g_red_extended=false; break;
    case ST_RED_YEL:      set_lights(true,  true,  false); g_target_ticks=T_REDY_TICKS; break;
    case ST_GREEN:        set_lights(false, false, true ); g_target_ticks=T_GRN_TICKS;  break;
    case ST_GRN_YEL_PED:  set_lights(false, true,  true ); g_target_ticks=T_PED_GY_TICKS; g_ped_active=true; HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); break;
  }
}
```
All transitions go through here, which keeps the logic readable and consistent.

4) TIM3 interrupt: the heartbeat (every 100 ms)
- First, if we’re in GREEN and there’s a pending pedestrian request, immediately switch to the overlap state and return (so we don’t “double process” the same tick).
- Otherwise, increment the 100 ms tick counter and check if it’s time to transition.
- For the pedestrian overlap, use ms timing to exit exactly after 5 s.

5) Button (PD3) EXTI callback with debounce
```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == BTN_Pin) {
    uint32_t now = HAL_GetTick();
    if ((now - g_last_btn_ms) < 50) return; // debounce
    g_last_btn_ms = now;
    if (g_ped_active) return;               // ignore during overlap
    if (g_state == ST_GREEN)       g_ped_request = true;  // served in TIM3 ISR
    else if (g_state == ST_RED && !g_red_extended) { g_target_ticks += 100; g_red_extended = true; }
  }
}
```
This keeps the ISR tiny: just set flags and let the timer ISR run the FSM.


## Customization

- Change durations: edit the `T_*_TICKS` macros in `Src/main.c`.
- Change polarities: set `LED_ACTIVE_HIGH` and `BTN_ACTIVE_HIGH` to match your wiring.
- Disable the boot‑probe LED sweep: set `LED_BOOT_PROBE_MS` to `0`.
- Build‑time defines: you can also move these to `platformio.ini` under `build_flags` if you prefer (e.g., `-D LED_ACTIVE_HIGH=1`).


## Troubleshooting

- LEDs inverted or always on/off: check `LED_ACTIVE_HIGH` and wiring (series resistors, correct port/pin).
- Button not detected: confirm PD3 wiring matches polarity; if using active‑low, ensure the button connects to GND and no external pull fights the internal pull‑up.
- Timing feels off: confirm the board clock is stable and TIM3 settings match the code (they are configured for a 100 ms base tick).
- Nothing uploads: make sure the board is selected as `nucleo_f767zi` and ST‑Link drivers are installed.


## Project layout

- `Src/main.c` – Part 2 traffic light with EXTI
- `part1.c` – Part 1 (no button) reference
- `Inc/main.h` – Pins and peripheral handles
- `platformio.ini` – PlatformIO configuration

To try Part 1 instead of Part 2
- Option A (quick view): Open `part1.c` to read and compare the simpler FSM.
- Option B (build it): Replace `Src/main.c` with the content from `part1.c` (or rename files accordingly) and build/upload. The PlatformIO `src_dir` is `Src`, so whatever is compiled as `main.c` there becomes the firmware entry point.

The rest of the files are STM32Cube/HAL sources generated or provided for the MCU and board support.

---

Made for learning: simple, readable, and easy to extend (e.g., add pedestrian LED/do‑not‑walk icons, buzzer, or a UART debug console).
