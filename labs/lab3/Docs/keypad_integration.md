# Keypad Integration Summary

## 1. CubeMX / Hardware Configuration (`.ioc`)

* **Pin `PF4` (`GPIO_EXTI4_KPAD_IRQ`)**: Set to **External Interrupt Mode with Falling edge trigger detection**, **No pull-up and no pull-down**.
* **NVIC Interrupt Priorities**:
  * `EXTI line4 interrupt` (`EXTI4_IRQn`): Enabled, Preemption Priority = **2**
  * `TIM6 global interrupt` (`TIM6_DAC_IRQn`): Preemption Priority = **1** (higher priority than keypad EXTI so I2C reads never delay the 100 Hz motor control loop).

---

## 2. Added / Modified Files

* **Added from Lab 1**:
  * `Core/Inc/SX1509_Registers.h`
  * `Core/Inc/SX1509_Utils.h`
  * `Core/Src/SX1509_Utils.c` (commented out `printf` debug lines to avoid SWO/UART overhead).
* **Modified**:
  * `Core/Src/main.c` (included SX1509 headers, called `Init_SX1509()` in `main()`, added keypad variables and callbacks).
  * `Core/Inc/main.h` & `Core/Src/stm32f7xx_it.c` (auto-generated `EXTI4_IRQHandler` via CubeMX).

---

## 3. Functions in `main.c`

* **`HAL_GPIO_EXTI_Callback()`**: Copied verbatim from Lab 1; triggered on `PF4` falling edge, reads row/column registers from the SX1509 over `I2C1`, decodes the pressed key, and calls `updateSpeedReference(key)`.
* **`updateSpeedReference(char key)`**: Custom logic adapted from Lab 1 to update `w_target_1` and `w_target_2`.

---

## 4. Keypad Control Logic

* **Boot / Default**: Both motors start at `W_DEFAULT` (`6.0 rad/s`) once MATLAB datalogging (`logger.tx_enable`) starts.
* **Motor Selection**:
  * `A`: Select Motor 1 only
  * `B`: Select Motor 2 only
  * `C`: Select Both Motors (default)
  * `D`: Immediate emergency stop (`0.0 rad/s` on both motors)
* **Speed Input**:
  * `0`–`9`: Type target angular speed in `rad/s` (clamped to `±16.0 rad/s`).
  * `*`: Sets negative sign `-` if pressed before typing digits; clears/resets current input if pressed while already typing.
  * `#`: Confirms and applies the typed speed to the selected motor(s).
