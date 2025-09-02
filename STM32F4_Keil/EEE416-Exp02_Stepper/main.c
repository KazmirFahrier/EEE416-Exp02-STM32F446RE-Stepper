
/**
  ******************************************************************************
  * @file    main.c
  * @author  ChatGPT
  * @brief   EEE 416 – Experiment 02 (STM32F446RE, CMSIS/Keil)
  *          - LED on PA5
  *          - Pushbutton on PC13 (active-LOW, pull-up)
  *          - 28BYJ-48 stepper via ULN2003 on PC5, PC6, PC8, PC9
  *          Behavior:
  *            - Button released: LED OFF, Full-step CCW rotation
  *            - Button pressed : LED blinks, Half-step CW rotation
  *
  * NOTE: This project uses CMSIS register-level code (no HAL). It is adapted
  *       from the provided lab template for STM32F446RE (Nucleo).
  ******************************************************************************
  */

#include "stm32f4xx.h"

/* === Tick timer for millisecond delays === */
static volatile uint32_t g_ms = 0;

void SysTick_Handler(void) { g_ms++; }

static void delay_ms(uint32_t ms) {
  uint32_t start = g_ms;
  while ((g_ms - start) < ms) { __NOP(); }
}

/* === GPIO helpers === */
static inline void gpio_enable_clk(void) {
  /* Enable clocks to GPIOA and GPIOC */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN);
  __DSB(); /* Ensure the write completes before further config */
}

static inline void gpio_config_outputs(void) {
  /* PA5: LED, output */
  GPIOA->MODER  &= ~(3u << (5*2));
  GPIOA->MODER  |=  (1u << (5*2));         /* 01: General purpose output */
  GPIOA->OTYPER &= ~(1u << 5);             /* Push-pull */
  GPIOA->OSPEEDR |= (2u << (5*2));         /* High speed */
  GPIOA->PUPDR  &= ~(3u << (5*2));         /* No pull */

  /* PC5, PC6, PC8, PC9: stepper coils -> outputs */
  const uint8_t out_pins[] = {5, 6, 8, 9};
  for (unsigned i = 0; i < 4; i++) {
    uint8_t p = out_pins[i];
    GPIOC->MODER   &= ~(3u << (p*2));
    GPIOC->MODER   |=  (1u << (p*2));
    GPIOC->OTYPER  &= ~(1u << p);
    GPIOC->OSPEEDR |=  (2u << (p*2));
    GPIOC->PUPDR   &= ~(3u << (p*2));
  }
}

static inline void gpio_config_button(void) {
  /* PC13: input with pull-up (user button) */
  GPIOC->MODER  &= ~(3u << (13*2));        /* 00: Input */
  GPIOC->PUPDR  &= ~(3u << (13*2));
  GPIOC->PUPDR  |=  (1u << (13*2));        /* 01: Pull-up */
}

/* LED control on PA5 via BSRR */
static inline void led_on(void)  { GPIOA->BSRR = (1u << 5); }
static inline void led_off(void) { GPIOA->BSRR = (1u << (5 + 16)); }
static inline void led_toggle(void) {
  if (GPIOA->ODR & (1u << 5)) led_off(); else led_on();
}

static inline uint8_t button_pressed(void) {
  /* Active-LOW on PC13 */
  return (GPIOC->IDR & (1u << 13)) == 0;
}

/* === Stepper driver ===
   Map 4-bit mask (A,B,C,D) -> (PC5, PC6, PC8, PC9)
*/
static inline void stepper_write_mask(uint8_t mask) {
  uint32_t set_mask = 0;
  if (mask & 0x8) set_mask |= (1u << 5);   /* A -> PC5 */
  if (mask & 0x4) set_mask |= (1u << 6);   /* B -> PC6 */
  if (mask & 0x2) set_mask |= (1u << 8);   /* C -> PC8 */
  if (mask & 0x1) set_mask |= (1u << 9);   /* D -> PC9 */

  uint32_t reset_mask = ((1u << 5) | (1u << 6) | (1u << 8) | (1u << 9)) & (~set_mask);
  GPIOC->BSRR = (set_mask) | (reset_mask << 16);
}

/* Full-step (A, B, C, D) */
static const uint8_t FULL_SEQ[4] = { 0x8, 0x4, 0x2, 0x1 };

/* Half-step (A, AB, B, BC, C, CD, D, DA) */
static const uint8_t HALF_SEQ[8] = { 0x8, 0xC, 0x4, 0x6, 0x2, 0x3, 0x1, 0x9 };

/* Direction: +1 (forward/CW), -1 (reverse/CCW) */
static void step_sequence(const uint8_t *seq, int len, int dir, uint32_t step_delay_ms) {
  static int idx_full = 0;
  static int idx_half = 0;

  int *pidx = (len == 4) ? &idx_full : &idx_half;
  *pidx = (*pidx + (dir > 0 ? 1 : -1) + len) % len;

  stepper_write_mask(seq[*pidx]);
  delay_ms(step_delay_ms);
}

int main(void) {
  /* System clock is configured by system_stm32f4xx.c.
     Configure SysTick to 1ms ticks. */
  SystemInit();
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000U);

  gpio_enable_clk();
  gpio_config_outputs();
  gpio_config_button();

  /* Idle all coils off */
  stepper_write_mask(0x0);
  led_off();

  const uint32_t HALF_DELAY_MS = 3;  /* tune as needed */
  const uint32_t FULL_DELAY_MS = 3;

  while (1) {
    if (button_pressed()) {
      /* Button pressed: LED blink + half-step CW */
      led_toggle();
      step_sequence(HALF_SEQ, 8, +1, HALF_DELAY_MS);
    } else {
      /* Button released: LED off + full-step CCW */
      led_off();
      step_sequence(FULL_SEQ, 4, -1, FULL_DELAY_MS);
    }
  }
}
