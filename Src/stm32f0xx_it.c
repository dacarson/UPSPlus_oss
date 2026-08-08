/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx.h"
#include "ups_state.h"
/* USER CODE END Includes */
#include "stm32f0xx_ll_dma.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  * Fail-safe: force a full IP5328 reset (KEY/IP_EN held LOW >= 10s per datasheet Section 6,
  * reusing the Section 9.1 mechanism -- see UPSPlus_Behavior_Spec.md Sections 9.1/10), keeping
  * PWR_EN HIGH throughout (MCU hold-up). MT_EN is left unchanged to avoid unnecessarily
  * power-cycling the Pi. IP_EN is released HIGH again before requesting a system reset.
  * Direct register writes/reads only; no HAL/LL calls. This bypasses the main-loop IP_EN
  * arbitration (Section 9.3) entirely -- it runs in a dedicated fault context and resets the MCU
  * immediately afterward, so it can never meaningfully overlap with the periodic-reset or
  * LED-check drivers there.
  */
void HardFault_Handler(void)
{
  static volatile uint8_t in_handler;
  if (in_handler) { for (;;) { __NOP(); } }
  in_handler = 1;

  /* Enable GPIOA clock (direct RCC; pins may be uninitialized). */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  __DSB();
  __ISB();

  /* PA5 (IP_EN) and PA7 (PWR_EN): output, push-pull, low speed, no pull. Leave PA6 (MT_EN) unchanged. */
  GPIOA->MODER   = (GPIOA->MODER   & ~((0x3U << 10) | (0x3U << 14))) | (0x1U << 10) | (0x1U << 14);
  GPIOA->OTYPER  = GPIOA->OTYPER  & ~((1U << 5) | (1U << 7));
  GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~((0x3U << 10) | (0x3U << 14)));
  GPIOA->PUPDR   = (GPIOA->PUPDR   & ~((0x3U << 10) | (0x3U << 14)));

  /* IP_EN LOW, PWR_EN HIGH. Do not drive MT_EN (leave unchanged). BSRR: 0–15 set, 16–31 reset. */
  GPIOA->BSRR = (1u << 21) | (1u << 7);

  /* Enable TIM3 directly and force it into the same free-running 1 MHz configuration
   * MX_TIM3_Init() uses, regardless of whether normal init already ran (this handler must not
   * assume any prior state). Reconfiguring/resetting the counter here is harmless even if TIM3
   * was already running -- only the elapsed time within this handler matters. A hardware timer
   * keeps counting during this handler even though it is the highest-priority exception (SysTick
   * cannot preempt it), so it gives a reliable elapsed-time source without needing interrupts. */
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->CR1 &= ~TIM_CR1_CEN;
  TIM3->PSC = 48u - 1u;   /* 48 MHz APB1 (per SystemClock_Config) / 48 = 1 MHz, i.e. 1 us/tick */
  TIM3->ARR = 0xFFFFu;
  TIM3->CNT = 0u;
  TIM3->EGR = TIM_EGR_UG;  /* Force PSC/ARR reload into shadow registers before starting */
  TIM3->CR1 |= TIM_CR1_CEN;

  /* Hold IP_EN LOW for IP5328_RESET_HOLD_MS (10.5 s: datasheet's 10 s full-reset minimum + 500 ms
   * margin -- same constant Section 9.1 uses). Busy-wait in <=50 ms chunks (well inside TIM3's
   * 65.536 ms wrap window) and feed the ~8 s IWDG every chunk so it cannot fire mid-hold and cut
   * the pulse short before the chip's reset threshold is reached. */
  {
      uint32_t remaining_ms = IP5328_RESET_HOLD_MS;
      while (remaining_ms > 0u)
      {
          uint32_t chunk_ms = (remaining_ms > 50u) ? 50u : remaining_ms;
          uint16_t start_us = (uint16_t)TIM3->CNT;
          uint16_t target_us = (uint16_t)(chunk_ms * 1000u);
          while ((uint16_t)((uint16_t)TIM3->CNT - start_us) < target_us) { }
          remaining_ms -= chunk_ms;
          IWDG->KR = 0xAAAAu;  /* Reload key: feed the watchdog */
      }
  }

  /* Release IP_EN back HIGH (its normal idle state) before resetting, mirroring the Section 9.1
   * sequence. BSRR bit 5 = PA5 set. */
  GPIOA->BSRR = (1u << 5);

  /* Request system reset; then NOP forever if reset is delayed. */
  SCB->AIRCR = (0x5FAu << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
  for (;;) { __NOP(); }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
 void SysTick_Handler(void)
 {
   /* USER CODE BEGIN SysTick_IRQn 0 */
   /* SysTick now fires every 10 ms directly (see SystemClock_Config in main.c); no divider needed. */
   Scheduler_ISR_Tick10ms();
   /* USER CODE END SysTick_IRQn 0 */
 
   /* USER CODE BEGIN SysTick_IRQn 1 */
 
   /* USER CODE END SysTick_IRQn 1 */
 }

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/**
 * @brief DMA1 Channel1 interrupt handler — fires on ADC DMA transfer complete.
 * DMA runs NORMAL (one-shot), re-armed each cycle in the main loop, so it's
 * idle between bursts and aADCxConvertedData is stable for the main loop to
 * read directly once this fires.
 * Sets adc_ready; main loop processes ADC buffer and updates state.
 * Defined here (not main.c) so the strong symbol survives LTO and correctly
 * overrides the weak assembly alias in the vector table.
 */
void DMA1_CH1_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC1(DMA1))
    {
        LL_DMA_ClearFlag_TC1(DMA1);
        adc_ready = 1;
    }
    if (LL_DMA_IsActiveFlag_TE1(DMA1))
    {
        LL_DMA_ClearFlag_TE1(DMA1);
    }
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
