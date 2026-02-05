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
/* USER CODE END Includes */

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
  * Fail-safe: drive MT_EN/IP_EN low (RPi off, charger off), PWR_EN high,
  * then request system reset. Direct register writes only; no HAL/LL calls.
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

  /* PA5/PA6/PA7: output, push-pull, low speed, no pull. */
  GPIOA->MODER   = (GPIOA->MODER   & ~(0x3FU << 10)) | (0x15U << 10);
  GPIOA->OTYPER  = GPIOA->OTYPER  & ~0xE0U;
  GPIOA->OSPEEDR = GPIOA->OSPEEDR & ~(0x3FU << 10);
  GPIOA->PUPDR   = GPIOA->PUPDR   & ~(0x3FU << 10);

  /* Safe outputs: MT_EN LOW, IP_EN LOW, PWR_EN HIGH. BSRR: bits 0–15 set (high), 16–31 reset (low). */
  GPIOA->BSRR = (1u << 21) | (1u << 22) | (1u << 7);

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

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
