/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
enum state {init, check_output, run, protect};
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define U_TARGET	1730
#define U_GIST		5
#define K_PROP		10

#define U_START		400
#define DUTY_START	50
#define START_MS	10

#define U_KZ		1000
#define TIME_KZ		3
#define TIMEOUT_PROTECT 3000

#define DEAD_TIME 20
#define PERIOD 250
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static int32_t duty = 0; //0...250
enum state powerState = init;
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
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
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

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
	 /* USER CODE BEGIN TIM14_IRQn 0 */
	LL_TIM_ClearFlag_UPDATE(TIM14);
	  /* USER CODE END TIM14_IRQn 0 */
	  /* USER CODE BEGIN TIM14_IRQn 1 */
	static uint32_t ADC_Result[5];
	static uint32_t ledCounter;
	static uint32_t startCounter = 0;
	static uint32_t kzCounter = 0;
	static uint32_t protectCounter = 0;
	static uint32_t ADC_Min[5] = {0x3FF,0x3FF,0x3FF,0x3FF,0x3FF};
	static uint32_t ADC_Max[5];

	ledCounter++;
	if (ledCounter == 2000)
	{
		ledCounter = 0;
		LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}

	LL_ADC_REG_StartConversion(ADC1);
	for (uint32_t i=0; i < 5; i++)
	{
		while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0) /* Wait end of conversion */
		{
			/* For robust implementation, add here time-out management */
		}
		ADC_Result[i] = ADC1->DR; /* Store the ADC conversion result */
	}
	LL_ADC_ClearFlag_EOS(ADC1);
	/*
	for (uint32_t i = 0; i < 5; i++)
	{
		if (ADC_Result[i] > ADC_Max[i])
		{
			ADC_Max[i] = ADC_Result[i];
		}

		if (ADC_Result[i] < ADC_Min[i])
		{
			ADC_Min[i] = ADC_Result[i];
		}
	}
	*/
	switch (powerState)
	{
		case init://Установим минимальный коэффициент заполнения на 1мс для проверки КЗ на выходе
		{
			duty = DEAD_TIME + DUTY_START;
			startCounter++;
			if (startCounter == START_MS)
			{
				powerState = check_output;
				startCounter = 0;
			}
			break;
		}
		//Подумать, может это состояние убрать
		case check_output://Проверяем есть ли напряжение на выходе
		{
			if (ADC_Result[1] > U_START)
			{
				powerState = run;
			}
			else
			{
				//отключаем Ш�?М
				duty = DEAD_TIME;
				//Зажигаем красный светодиод
				//LL_GPIO_SetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);

				powerState = protect;
			}
			break;
		}

		case run:
		{
			if (ADC_Result[1] > U_TARGET )
			{
				//duty -= (ADC_Result[0] - U_TARGET) / 32;
				//ограничение
				duty--;
				if (duty <DEAD_TIME ) duty = DEAD_TIME;
			}

			if (ADC_Result[1] < U_TARGET )
			{
				//duty += (U_TARGET - ADC_Result[0]) / 32;
				duty ++;
				if (duty > 250) duty = 250;
			}

			//if ((ADC_Result[0] < U_KZ) &&(duty == 250))
			if (ADC_Result[1] < U_KZ)
			{
				kzCounter++;
				if (kzCounter == TIME_KZ)
				{
					kzCounter = 0;
					//Вынести в отдельную функцию
					//отключаем Ш�?М
					duty = DEAD_TIME;
					//Зажигаем красный светодиод
					//LL_GPIO_SetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);

					powerState = protect;
				}
			}
			else
			{
				kzCounter = 0;
			}

			break;
		}

		case protect:
		{
			protectCounter++;
			if (protectCounter == TIMEOUT_PROTECT)
			{
				protectCounter = 0;
				//Тушим красный светодиод
				//LL_GPIO_ResetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);
				//Попробуем запуститься
				powerState = init;
			}
			break;
		}

		default: break;

	}



	//Обновляем значения в таймере

	LL_TIM_OC_SetCompareCH1(TIM3, duty - DEAD_TIME);
	LL_TIM_OC_SetCompareCH2(TIM3, 2*PERIOD - duty + DEAD_TIME);

	  /* USER CODE END TIM14_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
