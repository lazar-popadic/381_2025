/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static tactic_num *tactic_ptr;
int8_t main_fsm_case = 0;
uint32_t delay_1_start = 0xFFFFFFFF;
uint8_t flag_end = 0;

/* test promenljive */
volatile uint8_t ax_id_test = 10;
volatile uint16_t ax_angle_test = 511;
volatile uint16_t ax_speed_test = 200;
volatile uint16_t out_0, out_1, out_2, out_3;
volatile uint8_t in_0, in_1;
extern int16_t ax_1_offs;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void
SystemClock_Config (void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int
main (void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init ();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config ();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init ();
	MX_DMA_Init ();
	MX_TIM4_Init ();
	MX_TIM10_Init ();
	MX_TIM5_Init ();
	MX_TIM3_Init ();
	MX_USART6_UART_Init ();
	MX_TIM2_Init ();
	MX_USART1_UART_Init ();
	MX_I2C3_Init ();
	/* USER CODE BEGIN 2 */
	pwm_start ();
	time_start ();
	base_init ();
	odometry_init ();
	regulation_init ();
	rpi_init ();
	sg90_init ();
	ax_init ();
	tactic_ptr = get_tact_num_ptr ();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
		{
			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
			switch (main_fsm_case)
				{
				case 0:
					mechanism_init ();
					set_regulation_status (0);
					main_fsm_case = 1;
					break;

				case 1:
					choose_tactic (tactic_ptr);
					if (cinc_db ())
						{
							start_match ();
							display_ready ();
							main_fsm_case = tactic_ptr->num + 10;
							set_regulation_status (1);
						}
					break;

				case 10:
					if (tact_0 () == TASK_SUCCESS)
						main_fsm_case = -1;
					break;

				case 11:
					if (tact_1 () == TASK_SUCCESS)
						main_fsm_case = -1;
					break;

					//				case 13:
					//					if (tact_homologation () == TASK_SUCCESS)
					//						main_fsm_case = -1;
					//					break;

				case 12:
					if (tact_2 () == TASK_SUCCESS)
						main_fsm_case = -1;
					break;

				case 13:
					if (tact_dev_3 () == TASK_SUCCESS)
						main_fsm_case = -1;
//			tact_3 ();
					break;

				case 20:
					ax_move (ax_id_test, ax_angle_test, ax_speed_test, huart6);
					ax_move (1, ax_angle_test + ax_1_offs, ax_speed_test, huart6);
					vacuum_0 (out_0);
					vacuum_1 (out_1);
					vacuum_2 (out_2);
					vacuum_3 (out_3);
					break;

				case 21:
					sg90_1_move (out_0);
					sg90_2_move (out_1);
					sg90_3_move (out_2);
					sg90_4_move (out_3);
					break;

					// Go to HOME
				case -10:
					if (move_to_xy (x_side (-800), 500, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD))
						main_fsm_case = -1;
					break;

				case -1:
					if (!flag_end)
						{
							flag_end = 1;
							if (fabs (get_robot_base ()->x) > 750 && fabs (get_robot_base ()->y) > 450)
								add_points (10);
						}
					set_regulation_status (0);
					stop_match ();
					HAL_Delay (10);
					time_stop ();
					break;
				}

			display_fsm ();
		}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void
SystemClock_Config (void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
		{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
		{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK)
		{
			Error_Handler ();
		}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
		{
			Error_Handler ();
		}
}

/* USER CODE BEGIN 4 */

int
_write (int le, char *ptr, int len)
{
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++)
		{
			ITM_SendChar (*ptr++);
		}
	return len;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void
Error_Handler (void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq ();
	while (1)
		{
		}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
