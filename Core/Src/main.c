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
uint8_t sys_time_s = 0;
tactic_num tactic;
uint16_t main_fsm_case = 0;
uint32_t delay_1_start = 0xFFFFFFFF;

uint8_t points = 0;
uint16_t display_fsm_case = 0;
uint32_t display_delay = 0xFFFFFFFF;
char *tactic_name = "tactic";

/* test promenljive */
volatile uint8_t ax_id_test = 5;
volatile uint16_t ax_angle_test = 511;
volatile uint16_t ax_speed_test = 100;
int8_t dir_test = 1;
uint8_t in_0 = 0, in_1 = 0, in_2 = 0, in_3 = 0;
uint8_t out_0 = 0, out_1 = 0, out_2 = 0, out_3 = 0;
uint8_t sg90_1, sg90_2;

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

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
		{
			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
			sys_time_s = get_time_ms () / 1000;

			switch (main_fsm_case)
				{
				case 0:
					choose_tactic (&tactic);
					if (cinc_db () || 1)	// TODO: ISKLJUCEN JE CINC
						{
							start_match ();
//							display_fsm_case = 3;
							main_fsm_case = 100;
							// ovde se gasi brzinska petlja:	1 : upaljena,	0 : ugasena
							set_regulation_status (1);
						}
					break;

				case 1:
					mechanism_init ();
					if (delay_nb_2 (&delay_1_start, 500))
						{
							main_fsm_case = 2;
						}
					break;

				case 2:
					prepare_front ();
					if (delay_nb_2 (&delay_1_start, 500))
						{
							main_fsm_case = 3;
						}
					break;

				case 3:
					if (move_on_dir (200, FORWARD, 0.5) == TASK_SUCCESS)
						{
							main_fsm_case = 4;
						}
					break;

				case 4:
					if (move_on_dir (100, FORWARD, 0.2) == TASK_SUCCESS)
						{
							main_fsm_case = 5;
						}
					break;

				case 5:
					grtl_front_grip_all ();
					ruc_front_down ();
					if (delay_nb_2 (&delay_1_start, 500))
						{
							main_fsm_case = 6;
						}
					break;

				case 6:
					vacuum_front (1);
					if (delay_nb_2 (&delay_1_start, 500))
						{
							main_fsm_case = 7;
						}
					break;

				case 7:
					ruc_front_mid ();
					if (delay_nb_2 (&delay_1_start, 500))
						{
							main_fsm_case = 8;
						}
					break;

				case 8:
					lift_front_up ();
					if (delay_nb_2 (&delay_1_start, 2000))
						{
							main_fsm_case = 99;
						}
					break;

				case 9:
					if (move_on_dir (300, BACKWARD, 0.1) == TASK_SUCCESS)
						{
							main_fsm_case = 10;
						}
					break;

				case 100:
					grtl_front_grip_outside ();
					gurl_mid ();
					ruc_front_down ();
					if (delay_nb_2 (&delay_1_start, 500))
						{
							main_fsm_case = 101;
						}
					break;

				case 101:
					vacuum_front (1);
					if (delay_nb_2 (&delay_1_start, 500))
						{
							main_fsm_case = 102;
						}
					break;

				case 102:
					ruc_front_mid ();
					if (delay_nb_2 (&delay_1_start, 500))
						{
							main_fsm_case = 103;
						}
					break;

				case 103:
					gurl_front ();
					if (move_on_dir (70, BACKWARD, 0.2) == TASK_SUCCESS)
						{
							main_fsm_case = 104;
						}
					break;

				case 104:
					gurl_mid ();
					if (move_on_dir (130, BACKWARD, 0.5) == TASK_SUCCESS)
						{
							main_fsm_case = 105;
						}
					break;

				case 105:
					ruc_front_down ();
					if (delay_nb_2 (&delay_1_start, 100))
						{
							main_fsm_case = 106;
						}
					break;

				case 106:
					vacuum_front (0);
					if (delay_nb_2 (&delay_1_start, 100))
						{
							main_fsm_case = 107;
						}
					break;

				case 107:
					ruc_front_up ();
					if (delay_nb_2 (&delay_1_start, 1000))
						{
							main_fsm_case = 108;
						}
					break;

				case 108:
					lift_front_up ();
					if (delay_nb_2 (&delay_1_start, 1000))
						{
							main_fsm_case = 109;
						}
					break;

				case 109:
					if (move_on_dir (200, FORWARD, 0.5) == TASK_SUCCESS)
						{
							main_fsm_case = 110;
						}
					break;

				case 110:
					lift_front_drop ();
					if (delay_nb_2 (&delay_1_start, 100))
						{
							main_fsm_case = 111;
						}
					break;

				case 111:
					grtl_front_open ();
					if (delay_nb_2 (&delay_1_start, 100))
						{
							main_fsm_case = 112;
						}
					break;

				case 112:
					if (move_on_dir (200, BACKWARD, 0.5) == TASK_SUCCESS)
						{
							main_fsm_case = 99;
						}
					break;

				case 12:
					if (move_on_path (200, -1000, 180, 1, 0, 0.3, 0) == TASK_SUCCESS)
						{
							main_fsm_case = 99;
						}
//		  if (delay_nb_2 (&delay_1_start, 1000))
//			{
//			}
					break;

					//		  ax_move (ax_id_test, ax_angle_test, ax_speed_test, huart6);	// PA11
				}

			/* Display FSM */
			switch (display_fsm_case)
				{

				/* Inicijalizacija displeja */
				case 0:
					if (HD44780_Init (2))
						display_fsm_case = 1;
					break;

				/* Ispis pre cinca */
				case 1:
					HD44780_NoBacklight ();
					HD44780_Clear ();
					HD44780_SetCursor (0, 0);
					HD44780_PrintStr (" +381  Robotics ");
					HD44780_SetCursor (3, 1);
					HD44780_PrintStr (tactic_name);
					HD44780_Backlight ();
					display_fsm_case = 2;
					break;

					/* Cekanje pocetka */
				case 2:
					break;

					/* Ispis celog displeja */
				case 3:
					if (delay_nb_2 (&display_delay, 1000))
						display_write_all (points, sys_time_s, tactic_name);
					break;

					/* Ispisivanje samo brojeva svake sekunde */
				case 4:
					if (delay_nb_2 (&display_delay, 1000))
						display_write_numbers (points, sys_time_s);
					break;

					/* Isteklo vreme, nista vise ne ispisuj */
				case 5:
					break;
				}
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
