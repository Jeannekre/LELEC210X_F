/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "arm_math.h"
#include "usart.h"
#include "utils.h"
#include "s2lp.h"
#include "acquisition.h"
#include "computation.h"
#include "transfer.h"
#include "eval_radio.h"
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
// From acquisition
extern volatile uint16_t *samples_buf_to_process;
extern volatile uint8_t is_processing;
// From computation
extern q15_t mel_vectors[N_MELVECS][MELVEC_LENGTH];
extern volatile uint8_t cur_melvec;
// From transfer
extern uint8_t packet[PACKET_LENGTH];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ===========================================================================
 * P2a TIMING TEST
 *   P2A_TIMING_TEST 1 : the blue button runs the P2a test (10 000 samples
 *                       acquired by polling + raw print, both timed)
 *   P2A_TIMING_TEST 0 : normal application (acquisition_start(), DMA, ...)
 *
 *   USE_TIMER_TRIGGER 0 : software trigger, sampling freq. NOT controlled
 *   USE_TIMER_TRIGGER 1 : ADC triggered by TIM3
 * ========================================================================= */
#define P2A_TIMING_TEST    1
#define USE_TIMER_TRIGGER  0

#if P2A_TIMING_TEST
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

#define SAMPLE_COUNT 10000
// static: 20 kB buffer kept out of the (small) stack of the interrupt
static uint16_t test_samples[SAMPLE_COUNT];

/* Enables the DWT cycle counter (counts CPU clock cycles, works in interrupts) */
static void cycle_counter_init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Converts CPU cycles to microseconds */
static uint32_t cycles_to_us(uint32_t cycles)
{
	return (uint32_t)(((uint64_t)cycles * 1000000ULL) / SystemCoreClock);
}

static void p2a_timing_test(void)
{
	uint32_t t_start, t_acq_cycles, t_print_cycles;

	cycle_counter_init();

	// Select the ADC trigger source for this test
#if USE_TIMER_TRIGGER
	hadc1.Init.ExternalTrigConv     = ADC_EXTERNALTRIG_T3_TRGO;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
#else
	hadc1.Init.ExternalTrigConv     = ADC_SOFTWARE_START;
#endif
	HAL_ADC_Init(&hadc1);

	// LED ON during acquisition
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	/* ===================== ACQUISITION (timed) ===================== */
	t_start = DWT->CYCCNT;
#if USE_TIMER_TRIGGER
	HAL_ADC_Start(&hadc1);
	HAL_TIM_Base_Start(&htim3);
	for (int i = 0; i < SAMPLE_COUNT; i++) {
		HAL_ADC_PollForConversion(&hadc1, 100);
		test_samples[i] = HAL_ADC_GetValue(&hadc1);
	}
	HAL_TIM_Base_Stop(&htim3);
	HAL_ADC_Stop(&hadc1);
#else
	for (int i = 0; i < SAMPLE_COUNT; i++) {
		HAL_ADC_Start(&hadc1);                       // software trigger
		HAL_ADC_PollForConversion(&hadc1, 100);
		test_samples[i] = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
#endif
	t_acq_cycles = DWT->CYCCNT - t_start;
	/* =============================================================== */

	// LED OFF once acquisition is finished, BEFORE printing
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/* ===================== UART PRINT (timed) ====================== */
	t_start = DWT->CYCCNT;
	print_raw_samples(test_samples, SAMPLE_COUNT);
	t_print_cycles = DWT->CYCCNT - t_start;
	/* =============================================================== */

	// Results printed AFTER the measurements (printf is slow)
	uint32_t t_acq_us   = cycles_to_us(t_acq_cycles);
	uint32_t t_print_us = cycles_to_us(t_print_cycles);
	uint32_t fs_hz      = (t_acq_us > 0) ?
			(uint32_t)(((uint64_t)SAMPLE_COUNT * 1000000ULL) / t_acq_us) : 0;

	DEBUG_PRINT("\r\n--- Timing results (%s trigger) ---\r\n",
			USE_TIMER_TRIGGER ? "TIM3" : "software");
	DEBUG_PRINT("CPU clock          : %lu Hz\r\n", (unsigned long)SystemCoreClock);
	DEBUG_PRINT("Acquisition time   : %lu us (%lu cycles)\r\n",
			(unsigned long)t_acq_us, (unsigned long)t_acq_cycles);
	DEBUG_PRINT("Time per sample    : %lu ns\r\n",
			(unsigned long)((uint64_t)t_acq_us * 1000ULL / SAMPLE_COUNT));
	DEBUG_PRINT("Mean sampling freq : %lu Hz\r\n", (unsigned long)fs_hz);
	DEBUG_PRINT("UART print time    : %lu ms\r\n", (unsigned long)(t_print_us / 1000));
}
#endif // P2A_TIMING_TEST

/*
 * @brief Manages external interrupts : user button and S2LP radio.
 * @note This overrides the __weak__ function in stm32l4xx_hal_gpio.c to provide an user implementation.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin) { // User button
#if P2A_TIMING_TEST
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); // only one execution per button press
		p2a_timing_test();
		__HAL_GPIO_EXTI_CLEAR_FLAG(B1_Pin);
		HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
#else
		if (acquisition_start()) {
			printf("Acquisition not started, already running\r\n");
		} else {
			printf("Acquisition started\r\n");
		}
#endif
	}
	else if (GPIO_Pin == RADIO_INT_Pin) { // S2LP radio, will be used in P2d
		S2LP_IRQ_Handler();
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
#if DEBUGP
	MX_LPUART1_UART_Init();
	DEBUG_PRINT("Hello :)\r\n");
#endif
#if ENABLE_RADIO
  HAL_StatusTypeDef err = S2LP_Init(&hspi1);
  if (err)  {
	  DEBUG_PRINT("[S2LP] Error while initializing: %u\r\n", err);
	  Error_Handler();
  } else {
	  DEBUG_PRINT("[S2LP] Init OK\r\n");
  }
#endif
#if (RUN_CONFIG == EVAL_RADIO)
  eval_radio_start();
#else // MAIN_APP
  DEBUG_PRINT("Press the blue button to start acquisition.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (is_processing) {
			HAL_GPIO_TogglePin(GPOUT_GPIO_Port, GPOUT_Pin); // for oscillo timing measurements
			melvec_compute((q15_t *)samples_buf_to_process , mel_vectors[cur_melvec]);
			HAL_GPIO_TogglePin(GPOUT_GPIO_Port, GPOUT_Pin); // for oscillo timing measurements
			is_processing = 0;
			cur_melvec++;
			if (cur_melvec == N_MELVECS) {
				cur_melvec = 0;
				acquisition_stop();
				make_packet();
        send_packet();
#if CONTINUOUS_ACQ
				acquisition_start();
#endif // CONTINUOUS_ACQ
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
#endif // RUN_CONFIG == EVAL_RADIO
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();
	DEBUG_PRINT("Entering error Handler\r\n");
	while (1)
	{
		// Blink LED3 (red)
		HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
		for (volatile int i=0; i < SystemCoreClock/200; i++);
		HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
		for (volatile int i=0; i < SystemCoreClock/200; i++);
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */