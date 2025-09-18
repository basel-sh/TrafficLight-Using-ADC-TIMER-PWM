/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Traffic Light + LCD + ADC4bit + Timer + Interrupt Priorities
  *
  * Pins used (adjust if you used different mapping):
  *   PC13 -> RED LED
  *   PC14 -> YELLOW LED
  *   PC15 -> GREEN LED
  *   PA0  -> RUN/STOP Button (EXTI0 - HIGH priority)
  *   PA2  -> EMERGENCY Button (EXTI2 - LOW priority)
  *   PA1  -> ADC1_IN1 (POTENTIOMETER)  (12-bit ADC)
  *   PB0..PB5 -> LCD (your lcd.c should already use PB0..PB5)
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* ADC raw value from DMA (12-bit) */
volatile uint32_t adc_raw = 0;

/* Processed and shared variables (updated in ADC callback or TIM tick) */
volatile float adc_voltage = 0.0f;   /* volts */
volatile uint8_t adc_4bit = 0;       /* 0..15 */

volatile uint8_t traffic_on = 1;     /* 1 = running, 0 = stopped */
volatile uint8_t emergency_flag = 0; /* 1 = emergency */
volatile uint8_t state = 0;          /* 0=RED,1=YELLOW,2=GREEN */

/* motor duty placeholder (apply as PWM duty if you add a PWM channel) */
volatile uint8_t motor_duty_percent = 0;

/* timing and debounce */
#define EXTI_DEBOUNCE_MS 200
volatile uint32_t last_exti0 = 0;
volatile uint32_t last_exti2 = 0;

volatile uint8_t lcd_update_needed = 0;

/* LCD buffer */
char lcd_line2[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Update 4-bit LED outputs (PB4..PB7) to show bits b3..b0 */
static void update_adc_leds(uint8_t value4)
{
  // value4 is 0..15
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (value4 & 0x8) ? GPIO_PIN_SET : GPIO_PIN_RESET); // bit3 -> PB12
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (value4 & 0x4) ? GPIO_PIN_SET : GPIO_PIN_RESET); // bit2 -> PB13
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (value4 & 0x2) ? GPIO_PIN_SET : GPIO_PIN_RESET); // bit1 -> PB14
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (value4 & 0x1) ? GPIO_PIN_SET : GPIO_PIN_RESET); // bit0 -> PB15

}

/* Refresh LCD lines based on current mode and ADC */
static void show_status_on_lcd(void)
{
    static uint8_t last_state = 0xFF;
    static uint8_t last_traffic_on = 0xFF;
    static uint8_t last_emergency_flag = 0xFF;

    char line[20];

    // Always update Line 2 (ADC values), only refresh Line 1 when state changes
    if(state != last_state || traffic_on != last_traffic_on || emergency_flag != last_emergency_flag)
    {
        last_state = state;
        last_traffic_on = traffic_on;
        last_emergency_flag = emergency_flag;

        LCD_Clear();

        if(!traffic_on)
        {
            LCD_SetCursor(0,0);
            LCD_Print("Traffic: STOPPED ");
            // Line 2 will still show ADC
        }
        else if(emergency_flag)
        {
            LCD_SetCursor(0,0);
            LCD_Print("EMERGENCY");
        }
        else
        {
            LCD_SetCursor(0,0);
            if(state == 0) LCD_Print("Red Light");
            else if(state == 1) LCD_Print("Yellow Light");
            else if(state == 2) LCD_Print("Green Light");
        }
    }

    LCD_SetCursor(1,0);
    snprintf(line, sizeof(line), "V=%.2fV  B=%2u", adc_voltage, adc_4bit);
    LCD_Print(line);
    update_adc_leds(adc_4bit);

}

// Call this whenever you want to set motor speed 0..100%
void set_motor_speed(uint8_t speed_percent)
{
    if(speed_percent > 100) speed_percent = 100;  // clamp max
    motor_duty_percent = speed_percent;

    // Apply to PWM channel (TIM3_CH1)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, motor_duty_percent * 10);
}


/* EXTI callbacks (handled by HAL IRQ -> calls HAL_GPIO_EXTI_Callback) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t t = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_0) { /* RUN/STOP: EXTI0 (HIGH priority) */
        if ((t - last_exti0) > EXTI_DEBOUNCE_MS) {
            last_exti0 = t;
            traffic_on = !traffic_on;

            if (!traffic_on) {
                /* Traffic OFF -> turn all lights OFF */
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
                motor_duty_percent = 0;
            } else {
                /* Traffic ON -> restore normal duty */
                motor_duty_percent = 50;
            }

            lcd_update_needed = 1; // update LCD immediately
        }
    }
    else if (GPIO_Pin == GPIO_PIN_2) { /* EMERGENCY: EXTI2 (LOW priority) */
        if ((t - last_exti2) > EXTI_DEBOUNCE_MS) {
            last_exti2 = t;

            if (!traffic_on) {
                /* Traffic is OFF -> ignore emergency */
                return;
            }

            emergency_flag = !emergency_flag;

            if (emergency_flag) {
                /* Force RED blinking */
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                state = 0;
                motor_duty_percent = 75;
            } else {
                /* back to normal traffic */
                motor_duty_percent = 50;
            }

            lcd_update_needed = 1; // update LCD immediately
        }
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
  // User variables or early initialization can go here
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Any low-level init code before system clock config can go here
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Additional system-level configurations can go here
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */

  LCD_Init();                          // Initialize LCD
  HAL_TIM_Base_Start_IT(&htim2);       // Start TIM2 with interrupt enabled
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  set_motor_speed(0);   // normal speed
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Update LCD only when needed
    if(lcd_update_needed)
    {
        show_status_on_lcd();
        lcd_update_needed = 0;

    }
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 250-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12
                           PB13 PB14 PB15 PB3
                           PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        static uint8_t color = 0;      // Normal traffic: 0=RED,1=YELLOW,2=GREEN
        static uint8_t half_sec = 0;   // Counts 0..7 for 0.5s ON/OFF cycles

        // Turn all traffic lights OFF first
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

        if(!traffic_on)
        {
            // Traffic OFF -> keep all LEDs off
        }
        else if(emergency_flag)
        {
            // Emergency active -> RED blinking
            if(half_sec % 2 == 0)
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        }
        else
        {
            // Normal traffic cycle
            if(half_sec % 2 == 0)
            {
                switch(color)
                {
                    case 0: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); break; // RED
                    case 1: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); break; // YELLOW
                    case 2: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); break; // GREEN
                }
            }

            // Advance normal traffic color every 4 half_sec cycles
            if(half_sec >= 7)
                color = (color + 1) % 3;
        }

        // Update half_sec counter
        half_sec = (half_sec + 1) % 8;

        // Update global state for LCD
        state = (emergency_flag ? 0 : color);

        // ---- ADC Polling ----
        HAL_ADC_Start(&hadc1);                                   // Start conversion
           if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)     // Wait (timeout 10ms)
           {
               adc_raw = HAL_ADC_GetValue(&hadc1);                  // 12-bit result (0..4095)
               adc_voltage = (adc_raw * 3.3f) / 4095.0f;            // Convert to volts
               adc_4bit = adc_raw >> 8;                             // Scale to 4-bit (0..15)
           }
           HAL_ADC_Stop(&hadc1);


        // Signal main loop to update LCD
        lcd_update_needed = 1;


    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
