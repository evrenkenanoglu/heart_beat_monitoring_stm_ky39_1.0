/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "oled_ssd1306.h"
#include "stdio.h"
#include "string.h"

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
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void writeScreenCenter(uint8_t yPos, char* str, FontDef font);
void updateScreen(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RESOLUTION_10                  (1024)
#define RESOLUTION_12                  (4096)
#define ONE_MIN_IN_MS                  60000                            // ms
#define REF_VOLTAGE_UV                 3300000                          // uV
#define MIN_STEP_SIZE                  (REF_VOLTAGE_UV / RESOLUTION_12) //** 732.42 uVolt
#define FINGER_DETECTION_THRESHOLD_RAW (RESOLUTION_10 / 2)
#define RESULT_PERIOD                  10
#define HB_MIN_CHANGE                  ()
#define HB_MAX_CHANGE                  ()

#define AMPLIFIER_COEFF                (1) // (4096/RESULT_PERIOD)
#define MAX_VALUE_COEFF                (1)
#define MAX_VALUE_NEW_BASE_COEFF       (0.95)

#define ADC_SAMPLE_NO                  32
uint32_t adc_value[ADC_SAMPLE_NO] = {0};
uint32_t adcValue                 = 0;
uint32_t adcMeanValue             = 0;

uint8_t bpmCounter = 0;

typedef enum
{
    eHeartBeatIdle = 0,
    eHeartBeatFingerNotDetected,
    eHeartBeatNotDetected,
    eHeartBeatDetected,
} eState_t;

typedef enum
{

    eStatePPGIdle = 0,
    eStatePeakDetectedFirst,
    eStateSegment1,
    eStatePeakDetectedSecond,
} ePPGState_t;
ePPGState_t ppgState = eStatePPGIdle;
uint8_t     state    = eHeartBeatNotDetected;

int heartRateBPM = 0;

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
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_IT(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, adc_value, ADC_SAMPLE_NO);

    ssd1306_Init();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        HAL_Delay(100);
        /* USER CODE BEGIN 3 */
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

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 168;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
     */
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_10B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel      = ADC_CHANNEL_5;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
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

    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 999;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 25;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
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
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 3;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 20999;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

    /*Configure GPIO pin : PA0 */
    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PA7 */
    GPIO_InitStruct.Pin   = GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* USER CODE BEGIN 4 */

uint8_t heartbeatDetected(uint32_t adcRawValue, int delay)
{
    static int     maxValue = 0;
    static uint8_t isPeak   = 0;
    uint8_t        result   = 0;
    // Measuring of the voltage value at the photo transistor
    adcRawValue *= (1000 / delay);
    // If the difference of the current value and the last value is to high
    // ( maybe because you moved the finger away from the sensor)
    // maxValue will be resetted to get a new basic
    if (adcRawValue * 4L < maxValue)
    {
        maxValue = adcRawValue * 0.8;
    } // Detect new peak
    if (adcRawValue > maxValue - (1000 / delay))
    {
        // The peak will be detected here. If the new adcRawValue is bigger than
        // the last maxValue, it will be detected as a peak.
        if (adcRawValue > maxValue)
        {
            maxValue = adcRawValue;
        }
        // Allocate only one heartbeat to a peak.
        if (isPeak == 0)
        {
            result = 1;
        }
        isPeak = 1;
    }
    else if (adcRawValue < maxValue - (3000 / delay))
    {
        isPeak = 0;
        // Here the maxValue will be decreased at each run
        // because if you don't do that the value would be every time lower or the same.
        // Also if you move the finger a bit the signal would be weaker without that.
        maxValue -= (1000 / delay);
    }
    return result;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    UNUSED(hadc);
    adcMeanValue = 0;

    static int statePrev = 0;
    for (int i = 0; i < ADC_SAMPLE_NO; i++)
    {
        adcMeanValue += adc_value[i];
    }
    adcMeanValue /= ADC_SAMPLE_NO;

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, adc_value, ADC_SAMPLE_NO);

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);

    if (adcMeanValue > FINGER_DETECTION_THRESHOLD_RAW)
    {
        static uint16_t beatMs = 0;
        if (heartbeatDetected(adcMeanValue, RESULT_PERIOD))
        {
            state        = eHeartBeatDetected;
            heartRateBPM = 60000 / beatMs;
            beatMs       = 0;
						if(heartRateBPM > 45 && heartRateBPM < 180)
						{
							updateScreen();
						}
            statePrev = state;
        }
        else
        {
            state     = eHeartBeatNotDetected;
            statePrev = state;
        }
        beatMs += RESULT_PERIOD;
    }
    else
    {
        state        = eHeartBeatFingerNotDetected;
        heartRateBPM = 0;

        if (state != statePrev)
        {
            updateScreen();
        }

        statePrev = statePrev;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(GPIO_Pin);
    ssd1306_Fill(eBlack);
    char    str[30] = {0};
    uint8_t y       = 2;

    writeScreenCenter(y, "HeartBeat <3 Rate", Font_7x10);

    y = 20;
    writeScreenCenter(y, "Monitoring...", Font_7x10);

    y = 35;
    sprintf(str, "%d bpm", bpmCounter);
    writeScreenCenter(y, str, Font_16x26);

    ssd1306_UpdateScreen();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(htim);
    static uint32_t counter    = 0;
    static uint8_t  toggleLeds = 0;

    counter++;
    if (counter % 1000 == 0)
    {
    }

    static int beatMsec = 0;

    if (state == eHeartBeatNotDetected)
    {
    }
    else
    {
    }
}
void updateScreen(void)
{
    if (state == eHeartBeatDetected)
    {
        ssd1306_Fill(eBlack);
        char    str[30] = {0};
        uint8_t y       = 2;

        writeScreenCenter(y, "HeartBeat <3 Rate", Font_7x10);

        y = 20;
        writeScreenCenter(y, "Monitoring...", Font_7x10);

        y = 35;
        sprintf(str, "%d bpm", heartRateBPM);
        writeScreenCenter(y, str, Font_16x26);

        ssd1306_UpdateScreen();
    }
    else if (state == eHeartBeatFingerNotDetected)
    {
        ssd1306_Fill(eBlack);
        char    str[30] = {0};
        uint8_t y       = 2;

        writeScreenCenter(y, "HeartBeat <3 Rate", Font_7x10);

        y = 20;
        writeScreenCenter(y, "Monitoring...", Font_7x10);

        y = 35;
        sprintf(str, "No Finger");
        writeScreenCenter(y, str, Font_11x18);

        ssd1306_UpdateScreen();
    }
}

void writeScreenCenter(uint8_t yPos, char* str, FontDef font)
{
    uint8_t length = strlen(str);
    if (font.FontWidth == Font_16x26.FontWidth)
    {
        uint8_t xPos = (SSD1306_WIDTH - length * 16) / 2;
        ssd1306_SetCursor(xPos, yPos);
        ssd1306_WriteString(str, Font_16x26, eWhite);
    }
    else if (font.FontWidth == Font_7x10.FontWidth)
    {
        uint8_t xPos = (SSD1306_WIDTH - length * 7) / 2;
        ssd1306_SetCursor(xPos, yPos);
        ssd1306_WriteString(str, Font_7x10, eWhite);
    }
    else if (font.FontWidth == Font_11x18.FontWidth)
    {
        uint8_t xPos = (SSD1306_WIDTH - length * 11) / 2;
        ssd1306_SetCursor(xPos, yPos);
        ssd1306_WriteString(str, Font_11x18, eWhite);
    }
    else if (font.FontWidth == Font_6x8.FontWidth)
    {
        uint8_t xPos = (SSD1306_WIDTH - length * 6) / 2;
        ssd1306_SetCursor(xPos, yPos);
        ssd1306_WriteString(str, Font_6x8, eWhite);
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

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
