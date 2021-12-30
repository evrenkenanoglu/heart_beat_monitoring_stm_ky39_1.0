/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 30/12/2021 13:32:14
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated by STM32CubeMX (DO NOT EDIT!)
 ******************************************************************************/

#ifndef __MX_DEVICE_H
#define __MX_DEVICE_H

/*---------------------------- Clock Configuration ---------------------------*/

#define MX_LSI_VALUE                            32000
#define MX_LSE_VALUE                            32768
#define MX_HSI_VALUE                            16000000
#define MX_HSE_VALUE                            25000000
#define MX_EXTERNAL_CLOCK_VALUE                 12288000
#define MX_PLLCLKFreq_Value                     168000000
#define MX_SYSCLKFreq_VALUE                     168000000
#define MX_HCLKFreq_Value                       168000000
#define MX_FCLKCortexFreq_Value                 168000000
#define MX_CortexFreq_Value                     168000000
#define MX_AHBFreq_Value                        168000000
#define MX_APB1Freq_Value                       42000000
#define MX_APB2Freq_Value                       84000000
#define MX_APB1TimFreq_Value                    84000000
#define MX_APB2TimFreq_Value                    168000000
#define MX_48MHZClocksFreq_Value                84000000
#define MX_EthernetFreq_Value                   168000000
#define MX_I2SClocksFreq_Value                  192000000
#define MX_RTCFreq_Value                        32000
#define MX_WatchDogFreq_Value                   32000
#define MX_MCO1PinFreq_Value                    16000000
#define MX_MCO2PinFreq_Value                    168000000

/*-------------------------------- ADC1       --------------------------------*/

#define MX_ADC1                                 1

/* GPIO Configuration */

/* Pin PA5 */
#define MX_ADCx_IN5_Pin                         PA5
#define MX_ADCx_IN5_GPIOx                       GPIOA
#define MX_ADCx_IN5_GPIO_PuPd                   GPIO_NOPULL
#define MX_ADCx_IN5_GPIO_Pin                    GPIO_PIN_5
#define MX_ADCx_IN5_GPIO_Mode                   GPIO_MODE_ANALOG

/* DMA Configuration */

/* DMA ADC1 */
#define MX_ADC1_DMA_DMA_Handle
#define MX_ADC1_DMA_Instance                    DMA2_Stream0
#define MX_ADC1_DMA_FIFOMode                    DMA_FIFOMODE_DISABLE
#define MX_ADC1_DMA_Priority                    DMA_PRIORITY_LOW
#define MX_ADC1_DMA_Channel                     DMA_CHANNEL_0
#define MX_ADC1_DMA_PeriphDataAlignment         DMA_PDATAALIGN_WORD
#define MX_ADC1_DMA_MemDataAlignment            DMA_MDATAALIGN_WORD
#define MX_ADC1_DMA_Mode                        DMA_NORMAL
#define MX_ADC1_DMA_Direction                   DMA_PERIPH_TO_MEMORY
#define MX_ADC1_DMA_PeriphInc                   DMA_PINC_DISABLE
#define MX_ADC1_DMA_MemInc                      DMA_MINC_ENABLE
#define MX_ADC1_DMA_IpInstance

/* NVIC Configuration */

/* NVIC ADC_IRQn */
#define MX_ADC_IRQn_interruptPremptionPriority  0
#define MX_ADC_IRQn_PriorityGroup               NVIC_PRIORITYGROUP_4
#define MX_ADC_IRQn_Subriority                  0

/*-------------------------------- DMA        --------------------------------*/

#define MX_DMA                                  1

/* NVIC Configuration */

/* NVIC DMA2_Stream0_IRQn */
#define MX_DMA2_Stream0_IRQn_interruptPremptionPriority 0
#define MX_DMA2_Stream0_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_DMA2_Stream0_IRQn_Subriority         0

/*-------------------------------- I2C1       --------------------------------*/

#define MX_I2C1                                 1

/* GPIO Configuration */

/* Pin PB6 */
#define MX_I2C1_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_I2C1_SCL_Pin                         PB6
#define MX_I2C1_SCL_GPIOx                       GPIOB
#define MX_I2C1_SCL_GPIO_Pin                    GPIO_PIN_6
#define MX_I2C1_SCL_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SCL_GPIO_Pu                     GPIO_NOPULL
#define MX_I2C1_SCL_GPIO_Mode                   GPIO_MODE_AF_OD

/* Pin PB7 */
#define MX_I2C1_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_I2C1_SDA_Pin                         PB7
#define MX_I2C1_SDA_GPIOx                       GPIOB
#define MX_I2C1_SDA_GPIO_Pin                    GPIO_PIN_7
#define MX_I2C1_SDA_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SDA_GPIO_Pu                     GPIO_NOPULL
#define MX_I2C1_SDA_GPIO_Mode                   GPIO_MODE_AF_OD

/* NVIC Configuration */

/* NVIC I2C1_EV_IRQn */
#define MX_I2C1_EV_IRQn_interruptPremptionPriority 0
#define MX_I2C1_EV_IRQn_PriorityGroup           NVIC_PRIORITYGROUP_4
#define MX_I2C1_EV_IRQn_Subriority              0

/*-------------------------------- SYS        --------------------------------*/

#define MX_SYS                                  1

/* GPIO Configuration */

/*-------------------------------- TIM3       --------------------------------*/

#define MX_TIM3                                 1

/* GPIO Configuration */

/*-------------------------------- TIM4       --------------------------------*/

#define MX_TIM4                                 1

/* GPIO Configuration */

/* NVIC Configuration */

/* NVIC TIM4_IRQn */
#define MX_TIM4_IRQn_interruptPremptionPriority 0
#define MX_TIM4_IRQn_PriorityGroup              NVIC_PRIORITYGROUP_4
#define MX_TIM4_IRQn_Subriority                 0

/*-------------------------------- NVIC       --------------------------------*/

#define MX_NVIC                                 1

/*-------------------------------- GPIO       --------------------------------*/

#define MX_GPIO                                 1

/* GPIO Configuration */

/* Pin PA0-WKUP */
#define MX_PA0_WKUP_Pin                         PA0_WKUP
#define MX_PA0_WKUP_GPIOx                       GPIOA
#define MX_PA0_WKUP_GPIO_PuPd                   GPIO_NOPULL
#define MX_PA0_WKUP_GPIO_Pin                    GPIO_PIN_0
#define MX_PA0_WKUP_GPIO_ModeDefaultEXTI        GPIO_MODE_IT_RISING

/* Pin PA7 */
#define MX_PA7_GPIO_Speed                       GPIO_SPEED_FREQ_LOW
#define MX_PA7_Pin                              PA7
#define MX_PA7_GPIOx                            GPIOA
#define MX_PA7_PinState                         GPIO_PIN_RESET
#define MX_PA7_GPIO_PuPd                        GPIO_NOPULL
#define MX_PA7_GPIO_Pin                         GPIO_PIN_7
#define MX_PA7_GPIO_ModeDefaultOutputPP         GPIO_MODE_OUTPUT_PP

#endif  /* __MX_DEVICE_H */

