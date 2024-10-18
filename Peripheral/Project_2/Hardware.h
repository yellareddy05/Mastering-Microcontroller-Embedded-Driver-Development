/*
 * Hardware.h
 *
 *  Created on: 19-Nov-2022
 *      Author: Soft
 */

#ifndef SRC_HARDWARE_H_
#define SRC_HARDWARE_H_

# define RELAY_ON()         HAL_GPIO_WritePin(GPIOB, RELAY_Pin, GPIO_PIN_SET)
# define RELAY_OFF()        HAL_GPIO_WritePin(GPIOB, RELAY_Pin, GPIO_PIN_RESET)
# define AC_LED_ON()        HAL_GPIO_WritePin(GPIOD, AC_LED_Pin, GPIO_PIN_RESET)
# define AC_LED_OFF()       HAL_GPIO_WritePin(GPIOD, AC_LED_Pin, GPIO_PIN_SET)
# define FAN_OFF()          HAL_GPIO_WritePin(GPIOB, FAN_Pin, GPIO_PIN_RESET)
# define FAN_ON()           HAL_GPIO_WritePin(GPIOB, FAN_Pin, GPIO_PIN_SET)
# define BUZZER_ON()        HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_SET)
# define BUZZER_OFF()       HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_RESET)
# define SHUTDWON_ON()      HAL_GPIO_WritePin(GPIOB, SHUTDOWN_Pin, GPIO_PIN_SET)
# define SHUTDWON_OFF()     HAL_GPIO_WritePin(GPIOB, SHUTDOWN_Pin, GPIO_PIN_RESET)
# define LEDS_30_ON()       HAL_GPIO_WritePin(GPIOD, LED_30__Pin, GPIO_PIN_RESET)
# define LEDS_30_OFF()      HAL_GPIO_WritePin(GPIOD, LED_30__Pin, GPIO_PIN_SET)
# define LEDS_50_ON()       HAL_GPIO_WritePin(GPIOD, LED_50__Pin, GPIO_PIN_RESET)
# define LEDS_50_OFF()      HAL_GPIO_WritePin(GPIOD, LED_50__Pin, GPIO_PIN_SET)
# define LEDS_80_ON()       HAL_GPIO_WritePin(GPIOD, LED_80__Pin, GPIO_PIN_RESET)
# define LEDS_80_OFF()      HAL_GPIO_WritePin(GPIOD, LED_80__Pin, GPIO_PIN_SET)
# define LEDS_100_ON()      HAL_GPIO_WritePin(GPIOB, LED_80__Pin, GPIO_PIN_RESET)
# define LEDS_100_OFF()     HAL_GPIO_WritePin(GPIOB, LED_80__Pin, GPIO_PIN_SET)
# define LED_30_TOGGLE()    HAL_GPIO_TogglePin(GPIOD, LED_30__Pin);
# define LED_50_TOGGLE()    HAL_GPIO_TogglePin(GPIOD, LED_50__Pin);
# define LED_80_TOGGLE()    HAL_GPIO_TogglePin(GPIOD, LED_80__Pin);
# define LED_100_TOGGLE()   HAL_GPIO_TogglePin(GPIOB, LED_100__Pin);
# define AC_LED_TOGGLE()    HAL_GPIO_TogglePin(GPIOD, AC_LED_Pin);
# define BUZZER_TOGGLE()    HAL_GPIO_TogglePin(GPIOB, BUZZER_Pin);
# define DAC_VOLTAGE_SET()  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,DAC_VOLTAGE_VALUE)
# define PWM_CURRENT_SET()  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,PWM_CURRENT_VALUE)
# define DAC_VOLTAGE_ZERO()  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,0)
# define PWM_CURRENT_ZERO()  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0)
# define DELAY(uint16_t)    HAL_Delay(uint16_t)
# define REVERSE_PIN(GPIOC,uint16_t)       HAL_GPIO_ReadPin(GPIOC, uint16_t )
# define Aging_pin()        HAL_GPIO_ReadPin(GPIOB,AGING_Pin )
#endif /* SRC_HARDWARE_H_ */
