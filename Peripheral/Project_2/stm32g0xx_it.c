/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_it.h"
#include "math.h"
#include "DEFINE.h"
#include "Hardware.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

extern ADC_HandleTypeDef hadc1;

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac1_ch1;

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;

extern volatile uint16_t  Counter100msec;
extern volatile uint16_t  Counter1000msec;
extern volatile uint16_t  Counter5000msec;
extern volatile uint16_t  CAN_RxTimer;
extern volatile uint16_t  CAN_time_out;
extern volatile uint16_t General_timer;

extern volatile uint16_t Sec_Counter;
extern volatile uint16_t Min_Counter;
extern volatile uint16_t Hrs_Counter;
extern volatile uint16_t Msec_Counter;
extern volatile  uint8_t Time_out;
extern volatile uint8_t Charge_started;

extern volatile uint16_t Counter1000msec1;
extern volatile uint16_t Counter1000msec2;
extern volatile uint16_t Counter1000msec3;
extern volatile uint16_t Counter1000msec4;
extern volatile uint16_t Counter1000msec5;
extern volatile uint16_t  Full_CHARGE;

/****Interrupt based reading and SHTCKT protections extern variables************/

extern volatile uint8_t Initialisation_Completed;
extern volatile uint16_t Live_Charge;
extern volatile uint16_t Live_Voltage;
extern volatile uint16_t Live_Current;
extern volatile uint16_t Live_AC_Voltage;

extern volatile uint32_t Live_Voltage_Org;
extern volatile uint32_t Live_Current_Org;
extern volatile uint16_t Shkt_Current_Org;
extern volatile uint16_t OC_Average[];
extern volatile uint8_t OC_Count;
extern volatile uint16_t Short_Circuit_Average[];
extern volatile uint8_t Short_Circuit_Count;
extern volatile uint16_t OV_Average[OV_SAMPLES];
extern volatile uint8_t OV_Count;
extern volatile uint16_t Short_Circuit_Voltage;

extern volatile uint16_t Modified_Voltage_Read;
extern volatile uint16_t Modified_Current_Read;
extern volatile uint16_t Live_Current_Tune;
extern volatile uint16_t Live_Voltage_Tune;
extern volatile uint32_t Live_Current_ADC;
extern volatile uint32_t Live_Voltage_ADC;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim14;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
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
	uint32_t Counter;
	uint8_t Loop_Counter;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  Counter100msec ++;
  Counter1000msec ++;
  Counter5000msec ++;
  CAN_RxTimer++;
  CAN_time_out++;
  General_timer++;

  Counter1000msec1 ++;
  Counter1000msec2 ++;
  Counter1000msec3 ++;
  Counter1000msec4 ++;
  Counter1000msec5 ++;
  Full_CHARGE++;

  /********************Interrupt based reading and SHTCKT protections************/
  if (Initialisation_Completed == 1)
     {
       ADC_Select_CH0 (BATTERY_SENSE, 1);
       HAL_ADC_Start (&hadc1);
       HAL_ADC_PollForConversion (&hadc1, 2); //1);
       OV_Average[OV_Count] = HAL_ADC_GetValue (&hadc1);
       Short_Circuit_Voltage = OV_Average[OV_Count];

       ADC_Select_CH0 (CURRENT_SENSE, 1);
       HAL_ADC_Start (&hadc1);
       HAL_ADC_PollForConversion (&hadc1, 2); //1);
       OC_Average[OC_Count] = HAL_ADC_GetValue (&hadc1);
       Short_Circuit_Average[Short_Circuit_Count] = OC_Average[OC_Count];

       Live_Current_Org = 0;
       for (Counter = 0; Counter < OC_SAMPLES; Counter++)
 	{
 	  Live_Current_Org = Live_Current_Org + OC_Average[Counter];
 	}

       Live_Current_Org = Live_Current_Org / OC_SAMPLES;

       if (++OC_Count == OC_SAMPLES)
 	{
 	  OC_Count = 0;
 	}
       /*************************************************************************/
       Shkt_Current_Org = 0;
       for (Counter = 0; Counter < SHORT_CIRCUIT_SAMPLES; Counter++)
 	{
 	  Shkt_Current_Org = Shkt_Current_Org + Short_Circuit_Average[Counter];
 	}

       Shkt_Current_Org = Shkt_Current_Org / SHORT_CIRCUIT_SAMPLES;

       if (++Short_Circuit_Count == SHORT_CIRCUIT_SAMPLES)
 	{
 	  Short_Circuit_Count = 0;
 	}

       /********************************************************/
       Live_Voltage_Org = 0;
       for (Counter = 0; Counter < OV_SAMPLES; Counter++)
 	{
 	  Live_Voltage_Org = Live_Voltage_Org + OV_Average[Counter];
 	}

       Live_Voltage_Org = Live_Voltage_Org / OV_SAMPLES;

       if (++OV_Count == OV_SAMPLES)
 	{
 	  OV_Count = 0;
 	}

       /*********************************************************************/
 //      Live_Voltage = (unsigned int) ((float) (((Live_Voltage_Org * 0.0237605)
 //	  + 0.7) * 10));
 //
 //      Live_Current = (unsigned int) ((float) (Live_Current_Org * 0.0350877)); //1A = 285, 285 * x = 10 => x = 0.0350877

   	/*********************************************************************/
   	Live_Voltage_ADC = (Live_Voltage_Org * Modified_Voltage_Read);
   	// in mV used in tuning
   	Live_Voltage_Tune = Live_Voltage_ADC / 1000;
   	Live_Voltage = Live_Voltage_Tune / 10;	// legacy variable used in main

   	Live_Current_ADC = (Live_Current_Org * Modified_Current_Read);
   	// in mA used in tuning
   	Live_Current_Tune = Live_Current_ADC / 1000;
   	Live_Current = Live_Current_Tune / 10;	// Legacy variable used in main
       /*********************************************************************/
         if ((Shkt_Current_Org >= SHORT_CIRUCIT_CURRENT_ADC)
         		&& (Short_Circuit_Voltage <= UNDER_VOLT_VALUE_ADC))
 		{
 		  RELAY_OFF ();
 		  SHUTDWON_ON ();
 		  PWM_CURRENT_ZERO ();
 		  DAC_VOLTAGE_ZERO ();
 		  FAN_OFF ();
 		  LEDS_30_OFF ();
 		  LEDS_50_OFF ();
 		  LEDS_80_OFF ();
 		  LEDS_100_OFF ();
 		    while (1)
 			{

 		    	for (Loop_Counter = 0; Loop_Counter < 6; Loop_Counter++)
 			    {
 		    		for (Counter = 0; Counter <= 200000; Counter++)
 		    		{
 		    			AC_LED_ON();
 		    		}
 		    		for (Counter = 0; Counter <= 200000; Counter++)
 		    		{
 		    			AC_LED_OFF();
 		    		}
 			    }

 		    	for (Counter = 0; Counter <= 1000000; Counter++)
 			    {
 		    		AC_LED_OFF();
 			    }

 			}
 		}

     }       // initialization completed loop - Protections check
  /////////////////////////////////////////////////
  // Hours Counter
  ///////////////////////////////////////
  if(Charge_started == 1 )		// Charging in Progress
  {
	  if(++Msec_Counter == 1000)
	  {
		  Msec_Counter = 0;
		  if( ++Sec_Counter == 60)
		  {

			  Sec_Counter = 0;
			  if( ++Min_Counter == 60)
			  {
				  Min_Counter = 0;
				  if(++Hrs_Counter == 8)
				  {
					  Time_out =1;
				  }
			  }
		  }
	  }
  }


  //////////////////////////////////////////
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM16, FDCAN1_IT0 and FDCAN2_IT0 Interrupt.
  */
void TIM16_FDCAN_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_FDCAN_IT0_IRQn 0 */

  /* USER CODE END TIM16_FDCAN_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN TIM16_FDCAN_IT0_IRQn 1 */

  /* USER CODE END TIM16_FDCAN_IT0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}
/* USER CODE END 1 */
