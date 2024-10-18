/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "DEFINE.h"
#include "VARIABLES.h"
#include "Hardware.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
# define   FIXED_CURRENT	             0x50

# define   DAC_VOLTAGE_VALUE             1
# define   PWM_CURRENT_VALUE             1

# define   AC_HIGH_VOLTAGE				0x0D03//3200//0X0934
# define   AC_LOW_VOLTAGE			    0x0886//1700// 0x04BC

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

FDCAN_HandleTypeDef hfdcan1;

//IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
GPIO_TypeDef GPIO_READ;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t ubKeyNumber = 0x0;
uint8_t ubKeyNumberValue = 0x0;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];                              //can data array
uint32_t CANRXIdentifier;
static uint8_t Counter=0x2F;
uint8_t BatteryReverse = 0;
static uint8_t BMS_RECEIVE ;

static uint16_t Set_output_voltage ;
static uint16_t Set_output_current ;
uint16_t CRC_Return;
uint16_t CRC_Received;


union PROTECTION_FLAG
{
	struct
	{
		uint16_t Monomer_overvoltage_protection           :1;  //byte 0 start
		uint16_t Monomer_undervoltage_protection          :1;
		uint16_t Whole_group_overvoltage_protection	      :1;
		uint16_t Whole_group_undervoltage_protection      :1;
		uint16_t Chargeing_over_temperature_protection    :1;
		uint16_t Chargeing_low_temperature_protection     :1;
		uint16_t Dischargeing_over_temperature_protection :1;
		uint16_t Dischargeing_low_temperature_protection  :1;  //byte 0 end
		uint16_t Charge_overcurrent_protection		   	  :1;  //byte 1 start
		uint16_t Discharge_overcurrent_protection	      :1;
		uint16_t Short_circuit_protection                 :1;
		uint16_t Front_end_detection_IC_error             :1;
        uint16_t Software_lock_MOS      				  :1;
	    uint16_t Reserved_0                               :1;
	    uint16_t Reserved_1                               :1;
	    uint16_t Reserved_2                               :1;  //byte 1 end
	};
	uint16_t Fault;
	uint8_t BMS_CAN[2];		//MURTHY

}BMS_Fault;
uint8_t Fault_Counter;

union Charger
{
	struct
		{
		uint16_t Hardware_fault			  :1;
		uint16_t Temperature_fault		  :1;
		uint16_t Input_voltage_fault	  :1;
		uint16_t Input_voltage_status	  :1;
		uint16_t Output_over_current	  :1;
		uint16_t Charging_status		  :1;
		uint16_t Communication_status	  :1;
		uint16_t Battery_connetion_status :1;
		uint16_t Output_short_circuit	  :1;
		uint16_t Output_over_voltage	  :1;
		uint16_t Output_under_voltage	  :1;
		uint16_t chargerCMDoutofrange     :1;

		uint8_t chargerCANTimeout     :1;


		};
	uint16_t Fault;
	uint8_t Charger_CAN[2];

}Charger_Fault;

union Charger_Control  // Bhargav
{
	struct
		{
		    uint8_t chargerEnableB0		   :1;
			uint8_t chargerEnableB1        :1;
			uint8_t chargerModeRequestB2   :1;
			uint8_t chargerModeRequestB3   :1;
			uint8_t emergencyShutdown	   :1;
			uint8_t reserved0	           :1;
			uint8_t reserved1		       :1;
			uint8_t reserved2	           :1;
		};
	uint8_t data;
	//uint8_t Charger_CAN[2];

}chargerControl;

union Contents
{
  struct
  {
    uint32_t Received_Content_ID_100 :1;
    uint32_t Received_Content_ID_101 :1;
    uint32_t Received_Content_ID_102 :1;
    uint32_t Received_Content_ID_103 :1;
    uint32_t Received_Content_ID_104 :1;
    uint32_t Received_Content_ID_105 :1;
    uint32_t Received_Content_ID_106 :1;
    uint32_t Received_Content_ID_107 :1;
    uint32_t Received_Content_ID_108 :1;
    uint32_t Received_Content_ID_109 :1;
    uint32_t Received_Content_ID_10A :1;
    uint32_t Received_Content_ID_10B :1;
    uint32_t Received_Content_ID_10C :1;
    uint32_t Received_Content_ID_10D :1;
    uint32_t Received_Content_ID_10E :1;
    uint32_t Received_Content_ID_10F :1;
    uint32_t Received_Content_ID_110 :1;
    uint32_t Received_Content_ID_1806E5F4 :1;
  };
  uint32_t ID_Data;
}Received_BMS;

uint8_t Protection_Count;
//static uint32_t Rated_Capacity;
//static uint8_t Active_Cell;
//static uint8_t Active_Temp;
//static uint8_t DOD;
//static uint8_t Cell_Chemistry;


uint16_t Temperature_value_NTC[12];
uint8_t Can_Received_Data[138];

uint16_t Over_voltage ;
uint16_t Over_current;
uint16_t Present_voltage;
uint16_t Present_current;
uint8_t Full_Charge_start;
uint8_t Pre_Fullcharge_Iterations;
uint8_t Pre_Fullcharge_Checking;
uint8_t Power_on_check;
uint8_t MODE_Backup;

volatile uint16_t General_timer;
volatile uint16_t Counter100msec;
volatile uint16_t Counter1000msec;
volatile uint16_t Counter1000msec1;
volatile uint16_t Counter1000msec2;
volatile uint16_t Counter1000msec3;
volatile uint16_t Counter1000msec4;
volatile uint16_t Counter1000msec5;
volatile uint16_t Counter5000msec;
volatile uint16_t UART_Counter;
volatile uint16_t CAN_RxTimer;
volatile uint16_t CAN_time_out; //5 sec
volatile uint16_t Full_CHARGE;
volatile uint32_t Pre_Full_Charge;
uint16_t Full_charge_check;
uint8_t Pre_FullCharge;
volatile uint16_t Sec_Counter;
volatile uint16_t Min_Counter;
volatile uint16_t Hrs_Counter;
volatile uint16_t Msec_Counter;
volatile  uint8_t Time_out;
volatile uint8_t Charge_started;
volatile uint8_t Charger_Status;

volatile uint16_t Short_Circuit_Voltage;

uint32_t Charge_Status;

uint32_t Demand_voltage;
uint32_t Demand_current;

uint32_t Battery_voltage;
uint32_t Battery_current;
uint16_t Remaining_capacity;
uint16_t Cycle_times;
uint8_t No_of_battery_string;
uint8_t No_of_NTC_probes;
uint8_t BMS_Data;
static uint8_t State_of_Charge;
//static uint8_t State_of_Health;

uint8_t aTxBuffer[50];
uint8_t aRxBuffer[255];
uint8_t aTxTestData[]  = " *** UART and CAN Recorder  ***\r\n  ";
uint8_t aInitial_mode[]  = "Initialize Mode\r\n ";

uint16_t Over_voltage ;
uint16_t Over_current;
uint16_t Present_voltage;
uint16_t Present_current;
uint8_t Full_Charge_start;
uint8_t Pre_FullCharge;
uint8_t  Mode_Backup;


volatile uint8_t Initialisation_Completed;
volatile uint16_t Live_Charge;
volatile uint16_t Live_Voltage;
volatile uint16_t Live_Current;
volatile uint16_t Live_AC_Voltage;
volatile uint32_t Live_Voltage_Org;
volatile uint32_t Live_Current_Org;
volatile uint16_t Shkt_Current_Org;

volatile uint16_t Short_Circuit_Average[SHORT_CIRCUIT_SAMPLES];
volatile uint8_t Short_Circuit_Count;
volatile uint16_t OC_Average[OC_SAMPLES];
volatile uint8_t OC_Count;
volatile uint16_t OV_Average[OV_SAMPLES];
volatile uint8_t OV_Count;
volatile uint16_t Short_Circuit_Voltage;

volatile uint16_t General_timer;
volatile uint16_t Counter100msec;
volatile uint16_t Counter1000msec;
volatile uint16_t Counter1000msec1;
volatile uint16_t Counter1000msec2;
volatile uint16_t Counter1000msec3;
volatile uint16_t Counter1000msec4;
volatile uint16_t Counter1000msec5;
volatile uint16_t Counter5000msec;

volatile uint16_t Sec_Counter;
volatile uint16_t Min_Counter;
volatile uint16_t Hrs_Counter;
volatile uint16_t Msec_Counter;
volatile  uint8_t Time_out;
volatile uint8_t Charge_started;

volatile uint16_t CAN_RxTimer;
volatile uint16_t CAN_time_out; //5 sec
volatile uint16_t Full_CHARGE;
uint16_t Full_charge_check;
uint8_t Power_on_check;


uint8_t UART_RX_Identifier;
uint8_t UART_Data_Ready;

uint16_t Max_cell_volt;
uint8_t No_Of_Cell_With_Max_volt;
uint16_t Min_cell_volt;
uint8_t No_Of_Cell_With_Min_volt;

uint8_t Max_cell_Temp;
uint8_t No_Of_Cell_With_Max_Temp;
uint8_t Min_cell_Temp;
uint8_t No_Of_Cell_With_Min_Temp;

uint8_t No_of_battery_string;
uint8_t No_of_NTC_probes;

//static uint16_t State_of_Charge;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
//static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
void
AC_LED_BLINK (unsigned char NO_OF_BLINKS);
static void FDCAN_Config(void);
void CAN_TX (void);
void CAN_RX (void);
void Demanding_Swtich_status (void);
void Received_Swtich_status (void);
void volt_current_output (void);
void Charger_Authorization (void);

void Demanding_Content_ID_180FF50E5(void);
void Received_Content_ID_1806E5F4(void);


void Demanding_Content_ID_100(void);
void Received_Content_ID_100(void);
void Demanding_Content_ID_101(void);
void Received_Content_ID_101(void);
void Demanding_Content_ID_102(void);
void Received_Content_ID_102(void);
void Demanding_Content_ID_103(void);
void Received_Content_ID_103(void);
void Demanding_Content_ID_104(void);
void Received_Content_ID_104(void);
void Demanding_Content_ID_105(void);
void Received_Content_ID_105(void);
void Demanding_Content_ID_106(void);
void Received_Content_ID_106(void);
void Demanding_Content_ID_107(void);
void Received_Content_ID_107(void);
void Demanding_Content_ID_108(void);
void Received_Content_ID_108(void);
void Demanding_Content_ID_109(void);
void Received_Content_ID_109(void);
void Demanding_Content_ID_10A(void);
void Received_Content_ID_10A(void);
void Demanding_Content_ID_10B(void);
void Received_Content_ID_10B(void);
void Demanding_Content_ID_10C(void);
void Received_Content_ID_10C(void);
void Demanding_Content_ID_10D(void);
void Received_Content_ID_10D(void);
void Demanding_Content_ID_10E(void);
void Received_Content_ID_10E(void);
void Demanding_Content_ID_10F(void);
void Received_Content_ID_10F(void);
void Demanding_Content_ID_110(void);
void Received_Content_ID_110(void);

uint16_t Check_CRC16 (uint8_t *pchMsg,uint8_t wDataLen);

void LED_ON(void);
void LED_OFF(void);
void CAN_Process();
void CAN_Recovery();
void CAN_Send_Fault();
void TEST_LED(void);
void Voltage_Fine_tuning(unsigned int INIT_VOLTAG, unsigned int END_VOLT_VAL);
void Aging_process(void);
void UART_process(void);
void String_Copy(uint8_t *Dest,uint8_t *Src, uint8_t Length);
uint16_t Check_CRC16 (uint8_t *pchMsg,uint8_t wDataLen);
uint8_t AC_Range_Check (void);

uint16_t Temp_value_NTC[5];//4NTC 0-3
uint16_t Voltage_value_CELL[23];

uint8_t CANDataReady;
uint8_t TempRxData[8];
uint8_t MOS_Status;

void AGING_PROTECTION_CALL(void);
void Aging_process(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*********************UART TUNNING PATAMETERS***********************/
void Aging_VI_Output();
/*Abbreviation Table
 WWDT == Window Watch Dog Timer
 BAT == Battery
 SUB ==
 OVP == Over Voltage Protection
 LAT == Latch
 MIN == Minimum
 VOL == Voltage
 CLRWDT == Clear Watchdog Timer
 INC == Increment
 CC == Constant Current
 CV == Constant Voltage
 BLK == Blinking
 EXE == Execution
 DIV == Division
 Val == Value
 BLK == Blink
 ms == millisecond
 API == Application Programming Interface
 */

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
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  /* USER CODE BEGIN SysInit */
  Initialisation_Completed = 0;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();

  MX_FDCAN1_Init();

 // MX_IWDG_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();


  /* USER CODE BEGIN 2 */
  //Start DAC Output generation
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
    DAC_VOLTAGE_SET();

   /* Configure the FDCAN peripheral */
    FDCAN_Config();

    CANDataReady =0;
    Counter = 0x2F;
    BMS_RECEIVE = 0;

    Protection_Count=0;
    BatteryReverse = 0;
    Over_voltage = 0 ;
    Over_current = 0 ;
    Present_voltage = 0;
    Present_current = 0;
    Full_Charge_start = 0;
    Power_on_check = 0;
    Pre_FullCharge = 0;

    Battery_voltage = 0;
    Battery_current = 0;
    Remaining_capacity = 0;
  // Present_DAC =0;
    Counter1000msec =0;
    Counter100msec =0;
    Counter1000msec2 = 0;
    Counter1000msec1=0;

    Sec_Counter = 0;
    Min_Counter = 0;
    Hrs_Counter = 0;
    Msec_Counter = 0;
    Time_out =0;;
    Charge_started = 0;

    CAN_RxTimer =0;
    CAN_time_out =0;

    Max_cell_volt = 0;
    No_Of_Cell_With_Max_volt = 0;
    Min_cell_volt = 0;
    No_Of_Cell_With_Min_volt = 0;

    Max_cell_Temp = 0;
    No_Of_Cell_With_Max_Temp = 0;
    Min_cell_Temp = 0;
    No_Of_Cell_With_Min_Temp = 0;

    BMS_Fault.Fault = 0xFFFF;
    Charger_Fault.Fault = 0x0000;
    Set_output_voltage = 0;
    Set_output_current = 0;
  // Start Timer with Interrupt
    HAL_TIM_Base_Start_IT(&htim14);

  // Start Timer for PWM Generation
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //Current
    OC_Count = 0;
      for (Counter = 0; Counter < OC_SAMPLES; Counter++)
        {
          OC_Average[Counter] = 1; // to avoid numerator as 0 initially during averaging
        }

      Short_Circuit_Voltage = UNDER_VOLT_VALUE_ADC + 10;	// do not assign value 0
      OV_Count = 0;
      for (Counter = 0; Counter < OV_SAMPLES; Counter++)
        {
          OV_Average[Counter] = 1; // to avoid numerator as 0 initially during averaging
        }

      Short_Circuit_Count = 0;

      for (Counter = 0; Counter < SHORT_CIRCUIT_SAMPLES; Counter++)
        {
          Short_Circuit_Average[Counter] = 1; // to avoid numerator as 0 initially during averaging
        }


   	/**********************************************
   	 * Set the over current bit as Disable
   	 * OVER_CURRENT_BIT
   	 **********************************************/
   	INIT_COND_FUN_CALL();
   	OFF_CONDITION();
   	ALL_LEDS_OFF();
   	DELAY(50);
    AC_LED_ON();
   	/********************************************
   	 * Selection Mode from switch case
   	 *********************************************/


   	/*******************Checking Aging or Normal Mode******************************/
    /**************************Memory operations******************************************************/
         Tuning_Command = 0;
       FirstPage = ADDR_FLASH_PAGE;
       NbOfPages = 1;
       BankNumber = FLASH_BANK_1;

         Address = ADDR_FLASH_PAGE_63;
         Data64 = *(__IO uint32_t*) Address;

       if (Data64 != 1)
         {
           Page_Data[0] = 1;
           	Page_Data[1] = MAX_VOLTAGE;
           	Page_Data[2] = MAX_CURRENT;
           	Page_Data[3] = VOLATGE_MULTIPLIER;
           	Page_Data[4] = CURRENT_MULTIPLIER;


     	Flash_Erase_Page();
     	Address = ADDR_FLASH_PAGE_63;
     	for (Counter = 0; Counter < 5; Counter++)
     	{
     		HAL_FLASH_Program (FLASH_TYPEPROGRAM_DOUBLEWORD, Address,
     		Page_Data[Counter]);
     		Address = Address + 8;
     	}

     	HAL_FLASH_Lock();
     	DELAY(5);//  to close the memory writing operation, do not remove for safe memory operations
         }

           Address = DEFINED_VOLTAGE;
           Data64 = *(__IO uint32_t*) Address;
           Modified_Voltage = Data64;
           Modified_OVP = Modified_Voltage + BUFFER_VOLTAGE;

           Address = DEFINED_CURRENT;
           Data64 = *(__IO uint32_t*) Address;
           Modified_Current = Data64;
           Modified_OCP = Modified_Current + BUFFER_CURRENT;

           Address = VOLATGE_ADC_MULTIPLIER;
           Data64 = *(__IO uint32_t*) Address;
           Modified_Voltage_Read = Data64;

           Address = CURRENT_ADC_MULTIPLIER;
           Data64 = *(__IO uint32_t*) Address;
           Modified_Current_Read = Data64;
 /********************************************
    	 * Selection Mode from switch case
   *********************************************/

    if (Aging_pin () == 0)
    {
      Aging_process ();
    }  //Aging end

    Initialisation_Completed =1;
    DELAY(200);
     /* Infinite loop */
     /* USER CODE BEGIN WHILE */

//****************************************** Reverse Polarity check******************************************
  Counter1000msec5 = 0;
  //LED_ON();
  MODE = INITIAL_MODE;

  RELAY_ON();
  DELAY(30);
  while (Counter1000msec5 <= 5000)
    {
      REVERSE_BIT = REVERSE_PIN(GPIOC, REVERSE__Pin); //Battery reverse indication
      if (REVERSE_BIT == 1)
      {
		  LED_OFF ();
		  BUZZER (4);
		  ACCHECK_MODE = 0;
		  OVER_CURRENT_BIT = 0;
		  RELAY_OFF();
		  while (1)
		  {
			  if (Counter100msec >= 250)
			  {
				  Counter100msec = 0;
				  LED_OFF ();
				  AC_LED_TOGGLE();
			  }

		  } // inside while ends
      }

      OVP_AVG_VOLTGAE = OVP_AVG ();
      if (OVP_AVG_VOLTGAE > OVER_VOLT_VALUE_PROTECTION) //over voltage protection
      {
    	  SHUTDOWN_HIGH_FUN();
    	     	     		    //OFF_CONDITION();
    	     	  while(1){
    	     	  	         if (LED_TIMER >= 20000)
    	     	     		    {

    	     	     				  LED_OFF();
    	     	  		              AC_LED_BLINK (2);
    	     	  		              LED_TIMER=0;
    	     	     		    }
    	     	           }
      }

    }	// reverse battery while counter ends
  	BUZZER(2); //power on buzzer

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//											MAIN ROUTINE
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^




  	while (1)
   	{

  		//   		HAL_IWDG_Refresh(&hiwdg);

  		   		switch (MODE)
  		   		{
  		   		case INITIAL_MODE:
  		   		    INIT_COND_FUN_CALL();
  		   		    OFF_CONDITION();
  		   		    AC_LED_ON();	//ON

  		//**************Reverse battery /  Over current check  **************************//
  		     		 CURRENT= CURR_SENSE(CURRENT_SENSE);
  		     		 if(CURRENT > OVER_CURRENT_VALUE_PROTECTION) //over current protection
  		     		 	{
  		     		 	    OVC_BIT=1;
  		     		 	    if(OVC_TMR >= 20)
  		     		 	    {
  		     		 	    	RELAY_OFF();
  		     		 	    	PWM_CURRENT_ZERO();	//CURRENT
  		     		 	    	Present_current=0;
  		     		 	    	Set_output_current=0;
  		     		 	    	MODE=OVER_CURRENT_MODE;
  		     		 	    	BUZZER(4);
  		     		 	    	break;
  		     		 	    }
  		     		 	}
  		     		 	else
  		     		 	{
  		     		 	   OVC_BIT=0;
  		     		 	   OVC_TMR=0;
  		     		 	}


  		  		 if(Power_on_check > 2)
  		    	 {
  		    		 BMS_RECEIVE = 0;
  		    		 MODE = CAN_TIME_OUT;
  		    		 break;
  		    	 }
  		    	 Power_on_check++;

  		/******************* 2 Amp Pre-charge **************************/


  		   	Counter1000msec5 = 0;
  			  while (Counter1000msec5 <= 20000)
  			    {
  			      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PRE_CHARGE_CURRENT); //CURRENT
  			      Present_current = PRE_CHARGE_CURRENT;

  			      Set_output_voltage = Modified_Voltage;

  			      if (Present_voltage != Set_output_voltage)
  				{
  			    	VOLTAGE_INC_SLOWLY_ONE_AMP (Present_voltage, Set_output_voltage);
  			    	Present_voltage = Set_output_voltage;
  				}


  			      Present_current = PRE_CHARGE_CURRENT;
  			      CURRENT = CURR_SENSE (CURRENT_SENSE);
  			      DELAY(10);
  			      if (CURRENT >= CURRENT_0_5A)
  				{
  				  FAN_ON();
  				}
  		//	      else
  		//		{
  		//		  FAN_OFF();
  		//		}



  			      LEDS_50_OFF();
  			      LEDS_80_OFF();
  			      LEDS_100_OFF();
  			      if (LED_TIMER >= 1000)
  				{
  				  LED_TIMER = 0;
  				  LED_30_TOGGLE();
  				}

  			      CAN_RxTimer = 0;

  		 //****************************************** AV voltage check ******************************************
  		   			/*ACVOLTAGE = AC_VOLT_SENSE(AC_SENSE);
  		   			if((ACVOLTAGE > AC_HIGH_VOLTAGE) || (ACVOLTAGE < AC_LOW_VOLTAGE)) // Input AC voltage sensing
  		   			{
  		   				AC_OVP=1;
  		   				if(AC_OVP_TMR >= 50)
  		   				{
  		   				   		//BUZZER(1);
  		   				   		PWM_CURRENT_ZERO();	//CURRENT
  		   				   		Present_current=0;
  		   				   		Set_output_current=0;
  		   				   		MODE_Backup = MODE;
  		   				   		MODE= AC_RECOVERY_MODE;
  		   				   		break; // while break;
  		   				}

  		   			}
  		   			else
  		   			{
  		   				 AC_OVP=0;
  		   				 AC_OVP_TMR = 0;
  		   			}*/

  		   			}//while end

  		//*************** Under voltage  & Full charge******************************************
  		   		if(MODE == INITIAL_MODE)
  		   		{
  		   			// volt_under_oneamp = ((Active_Cell * (MIN_CELL_VOLTAGE * 0.001))*10);
  		   			OVP_AVG_VOLTGAE=OVP_AVG();
  		   			CURRENT = CURR_SENSE(CURRENT_SENSE);
  		   			if(OVP_AVG_VOLTGAE < UNDER_VOLT_VALUE_ONEAMP) //volt_under_oneamp) //under voltage protection
  		    		{
  		//    				PWM_CURRENT_ZERO();
  		//    		     	Present_current=0;
  		//   		     	Set_output_current =0;
  		    		     	MODE=ONE_AMP_MODE;
  		//    		    	BUZZER(1);
  		    		     	break;
  		    		}

  		   			if((CURRENT <= BATTERY_CURRENT) && (Battery_voltage >= Modified_Voltage))//FULL charge call
  		   			{
  		   			   		PWM_CURRENT_ZERO();	//CURRENT
  		   			   		Present_current=0;
  		   			   		Set_output_current =0;
  		   			   		Full_CHARGE=0;
  		   			   		MODE=FULL_CHARGE_MODE;
  		   			   		break;
  		   			 }

  		 /*******************CAN communication checking**************************/

  		   				// to send query 2
  						CAN_Process();

  		 			//PROTECTION_CALL();
  		   		}
  		   			break;

  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		//										CC_CV MODE
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		   		case CC_CV_MODE:

  		   			Charge_started = 1;
  		   			if(Time_out == 1)
  		   			   	{
  		   			   		MODE = FULL_CHARGE_MODE;
  		   			   		break;
  		   			   	}
  		   			AC_LED_ON();
  		   		    LED_BLINK();
  		           // BUZZER(1);

  		            Set_output_voltage = Modifier_Voltage;
  		   			if(Present_voltage != Modifier_Voltage)
  		   			{
  		   				VOLTAGE_INC_SLOWLY(Present_voltage,Set_output_voltage);
  		  			}
  		   			Set_output_current = Modifier_Current;
  		   			if(Present_current != Set_output_current)
  		   			{
  		   				SLOW_CURRENT_INCREMENT(Present_current,Set_output_current);
  		   				CAN_RxTimer = 0;
  		   			}

  		   			CURRENT = CURR_SENSE(CURRENT_SENSE);

  		   			if(CURRENT>=CURRENT_0_5A)
  		   			{
  		   				FAN_ON ();
  		   			}
  			  		else
  			  		{
  			  			FAN_OFF ();
  			  		}

  		//******************************************Full_charge_checking***************************

  		   			if (Full_charge_check==1)
  		   			{
  		   			    LED_BLINK();
//  		   			    Full_CHARGE=0;
  		   				Full_charge_check=0;
  		   			   	CURRENT = CURR_SENSE(CURRENT_SENSE);
  		   			   	if(CURRENT <= BATTERY_CURRENT)//&&(Battery_voltage >= BATTERY_VOLTAGE))//FULL charge call
  		   			   	{

  		   			   		PWM_CURRENT_ZERO();	//CURRENT
  		   			   		Present_current=0;
  		   			   		Set_output_current =0;
  		   			   		Full_CHARGE=0;
  		   			   	    Pre_Full_Charge = 0;
  		   			   		MODE=FULL_CHARGE_MODE;
  		   			   		break;
  		   			   	}
  		   			}

  		   			if (Full_CHARGE >=5000)
  		   			{
  		   				Full_CHARGE=0;
  		   			   	Full_charge_check=1;
  		   			}
  		//******************************************************************************************
  		   		    LED_BLINK();
  		   			CAN_Process();
  		/************************* CAN Time out ********************************/

  		   			if(CAN_RxTimer >= 10000)
  		   			{
  		   				BMS_Data =0;
  		   				BMS_RECEIVE = 0;
  		   				MODE=CAN_TIME_OUT;
  		   				Charger_Fault.Communication_status = TRUE;
  		   				break;

  		   			}

  		   			PROTECTION_CALL();
  		   		//	AC_LED_ON();
  		   			Charger_Fault.Charging_status = TRUE;
  		   			break;
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		//											 ONE_AMP_MODE
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  		   	case ONE_AMP_MODE:

  		   		Charge_started = 1;
  		   		if(Time_out == 1)
  		   				{
  		   			   		MODE = FULL_CHARGE_MODE;
  		   			   		break;
  		   			   	}
  		   		if(Present_current != DEEP_DISCHARGE_CURRENT)
  		   			   	{
  		   					 SLOW_CURRENT_INCREMENT_ONE_AMP(Present_current,DEEP_DISCHARGE_CURRENT);
  		   			   	}
  		   		Present_current= DEEP_DISCHARGE_CURRENT;

  		   		if(Present_voltage != Set_output_voltage)
  		   			   	{
  		   					VOLTAGE_INC_SLOWLY_ONE_AMP(Present_voltage,Set_output_voltage);
  		   			   	}
  		   		Present_voltage = Set_output_voltage;

  		   		CURRENT = CURR_SENSE(CURRENT_SENSE);

  		   		if(CURRENT>=CURRENT_0_5A)
  		   		{
  		   			FAN_ON ();
  		   		}
  		   		else
  		   		{
  		   			FAN_OFF ();
  		   		}


  		   		AC_LED_ON();
  		   		LEDS_50_OFF();
  		   		LEDS_80_OFF();
  		   		LEDS_100_OFF();
  		   		if (LED_TIMER >= 500)
  				{
  					LED_TIMER = 0;
  					LED_30_TOGGLE();
  				}
  		   		//BUZZER(1);
  		   		OVP_AVG_VOLTGAE=OVP_AVG();
  		   		if((OVP_AVG_VOLTGAE > UNDER_VOLT_VALUE_ONEAMP)||(Hrs_Counter == 3)) //volt_under_oneamp) //under voltage protection
  		   		{
  		   			CAN_RxTimer =0;
  		   			if(BMS_RECEIVE == 0)
  		 			{
  		 				 //BUZZER(1);
  		 				 MODE = CAN_TIME_OUT;
  		 				 break;
  		 			}
  		 			else
  		 			{
  		 				 MODE = CC_CV_MODE;
  		 				 break;
  		 			}
  				   	AC_LED_ON();
  				   	LEDS_30_ON();
  		//		    BUZZER(2);
  				   	break;
  		 		}

  		/*******************CAN communication checking**************************/

  							// to send query 2
  						CAN_Process();

  					PROTECTION_CALL();

  		   		break;
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		//					PRE_FULL CHARGE_MODE
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		   	case PRE_FULL_CHARGE:

  		   	   			if(Pre_Full_Charge <= 300000)  //5 minute
  		   	   			{
  		   	   				TIMER_RESUME = 0;
  		   	   				SHUTDWON_ON();
  		   	   				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);	            //CURRENT
  		   	   				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);    //VOLTAGE
  		   	   				Present_voltage = 0;
  		   	   				Present_current = 0;
  		   	   			}
  		   	   			else if(Pre_Full_Charge > 300000)
  		   	   			{

  		   	   				Set_output_voltage = Modified_Voltage;
  		   	   				if(Present_voltage != Set_output_voltage)
  		   	   	   			{
  		   	   	   			   	VOLTAGE_INC_SLOWLY(Present_voltage,Set_output_voltage);
  		   	   	   			}
  		   	   	   			Present_voltage = Set_output_voltage;

  		   	   	   			if(Present_current != DEEP_DISCHARGE_CURRENT)
  		   	   	   			{
  		   	   	   				SLOW_CURRENT_INCREMENT(Present_current,DEEP_DISCHARGE_CURRENT);
  		   	   	   			}
  		   	   	   			Present_current= DEEP_DISCHARGE_CURRENT;


  		   	   	   			while(1)
  		   	   	   			{
  		   	   	   				DELAY(100);
  		   	   	   		 //  			   	if(State_of_Charge >= 100)//FULL charge call
  		   	   	   				CURRENT = CURR_SENSE(CURRENT_SENSE);
  		   	   	   				if( CURRENT >= CURRENT_0_5A )
  		   	   	   				{
  		   	   	   					Pre_Fullcharge_Iterations ++;
  		   	   	   					if(Pre_Fullcharge_Iterations < 3)
  		   	   	   						MODE = CC_CV_MODE;
  		   	   	   					else
  		   	   	   						MODE = FULL_CHARGE_MODE;
  		   	   	   					    break;
  		   	   	   				}
  		   	   	   				else
  		   	   	   				{
  		   	   	   					Pre_Fullcharge_Checking++;
  		   	   	   					if(Pre_Fullcharge_Checking >= 3)
  		   	   	   					{
  		   	   	   						MODE = FULL_CHARGE_MODE;
  		   	   	   						break;
  		   	   	   					}
  		   	   	   				//else
  		   	   	   				//{
  		   	   	   				//	Pre_Full_Charge = 0;
  		   	   	   				//}
  		   	   	   				}
  		   	   	   			}// while ends
  		   	   			}

  		   	   			LED_BLINK();
  		   	   			CAN_Process();
  		   	   			PROTECTION_CALL();

  		   	   			break;

  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		//											FULLCHARGE MODE
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		   		case FULL_CHARGE_MODE:

  		   			General_timer = 0;
  		   			if(Full_Charge_start == 0)
  		   			{
  		   				while(1)
  		   				{
  		   				BUZZER_ON();
  		   				if(General_timer >= 2000)
  		   					{
  		   					    BUZZER_OFF();
  		   					    Full_Charge_start = 1;
  		   					    break;
  		   					}
  		   				}
  		   			}

  		   		    OFF_CONDITION();
  		            LED_ON();
  		   		    AC_LED_OFF();
  		   			OVP_AVG_VOLTGAE=OVP_AVG();
  		   		    if(OVP_AVG_VOLTGAE > OVER_VOLT_VALUE_PROTECTION) //over voltage protection
  		   		        {
  		   		            OVP_BIT=1;
  		   		            if(OVP_TMR >= 50)//100
  		   		            {
  		   		            	PWM_CURRENT_ZERO();	//CURRENT
  		   		            	MODE=OVR_VOLT_MODE;
  		   		            }
  		   		        }
  		   		        else
  		   		        {
  		   		            OVP_BIT=0;
  		   		            OVP_TMR=0;
  		   		        }
  		   	   	    //CAN_Process();

  		   		    break;
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		//											SHORT CKT MODE
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		   		case SHORT_CKT_MODE:
  		   		    SHUTDOWN_HIGH_FUN();
  		   		    //OFF_CONDITION();
  		   		    Charger_Fault.Output_short_circuit = TRUE;
  		   		    Charger_Fault.Charging_status = FALSE;
  		   		    if(LED_TIMER >= 20000)
  		   		    {
  		   				LED_OFF();
  		   				AC_LED_BLINK (6);
  		   				LED_TIMER=0;
  		   		    }

  		   		    //CAN_Process();

  		   		    break;

  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		//							OVER CURRENT MODE
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		   		case OVER_CURRENT_MODE:

  		   		    OVC_BIT=0;
  		   		    OVC_TMR=0;
  		   		    //SHUTDOWN_HIGH_FUN();
  		   		    Charger_Fault.Output_over_current = TRUE;
  		   		    Charger_Fault.Charging_status = FALSE;
  		   		    OFF_CONDITION();
  		  		  	if(LED_TIMER >= 20000)
  		   		   	{

  		                LED_OFF();
  		                AC_LED_BLINK (5);
  		                LED_TIMER=0;
  		   		   	}

  		   	   	    //CAN_Process();

  		   		    break;
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		//							OVER VOLTAGE MODE
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		   		case OVR_VOLT_MODE:

  		   		    OVP_BIT=0;
  		   		    OVP_TMR=0;

  		   		    Charger_Fault.Output_over_voltage = TRUE;
  		   		   	Charger_Fault.Charging_status = FALSE;
  		   		    SHUTDOWN_HIGH_FUN();
  		   		    //OFF_CONDITION();
  		   		    if(LED_TIMER >= 20000)
  		   		    {
  		   		    	LED_OFF();
  		   				AC_LED_BLINK (2);
  		   				LED_TIMER=0;
  		   		    }
  		   	   	    //CAN_Process();

  		   		    break;
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		//						   OVER TEMPERATURE MODE
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		   		case OVR_TEMP_MODE:
  		   		    OVT_BIT=0;
  		   		    OVT_TMR=0;
  		   		    OFF_CONDITION();
  		  		    if(LED_TIMER >= 20000)
  		   		    {
  		   				LED_OFF();
  		   				AC_LED_BLINK (4);
  		   				LED_TIMER=0;
  		   		    }

  		   		    if(0)//(TEMPERATURE < REON_TEMP_VAL)
  		   		    {
  		   		    	OVT_REON_BIT=1;
  		   		    	if(OVT_REON_TMR > 1500)
  		   		    	{
  		   		    		OVT_REON_BIT=0;
  		   		    		OVT_REON_TMR=0;
  		   		    		MODE=INITIAL_MODE;
  		   		    		break;
  		   		    	}
  		   		    }
  		   		    else
  		   		    {
  		   		        OVT_REON_BIT=0;
  		   		        OVT_REON_TMR=0;
  		   		    }

  		      	    CAN_Process();

  		   		    break;

  		 //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		 //											 AC RECOVERY MODE
  		 //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  		   		case AC_RECOVERY_MODE:

  		   			LED_ON();
  				    AC_LED_ON();
  				    DELAY(250);
  				    AC_LED_OFF();
  				    LED_OFF();
  				    DELAY(250);
  				    //BUZZER(1);
  		   		    SHUTDOWN_HIGH_FUN();

  		   		    ACVOLTAGE = AC_VOLT_SENSE(AC_SENSE);
  		   		//  PROTECTION_CALL(); //Going in CC mode when AC voltage in the operating range
  		   		    if((ACVOLTAGE < AC_HIGH_VOLTAGE) && (ACVOLTAGE > AC_LOW_VOLTAGE))
  		   		    {
  		   		    	Full_CHARGE=0;
  		   		    	//BUZZER (1);
  		   		    	RELAY_ON();
  		   		    	SHUTDWON_OFF();
  		   		    	MODE = MODE_Backup;
  		   		    	CAN_RxTimer = 0;
  		   		    	AC_OVP=0;
  		   		    	Present_current = 0;
  		   		    	Set_output_current = 0;
  		   		    }

  		   		    break;
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		//											 IDLE_MODE MODE
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


  		   		case IDLE_MODE:

  		   			if (LED_TIMER >= 10000)
  		   		    {
  		   				LED_OFF ();
  		   				AC_LED_TOGGLE();
  		   				LED_TIMER = 0;
  		   		    }
  		   			Charger_Fault.Charging_status = FALSE;
  		   			SHUTDOWN_HIGH_FUN();
  		   		    //CAN_Process();

  		   	    break;
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		//								CHG OFF MODE
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		   		case CHG_OFF_MODE:

  		   			AC_LED_OFF();
  		   			LED_OFF();
  		   		   	Charger_Fault.Charging_status = FALSE;
  		   		   	SHUTDOWN_HIGH_FUN();
  		   		   	//CAN_Process();

  		   	    break;

  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		//											 CAN_time_out
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  		   		case CAN_TIME_OUT :

  		   			Charger_Fault.Communication_status = TRUE;
  		   		   SHUTDOWN_HIGH_FUN();
  		   			if(LED_TIMER >= 20000)
  		   		    {
  		   				LED_OFF();
  		   				AC_LED_BLINK (3);
  		   				LED_TIMER=0;
  		   		    }



  		/*******************CAN communication checking**************************/

  		   		Received_BMS.Received_Content_ID_100 = 0;
  		   		   			CAN_Recovery();
  		   		   			if(Received_BMS.Received_Content_ID_100 == 1)
  		   		   			{

  		   						/*OVP_AVG_VOLTGAE = OVP_AVG();
  		   						if (OVP_AVG_VOLTGAE > UNDER_VOLT_VALUE_ONEAMP) {
  		   							MODE = CC_CV_MODE;
  		   							Present_current = 0;
  		   							Set_output_current = 0;
  		   							//RELAY_ON();

  		   						}
  		   						else if (OVP_AVG_VOLTGAE <= UNDER_VOLT_VALUE_ONEAMP) {
  		   							MODE = ONE_AMP_MODE;
  		   							Present_current = 0;
  		   							Set_output_current = 0;
  		   							//RELAY_ON();

  		   						}
  		   						else {*/
  		   							MODE = CC_CV_MODE;
  		   							Present_current = 0;
  		   							Set_output_current = 0;
  		   						//}
  		   		   				SHUTDWON_OFF();
  		   						Power_on_check = 0;
  		   						BMS_RECEIVE = 0;
  		   						RELAY_ON();
  		   					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Modified_Current * 0.1);
  		   		   				break;
  		   		   			}

  		  			break;


  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  		//											 UNDER_VOLT_MODE
  		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


  		   		case UNDER_VOLT_MODE:

  		   			OVP_AVG_VOLTGAE=OVP_AVG();
  		   			SHUTDOWN_HIGH_FUN();

  		   			if(LED_TIMER >= 20000)
  		   		    {
  		   				LED_OFF();
  		   				AC_LED_BLINK (7);
  		   				LED_TIMER=0;
  		   		    }

  		   			if(Counter1000msec >= 1000)
  		   			{
  		   				Charger_Fault.Output_under_voltage = TRUE;
  		   			    Charger_Fault.Charging_status = FALSE;
  		   			    CAN_Send_Fault();
  		   			    Counter1000msec =0;
  		   			}

  		   			if(OVP_AVG_VOLTGAE >= UNDER_VOLT_VALUE_PROTECTION)
  		   			{
  		   				Charger_Fault.Output_under_voltage = FALSE;
  		   			}

  		   			//CAN_Process();

  		   			break;

  		   		default:
  		   			break;

  		   		}//Switch End

  		       /* USER CODE END WHILE */

  		       /* USER CODE BEGIN 3 */
  		   	}
   	return 1;
     /* USER CODE END 3 */
}

   /* Set the data to be transmitted */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;		//6
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
          {
            // ADC calibration error handling
            Error_Handler();
          }

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger =  DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;//DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */

/***************************250 CAN BAUDRATE******************/
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = ENABLE;
	hfdcan1.Init.TransmitPause = ENABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	hfdcan1.Init.NominalPrescaler =2;//1;
	hfdcan1.Init.NominalSyncJumpWidth =20 ;//16;
	hfdcan1.Init.NominalTimeSeg1 = 107;//47//63
	hfdcan1.Init.NominalTimeSeg2 = 20;
	hfdcan1.Init.DataPrescaler = 1;
	hfdcan1.Init.DataSyncJumpWidth = 24;
	hfdcan1.Init.DataTimeSeg1 = 39;
	hfdcan1.Init.DataTimeSeg2 = 24;
	hfdcan1.Init.StdFiltersNbr = 1;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}

  /************************500 CAN BAUD ********************************/
//	  hfdcan1.Instance = FDCAN1;
//	  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
//	  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
//	  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
//	  hfdcan1.Init.AutoRetransmission = ENABLE;
//	  hfdcan1.Init.TransmitPause = ENABLE;
//	  hfdcan1.Init.ProtocolException = DISABLE;
//	  hfdcan1.Init.NominalPrescaler =2;//1;
//	  hfdcan1.Init.NominalSyncJumpWidth = 16;
//	  hfdcan1.Init.NominalTimeSeg1 = 47;//63;
//	  hfdcan1.Init.NominalTimeSeg2 = 16;
//	  hfdcan1.Init.DataPrescaler = 1;
//	  hfdcan1.Init.DataSyncJumpWidth = 4;
//	  hfdcan1.Init.DataTimeSeg1 = 5;
//	  hfdcan1.Init.DataTimeSeg2 = 4;
//	  hfdcan1.Init.StdFiltersNbr = 1;
//	  hfdcan1.Init.ExtFiltersNbr = 0;
//	  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
//	  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
//	  {
//	    Error_Handler();
//	  }
  /************************************************************************/
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 256;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 300;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0XFFF; //256;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 300;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 1000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 4-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
	/* USER CODE BEGIN USART1_Init 0 */
		/* USER CODE END USART1_Init 1 */
	    huart1.Instance = USART1;
	    huart1.Init.BaudRate = 115200;
	    huart1.Init.WordLength = UART_WORDLENGTH_8B;
	    huart1.Init.StopBits = UART_STOPBITS_1;
	    huart1.Init.Parity = UART_PARITY_NONE;
	    huart1.Init.Mode = UART_MODE_TX_RX;
	    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	    if (HAL_UART_Init(&huart1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
		    != HAL_OK)
	  {
	    Error_Handler();
	  }
	    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
		    != HAL_OK)
	  {
	    Error_Handler();
	  }
	    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
	  {
	    Error_Handler();
	  }


  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHUTDOWN_Pin|FAN_Pin|LED_100__Pin|RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, AC_LED_Pin|LED_30__Pin|LED_50__Pin|LED_80__Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : REVERSE__Pin */
  GPIO_InitStruct.Pin = REVERSE__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(REVERSE__GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins :  AGING_Pin */
  GPIO_InitStruct.Pin = AGING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SHUTDOWN_Pin FAN_Pin LED_100__Pin RELAY_Pin */
  GPIO_InitStruct.Pin = SHUTDOWN_Pin|FAN_Pin|LED_100__Pin|RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : AC_LED_Pin LED_30__Pin LED_50__Pin LED_80__Pin */
  GPIO_InitStruct.Pin = AC_LED_Pin|LED_30__Pin|LED_50__Pin|LED_80__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin */
   GPIO_InitStruct.Pin = BUZZER_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   /*Configure GPIO pins : CAN_SLEW RATE */
     GPIO_InitStruct.Pin = GPIO_PIN_5;
       GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
       HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/************************************************
 * Function Name: void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: GPIO Interrupt
 * Notes:
 * Usage: HAL_GPIO_EXTI_Callback(GPIO_Pin);
 ************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	HAL_GPIO_WritePin(GPIOD, AC_LED_Pin, GPIO_PIN_RESET);
//	if (GPIO_Pin == BATT_REVERSE_Pin)
//	{
//		if(HAL_GPIO_ReadPin(GPIOC, BATT_REVERSE_Pin) == 1)
//		{
////			if(LED_TIMER >= 100)
////			{
////				LED_TIMER=0;
////				HAL_GPIO_WritePin(GPIOA, LED_30__Pin, GPIO_PIN_SET);
////				HAL_GPIO_WritePin(GPIOA, LED_50__Pin, GPIO_PIN_SET);
////				HAL_GPIO_WritePin(GPIOA, LED_80__Pin, GPIO_PIN_SET);
////				HAL_GPIO_WritePin(GPIOA, LED_100__Pin, GPIO_PIN_SET);
////				HAL_GPIO_TogglePin(GPIOD, AC_LED_Pin);
////			}
//			HAL_GPIO_WritePin(GPIOD, AC_LED_Pin, GPIO_PIN_RESET);
//			//HAL_GPIO_TogglePin(GPIOD, AC_LED_Pin);
//			//HAL_GPIO_WritePin(GPIOD, AC_LED_Pin, GPIO_PIN_RESET);
//			//__HAL_GPIO_EXTI_CLEAR_IT(AC_SENSE_Pin);
//		} else
//		{
//			__NOP();
//			//		HAL_GPIO_WritePin(GPIOB, AC_LED_Pin, GPIO_PIN_SET);
//		}
//	}
}

/************************************************
 * Function Name: void ADC_Select_CH0(uint32_t ADC_Channel, uint8_t val)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: ADC Channel selection
 * Notes: Select channel 0
 * Usage: ADC_Select_CH1(ADC_Channel, 0);
 ************************************************/
//void ADC_Select_CH0(void)
void ADC_Select_CH0(uint32_t ADC_Channel, uint8_t val)
{
	ADC_ChannelConfTypeDef sConfig;// = { 0 };
	/** Configure for the BATTERY_SENSE ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_Channel;//ADC_CHANNEL_0;
	sConfig.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;//ADC_SAMPLETIME_71CYCLES_5;
	//sConfig.Rank = 0;	//ADC_RANK_CHANNEL_NUMBER;
	if(val == 1)
	  {
	 //   sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
		 sConfig.Rank = ADC_REGULAR_RANK_1;
	  }

	  else
	  {
	    sConfig.Rank = ADC_RANK_NONE;
	  }
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}
}

/************************************************
 * Function Name: void ADC_Select_CH1(uint32_t ADC_Channel, uint8_t val)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: ADC Channel selection
 * Notes: Select channel 1
 * Usage: ADC_Select_CH1(ADC_Channel, 1);
 ************************************************/
//void ADC_Select_CH1(void)
void ADC_Select_CH1(uint32_t ADC_Channel, uint8_t val)
{
	ADC_ChannelConfTypeDef sConfig;// = { 1 };
	/** Configure for the CHARGER_SENSE ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_Channel;//ADC_CHANNEL_1;
//	sConfig.Rank = 1;	//ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
	if(val == 1)
	{
		sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	}
	else
	{
		sConfig.Rank = ADC_RANK_NONE;
	}
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}
}

/************************************************
 * Function Name: void ADC_Select_CH2(uint32_t ADC_Channel, uint8_t val)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: ADC Channel selection
 * Notes: Select channel 2
 * Usage: ADC_Select_CH2(ADC_Channel, 1);
 ************************************************/
//void ADC_Select_CH2(void)
void ADC_Select_CH2(uint32_t ADC_Channel, uint8_t val)
{
	ADC_ChannelConfTypeDef sConfig;// = { 2 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_Channel;//ADC_CHANNEL_2;
//	sConfig.Rank = 2;	//ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;
	if(val == 1)
	{
		sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	}
	else
	{
		sConfig.Rank = ADC_RANK_NONE;
	}
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}
}

/************************************************
 * Function Name: void ADC_Select_CH3(uint32_t ADC_Channel, uint8_t val)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: ADC Channel selection
 * Notes: Select channel 3
 * Usage: ADC_Select_CH2(ADC_Channel, 3);
 ************************************************/
void ADC_Select_CH10(uint32_t ADC_Channel, uint8_t val)
{
	ADC_ChannelConfTypeDef sConfig;// = { 3 };
//	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//	 */
	sConfig.Channel = ADC_Channel;//ADC_CHANNEL_3;
//	sConfig.Rank = 3;	//ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
	if(val == 1)
	{
		sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	}
	else
	{
		sConfig.Rank = ADC_RANK_NONE;
	}
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}
}

/************************************************
 * Function Name: unsigned int CHAG_GETADC(uint32_t ADC_Channel)
 * Returns: Unsigned integer
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains ADC Value of Charger Sense Channel
 * Notes: ADC value of Charger Sense Voltage
 * Usage:CHAG_GETADC(ADC_Channel);
 ************************************************/
//unsigned int CHAG_GETADC(unsigned char ADC_Channel)
unsigned int CHAG_GETADC(uint32_t ADC_Channel)
{
	uint32_t Result;
//	unsigned char k;
	uint32_t k;
	Result = 0;
//	ADC_Select_CH0 ();
//	ADC_Select_CH1();
	ADC_Select_CH1(ADC_Channel, 1);
	HAL_ADCEx_Calibration_Start(&hadc1);
	for (k = 0; k < NO_SAMPLES; k++)
	{
		//--Result += ADC_GetConversion(ADC_Channel);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);//1);
		Result += HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}
	Result = (Result >> DIV_VAL);
	ADC_Select_CH1(ADC_Channel, 0);
	return Result;
}

/************************************************
 * Function Name: unsigned int BATT_GETADC(uint32_t ADC_Channel)
 * Returns: Unsigned integer
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains ADC Value of Battery Sense Voltage
 * Notes: ADC value of Battery Sense Voltage
 * Usage: BATT_GETADC(ADC_Channel);
 ************************************************/
//unsigned int BATT_GETADC(unsigned char ADC_Channel)
unsigned int BATT_GETADC(uint32_t ADC_Channel)
{

	uint32_t Result;
//	unsigned char k;
	uint32_t k;
	Result = 0;
//	ADC_Select_CH0();
	ADC_Select_CH0(ADC_Channel, 1);
	//ADC_Select_CH1 ();
	HAL_ADCEx_Calibration_Start(&hadc1);
	for (k = 0; k < NO_SAMPLES; k++)
	{

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);//1);
		Result+= HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}
	Result = (Result >> DIV_VAL);




	ADC_Select_CH0(ADC_Channel, 0);
	return Result;
}

/************************************************
 * Function Name: unsigned int CURR_GETADC(uint32_t ADC_Channel)
 * Returns: Unsigned integer
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains ADC Value of Current sense Channel
 * Notes: ADC value of Output Current Sense
 * Usage: CURR_GETADC(ADC_Channel);
 ************************************************/
//unsigned int CURR_GETADC(unsigned char ADC_Channel)
unsigned int CURR_GETADC(uint32_t ADC_Channel)
{
//   	ADC_ChannelConfTypeDef sConfig = {0};
//   	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//   	*/
//	sConfig.Channel = ADC_Channel;//ADC_CHANNEL_0;
//	sConfig.Rank = 2;//ADC_RANK_CHANNEL_NUMBER;
//	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;//ADC_SAMPLETIME_239CYCLES_5;;
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}

	//unsigned int Result;
	uint32_t Result;
	//unsigned char k;
	uint32_t k;
	Result = 0;
	//ADC_Select_CH0 ();
	//ADC_Select_CH1 ();
	//ADC_Select_CH2();
	ADC_Select_CH2(ADC_Channel, 1);
	HAL_ADCEx_Calibration_Start(&hadc1);
	for (k = 0; k < NO_SAMPLES; k++)
	{
		//--Result += ADC_GetConversion(ADC_Channel);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);//1);
		Result+= HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}
	Result = (Result >> DIV_VAL);
	ADC_Select_CH2(ADC_Channel, 0);
	return Result;
}

/************************************************
 * Function Name: unsigned int AC_SENSE_GETADC(uint32_t ADC_Channel)
 * Returns: Unsigned integer
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains ADC Value of AC Sense Voltage
 * Notes: ADC value of AC Sense Voltage
 * Usage: AC_SENSE_GETADC(ADC_Channel);
 ************************************************/
//unsigned int BATT_GETADC(unsigned char ADC_Channel)
unsigned int AC_SENSE_GETADC(uint32_t ADC_Channel)
{

	uint32_t Result;
//	unsigned char k;
	uint32_t k;
	Result = 0;
//	ADC_Select_CH0();
	ADC_Select_CH0(ADC_Channel, 1);
	//ADC_Select_CH1 ();
	HAL_ADCEx_Calibration_Start(&hadc1);
	for (k = 0; k < NO_SAMPLES; k++)
	{

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);//1);
		Result+= HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}
	Result = (Result >> DIV_VAL);

	ADC_Select_CH10(ADC_Channel, 0);
	return Result;
}

/************************************************
 * Function Name: unsigned int CHAG_SENSE(uint32_t ADC_Channel)
 * Returns: Unsigned integer
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains ADC Voltage Value
 * Notes: ADC value of Charger Sense Voltage
 * Usage: CHAG_SENSE(Channel);
 ************************************************/
unsigned int CHAG_SENSE(uint32_t ADC_Channel)
{
//	unsigned int chag_volt;
//	chag_volt = BATT_GETADC(ADC_Channel);;//CHAG_GETADC(ADC_Channel);
//	chag_volt = (unsigned int) ((float) (((chag_volt*0.0237605)+0.7)*10));//40V = 1654,Ex = ((1654*x)+0.7)*10) So x = 0.0237605
//	return chag_volt;
	return Live_Voltage;
}

/************************************************
 * Function Name: unsigned int CURR_SENSE(uint32_t ADC_Channel)
 * Returns: Unsigned integer
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains ADC Current Value
 * Notes: ADC value of Output Current Sense
 * Usage: CURR_SENSE(Channel);
 ************************************************/
unsigned int CURR_SENSE(uint32_t ADC_Channel)
{
//	unsigned int curr;
//	curr = BATT_GETADC(ADC_Channel);//CURR_GETADC(ADC_Channel);
//	curr = (unsigned int) ((float) (curr * 0.0350877));//1A = 285, 285 * x = 10 => x = 0.0350877
//	return curr;
	return Live_Current;
}

/************************************************
 * Function Name: unsigned int BATT_SENSE(uint32_t ADC_Channel)
 * Returns: Unsigned integer
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains ADC Voltage Value
 * Notes: ADC value of Battery Sense Voltage
 * Usage: BATT_SENSE(Channel);
 ************************************************/
unsigned int BATT_SENSE(uint32_t ADC_Channel)
{
//	unsigned int batt_volt;
//
//	batt_volt = BATT_GETADC(ADC_Channel);
//
//	batt_volt = (unsigned int) ((float) (((batt_volt*0.0237605)+0.7)*10));//40V = 1654,Ex = ((1654*x)+0.7)*10) So x = 0.0237605//(VOLTAGE/ADC)//EX-400/2244 = 0.1782531//(((float)(volt*0.1782531)+0.7)*10);
//	return batt_volt;
	return Live_Voltage;
}

/************************************************
 * Function Name: unsigned int BATT_SENSE(uint32_t ADC_Channel)
 * Returns: Unsigned integer
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains ADC Voltage Value
 * Notes: ADC value of Battery Sense Voltage
 * Usage: BATT_SENSE(Channel);
 ************************************************/
unsigned int AC_VOLT_SENSE(uint32_t ADC_Channel)
{
	unsigned int ac_sense_volt;

	ac_sense_volt = AC_SENSE_GETADC(ADC_Channel);

	//ac_sense_volt = (unsigned int) ((float) (((ac_sense_volt*0.0237605)+0.7)*10));//40V = 1654,Ex = ((1654*x)+0.7)*10) So x = 0.0237605//(VOLTAGE/ADC)//EX-400/2244 = 0.1782531//(((float)(volt*0.1782531)+0.7)*10);
	return ac_sense_volt;
}
/************************************************
 * Function Name: unsigned int TEMP_SENSE(unsigned char Channel)
 * Returns: Unsigned integer
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains ADC Temperature Value of ADC Channel
 * Notes: ADC value of Output Current Sense
 * Usage: TEMP_SENSE(Channel);
 ************************************************/
unsigned int TEMP_SENSE(uint32_t ADC_Channel)
{
	unsigned int temp;
	temp = BATT_GETADC(ADC_Channel);
	//temp = (unsigned int) ((float) (temp));
	return temp;
}

/************************************************
 * Function Name: unsigned int OVP_AVG()
 * Returns: Unsigned integer
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description:Store average value of Over Voltage
 * Notes:Store Battery Sense Voltage
 * Usage:OVP_AVG()
 ************************************************/
unsigned int OVP_AVG()
{
 unsigned int  Result;
	unsigned char k;
	Result = 0;
	for ( k=0; k < OVP_SAMPLE ; k++)
	{
		DELAY(1);//100
        Result += BATT_SENSE(BATTERY_SENSE);
//        HAL_IWDG_Refresh(&hiwdg);
	}
	Result = (Result >> OVP_DIV);
	return Result;
}

uint8_t PROTECTION_CALL(void)
{
	uint8_t Error;
    Error = 0;
//    DELAY(300);
	OVP_AVG_VOLTGAE=OVP_AVG();
	CURRENT = CURR_SENSE(CURRENT_SENSE);

//	if(OVP_AVG_VOLTGAE < UNDER_VOLT_VALUE_PROTECTION) //under voltage protection
//	{
//		UVT_BIT=1;
//		if(UVT_TMR >= 1000)//100
//		{
//			PWM_CURRENT_ZERO();	//CURRENT
//			Present_current=0;
//			Set_output_current =0;
//			MODE=UNDER_VOLT_MODE;
//			Error =1;
//		}
//	}
//	else
//	{
//		UVT_BIT=0;
//		UVT_TMR=0;
//	}


	if(OVP_AVG_VOLTGAE > OVER_VOLT_VALUE_PROTECTION) //over voltage protection
	{
	    OVP_BIT=1;
	    if(OVP_TMR >= 1000)//100
	    {
	    	PWM_CURRENT_ZERO();	//CURRENT
	    	Present_current=0;
	    	Set_output_current =0;
	    	MODE=OVR_VOLT_MODE;
	    	Error =1;
	    }
	}
	else
	{
	    OVP_BIT=0;
	    OVP_TMR=0;
	}


	if(CURRENT > OVER_CURRENT_VALUE_PROTECTION) //over current protection
	{
	    OVC_BIT=1;
	    if(OVC_TMR >= 1000)
	    {
	    	PWM_CURRENT_ZERO();	//CURRENT
	    	Present_current=0;
	    	Set_output_current=0;
	    	MODE=OVER_CURRENT_MODE;
	    	Error =1;
	    }
	}
	else
	{
	   OVC_BIT=0;
	   OVC_TMR=0;
	}

	if(0)//(TEMPERATURE > OVR_TEMP_VAL) //over temperature protection
	{
	    OVT_BIT=1;
	    if(OVT_TMR >= 100)
	    {
	    	PWM_CURRENT_ZERO();	//CURRENT
	    	Present_current=0;
	    	Set_output_current=0;
	     MODE=OVR_TEMP_MODE;
	     Error =1 ;
	    }
	}
	else
	{
	    OVT_BIT=0;
	    OVT_TMR=0;
	}

	return Error;

}

void  AGING_PROTECTION_CALL(void)
{

	OVP_AVG_VOLTGAE=  Live_Voltage;
	CURRENT = CURR_SENSE(CURRENT_SENSE);

	if(OVP_AVG_VOLTGAE > OVER_VOLT_VALUE_PROTECTION) //over voltage protection
	{
	    OVP_BIT=1;
	if (OVP_TMR >= 2500) //100
	    {
		Present_current=0;
	    	Set_output_current =0;
	    	AGING_MODE_NO=AGING_OVR_VOLT_MODE;

	    }
	}
	else
	{
	    OVP_BIT=0;
	    OVP_TMR=0;
	}
    if (CURRENT > OVER_CURRENT_VALUE_PROTECTION) //over current protection
	{
	    OVC_BIT=1;
	if (OVC_TMR >= 2500)
	    {
	    	Present_current=0;
	    	Set_output_current=0;
	    	AGING_MODE_NO=AGING_OVER_CURRENT_MODE;
	    }
	}
	else
	{
	   OVC_BIT=0;
	   OVC_TMR=0;
	}
}

/************************************************
 * Function Name: CC_CV_AC_CALL(void)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains Timer Enable PWM1(Charger Voltage Set)
 and PWM2(Charger Current Set) Duty Cycle
 Store Battery Sense Voltage
 Store Current Sense Voltage
 LED ON, FAN ON
 * Notes:
 * Usage: CC_CV_AC_CALL();
 ************************************************/
//void CC_CV_AC_CALL(void)
//{
//	TIMER_RESUME = 1;
//	//--PWM2_LoadDutyValue(LIMIT_59_2VOLTS);//V setting
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LIMIT_65_5VOLTS + 2.5);//2.5//67V//5);68V
//	//--PWM1_LoadDutyValue(LIMIT_18A_CURRENT);//I setting
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, LIMIT_6A_CURRENT);//CURRENT
//	//--LED_CHARGER_PIN_LAT =ON;
//	//--VOLTAGE = VOLT_SENSE(BATTERY_SENSE_PIN);
//	//VOLTAGE = BATT_GETADC(ADC_CHANNEL_0);
//	VOLTAGE = BATT_SENSE(BATTERY_SENSE);
//	//--CURRENT = CURR_SENSE(CURRENT_SENSE_PIN);
////	CURRENT = CURR_SENSE(ADC_CHANNEL_2);
////	CURRENT = CURR_SENSE(CURRENT_SENSE);
//	//--FAN_PIN_LAT=1;
//	FAN_ON();
//	LED_BLINK();
//}

/************************************************
 * Function Name: void FULLCHARGE_CALL(void)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains Timer Enable PWM1(Charger Voltage Set)
 and PWM2(Charger Current Set) Duty Cycle
 Store Battery Sense Voltage
 Store Current Sense Voltage
 LED ON, FAN ON
 * Notes:
 * Usage: FULLCHARGE_CALL();
 ************************************************/
//void FULLCHARGE_CALL(void)
//{
//	TIMER_RESUME = 0;
//	//--SHUTDOWN_PIN_LAT=1;
//	SHUTDWON_ON();
//	//--PWM1_LoadDutyValue(0);//I setting
//	PWM_CURRENT_ZERO();                //CURRENT
//	//--PWM2_LoadDutyValue(0);//V setting
//	DAC_VOLTAGE_ZERO();    //VOLTAGE
//	//--__delay_ms(200);
//	Present_current=0;
//	Present_voltage =0;
//	Set_output_current =0;
//	Set_output_voltage =0;
//
//	DELAY(200);
//	//--RELAY_PIN_LAT=0;
//	RELAY_OFF();
//	//--LED_CHARGER_PIN_LAT = ON;
//	//--FAN_PIN_LAT=0;
//	FAN_OFF();
//	PULSE_MODEBIT = 0;
//	PULSE_OVER = 0;
//	ACCHECK_MODEBIT = 0;
//	ACCHECK_MODE = 0;
//}

/************************************************
 * Function Name: ALL_LEDS_OFF(void)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains 30% LED Blink Bit as Enable
 * Usage: ALL_LEDS_OFF();
 ************************************************/
void ALL_LEDS_OFF(void)
{
	BLINK_30_BIT = 1;
	BLINK_50_BIT = 0;
	BLINK_80_BIT = 0;
	BLINK_100_BIT = 0;
	CONST_100_BIT = 0;
	//--LED_30_PIN_LAT=OFF;
	//--LED_50_PIN_LAT=OFF;
	//--LED_80_PIN_LAT=OFF;
	//--LED_100_PIN_LAT=OFF;
	LED_OFF();
	LED_TIMER = 0;
}
/************************************************
 * Function Name: ALL_LEDS_OFF(void)
 * Returns: Nothing
 * Created By: RAM SINGH
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: ALL LED OFF state
 * Usage: ALL_LEDS_OFF();
 ************************************************/
void LED_OFF(void)
{

	HAL_GPIO_WritePin(GPIOD, LED_30__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LED_50__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LED_80__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, LED_100__Pin, GPIO_PIN_SET);

}
void LED_ON(void)
{

	HAL_GPIO_WritePin(GPIOD, LED_30__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED_50__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED_80__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LED_100__Pin, GPIO_PIN_RESET);

}





/************************************************
 * Function Name: OFF_CONDITION(void)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains Shutdown Enable,PWM1 Off,PWM2 Off
 Output Relay OFF,Charger LED OFF,FAN OFF
 * Usage: Charger is in OFF condition
 ************************************************/
void OFF_CONDITION(void)
{
	TIMER_RESUME = 0;
	BUZZER_OFF();
	SHUTDWON_ON();
	PWM_CURRENT_ZERO();
	DAC_VOLTAGE_ZERO();

	Present_current=0;
	Present_voltage =0;
	Set_output_current =0;
	Set_output_voltage =0;

	DELAY(200);
	RELAY_OFF();
    FAN_OFF();
}

/************************************************
 * Function Name: void INIT_COND_FUN_CALL(void)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains Shutdown Enable,PWM1 Off,PWM2 Off
 Output Relay OFF,Charger LED OFF,FAN OFF
 * Usage: Charger is in OFF condition
 ************************************************/
void INIT_COND_FUN_CALL(void)
{

	SHUTDWON_ON();
	TIMER_RESUME = 0;
	RELAY_OFF();
	OVER_ALL_TIMER = 0;
	TIMER_ROLL = 0;
	EMERGENCY_SHUTDOWN_TIME_BIT = 0;
	EMERGENCY_SHUTDOWN_TIME = 0;
	REON_CHECK_TIME_BIT = 0;
	REON_CHECK_TIME = 0;
	PWM_CURRENT_ZERO();
	DAC_VOLTAGE_ZERO();
	AC_LED_ON();
	FAN_OFF ();
	BUZZER_OFF();
	ACCHECK_MODEBIT = 0;
	ACCHECK_MODE = 0;
	AGING_MODE = 0;
	ITARATIONS = 0;
	AGING_NORMAL = 0;
	ONE_TIME_EXE_BIT=0;
	NO_BAT_DETECT_TIMES = 0;
	LED_OFF();
	LEDS_100_OFF() ;
}

/************************************************
 * Function Name: void MIN_CURR_FUN(void)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Charger is in OFF Condition,LEDs OFF
 * Usage: MIN_CURR_FUN();
 ************************************************/
void MIN_CURR_FUN(void)
{
	ITARATIONS++;
	ALL_LEDS_OFF();
	ACCHECK_MODEBIT = 0;
	ACCHECK_MODE = 0;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);	                //CURRENT
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);		//VOLTAGE
	DELAY(200);
	HAL_GPIO_WritePin(GPIOB, RELAY_Pin, GPIO_PIN_RESET);
	FAN_OFF();
	DELAY(200);
	VOLTAGE = BATT_SENSE(BATTERY_SENSE);
}

/************************************************
 * Function Name: void SHUTDOWN_HIGH_FUN(void)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Charger is in OFF Condition,LEDs OFF,Relay OFF
 * Usage: SHUTDOWN_HIGH_FUN();
 ************************************************/
void SHUTDOWN_HIGH_FUN(void)
{
	TIMER_RESUME = 0;
	SHUTDWON_ON();
//	ALL_LEDS_OFF();
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);	            //CURRENT
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);    //VOLTAGE
	Present_voltage = 0;
	Present_current = 0;
	DELAY(200);
    RELAY_OFF();
	FAN_OFF();

	Present_voltage = 0;
	Present_current = 0;
}

/************************************************
 * Function Name: void BUZZER(unsigned char SHORT_BEEPS)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Buzzer Beeps
 * Usage: BUZZER(x); x = 1,2,3,..
 ************************************************/
void BUZZER(unsigned char SHORT_BEEPS)
{
	unsigned char BEEP_TEMP;
	if(1)    //BUZZER_TIMER > 150)
	{
		BUZZER_OFF();
		for (BEEP_TEMP = 0; BEEP_TEMP < (SHORT_BEEPS * 2); BEEP_TEMP++)
		{
			BUZZER_TIMER = 1001;
			BUZZER_TOGGLE();
			DELAY(250);
		}
		BUZZER_TIMER = 0;
		BUZZER_OFF();
	}
}

/************************************************
 * Function Name: void SLOW_CURRENT_INCREMENT(unsigned int INIT_CURR,unsigned int END_CURRENT_VAL)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains Enable PWM2 with duty cycle
 module for slow current increment
 Store the initial current and final value current
 * Usage: SLOW_CURRENT_INCREMENT(INIT_CURR,END_CURRENT_VAL);
 * Notes:
 ************************************************/
void SLOW_CURRENT_INCREMENT(unsigned int INIT_CURR,unsigned int END_CURRENT_VAL)
{
	unsigned int INCREMENT_VAL;
	INCREMENT_VAL = INIT_CURR;
	//FAN_OFF();
	Present_current = END_CURRENT_VAL;

	if(INCREMENT_VAL < END_CURRENT_VAL)
	{
		while (INCREMENT_VAL < END_CURRENT_VAL) // && (AC_SENSE == 1))// && (OVER_CURRENT_BIT == 0))
		{
			//--CLRWDT();
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, INCREMENT_VAL);   //CURRENT
			DELAY(5);

			CURRENT = CURR_SENSE (CURRENT_SENSE);

			if (CURRENT >= CURRENT_0_5A)
			{
			  FAN_ON();
			}
			else
			{
			  FAN_OFF();
			}
	//		CAN_RX();
			if( INCREMENT_VAL >= OVER_CURRENT_VALUE)
			{
				 DAC_VOLTAGE_ZERO();
				 MODE= OVER_CURRENT_MODE;
				 OVC_BIT=1;
				 return ;
			}
			INCREMENT_VAL = INCREMENT_VAL+5;
			LED_BLINK ();
			DELAY(5);
		}
		INCREMENT_VAL = END_CURRENT_VAL;
	}

	if(INCREMENT_VAL > END_CURRENT_VAL)
	{
		while (INCREMENT_VAL > END_CURRENT_VAL) // && (AC_SENSE == 1))// && (OVER_CURRENT_BIT == 0))
		{
			//--CLRWDT();
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, INCREMENT_VAL);   //CURRENT
			DELAY(20);
//			CAN_RX();
			INCREMENT_VAL--;
			DELAY(10);
			CURRENT = CURR_SENSE (CURRENT_SENSE);

						if (CURRENT >= CURRENT_0_5A)
						{
						  FAN_ON();
						}
						else
						{
						  FAN_OFF();
						}
		}
		INCREMENT_VAL = END_CURRENT_VAL;
	}



	if (1)    //(AC_SENSE == 1)
	{

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, END_CURRENT_VAL); //CURRENT
	} else
	{

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);    //CURRENT
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);    	//VOLTAGE
		//HAL_GPIO_WritePin(GPIOB, RELAY_Pin, GPIO_PIN_RESET);
	}
}


void SLOW_CURRENT_INCREMENT_ONE_AMP(unsigned int INIT_CURR,unsigned int END_CURRENT_VAL)
{
	unsigned int INCREMENT_VAL;
	INCREMENT_VAL = INIT_CURR;
	//FAN_OFF();
	Present_current = END_CURRENT_VAL;

	if(INCREMENT_VAL < END_CURRENT_VAL)
	{
		while (INCREMENT_VAL < END_CURRENT_VAL) // && (AC_SENSE == 1))// && (OVER_CURRENT_BIT == 0))
		{
			//--CLRWDT();
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, INCREMENT_VAL);   //CURRENT
			DELAY(150);

			CURRENT = CURR_SENSE (CURRENT_SENSE);

			if (CURRENT >= CURRENT_0_5A)
			{
			  FAN_ON();
			}
			else
			{
			  FAN_OFF();
			}
	//		CAN_RX();
			if( INCREMENT_VAL >= OVER_CURRENT_VALUE)
			{
				 DAC_VOLTAGE_ZERO();
				 MODE= OVER_CURRENT_MODE;
				 OVC_BIT=1;
				 return ;
			}
			INCREMENT_VAL++;
//			LED_BLINK ();
//			DELAY(10);
		}
		INCREMENT_VAL = END_CURRENT_VAL;
	}

	if(INCREMENT_VAL > END_CURRENT_VAL)
	{
		while (INCREMENT_VAL > END_CURRENT_VAL) // && (AC_SENSE == 1))// && (OVER_CURRENT_BIT == 0))
		{
			//--CLRWDT();
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, INCREMENT_VAL);   //CURRENT
			DELAY(20);
//			CAN_RX();
			INCREMENT_VAL--;
			DELAY(10);

		}
		INCREMENT_VAL = END_CURRENT_VAL;
	}



	if (1)    //(AC_SENSE == 1)
	{

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, END_CURRENT_VAL); //CURRENT
	} else
	{

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);    //CURRENT
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);    	//VOLTAGE
		//HAL_GPIO_WritePin(GPIOB, RELAY_Pin, GPIO_PIN_RESET);
	}
}


void AGING_SLOW_CURRENT_INCREMENT(unsigned int INIT_CURR,unsigned int END_CURRENT_VAL)
{

	unsigned int INCREMENT_VAL;
	INCREMENT_VAL = INIT_CURR;
	//FAN_OFF();
	Present_current = END_CURRENT_VAL;

	if(INCREMENT_VAL < END_CURRENT_VAL)
	{
		while (INCREMENT_VAL < END_CURRENT_VAL) // && (AC_SENSE == 1))// && (OVER_CURRENT_BIT == 0))
		{
			//--CLRWDT();
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, INCREMENT_VAL);   //CURRENT
			DELAY(2);

			CURRENT = CURR_SENSE (CURRENT_SENSE);


			if (CURRENT >= CURRENT_0_5A)
			{
			  FAN_ON();
			}
			else
			{
			  FAN_OFF();
			}
	//		CAN_RX();
			if( INCREMENT_VAL >= OVER_CURRENT_VALUE)
			{
				 DAC_VOLTAGE_ZERO();
				 MODE= OVER_CURRENT_MODE;
				 OVC_BIT=1;
				 return ;
			}
			INCREMENT_VAL = INCREMENT_VAL + 5;
//			LED_BLINK ();
			DELAY(2);
		}
		INCREMENT_VAL = END_CURRENT_VAL;
	}

	if(INCREMENT_VAL > END_CURRENT_VAL)
	{
		while (INCREMENT_VAL > END_CURRENT_VAL) // && (AC_SENSE == 1))// && (OVER_CURRENT_BIT == 0))
		{
			//--CLRWDT();
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, INCREMENT_VAL);   //CURRENT
			DELAY(20);
//			CAN_RX();
			INCREMENT_VAL--;
			DELAY(10);

		}
		INCREMENT_VAL = END_CURRENT_VAL;
	}



	if (1)    //(AC_SENSE == 1)
	{

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, END_CURRENT_VAL); //CURRENT
	} else
	{

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);    //CURRENT
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);    	//VOLTAGE
		//HAL_GPIO_WritePin(GPIOB, RELAY_Pin, GPIO_PIN_RESET);
	}
}




/************************************************
 * Function Name: void VOLTAGE_INC_SLOWLY(unsigned int INIT_VOLTAG,unsigned int END_VOLT_VAL)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains Enable PWM1 with duty cycle
 module for slow voltage increment
 Store the Initial Voltage and Final Value Voltage
 * Usage: VOLTAGE_INC_SLOWLY(INIT_VOLTAG, END_VOLT_VAL);
 * Notes:
 ************************************************/
void VOLTAGE_INC_SLOWLY(unsigned int INIT_VOLTAG, unsigned int END_VOLT_VAL)
{
	unsigned int INCREMENT_VAL;
//	uint8_t Error;
//	Error = 0;

	INCREMENT_VAL = INIT_VOLTAG;
	//--VOLTAGE=VOLT_SENSE(BATTERY_SENSE_PIN);
	//--CHARGER_VOLTAGE=VOLT_SENSE(CHARGER_SENSE_PIN);
	VOLTAGE = BATT_SENSE(BATTERY_SENSE);
	CHARGER_VOLTAGE = CHAG_SENSE(CHARGER_SENSE);
	Present_voltage = END_VOLT_VAL;
	DELAY(20);
	RELAY_ON();
    //--__delay_ms(20);
	DELAY(20);

  if(INCREMENT_VAL < END_VOLT_VAL)
  {
	while (INCREMENT_VAL < END_VOLT_VAL) //((AC_SENSE==1) && )// && OVER_CURRENT_BIT==0)//(CHARGER_VOLTAGE < VOLTAGE) &&
	{
/*		Error = PROTECTION_CALL();
		if(Error == 1)
		{
			return;
		}*/

		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, INCREMENT_VAL);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, INCREMENT_VAL);   //VOLTAGE
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	//	CHARGER_VOLTAGE = CHAG_SENSE(CHARGER_SENSE);

		DELAY(5);
//		CAN_RX();
		INCREMENT_VAL = INCREMENT_VAL + 2;
		if( INCREMENT_VAL >= OVER_VOLT_VALUE)
		{
				 DAC_VOLTAGE_ZERO();
				 MODE= OVR_VOLT_MODE;
				 OVP_BIT=1;
				 return ;
		}
		//PROTECTION_CALL();
		 LED_BLINK ();
	}
	INCREMENT_VAL = END_VOLT_VAL;

  }
  else if (INCREMENT_VAL > END_VOLT_VAL)
  {
	  while (INCREMENT_VAL > END_VOLT_VAL) //((AC_SENSE==1) && )// && OVER_CURRENT_BIT==0)//(CHARGER_VOLTAGE < VOLTAGE) &&
	  	{
	/*	    Error = PROTECTION_CALL();
		    if(Error == 1)
		    {
		    	return;
		    }*/
	  		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, INCREMENT_VAL);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, INCREMENT_VAL);   //VOLTAGE
	  		HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	  		//--__delay_ms(6);//5Sec//12//8.4sec
	  		DELAY(100);
//	  		CAN_RX();
	  		INCREMENT_VAL=INCREMENT_VAL-1;
	  		CHARGER_VOLTAGE = CHAG_SENSE(CHARGER_SENSE);
	  	}

	  	INCREMENT_VAL = END_VOLT_VAL;

  }
	if (1)		//((AC_SENSE==1) && OVER_CURRENT_BIT==0)
	{
		//--CLRWDT();

		//--PWM2_LoadDutyValue (INCREMENT_VAL+6);//V setting
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,END_VOLT_VAL);//INCREMENT_VAL+6);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, INCREMENT_VAL + 6);//VOLTAGE
		//--__delay_ms(10);
		DELAY(10);


	} else
	{
		//PWM1_LoadDutyValue(0); //current
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);		//CURRENT
		//--PWM2_LoadDutyValue(0); //V setting
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);		//VOLTAGE
		//--RELAY_PIN_LAT = 0;
		HAL_GPIO_WritePin(GPIOB, RELAY_Pin, GPIO_PIN_RESET);
	}

}



void VOLTAGE_INC_SLOWLY_ONE_AMP(unsigned int INIT_VOLTAG, unsigned int END_VOLT_VAL)
{
	unsigned int INCREMENT_VAL;
//	uint8_t Error;
//	Error = 0;

	INCREMENT_VAL = INIT_VOLTAG;
	//--VOLTAGE=VOLT_SENSE(BATTERY_SENSE_PIN);
	//--CHARGER_VOLTAGE=VOLT_SENSE(CHARGER_SENSE_PIN);
	VOLTAGE = BATT_SENSE(BATTERY_SENSE);
	CHARGER_VOLTAGE = CHAG_SENSE(CHARGER_SENSE);
	Present_voltage = END_VOLT_VAL;
	DELAY(20);
	RELAY_ON();
    //--__delay_ms(20);
	DELAY(20);

  if(INCREMENT_VAL < END_VOLT_VAL)
  {
	while (INCREMENT_VAL < END_VOLT_VAL) //((AC_SENSE==1) && )// && OVER_CURRENT_BIT==0)//(CHARGER_VOLTAGE < VOLTAGE) &&
	{
/*		Error = PROTECTION_CALL();
		if(Error == 1)
		{
			return;
		}*/

		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, INCREMENT_VAL);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, INCREMENT_VAL);   //VOLTAGE
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	//	CHARGER_VOLTAGE = CHAG_SENSE(CHARGER_SENSE);

		DELAY(3);
//		CAN_RX();
		 INCREMENT_VAL ++;
		if( INCREMENT_VAL >= OVER_VOLT_VALUE)
		{
				 DAC_VOLTAGE_ZERO();
				 MODE= OVR_VOLT_MODE;
				 OVP_BIT=1;
				 return ;
		}
		//PROTECTION_CALL();
	}
	INCREMENT_VAL = END_VOLT_VAL;
//      LED_BLINK ();
  }
  else if (INCREMENT_VAL > END_VOLT_VAL)
  {
	  while (INCREMENT_VAL > END_VOLT_VAL) //((AC_SENSE==1) && )// && OVER_CURRENT_BIT==0)//(CHARGER_VOLTAGE < VOLTAGE) &&
	  	{
	/*	    Error = PROTECTION_CALL();
		    if(Error == 1)
		    {
		    	return;
		    }*/
	  		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, INCREMENT_VAL);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, INCREMENT_VAL);   //VOLTAGE
	  		HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	  		//--__delay_ms(6);//5Sec//12//8.4sec
//	  		DELAY(100);
//	  		CAN_RX();
	  		INCREMENT_VAL=INCREMENT_VAL-1;
	  		CHARGER_VOLTAGE = CHAG_SENSE(CHARGER_SENSE);
	  	}

	  	INCREMENT_VAL = END_VOLT_VAL;

  }
	if (1)		//((AC_SENSE==1) && OVER_CURRENT_BIT==0)
	{
		//--CLRWDT();

		//--PWM2_LoadDutyValue (INCREMENT_VAL+6);//V setting
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,END_VOLT_VAL);//INCREMENT_VAL+6);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, INCREMENT_VAL + 6);//VOLTAGE
		//--__delay_ms(10);
		DELAY(10);


	} else
	{
		//PWM1_LoadDutyValue(0); //current
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);		//CURRENT
		//--PWM2_LoadDutyValue(0); //V setting
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);		//VOLTAGE
		//--RELAY_PIN_LAT = 0;
		HAL_GPIO_WritePin(GPIOB, RELAY_Pin, GPIO_PIN_RESET);
	}

}

void AGING_VOLTAGE_INC_SLOWLY(unsigned int INIT_VOLTAG,unsigned int END_VOLT_VAL)
    {
	unsigned int INCREMENT_VAL;
	//	uint8_t Error;
	//	Error = 0;

		INCREMENT_VAL = INIT_VOLTAG;
		//--VOLTAGE=VOLT_SENSE(BATTERY_SENSE_PIN);
		//--CHARGER_VOLTAGE=VOLT_SENSE(CHARGER_SENSE_PIN);
		VOLTAGE = BATT_SENSE(BATTERY_SENSE);
		CHARGER_VOLTAGE = CHAG_SENSE(CHARGER_SENSE);
		Present_voltage = END_VOLT_VAL;
		DELAY(20);
		RELAY_ON();
	    //--__delay_ms(20);
		DELAY(20);

	  if(INCREMENT_VAL < END_VOLT_VAL)
	  {
		while (INCREMENT_VAL < END_VOLT_VAL) //((AC_SENSE==1) && )// && OVER_CURRENT_BIT==0)//(CHARGER_VOLTAGE < VOLTAGE) &&
		{
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, INCREMENT_VAL);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, INCREMENT_VAL);   //VOLTAGE
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
		//	CHARGER_VOLTAGE = CHAG_SENSE(CHARGER_SENSE);

		  DELAY(10);
	//		CAN_RX();
			INCREMENT_VAL=INCREMENT_VAL+10;
			if( INCREMENT_VAL >= OVER_VOLT_VALUE)
			{
					 DAC_VOLTAGE_ZERO();
					 MODE= OVR_VOLT_MODE;
					 OVP_BIT=1;
					 return ;
			}
		}
		INCREMENT_VAL = END_VOLT_VAL;
	  }
	  else if (INCREMENT_VAL > END_VOLT_VAL)
	  {
		  while (INCREMENT_VAL > END_VOLT_VAL) //((AC_SENSE==1) && )// && OVER_CURRENT_BIT==0)//(CHARGER_VOLTAGE < VOLTAGE) &&
		  	{
		  		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, INCREMENT_VAL);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, INCREMENT_VAL);   //VOLTAGE
		  		HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
		  		//--__delay_ms(6);//5Sec//12//8.4sec
		  DELAY(10);
	//	  		CAN_RX();
		  		INCREMENT_VAL=INCREMENT_VAL-1;
		  		CHARGER_VOLTAGE = CHAG_SENSE(CHARGER_SENSE);
		  	}

		  	INCREMENT_VAL = END_VOLT_VAL;

	  }
		if (1)		//((AC_SENSE==1) && OVER_CURRENT_BIT==0)
		{
			//--CLRWDT();

	//		HAL_IWDG_Refresh(&hiwdg);
			//--PWM2_LoadDutyValue (INCREMENT_VAL+6);//V setting
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,END_VOLT_VAL);//INCREMENT_VAL+6);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, INCREMENT_VAL + 6);//VOLTAGE
			//--__delay_ms(10);
			DELAY(10);


		} else
		{
			//PWM1_LoadDutyValue(0); //current
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);		//CURRENT
			//--PWM2_LoadDutyValue(0); //V setting
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);		//VOLTAGE
			//--RELAY_PIN_LAT = 0;
			HAL_GPIO_WritePin(GPIOB, RELAY_Pin, GPIO_PIN_RESET);
		}
}

///////////////////////////////////////
void Voltage_Fine_tuning(unsigned int INIT_VOLTAG, unsigned int END_VOLT_VAL)
{
	unsigned int INCREMENT_VAL;
	INCREMENT_VAL = INIT_VOLTAG;
	//--VOLTAGE=VOLT_SENSE(BATTERY_SENSE_PIN);
	//--CHARGER_VOLTAGE=VOLT_SENSE(CHARGER_SENSE_PIN);


  if(INCREMENT_VAL < END_VOLT_VAL)
  {
	while (INCREMENT_VAL < END_VOLT_VAL) //((AC_SENSE==1) && )// && OVER_CURRENT_BIT==0)//(CHARGER_VOLTAGE < VOLTAGE) &&
	{

		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, INCREMENT_VAL);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, INCREMENT_VAL);   //VOLTAGE
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
		//--__delay_ms(6);//5Sec//12//8.4sec
		DELAY(5);
		CAN_RX();
		INCREMENT_VAL=INCREMENT_VAL+1;

	}

	INCREMENT_VAL = END_VOLT_VAL;
  }
  else if (INCREMENT_VAL > END_VOLT_VAL)
  {
	  while (INCREMENT_VAL > END_VOLT_VAL) //((AC_SENSE==1) && )// && OVER_CURRENT_BIT==0)//(CHARGER_VOLTAGE < VOLTAGE) &&
	  	{

	  		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, INCREMENT_VAL);//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, INCREMENT_VAL);   //VOLTAGE
	  		HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	  		//--__delay_ms(6);//5Sec//12//8.4sec
	  		DELAY(5);
	  		CAN_RX();
	  		INCREMENT_VAL=INCREMENT_VAL-1;

	  	}

	  	INCREMENT_VAL = END_VOLT_VAL;

  }
}

/************************************************
 * Function Name: void TEST_LED(void)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains 30%, 50%, 80% and 100%
 LED Blinking based on Battery Sense Voltage
 * Usage: LED_BLINK();
 * Notes:
// ************************************************/

void TEST_LED(void)
{
	while(1)
	{
	   		LED_ON();
	   		DELAY(100);
	   		LED_OFF();
	   		DELAY(100);
	   		SHUTDOWN_HIGH_FUN();
	}

}
/************************************************
 * Function Name: LED_BLINK(void)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains 30%, 50%, 80% and 100%
 LED Blinking based on Battery Sense Voltage
 * Usage: LED_BLINK();
 * Notes:
// ************************************************/
void LED_BLINK_AG(void)
{
	VOLTAGE = BATT_SENSE(BATTERY_SENSE);

	if (BLINK_30_BIT)
	{
		if (VOLTAGE >= LED30_CHANGE_POINT_AG)
		{
			if (LED_SENSE_VOLTAGE_TIME++ >= LED_BLINK_SENSE_TIME)
			{
				BLINK_30_BIT = 0;
				BLINK_50_BIT = 1;
				BLINK_80_BIT = 0;
				BLINK_100_BIT = 0;
				CONST_100_BIT = 0;
				LED_SENSE_VOLTAGE_TIME = 0;
			}
		} else
			LED_SENSE_VOLTAGE_TIME = 0;
	}
	if (BLINK_50_BIT)
	{
		if (VOLTAGE > LED50_CHANGE_POINT_AG)
		{
			if (LED_SENSE_VOLTAGE_TIME++ >= LED_BLINK_SENSE_TIME)
			{
				BLINK_30_BIT = 0;
				BLINK_50_BIT = 0;
				BLINK_80_BIT = 1;
				BLINK_100_BIT = 0;
				CONST_100_BIT = 0;
				LED_SENSE_VOLTAGE_TIME = 0;
			}
		} else
			LED_SENSE_VOLTAGE_TIME = 0;
	}
	if (BLINK_80_BIT)
		{
			if (VOLTAGE > LED80_CHANGE_POINT_AG)
			{
				if (LED_SENSE_VOLTAGE_TIME++ >= LED_BLINK_SENSE_TIME)
				{
					BLINK_30_BIT = 0;
					BLINK_50_BIT = 0;
					BLINK_80_BIT = 0;
					BLINK_100_BIT = 1;
					CONST_100_BIT = 0;
					LED_SENSE_VOLTAGE_TIME = 0;
				}
			} else
				LED_SENSE_VOLTAGE_TIME = 0;
		}
	if(BLINK_100_BIT)// && (AGING_NORMAL == 1))//AGING_MODE)
	{
		if ((VOLTAGE > LED100_CHANGE_POINT_AG) && (CURRENT == 0))
		{
			if (LED_SENSE_VOLTAGE_TIME++ >= LED_BLINK_SENSE_TIME)
			{
				BLINK_30_BIT = 0;
				BLINK_50_BIT = 0;
				BLINK_80_BIT = 0;
				BLINK_100_BIT = 0;
				CONST_100_BIT = 1;
				LED_SENSE_VOLTAGE_TIME = 0;
			}
		} else
			LED_SENSE_VOLTAGE_TIME = 0;
	}
	if (BLINK_30_BIT)
	{
		if (LED_TIMER >= 5000)
		{
			LED_TIMER = 0;

			LED_30_TOGGLE();
		}

		LEDS_50_OFF();
		LEDS_80_OFF();
	    LEDS_100_OFF();
	}
	if (BLINK_50_BIT)
	{

		LEDS_30_ON();
		if (LED_TIMER >= 5000)
		{
			LED_TIMER = 0;
			LED_50_TOGGLE();
		}

		LEDS_80_OFF();
		LEDS_100_OFF();
	}
	if (BLINK_80_BIT)
	{

		LEDS_30_ON();
	    LEDS_50_ON();
		if (LED_TIMER >= 5000)
		{
			LED_TIMER = 0;
			LED_80_TOGGLE();
		}

		LEDS_100_OFF();
	}
	if (BLINK_100_BIT)
	{
		LEDS_30_ON();
		LEDS_50_ON();
		LEDS_80_ON();
		if (LED_TIMER >= 5000)
		{
			LED_TIMER = 0;
			LED_100_TOGGLE();
		}
	}
	if (CONST_100_BIT)
	{
		LED_ON();
	}
}


void LED_BLINK(void)
{
//	VOLTAGE = BATT_SENSE(BATTERY_SENSE);

	if (BLINK_30_BIT)
	{
		if (State_of_Charge >= LED30_CHANGE_POINT)
		{
			if (LED_SENSE_VOLTAGE_TIME++ >= LED_BLINK_SENSE_TIME)
			{
				BLINK_30_BIT = 0;
				BLINK_50_BIT = 1;
				BLINK_80_BIT = 0;
				BLINK_100_BIT = 0;
				CONST_100_BIT = 0;
				LED_SENSE_VOLTAGE_TIME = 0;
			}
		} else
			LED_SENSE_VOLTAGE_TIME = 0;
	}
	if (BLINK_50_BIT)
	{
		if (State_of_Charge > LED50_CHANGE_POINT)
		{
			if (LED_SENSE_VOLTAGE_TIME++ >= LED_BLINK_SENSE_TIME)
			{
				BLINK_30_BIT = 0;
				BLINK_50_BIT = 0;
				BLINK_80_BIT = 1;
				BLINK_100_BIT = 0;
				CONST_100_BIT = 0;
				LED_SENSE_VOLTAGE_TIME = 0;
			}
		} else
			LED_SENSE_VOLTAGE_TIME = 0;
	}
	if (BLINK_80_BIT)
		{
			if (State_of_Charge > LED80_CHANGE_POINT)
			{
				if (LED_SENSE_VOLTAGE_TIME++ >= LED_BLINK_SENSE_TIME)
				{
					BLINK_30_BIT = 0;
					BLINK_50_BIT = 0;
					BLINK_80_BIT = 0;
					BLINK_100_BIT = 1;
					CONST_100_BIT = 0;
					LED_SENSE_VOLTAGE_TIME = 0;
				}
			} else
				LED_SENSE_VOLTAGE_TIME = 0;
		}
	if(BLINK_100_BIT)// && (AGING_NORMAL == 1))//AGING_MODE)
	{
		if ((State_of_Charge > LED100_CHANGE_POINT) && (CURRENT == 0))
		{
			if (LED_SENSE_VOLTAGE_TIME++ >= LED_BLINK_SENSE_TIME)
			{
				BLINK_30_BIT = 0;
				BLINK_50_BIT = 0;
				BLINK_80_BIT = 0;
				BLINK_100_BIT = 0;
				CONST_100_BIT = 1;
				LED_SENSE_VOLTAGE_TIME = 0;
			}
		} else
			LED_SENSE_VOLTAGE_TIME = 0;
	}
	if (BLINK_30_BIT)
	{
		if (LED_TIMER >= 5000)
		{
			LED_TIMER = 0;

			LED_30_TOGGLE();
		}

		LEDS_50_OFF();
		LEDS_80_OFF();
	    LEDS_100_OFF();
	}
	if (BLINK_50_BIT)
	{

		LEDS_30_ON();
		if (LED_TIMER >= 5000)
		{
			LED_TIMER = 0;
			LED_50_TOGGLE();
		}

		LEDS_80_OFF();
		LEDS_100_OFF();
	}
	if (BLINK_80_BIT)
	{

		LEDS_30_ON();
	    LEDS_50_ON();
		if (LED_TIMER >= 5000)
		{
			LED_TIMER = 0;
			LED_80_TOGGLE();
		}

		LEDS_100_OFF();
	}
	if (BLINK_100_BIT)
	{
		LEDS_30_ON();
		LEDS_50_ON();
		LEDS_80_ON();
		if (LED_TIMER >= 5000)
		{
			LED_TIMER = 0;
			LED_100_TOGGLE();
		}
	}
	if (CONST_100_BIT)
	{
		LED_ON();
	}
}

/********************************************************
 * Function Name: void AGING_LED_INDICATION(void)
 * Returns: Nothing
 * Created By: Nilakntha
 * Date Created: 08/12/2021
 * Date modified:
 * Modified By:
 * Description: Contains Status of LED during Self Start Mode
 * Usage: AGING_LED_INDICATION();
 * Notes:
 ********************************************************/
void AGING_LED_INDICATION(void)
{

	LED_ON();
	DELAY(100);
	LEDS_30_OFF();
	DELAY(100);
	LEDS_50_OFF();
	DELAY(100);
	LEDS_80_OFF();
	DELAY(100);
	LEDS_100_OFF();
	DELAY(100);
	LED_ON();
}

//CallBack Function for TIMER 6
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{



	if (EMERGENCY_SHUTDOWN_TIME_BIT)
	{
		EMERGENCY_SHUTDOWN_TIME++;
		if (EMERGENCY_SHUTDOWN_TIME > 4000)
		{
			EMERGENCY_SHUTDOWN_TIME_BIT = 0;
			EMERGENCY_SHUTDOWN_TIME = 0;
			REON_CHECK_TIME_BIT = 1;
			                       //rs19112022
		}
	}
	if (REON_CHECK_TIME_BIT)
	{
		REON_CHECK_TIME++;
		if (REON_CHECK_TIME > 1000)
		{
			REON_CHECK_TIME_BIT = 0;
			REON_CHECK_TIME = 0;
			//--SHUTDOWN_PIN_LAT = 0;
		}
	}

	if (TIMER_RESUME)
	{
		if (SEC_TIMER++ > 999)
		{
			TIMER_ROLL++;
			OVER_ALL_TIMER++;
			SEC_TIMER = 0;
		}
	}
	if (CHECK_NO_BATTERY_BIT)
	{
		CHECK_NO_BATTERY_TIME++;
	}
	if (PULSE_MODEBIT)
	{
		PULSE_OVER++;
	}
	if (ACCHECK_MODEBIT)
	{
		ACCHECK_MODE++;
	}
	if (UVT_BIT == 1)
	{
	    UVT_TMR++;
	}
	if (OVP_BIT == 1)
	{
		OVP_TMR++;
	}
	if (OVC_BIT == 1)
	{
		OVC_TMR++;
	}

	if (SHT_BIT == 1)
		{
			SHT_TMR++;
		}

	if (AC_OVP == 1)
			{
				AC_OVP_TMR++;
			}

	if (OVT_BIT == 1)
	{
		OVT_TMR++;
	}
	if (SAFE_MOD_BIT)
	{
		if (SAFE_SEC++ > 999)
		{
			SAFE_MOD_TIMER++;
			SAFE_SEC = 0;
		}
	}
	LED_TIMER++;
	BUZZER_TIMER++;
	LED_SENSE_VOLTAGE_TIME++;
	Pre_Full_Charge++;
   // BUZZER_OFF();
}

//rs14112022
static void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType =FDCAN_EXTENDED_ID;//FDCAN_STANDARD_ID; FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;// FDCAN_FILTER_MASK; MAMIDI
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0A6D0D09;//0x1;
  sFilterConfig.FilterID2 = 0x0AB40D09;//0x7FFFFFF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  Start the FDCAN module*/
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
	  Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
	  Error_Handler();
  }

  /* Prepare Tx Header */
  TxHeader.Identifier = 0x445;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;		//FDCAN_DLC_BYTES_2;  MAMIDI
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;       //FOR RECIVEING OF CAN CLASSIC AND CAN FD
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

	uint8_t Counter;
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
	  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
    /* Retrieve Rx messages from RX FIFO0
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
    	Error_Handler();
    }
     */

    CANRXIdentifier = RxHeader.Identifier;      //Received ID copy into CANRXIDENTIFIER


        for(Counter =0; Counter<8;Counter++)    //copy received data into rx buffer
    	{
    	    TempRxData[Counter]= RxData[Counter];
    	}

    	CANDataReady = 1;                       //Call back is working for that
  }
  if (CANRXIdentifier == 0x1806E5F4)
  {
	  Received_Content_ID_1806E5F4();
  }

}

//void CAN_TX (void)
//{
//	Demanding_Swtich_status();
//}

void CAN_Send_Fault()
{
	volt_current_output();
}

/////////////////////////////////////////////////

void CAN_RX (void)
{
	  CANDataReady = 0;
	  CAN_RxTimer = 0;	//MURTHY 20-02
		if  (CANRXIdentifier == 0x100) // Reading data voltage,current and remaining capacity
		{
			Received_Content_ID_100();
		}
		else if(CANRXIdentifier == 0x101)//Charging full cycle,cycle times,RSOC
		{
			Received_Content_ID_101();
		}
		else if(CANRXIdentifier == 0x102)//Protection status
		{
			Received_Content_ID_102();
		}
		else if(CANRXIdentifier == 0x105) //Reading Status information
		{
			Received_Content_ID_105();
		}
		else if(CANRXIdentifier == 0x106) //Reading Cell voltage 1-48
		{
			Received_Content_ID_106();
		}


}// CAN RX ends

/**********************************************************************

//Charger to BMS response Messages:
//Extended ID : 0x18 FE 01 01
//Priority Level : 4
//PDU1 : 0xFE
//PGN ID : 0x01
//Source Address : 0x01
//DLC : 8 Bytes
//VOLTAGE = BATT_SENSE(BATTERY_SENSE);
*****************************************************************/
void Demanding_Swtich_status (void)
{
	uint8_t TxData[8];

	TxData[1] = CRC_Return & 0XFF;
	TxData[0] = (CRC_Return >> 8) & 0XFF;
	TxData[3] = Set_output_voltage & 0xFF;
	TxData[2] = (Set_output_voltage >> 8) & 0XFF;
	TxData[4] = 0x00;
	TxData[5] = 0x00;
	TxData[6] = 0x00;
	TxData[7] = 0x00;

	TxHeader.Identifier = 0x1B8;  //switch status
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    DELAY(10);

}


  /************************************************
   * Function Name: void AC_LED_BLINK(unsigned char NO_OF_BLINKS)
   * Returns: Nothing
   * Created By: Praveen.K
   * Date Created: 03/03/2023
   * Date modified:
   * Modified By:
   * Description: AC LED BLINKING
   * Usage: AC_LED_BLINK(x); x = 1,2,3,..
   ************************************************/
  void
  AC_LED_BLINK (unsigned char NO_OF_BLINKS)
  {
    unsigned char AC_LED_TEMP;
    for (AC_LED_TEMP = 0; AC_LED_TEMP < NO_OF_BLINKS; AC_LED_TEMP++)
      {
	AC_LED_ON();
	DELAY(300);
	AC_LED_OFF();
	DELAY(300);
      }

  }
/**********************************************************************

//Charger to BMS response Messages:
//Extended ID : 0x18 FE 01 01
//Priority Level : 4
//PDU1 : 0xFE
//PGN ID : 0x01
//Source Address : 0x01
//DLC : 8 Bytes
//VOLTAGE = BATT_SENSE(BATTERY_SENSE);
*****************************************************************/

void volt_current_output (void)
{

	 uint8_t TxData[8];

	 CURRENT = CURR_SENSE(CURRENT_SENSE);
	 CHARGER_VOLTAGE = CHAG_SENSE(CHARGER_SENSE);
	 ACVOLTAGE = AC_VOLT_SENSE(AC_SENSE);

	TxData[1] =  CHARGER_VOLTAGE & 0XFF;
	TxData[0] =  (CHARGER_VOLTAGE >> 8) & 0XFF;
	TxData[3] =  CURRENT & 0XFF;
	TxData[2] =  (CURRENT >> 8) & 0XFF;
	TxData[5] =  ACVOLTAGE & 0XFF;
	TxData[4] =  (ACVOLTAGE >> 8) & 0XFF;
	TxData[6] =  Charger_Fault.Charger_CAN[1];
	TxData[7] =  Charger_Fault.Charger_CAN[0];

	TxHeader.Identifier = 0x18000000;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
	DELAY(10);

}

void Charger_Authorization (void)
{

	uint8_t TxData[8];
	TxData[1] = Temp_value_NTC[0] & 0xFF;
	TxData[0] = (Temp_value_NTC[0] >> 8) & 0xFF;
	TxData[3] = Temp_value_NTC[1] & 0xFF;
	TxData[2] = (Temp_value_NTC[1] >> 8) & 0xFF;
	TxData[5] = Temp_value_NTC[2] & 0xFF;
	TxData[4] = (Temp_value_NTC[2] >> 8) & 0xFF;
	TxData[7] = Temp_value_NTC[3] & 0xFF;
	TxData[6] = (Temp_value_NTC[3] >> 8) & 0xFF;

	TxHeader.Identifier = 0x1E100000;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    DELAY(10);

}
//////////////////////////////////////////////////////////////////////////////////

void Demanding_Content_ID_180FF50E5(void)
{
	uint16_t Live_Voltage_0_01 = Live_Voltage * 10;
	uint16_t Live_Current_0_01 = Live_Current;
	uint8_t TxData[8];

	TxData[0] = (Live_Current_0_01 >> 8) & 0xFF;
	TxData[1] = Live_Current_0_01 & 0xFF;
	TxData[2] = (Live_Voltage_0_01 >> 8) & 0xFF;
	TxData[3] = Live_Voltage_0_01 & 0xFF;
    TxData[4] = Charger_Fault.Fault;
    TxData[5] = State_of_Charge;

    TxHeader.Identifier = 0x18FF50E5;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    DELAY(1);
}

void Received_Content_ID_1806E5F4(void)
{
	Demand_voltage = TempRxData[0];
	Demand_voltage = (Demand_voltage << 8) | TempRxData[1];

	Demand_current = TempRxData[2];
	Demand_current = (Demand_current << 8) | TempRxData[3];

	State_of_Charge = TempRxData[5];

	CAN_RxTimer = 0;
	Received_BMS.Received_Content_ID_1806E5F4 = 1;
	BMS_RECEIVE = 1;
}

void CAN_Process()
{
	uint8_t Counter;
	if (Counter100msec <= 1000)		// one sec checking
		return;
//	Power_on_check++;
	Counter100msec = 0;				// MURTHY
	Received_BMS.ID_Data = 0x00;
/////////////////////////////////////////////////////////////
	Counter = 0;
//	LED_BLINK();
	Counter1000msec1 = 0;
		Demanding_Content_ID_100();

		Demanding_Content_ID_180FF50E5();
		DELAY(1);


		if(BMS_RECEIVE == 1)
		{
			Charger_Fault.Fault = 0x00;

			Modifier_Voltage = Demand_voltage;
			Modifier_Voltage = Modifier_Voltage << 8;
			Modifier_Voltage = Modifier_Voltage | Demand_voltage;
			Modifier_Voltage = (Modifier_Voltage / 10) * 42;

			if (Modifier_Voltage > MAX_VOLTAGE)
			{
				Charger_Fault.chargerCMDoutofrange = TRUE;
			}

			Modifier_Current = Demand_current;
			Modifier_Current = Modifier_Current << 8;
			Modifier_Current = Modifier_Current | Demand_current;
			Modifier_Current = ((Modifier_Current / 10) + 1) * 320;

			if (Modifier_Current > MAX_CURRENT)
			{
				Modifier_Current = MAX_CURRENT;
			}
			BMS_RECEIVE = 0;
		}

	while (1)
			{

			if(CANDataReady)
			{
				CANDataReady = 0;
				CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x100) // Voltage , Current and remaining capacity
					{

						Received_Content_ID_100();
						if(Received_BMS.Received_Content_ID_100 == 1)
							{
								break;
							}
						else
							{
					if (++Counter <= 2) {
						Demanding_Content_ID_100();
						Counter1000msec1 = 0;
					} else {
						break;
					}
							}
					}
		}
      else if (Counter1000msec1 >= 50)
				{
			if (++Counter <= 2) {
				Demanding_Content_ID_100();
				Counter1000msec1 = 0;
			} else {
				break;
			}

				}

	} // while ends

//////////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_101();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1)
			{

			if(CANDataReady)
			{
				CANDataReady = 0;
				CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x101) // Charging capacity and SOC
					{
						Received_Content_ID_101();
						if(Received_BMS.Received_Content_ID_101 == 1)
							{
								break;
							}
						else
							{
					if (++Counter <= 2) {
						Demanding_Content_ID_101();
						Counter1000msec1 = 0;
					} else {
						break;
					}
							}
					}
		}
      else if (Counter1000msec1 >= 50)
				{
			if (++Counter <= 2) {
				Demanding_Content_ID_101();
				Counter1000msec1 = 0;
			} else {
				break;
			}

				}

			} // while ends

/////////////////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_102();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1)
			{

			if(CANDataReady)
			{
				CANDataReady = 0;
				CAN_RxTimer = 0;
				if(CANRXIdentifier == 0x102) // Balance status
					{
						Received_Content_ID_102();
						if(Received_BMS.Received_Content_ID_102 == 1)
							{
								break;
							}
						else
							{
					if (++Counter <= 2) {
						Demanding_Content_ID_102();
						Counter1000msec1 = 0;
					} else {
						break;
					}
							}
					}
		}
      else if (Counter1000msec1 >= 50)
				{
			if (++Counter <= 2) {
				Demanding_Content_ID_102();
				Counter1000msec1 = 0;
			} else {
				break;
			}

				}

			} // while ends

///////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_103();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1) {

		if (CANDataReady) {
			CANDataReady = 0;
			CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x103) // FET status
					{
				Received_Content_ID_103();
				if (Received_BMS.Received_Content_ID_103 == 1) {
					break;
				} else
							{
					if (++Counter <= 2) {
						Demanding_Content_ID_103();
						Counter1000msec1 = 0;
					} else {
						break;
					}
				}
			}
	}
      else if (Counter1000msec1 >= 50)
	{
			if (++Counter <= 2) {
				Demanding_Content_ID_103();
				Counter1000msec1 = 0;
			} else {
				break;
			}

		}

	} // while ends

	///////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_104();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1)
			{

			if(CANDataReady)
			{
				CANDataReady = 0;
				CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x104) // No of NTC and Cells
					{
						Received_Content_ID_104();
						if(Received_BMS.Received_Content_ID_104 == 1)
							{
								break;
							}
						else
							{
					if (++Counter <= 2) {
						Demanding_Content_ID_104();
						Counter1000msec1 = 0;
					} else {
						break;
					}
							}
					}
		}
      else if (Counter1000msec1 >= 50)
				{
			if (++Counter <= 2) {
				Demanding_Content_ID_104();
				Counter1000msec1 = 0;
			} else {
				break;
			}

				}

			} // while ends

///////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_105();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1)
			{

			if(CANDataReady)
			{
				CANDataReady = 0;
				CAN_RxTimer = 0;
				if(CANRXIdentifier == 0x105) // Cell Temperature
					{
						Received_Content_ID_105();
						if(Received_BMS.Received_Content_ID_105 == 1)
							{
								break;
							}
						else
							{
					if (++Counter <= 2) {
						Demanding_Content_ID_105();
						Counter1000msec1 = 0;
					} else {
						break;
					}
							}
					}
		}
      else if (Counter1000msec1 >= 50)
				{
			if (++Counter <= 2) {
				Demanding_Content_ID_105();
				Counter1000msec1 = 0;
			} else {
				break;
			}

				}

			} // while ends

/////////////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_106();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1)
			{

			if(CANDataReady)
			{
				CANDataReady = 0;
				CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x106) // Cell Temperature 2
					{
						Received_Content_ID_106();
						if(Received_BMS.Received_Content_ID_106 == 1)
							{
								break;
							}
						else
							{
					if (++Counter <= 2) {
						Demanding_Content_ID_106();
						Counter1000msec1 = 0;
					} else {
						break;
					}
							}
					}
		}
      else if (Counter1000msec1 >= 50)
				{
			if (++Counter <= 2) {
				Demanding_Content_ID_106();
				Counter1000msec1 = 0;
			} else {
				break;
			}

				}

			} // while ends


///////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_107();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1) {

		if (CANDataReady) {
			CANDataReady = 0;
			CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x107) // 1-3 Cell voltage
					{
				Received_Content_ID_107();
				if (Received_BMS.Received_Content_ID_107 == 1) {
					break;
				} else
							{
					if (++Counter <= 2) {
						Demanding_Content_ID_107();
						Counter1000msec1 = 0;
					} else {
						break;
					}
							}
			}
	}
      else if (Counter1000msec1 >= 50)
	{
			if (++Counter <= 2) {
				Demanding_Content_ID_107();
				Counter1000msec1 = 0;
			} else {
				break;
			}

		}

	} // while ends

///////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_108();
	LED_BLINK();
	Counter1000msec1 = 0;

	while (1)
			{

		if (CANDataReady) {
			CANDataReady = 0;
			CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x108) // 4-6 call voltage
					{
						Received_Content_ID_108();
						if(Received_BMS.Received_Content_ID_108 == 1)
							{
					break;
							}
						else
							{
					if (++Counter <= 2) {
						Demanding_Content_ID_108();
						Counter1000msec1 = 0;
					} else {
						break;
					}
							}
					}
	}
      else if (Counter1000msec1 >= 50)
	{
			if (++Counter <= 2) {
				Demanding_Content_ID_108();
				Counter1000msec1 = 0;
			} else {
				break;
			}

		}

	} // while ends

//////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_109();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1)
			{

		if (CANDataReady) {
				CANDataReady = 0;
				CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x109) // 7- 9 cell voltage
					{
				Received_Content_ID_109();
				if (Received_BMS.Received_Content_ID_109 == 1) {
					break;
				} else {
					if (++Counter <= 2) {
						Demanding_Content_ID_109();
						Counter1000msec1 = 0;
					} else {
						break;
					}
				}
					}
	}
      else if (Counter1000msec1 >= 50)
	{
			if (++Counter <= 2) {
				Demanding_Content_ID_109();
				Counter1000msec1 = 0;
			} else {
				break;
			}

				}

			} // while ends

/////////////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_10A();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1) {

		if (CANDataReady) {
			CANDataReady = 0;
			CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x10A) // 10-12 cell voltages
					{
				Received_Content_ID_10A();
				if (Received_BMS.Received_Content_ID_10A == 1) {
					break;
				} else {
					if (++Counter <= 2) {
						Demanding_Content_ID_10A();
						Counter1000msec1 = 0;
					} else {
						break;
					}
							}
					}
	}
      else if (Counter1000msec1 >= 50)
	{
			if (++Counter <= 2) {
				Demanding_Content_ID_10A();
				Counter1000msec1 = 0;
			} else {
				break;
			}

		}

	} // while ends

///////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_10B();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1) {

		if (CANDataReady) {
			CANDataReady = 0;
			CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x10B) // 13- 15 cell voltages
					{
				Received_Content_ID_10B();
				if (Received_BMS.Received_Content_ID_10B == 1) {
					break;
				} else {
					if (++Counter <= 2) {
						Demanding_Content_ID_10B();
						Counter1000msec1 = 0;
					} else {
						break;
					}
				}
			}
	}
      else if (Counter1000msec1 >= 50)
	{
			if (++Counter <= 2) {
				Demanding_Content_ID_10B();
				Counter1000msec1 = 0;
			} else {
				break;
			}

		}

	} // while ends

///////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_10C();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1) {

		if (CANDataReady) {
			CANDataReady = 0;
			CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x10C) // 16- 18 cells
					{
				Received_Content_ID_10C();
				if (Received_BMS.Received_Content_ID_10C == 1) {
					break;
				} else {
					if (++Counter <= 2) {
						Demanding_Content_ID_10C();
						Counter1000msec1 = 0;
					} else {
						break;
					}
							}
			}
	}
      else if (Counter1000msec1 >= 50)
	{
			if (++Counter <= 2) {
				Demanding_Content_ID_10C();
				Counter1000msec1 = 0;
			} else {
				break;
			}

		}

	} // while ends

/////////////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_10D();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1) {

		if (CANDataReady) {
			CANDataReady = 0;
			CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x10D) // 19-21 cells
					{
				Received_Content_ID_10D();
				if (Received_BMS.Received_Content_ID_10D == 1) {
					break;
				} else {
					if (++Counter <= 2) {
						Demanding_Content_ID_10D();
						Counter1000msec1 = 0;
					} else {
						break;
					}
				}
			}
	}
      else if (Counter1000msec1 >= 50)
	{
			if (++Counter <= 2) {
				Demanding_Content_ID_10D();
				Counter1000msec1 = 0;
			} else {
				break;
			}

		}

	} // while ends
//	LED_BLINK();
///////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_10E();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1) {

		if (CANDataReady) {
			CANDataReady = 0;
			CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x10E) // 22-24 cells
					{
				Received_Content_ID_10E();
				if (Received_BMS.Received_Content_ID_10E == 1) {
					break;
				} else {
					if (++Counter <= 2) {
						Demanding_Content_ID_10E();
						Counter1000msec1 = 0;
					} else {
						break;
					}
				}
			}
	}
      else if (Counter1000msec1 >= 50)
	{
			if (++Counter <= 2) {
				Demanding_Content_ID_10E();
				Counter1000msec1 = 0;
			} else {
				break;
			}

		}

	} // while ends

///////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_10F();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1) {

		if (CANDataReady) {
			CANDataReady = 0;
			CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x10F) // 25-27 cells
					{
				Received_Content_ID_10F();
				if (Received_BMS.Received_Content_ID_10F == 1) {
					break;
				} else {
					if (++Counter <= 2) {
						Demanding_Content_ID_10F();
						Counter1000msec1 = 0;
					} else {
						break;
					}
				}
			}
	}
      else if (Counter1000msec1 >= 50)
	{
			if (++Counter <= 2) {
				Demanding_Content_ID_10F();
				Counter1000msec1 = 0;
			} else {
				break;
			}

		}

	} // while ends

//////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_110();
//	LED_BLINK();
	Counter1000msec1 = 0;

	while (1) {

		if (CANDataReady) {
			CANDataReady = 0;
			CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x110) // 28-30 cells
					{
				Received_Content_ID_110();
				if (Received_BMS.Received_Content_ID_110 == 1) {
					break;
				} else {
					if (++Counter <= 2) {
						Demanding_Content_ID_110();
						Counter1000msec1 = 0;
					} else {
						break;
					}
				}
			}
	}
      else if (Counter1000msec1 >= 50)
	{
			if (++Counter <= 2) {
				Demanding_Content_ID_110();
				Counter1000msec1 = 0;
			} else {
				break;
			}

		}

	} // while ends

//////////////////////////////////////////////////////////////

  if ((BMS_RECEIVE == 1) && (CAN_RxTimer >= 10000))
	{
		CAN_RxTimer = 0;
		Charger_Fault.Communication_status = TRUE;
		//BUZZER_ON();
		//DELAY(1000);
		//BUZZER_OFF();
		MODE = CAN_TIME_OUT;
		return;
	}

  if (Received_BMS.ID_Data != 0 && BMS_RECEIVE == 0)
	{
		CAN_RxTimer = 0;
		BMS_RECEIVE = 1;
		Received_BMS.ID_Data = 0x00;
		if(OVP_AVG_VOLTGAE < UNDER_VOLT_VALUE_ONEAMP) //under voltage protection
		{
		   	MODE=ONE_AMP_MODE;
//		   	BUZZER(1);
		    return;
		}
		else
		{
			MODE=CC_CV_MODE;
//			BUZZER(2);
			return;
		}
	}

	if(BMS_RECEIVE == 1)
	{
		//int No_of_min_NTC = 0;
		int No_of_max_NTC = 0;

      for (int NTC_NO = 0; NTC_NO < No_of_NTC_probes; NTC_NO++)
		{
			if (Temp_value_NTC[NTC_NO] >= MAX_CELL_TEMP)
			{
				No_of_max_NTC ++;
				if(No_of_max_NTC > 0)
				{
					if (++Fault_Counter >= 5) {
						MODE_Backup = MODE;
						MODE = OVR_TEMP_MODE;
						return;
					}
				}
			}

			/*else if (Temp_value_NTC[NTC_NO] <= MIN_CELL_TEMP)
			{
				No_of_min_NTC ++;
				if(No_of_min_NTC > 0)
				{
					MODE=IDLE_MODE;
					break;
				}
			}*/

		}

		int No_of_max_Cells = 0;
      /*
		for (int Cell_No = 0; Cell_No < No_of_battery_string; Cell_No++) {
			if (Voltage_value_CELL[Cell_No] >= MAX_CELL_VOLTAGE) {
				No_of_max_Cells++;
				if (No_of_max_Cells > 0) {
					if (State_of_Charge >= 90) //FULL charge call
					{
						Pre_Full_Charge=0;
						MODE = PRE_FULL_CHARGE_MODE;
						return;
					}
				}
			}
		}

       */
      for (int Cell_No = 0; Cell_No < No_of_battery_string; Cell_No++)
	{
	  if (Voltage_value_CELL[Cell_No] >= MAX_CELL_VOLTAGE)
	    {
	      No_of_max_Cells++;
	      if (No_of_max_Cells > 0)
		{

		  MODE = FULL_CHARGE_MODE;
		  break;

		}
	    }
	}
//      LED_BLINK();

      if ((BMS_Fault.Fault & 0x1FFF) != 0)
	{
	  if (BMS_Fault.Charge_overcurrent_protection == 1)
	    {
	      MODE = OVER_CURRENT_MODE;
	      return;
	    }
	  else if (BMS_Fault.Short_circuit_protection == 1)
	    {
	      MODE = SHORT_CKT_MODE;
	      return;
	    }
	  else if ((BMS_Fault.Monomer_overvoltage_protection == 1)
	      || (BMS_Fault.Whole_group_overvoltage_protection == 1))
			{
	      MODE = FULL_CHARGE_MODE;
	      return;
			}
	  else if ((BMS_Fault.Chargeing_over_temperature_protection == 1)
	      || (BMS_Fault.Chargeing_low_temperature_protection == 1))
			{
	      MODE = OVR_TEMP_MODE;
	      return;
			}
	  if ((BMS_Fault.Fault & 0x1FC0) != 0)
			{
	      BUZZER (1);
	      MODE = IDLE_MODE;
	      return;
			}


	}

	}
//	LED_BLINK();
//	 BMS_RECEIVE = 0;

}///Can process ends

void CAN_Recovery()
{
	uint8_t Counter;


	/////////////////////////////////////////////////////////////
	Counter = 0;
	Demanding_Content_ID_100();
	Counter1000msec1 = 0;
	while (1) {

		if (CANDataReady) {
			CANDataReady = 0;
			CAN_RxTimer = 0;
			if (CANRXIdentifier == 0x100) // Voltage , Current and remaining capacity
					{
				Received_Content_ID_100();
				if (Received_BMS.Received_Content_ID_100 == 1) {
					break;
				} else {
					if (++Counter <= 2) {
						Demanding_Content_ID_100();
						Counter1000msec1 = 0;
					} else {
						break;
					}
				}
			}
		} else if (Counter1000msec1 >= 20)
				{
			if (++Counter <= 2) {
				Demanding_Content_ID_100();
				Counter1000msec1 = 0;
			} else {
				break;
			}

		}

	} // while ends
}

void Demanding_Content_ID_100(void)
{

	uint8_t  TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x100;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);


}

void Received_Content_ID_100(void)
{
	  //Applying offset
		  CRC_Received    = TempRxData[6];
		  CRC_Received    = CRC_Received<<8;
		  CRC_Received    = CRC_Received | TempRxData[7];

		  CRC_Return = Check_CRC16 (TempRxData,6);

//		  if(CRC_Received == CRC_Return)
//		  {
			  Battery_voltage    = TempRxData[0];
			  Battery_voltage    = Battery_voltage<<8;
			  Battery_voltage    = Battery_voltage | TempRxData[1];
		  	  //Applying offset
			  Battery_voltage    = (Battery_voltage*(0.01));
		  	  //multiplication factor for voltage for DAC
			  //Battery_voltage    = (Battery_voltage*(42));
		      //byte 2 and 3 are reserved

			  Battery_current    = TempRxData[2];
			  Battery_current    = Battery_current<<8;
			  Battery_current    = Battery_current | TempRxData[3];
		  	  //Applying offset
			  Battery_current    = (Battery_current*(0.01));
		  	  //multiplication factor for current for PWM
			  //Battery_current    = (Battery_current * 2);

			  Remaining_capacity    = TempRxData[4];
			  Remaining_capacity    = Remaining_capacity<<8;
			  Remaining_capacity    = Remaining_capacity | TempRxData[5];

			  CAN_RxTimer = 0;
			  Received_BMS.Received_Content_ID_100 = 1;
			  //Charger_Authorization ();
//		  }

}

void Demanding_Content_ID_101(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x101;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);

}

void Received_Content_ID_101(void)
{
	uint16_t Charging_full_capacity;
	uint16_t Cycle_times;
	//uint16_t R_State_of_Charge;

		 CRC_Received    = TempRxData[6];
		 CRC_Received    = CRC_Received<<8;
		 CRC_Received    = CRC_Received | TempRxData[7];

		 CRC_Return = Check_CRC16 (TempRxData,6);

//		 if(CRC_Received == CRC_Return)
//		 {
			 Charging_full_capacity  = TempRxData[0];
			 Charging_full_capacity  = Charging_full_capacity<<8;
			 Charging_full_capacity  = Charging_full_capacity | TempRxData[1];//10mAh unit

			 Cycle_times             = TempRxData[2];
			 Cycle_times             = Cycle_times<<8;
			 Cycle_times             = Cycle_times | TempRxData[3];//1 time unit

//			 State_of_Charge         = TempRxData[4];
//			 State_of_Charge         = State_of_Charge<<8;
			 State_of_Charge         = TempRxData[5];//1%

			 CAN_RxTimer = 0;
			 Received_BMS.Received_Content_ID_101 = 1;
			 //Charger_Authorization ();

//		 }

}

void Demanding_Content_ID_102(void)
{

	uint8_t TxData[8];

	TxData[0] = 0x5A;
	TxData[1] = 0x00;
	TxData[2] = 0x00;
	TxData[3] = 0x00;
	TxData[4] = 0x00;
	TxData[5] = 0x00;
	TxData[6] = 0x00;
	TxData[7] = 0x00;

    TxHeader.Identifier = 0x102;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);

}

void Received_Content_ID_102(void)
{
		CRC_Received    = TempRxData[6];
		CRC_Received    = CRC_Received<<8;
		CRC_Received    = CRC_Received | TempRxData[7];

		CRC_Return = Check_CRC16 (TempRxData,6);

//		if(CRC_Received == CRC_Return)
//		{

			BMS_Fault.BMS_CAN[0] = TempRxData[4];
			BMS_Fault.BMS_CAN[1] = TempRxData[5];

			CAN_RxTimer = 0;
			Received_BMS.Received_Content_ID_102 = 1;
			//Charger_Authorization ();

//		}

}

void Demanding_Content_ID_103(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x103;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);

}

void Received_Content_ID_103(void)
{
  MOS_Status = TempRxData[1];
	Received_BMS.Received_Content_ID_103 = 1;

}

void Demanding_Content_ID_104(void)
{

	uint8_t TxData[8];

	TxData[0] = 0x5A;
	TxData[1] = 0x00;
	TxData[2] = 0x00;
	TxData[3] = 0x00;
	TxData[4] = 0x00;
	TxData[5] = 0x00;
	TxData[6] = 0x00;
	TxData[7] = 0x00;

    TxHeader.Identifier = 0x104;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);

}

void Received_Content_ID_104(void)
{
		CRC_Received    = TempRxData[2];
		CRC_Received    = CRC_Received<<8;
		CRC_Received    = CRC_Received | TempRxData[3];

		CRC_Return = Check_CRC16 (TempRxData,2);

//		if(CRC_Received == CRC_Return)
//		{
			No_of_battery_string     = TempRxData[0];
			No_of_NTC_probes         = TempRxData[1];

			CAN_RxTimer = 0;
			Received_BMS.Received_Content_ID_104 = 1;
			//Charger_Authorization ();
//		}
}

void Demanding_Content_ID_105(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x105;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);

}
void Received_Content_ID_105(void)
{

  CRC_Received = TempRxData[6];
  CRC_Received = CRC_Received << 8;
  CRC_Received = CRC_Received | TempRxData[7];

  CRC_Return = Check_CRC16 (TempRxData, 6);

  if (No_of_NTC_probes == 1)
    {
      Temp_value_NTC[0] = TempRxData[0];
      Temp_value_NTC[0] = Temp_value_NTC[0] << 8;
      Temp_value_NTC[0] = Temp_value_NTC[0] | TempRxData[1];
      Temp_value_NTC[0] = ((Temp_value_NTC[0] - 2731) / 10);
      Temp_value_NTC[1] = 0;
      Temp_value_NTC[2] = 0;
      CAN_RxTimer = 0;
      Received_BMS.Received_Content_ID_105 = 1;
      return;
    }
  if (No_of_NTC_probes == 2)
    {
      Temp_value_NTC[0] = TempRxData[0];
      Temp_value_NTC[0] = Temp_value_NTC[0] << 8;
      Temp_value_NTC[0] = Temp_value_NTC[0] | TempRxData[1];
      Temp_value_NTC[0] = ((Temp_value_NTC[0] - 2731) / 10);

      Temp_value_NTC[1] = TempRxData[2];
      Temp_value_NTC[1] = Temp_value_NTC[1] << 8;
      Temp_value_NTC[1] = Temp_value_NTC[1] | TempRxData[3];
      Temp_value_NTC[1] = ((Temp_value_NTC[1] - 2731) / 10);
      CAN_RxTimer = 0;
      Received_BMS.Received_Content_ID_105 = 1;
      return;
    }

  if (No_of_NTC_probes >= 3)
    {
      Temp_value_NTC[0] = TempRxData[0];
      Temp_value_NTC[0] = Temp_value_NTC[0] << 8;
      Temp_value_NTC[0] = Temp_value_NTC[0] | TempRxData[1];
      Temp_value_NTC[0] = ((Temp_value_NTC[0] - 2731) / 10);

      Temp_value_NTC[1] = TempRxData[2];
      Temp_value_NTC[1] = Temp_value_NTC[1] << 8;
      Temp_value_NTC[1] = Temp_value_NTC[1] | TempRxData[3];
      Temp_value_NTC[1] = ((Temp_value_NTC[1] - 2731) / 10);

      Temp_value_NTC[2] = TempRxData[4];
      Temp_value_NTC[2] = Temp_value_NTC[2] << 8;
      Temp_value_NTC[2] = Temp_value_NTC[2] | TempRxData[5];
      Temp_value_NTC[2] = ((Temp_value_NTC[2] - 2731) / 10);

      CAN_RxTimer = 0;
      Received_BMS.Received_Content_ID_105 = 1;
    }


}

void Demanding_Content_ID_106(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x106;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);

}

void Received_Content_ID_106(void)
{
  if (No_of_NTC_probes <= 3)
    {
      Temp_value_NTC[3] = 0;
      Temp_value_NTC[4] = 0;
      Temp_value_NTC[5] = 0;
      CAN_RxTimer = 0;
      Received_BMS.Received_Content_ID_106 = 1;
    }
  /*
   if(No_of_NTC_probes == 4)
		{
			CRC_Received    = TempRxData[2];
			CRC_Received    = CRC_Received<<8;
			CRC_Received    = CRC_Received | TempRxData[3];

			CRC_Return = Check_CRC16 (TempRxData,2);

			if(CRC_Received == CRC_Return)
			{
				Temp_value_NTC[3] = TempRxData[0];
				Temp_value_NTC[3] = Temp_value_NTC[3] << 8;
				Temp_value_NTC[3] = Temp_value_NTC[3] | TempRxData[1];
				Temp_value_NTC[3] = ((Temp_value_NTC[3]-2731)/10);

				CAN_RxTimer = 0;
				Received_BMS.Received_Content_ID_106 = 1;


			}
		}

   if(No_of_NTC_probes == 5)
		{
			CRC_Received    = TempRxData[4];
			CRC_Received    = CRC_Received<<8;
			CRC_Received    = CRC_Received | TempRxData[5];

			CRC_Return = Check_CRC16 (TempRxData,4);

			if(CRC_Received == CRC_Return)
			{
				Temp_value_NTC[3] = TempRxData[0];
				Temp_value_NTC[3] = Temp_value_NTC[3] << 8;
				Temp_value_NTC[3] = Temp_value_NTC[3] | TempRxData[1];
				Temp_value_NTC[3] = ((Temp_value_NTC[3]-2731)/10);

				Temp_value_NTC[4] = TempRxData[2];
				Temp_value_NTC[4] = Temp_value_NTC[4] << 8;
				Temp_value_NTC[4] = Temp_value_NTC[4] | TempRxData[3];
				Temp_value_NTC[4] = ((Temp_value_NTC[4]-2731)/10);

				CAN_RxTimer = 0;
				Received_BMS.Received_Content_ID_106 = 1;

				//Charger_Authorization ();
			}
		}
   */
//		if(No_of_NTC_probes == 6)
//		{
			CRC_Received    = TempRxData[6];
			CRC_Received    = CRC_Received<<8;
			CRC_Received    = CRC_Received | TempRxData[7];

			CRC_Return = Check_CRC16 (TempRxData,6);

//			if(CRC_Received == CRC_Return)
//			{
				Temp_value_NTC[3] = TempRxData[0];
				Temp_value_NTC[3] = Temp_value_NTC[3] << 8;
				Temp_value_NTC[3] = Temp_value_NTC[3] | TempRxData[1];
				Temp_value_NTC[3] = ((Temp_value_NTC[3]-2731)/10);

				Temp_value_NTC[4] = TempRxData[2];
				Temp_value_NTC[4] = Temp_value_NTC[4] << 8;
				Temp_value_NTC[4] = Temp_value_NTC[4] | TempRxData[3];
				Temp_value_NTC[4] = ((Temp_value_NTC[4]-2731)/10);

				Temp_value_NTC[5] = TempRxData[4];
				Temp_value_NTC[5] = Temp_value_NTC[5] << 8;
				Temp_value_NTC[5] = Temp_value_NTC[5] | TempRxData[5];
				Temp_value_NTC[5] = ((Temp_value_NTC[5]-2731)/10);

				CAN_RxTimer = 0;
				Received_BMS.Received_Content_ID_106 = 1;

				//Charger_Authorization ();
//			}
//		}


}

void Demanding_Content_ID_107(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x107;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);

}

void Received_Content_ID_107(void)
{

		CRC_Received    = TempRxData[6];
		CRC_Received    = CRC_Received<<8;
		CRC_Received    = CRC_Received | TempRxData[7];

		CRC_Return = Check_CRC16 (TempRxData,6);

//		if(CRC_Received == CRC_Return)
//		{
			Voltage_value_CELL[0] = TempRxData[0];
			Voltage_value_CELL[0] = Voltage_value_CELL[0] << 8;
			Voltage_value_CELL[0] = Voltage_value_CELL[0] | TempRxData[1];
			Voltage_value_CELL[1] = TempRxData[2];
			Voltage_value_CELL[1] = Voltage_value_CELL[1] << 8;
			Voltage_value_CELL[1] = Voltage_value_CELL[1] | TempRxData[3];
			Voltage_value_CELL[2] = TempRxData[4];
			Voltage_value_CELL[2] = Voltage_value_CELL[2] << 8;
			Voltage_value_CELL[2] = Voltage_value_CELL[2] | TempRxData[5];

			CAN_RxTimer = 0;
			Received_BMS.Received_Content_ID_107 = 1;
//		}

}

void Demanding_Content_ID_108(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x108;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);

}

void Received_Content_ID_108(void)
{

		CRC_Received    = TempRxData[6];
		CRC_Received    = CRC_Received<<8;
		CRC_Received    = CRC_Received | TempRxData[7];

		CRC_Return = Check_CRC16 (TempRxData,6);

//		if(CRC_Received == CRC_Return)
//		{
			Voltage_value_CELL[3] = TempRxData[0];
			Voltage_value_CELL[3] = Voltage_value_CELL[3] << 8;
			Voltage_value_CELL[3] = Voltage_value_CELL[3] | TempRxData[1];
			Voltage_value_CELL[4] = TempRxData[2];
			Voltage_value_CELL[4] = Voltage_value_CELL[4] << 8;
			Voltage_value_CELL[4] = Voltage_value_CELL[4] | TempRxData[3];
			Voltage_value_CELL[5] = TempRxData[4];
			Voltage_value_CELL[5] = Voltage_value_CELL[5] << 8;
			Voltage_value_CELL[5] = Voltage_value_CELL[5] | TempRxData[5];

			CAN_RxTimer = 0;
			Received_BMS.Received_Content_ID_108 = 1;
//		}

}

void Demanding_Content_ID_109(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x109;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);
}

void Received_Content_ID_109(void)
{

		CRC_Received    = TempRxData[6];
		CRC_Received    = CRC_Received<<8;
		CRC_Received    = CRC_Received | TempRxData[7];

		CRC_Return = Check_CRC16 (TempRxData,6);

//		if(CRC_Received == CRC_Return)
//		{
			Voltage_value_CELL[6] = TempRxData[0];
			Voltage_value_CELL[6] = Voltage_value_CELL[6] << 8;
			Voltage_value_CELL[6] = Voltage_value_CELL[6] | TempRxData[1];
			Voltage_value_CELL[7] = TempRxData[2];
			Voltage_value_CELL[7] = Voltage_value_CELL[7] << 8;
			Voltage_value_CELL[7] = Voltage_value_CELL[7] | TempRxData[3];
			Voltage_value_CELL[8] = TempRxData[4];
			Voltage_value_CELL[8] = Voltage_value_CELL[8] << 8;
			Voltage_value_CELL[8] = Voltage_value_CELL[8] | TempRxData[5];

			CAN_RxTimer = 0;
			Received_BMS.Received_Content_ID_109 = 1;
//		}

}

void Demanding_Content_ID_10A(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x10A;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);

}

void Received_Content_ID_10A(void)
{

		CRC_Received    = TempRxData[6];
		CRC_Received    = CRC_Received<<8;
		CRC_Received    = CRC_Received | TempRxData[7];

		CRC_Return = Check_CRC16 (TempRxData,6);

//		if(CRC_Received == CRC_Return)
//		{
			Voltage_value_CELL[9] = TempRxData[0];
			Voltage_value_CELL[9] = Voltage_value_CELL[9] << 8;
			Voltage_value_CELL[9] = Voltage_value_CELL[9] | TempRxData[1];
			Voltage_value_CELL[10] = TempRxData[2];
			Voltage_value_CELL[10] = Voltage_value_CELL[10] << 8;
			Voltage_value_CELL[10] = Voltage_value_CELL[10] | TempRxData[3];
			Voltage_value_CELL[11] = TempRxData[4];
			Voltage_value_CELL[11] = Voltage_value_CELL[11] << 8;
			Voltage_value_CELL[11] = Voltage_value_CELL[11] | TempRxData[5];

			CAN_RxTimer = 0;
			Received_BMS.Received_Content_ID_10A = 1;
//		}

}

void Demanding_Content_ID_10B(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x10B;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);

}

void Received_Content_ID_10B(void)
{

		CRC_Received    = TempRxData[6];
		CRC_Received    = CRC_Received<<8;
		CRC_Received    = CRC_Received | TempRxData[7];

		CRC_Return = Check_CRC16 (TempRxData,6);

//		if(CRC_Received == CRC_Return)
//		{
			Voltage_value_CELL[12] = TempRxData[0];
			Voltage_value_CELL[12] = Voltage_value_CELL[12] << 8;
			Voltage_value_CELL[12] = Voltage_value_CELL[12] | TempRxData[1];
			Voltage_value_CELL[13] = TempRxData[2];
			Voltage_value_CELL[13] = Voltage_value_CELL[13] << 8;
			Voltage_value_CELL[13] = Voltage_value_CELL[13] | TempRxData[3];
			Voltage_value_CELL[14] = TempRxData[4];
			Voltage_value_CELL[14] = Voltage_value_CELL[14] << 8;
			Voltage_value_CELL[14] = Voltage_value_CELL[14] | TempRxData[5];

			CAN_RxTimer = 0;
			Received_BMS.Received_Content_ID_10B = 1;
//		}

}

void Demanding_Content_ID_10C(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x10C;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
   // DELAY(10);

}

void Received_Content_ID_10C(void)
{

		CRC_Received    = TempRxData[6];
		CRC_Received    = CRC_Received<<8;
		CRC_Received    = CRC_Received | TempRxData[7];

		CRC_Return = Check_CRC16 (TempRxData,6);

//		if(CRC_Received == CRC_Return)
//		{
			Voltage_value_CELL[15] = TempRxData[0];
			Voltage_value_CELL[15] = Voltage_value_CELL[15] << 8;
			Voltage_value_CELL[15] = Voltage_value_CELL[15] | TempRxData[1];
			Voltage_value_CELL[16] = TempRxData[2];
			Voltage_value_CELL[16] = Voltage_value_CELL[16] << 8;
			Voltage_value_CELL[16] = Voltage_value_CELL[16] | TempRxData[3];
			Voltage_value_CELL[17] = TempRxData[4];
			Voltage_value_CELL[17] = Voltage_value_CELL[17] << 8;
			Voltage_value_CELL[17] = Voltage_value_CELL[17] | TempRxData[5];

			CAN_RxTimer = 0;
			Received_BMS.Received_Content_ID_10C = 1;
//		}

}

void Demanding_Content_ID_10D(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;
    TxHeader.Identifier = 0x10D;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);

}
void Received_Content_ID_10D(void)
{

	if(No_of_battery_string == 23)
		{
			CRC_Received    = TempRxData[6];
			CRC_Received    = CRC_Received<<8;
			CRC_Received    = CRC_Received | TempRxData[7];

			CRC_Return = Check_CRC16 (TempRxData,6);

//			if(CRC_Received == CRC_Return)
//			{
				Voltage_value_CELL[18] = TempRxData[0];
				Voltage_value_CELL[18] = Voltage_value_CELL[18] << 8;
				Voltage_value_CELL[18] = Voltage_value_CELL[18] | TempRxData[1];
				Voltage_value_CELL[19] = TempRxData[2];
				Voltage_value_CELL[19] = Voltage_value_CELL[19] << 8;
				Voltage_value_CELL[19] = Voltage_value_CELL[19] | TempRxData[3];
				Voltage_value_CELL[20] = TempRxData[4];
				Voltage_value_CELL[20] = Voltage_value_CELL[20] << 8;
				Voltage_value_CELL[20] = Voltage_value_CELL[20] | TempRxData[5];

				CAN_RxTimer = 0;
				Received_BMS.Received_Content_ID_10D = 1;
//			}
		}

		if(No_of_battery_string == 19)
		{
      CRC_Received = TempRxData[6];
      CRC_Received = CRC_Received << 8;
      CRC_Received = CRC_Received | TempRxData[7];

      CRC_Return = Check_CRC16 (TempRxData, 6);

//			if(CRC_Received == CRC_Return)
//			{
				Voltage_value_CELL[18] = TempRxData[0];
				Voltage_value_CELL[18] = Voltage_value_CELL[18] << 8;
				Voltage_value_CELL[18] = Voltage_value_CELL[18] | TempRxData[1];
//				Voltage_value_CELL[19] = TempRxData[2];
//				Voltage_value_CELL[19] = Voltage_value_CELL[19] << 8;
//				Voltage_value_CELL[19] = Voltage_value_CELL[19] | TempRxData[3];
//				Voltage_value_CELL[20] = TempRxData[4];
//				Voltage_value_CELL[20] = Voltage_value_CELL[20] << 8;
//				Voltage_value_CELL[20] = Voltage_value_CELL[20] | TempRxData[5];

				CAN_RxTimer = 0;
				Received_BMS.Received_Content_ID_10D = 1;
//			}
		}


}

void Demanding_Content_ID_10E(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x10E;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    //DELAY(10);

}

void Received_Content_ID_10E(void)
{

	if(No_of_battery_string == 23)
		{
      CRC_Received = TempRxData[6];
			CRC_Received    = CRC_Received<<8;
      CRC_Received = CRC_Received | TempRxData[7];

      CRC_Return = Check_CRC16 (TempRxData, 6);

//			if(CRC_Received == CRC_Return)
//			{
				Voltage_value_CELL[21] = TempRxData[0];
				Voltage_value_CELL[21] = Voltage_value_CELL[21] << 8;
				Voltage_value_CELL[21] = Voltage_value_CELL[21] | TempRxData[1];
				Voltage_value_CELL[22] = TempRxData[2];
				Voltage_value_CELL[22] = Voltage_value_CELL[22] << 8;
				Voltage_value_CELL[22] = Voltage_value_CELL[22] | TempRxData[3];

				CAN_RxTimer = 0;
				Received_BMS.Received_Content_ID_10E = 1;
//			}
		}

		if(No_of_battery_string == 19)
		{
			CAN_RxTimer = 0;
			Received_BMS.Received_Content_ID_10E = 1;
		}


}

void Demanding_Content_ID_10F(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x10F;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
   // DELAY(10);

}

void Received_Content_ID_10F(void)
{
	CAN_RxTimer = 0;
	Received_BMS.Received_Content_ID_10F = 1;
}

void Demanding_Content_ID_110(void)
{

	uint8_t TxData[8];

    TxData[0] = 0x5A;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    TxHeader.Identifier = 0x110;  //switch status
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.DataLength=FDCAN_DLC_BYTES_1;
    DELAY(1);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
   // DELAY(10);

}

void Received_Content_ID_110(void)
{
	CAN_RxTimer = 0;
	Received_BMS.Received_Content_ID_110 = 1;
}
void UART_process()
{
if(UART_Counter>=2000)
{
switch(MODE)
{

case INITIAL_MODE:
	String_Copy(aTxTestData, aInitial_mode,20);
	HAL_UART_Transmit(&huart1, (uint8_t *)aTxTestData, 20, 10) ;

break;


default:
break;

}// if ends

}// switch ends


}


void String_Copy(uint8_t *Dest,uint8_t *Src, uint8_t Length)
{
	uint8_t Count;

	for(Count =0; Count<Length;Count++)
	{
		*Dest = *Src;
		Dest++;
		Src++;
	}


}

/*************************CRC Calculation ****************************/
uint16_t Check_CRC16 (uint8_t *pchMsg,uint8_t wDataLen)
{
	uint8_t i,chChar;
	uint16_t wCRC = 0xFFFF;
	while(wDataLen--)
	{
		chChar = *pchMsg++;
		wCRC ^= (uint16_t) chChar;
		for( i = 0; i < 8; i++)
		{
			if(wCRC & 0x0001)
			{
				wCRC = (wCRC >> 1) ^ CRC_16_POLYNOMIALS;
			}
			else
			{
				wCRC >>= 1;
			}
		}
	}
	return wCRC;
}

/********************************* Aging ********************************************/

void Aging_process(void)
{

    uint8_t Temp_Counter;

    Initialisation_Completed = 1;
	        DELAY(200);
	BUZZER (3); //power on buzzer for aging indication
	AGING_LED_INDICATION ();
	INIT_COND_FUN_CALL ();
	ALL_LEDS_OFF ();


	RELAY_ON();
	aging_soft_bit = 1;

	AGING_MODE = 1;

	AC_LED_ON();
/*
Aging_Communication ();
while (1)
{
  if ((CANDataReady) && (CANRXIdentifier == 0x18904001)) //Reading max and min Temperature
    {
      DELAY(10);
      CANDataReady = 0;
      Aging_Parameters ();
    }
}

*/
	RELAY_ON();
	SHUTDWON_OFF();
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Modified_Current * 0.1);//(Modified_Current * 0.2));
	AGING_VOLTAGE_INC_SLOWLY (0, MAX_VOLTAGE);
	AGING_SLOW_CURRENT_INCREMENT ((Modified_Current * 0.1), Modified_Current);
	DELAY(10);

	AGING_MODE_NO = AGING_IDLE_MODE;

	UART_RX_Identifier = 0;
	UART_Rx_Buffer[0] = 0;
	UART_Data_Ready = 0;


while (1)
{
	if (HAL_UART_Receive(&huart1,  UART_Rx_Buffer, 1, 100)
		== HAL_OK)
		{
		UART_RX_Identifier = UART_Rx_Buffer[0];
		UART_Data_Ready = 1;
		}

	if (UART_Data_Ready)
	{
	UART_Data_Ready = 0;

	if (UART_RX_Identifier == 'A')
		{
		VSet_Tune();
		}
	else if (UART_RX_Identifier == 'C')
		{
		ISet_Tune();
		}
	else if (UART_RX_Identifier == 'B')
		{
		VADC_Tune();
		}
	else if (UART_RX_Identifier == 'D')
		{
		IADC_Tune();
		}
	else if (UART_RX_Identifier == '*')
		{
		OVP_Check();
		VOLTAGE_INC_SLOWLY(Present_voltage, Modified_Voltage);
		DELAY(250);
		AGING_MODE_NO = AGING_IDLE_MODE;
		}
	else if (UART_RX_Identifier == '0')
		{
		DELAY(10);
		LED_OFF();
		DELAY(100);
	for (Temp_Counter = 0; Temp_Counter < 20; Temp_Counter++)
			{
				UART_Tx_Buffer[Temp_Counter] = Config_Message1[Temp_Counter];
			}
		HAL_UART_Transmit(&huart1, (uint8_t*) UART_Tx_Buffer, 20, 1000);

		for (Temp_Counter = 0; Temp_Counter < 20; Temp_Counter++)
			{
				UART_Tx_Buffer[Temp_Counter] = Config_Message2[Temp_Counter];
			}
		HAL_UART_Transmit(&huart1, (uint8_t*) UART_Tx_Buffer, 20, 1000);

		}
		UART_Rx_Buffer[0] = 0;
		UART_RX_Identifier = 0;

	}	// UARTREADY

/*************************************AGING SHORT CKT MODE*********************************************/
switch (AGING_MODE_NO)
{
case AGING_IDLE_MODE:
  AC_LED_ON();
  LED_BLINK_AG ();

  break;

case AGING_SHORT_CKT_MODE:
  SHUTDOWN_HIGH_FUN ();
  Charger_Fault.Output_short_circuit = TRUE;
  Charger_Fault.Charging_status = FALSE;

  	if(LED_TIMER >= 20000)
	{
		LED_OFF();
		AC_LED_BLINK(6);
		LED_TIMER = 0;
	}

  break;

/**********************AGING OVER CURRENT MODE*****************************/

case AGING_OVER_CURRENT_MODE:

  OVC_BIT = 0;
  OVC_TMR = 0;
  //SHUTDOWN_HIGH_FUN();
  Charger_Fault.Output_over_current = TRUE;
  Charger_Fault.Charging_status = FALSE;
  OFF_CONDITION ();
  	if(LED_TIMER >= 20000)
	{
		LED_OFF();
		AC_LED_BLINK(5);
		LED_TIMER = 0;
	}
  break;

/**********************AGING OVER VOLTAGE MODE*****************************/

case AGING_OVR_VOLT_MODE:

  OVP_BIT = 0;
  OVP_TMR = 0;

  Charger_Fault.Output_over_voltage = TRUE;
  Charger_Fault.Charging_status = FALSE;
  SHUTDOWN_HIGH_FUN ();
  //OFF_CONDITION();
  	if(LED_TIMER >= 20000)
	{
		LED_OFF();
		AC_LED_BLINK(2);
		LED_TIMER = 0;
	}

  break;

/**********************AGING UNDER VOLTAGE MODE*****************************/

case AGING_UNDER_VOLT_MODE:

  OVP_AVG_VOLTGAE = OVP_AVG ();
  SHUTDOWN_HIGH_FUN ();
  if(LED_TIMER >= 20000)
	{
		LED_OFF();
		AC_LED_BLINK(7);
		LED_TIMER=0;
	}

  if (Counter1000msec >= 1000)
    {
      Charger_Fault.Output_under_voltage = TRUE;
      Charger_Fault.Charging_status = FALSE;
//	      CAN_Send_Fault ();
      Counter1000msec = 0;
    }
  if (OVP_AVG_VOLTGAE >= UNDER_VOLT_VALUE_PROTECTION)
    {
      Charger_Fault.Output_under_voltage = FALSE;
    }

  break;

default:
  LED_OFF ();
  break;

} //Switch End

  AGING_PROTECTION_CALL ();
  CURRENT = CURR_SENSE (CURRENT_SENSE);

  if (CURRENT >= CURRENT_0_5A)
  {
      FAN_ON();
  }
  else
  {
      FAN_OFF();
  }
} //switch while end
}

void Aging_VI_Output()
{

    uint8_t aTxBuffer[16];
    uint16_t OVP_Avg_Voltage;
    uint16_t Current;

    OVP_Avg_Voltage = Live_Voltage_Tune;

    aTxBuffer[4] = (OVP_Avg_Voltage % 10) + '0';

    OVP_Avg_Voltage = OVP_Avg_Voltage / 10;
    aTxBuffer[3] = (OVP_Avg_Voltage % 10) + '0';

    aTxBuffer[2] = '.';

    OVP_Avg_Voltage = OVP_Avg_Voltage / 10;
    aTxBuffer[1] = (OVP_Avg_Voltage % 10) + '0';

    OVP_Avg_Voltage = OVP_Avg_Voltage / 10;
    aTxBuffer[0] = (OVP_Avg_Voltage) + '0';;



////////////////////////////////////////////////////////////////////

    aTxBuffer[5] = ' ';
    aTxBuffer[6] = ' ';
    aTxBuffer[7] = ' ';
//////////////////////////////////////////////////////////////////////
    Current = Live_Current_Tune;

    aTxBuffer[12] = (Current % 10) + '0';

    Current = Current / 10;
    aTxBuffer[11] =  (Current % 10) + '0';

    aTxBuffer[10] = '.';

    Current = Current / 10;
    aTxBuffer[9] = (Current % 10) + '0';

    Current = Current / 10;
    aTxBuffer[8] = (Current) + '0';

    aTxBuffer[13] = '\r';

    HAL_UART_Transmit(&huart1, aTxBuffer, 14, 100);
      DELAY(2);

    }

/***********************************************************/

void Flash_Erase_Page()
{
    uint8_t Status;
    //  uint32_t Flash_Error;

    HAL_FLASH_Unlock();

    FirstPage = ADDR_FLASH_PAGE;
    NbOfPages = 1;
    BankNumber = FLASH_BANK_1;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    __HAL_FLASH_CLEAR_FLAG(
	    FLASH_FLAG_SIZERR | FLASH_FLAG_EOP | FLASH_FLAG_OPERR |FLASH_FLAG_MISERR | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR |FLASH_FLAG_FASTERR);

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks = BankNumber;
  EraseInitStruct.Page = FirstPage;
  EraseInitStruct.NbPages = NbOfPages;
	//   FLASH_PageErase(FLASH_BANK_1, ADDR_FLASH_PAGE);
	Status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    if (Status != HAL_OK)
	    {
	BUZZER(1);
	PageError = Status;
	    }
	DELAY(1);

}
/****************************************************************/

void Flash_Write()
    {
    uint8_t Counter;
    uint8_t Status;
    LEDS_30_ON();
//    Initialisation_Completed = 0;


    /****************************Erase*******************************/
    HAL_FLASH_Unlock();

    FirstPage = ADDR_FLASH_PAGE;
    NbOfPages = 1;
    BankNumber = FLASH_BANK_1;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    //   __HAL_FLASH_CLEAR_FLAG(
//	    FLASH_FLAG_SIZERR | FLASH_FLAG_EOP | FLASH_FLAG_OPERR |FLASH_FLAG_MISERR | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR |FLASH_FLAG_FASTERR);

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks = BankNumber;
    EraseInitStruct.Page = FirstPage;
    EraseInitStruct.NbPages = NbOfPages;

    Status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    if (Status != HAL_OK)
	{
	BUZZER(1);

	PageError = Status;
//	Flash_Erase_Error(PageError);
	return;
	}


    LEDS_50_ON();
    /*************************************************************/

	    Page_Data[0] = 1;
	    Page_Data[1] = Modified_Voltage;
	    Page_Data[2] = Modified_Current;
	    Page_Data[3] = Modified_Voltage_Read;
	    Page_Data[4] = Modified_Current_Read;
	    Address = ADDR_FLASH_PAGE_63;
    for (Counter = 0; Counter < 5; Counter++)
		{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address,
		Page_Data[Counter]);
		Address = Address + 8;
		}
	    HAL_FLASH_Lock();
	    LEDS_80_ON();
	    LEDS_100_ON();
	    DELAY(100);
    //   Initialisation_Completed = 1;
	    }

////////////////////////////////////////////////////////
void VSet_Tune()
    {
    LED_OFF();
    LEDS_30_ON();
    AC_LED_OFF();
 //   HAL_UART_Transmit(&huart1, (uint8_t*) "V Set \r\n", 8, 10);

    while (1)
	{
	while (1)
	    {
	    CURRENT = CURR_SENSE(CURRENT_SENSE);
	    if (CURRENT >= CURRENT_0_5A)
	    {
	    FAN_ON();
	    }
	    else
	    {
	    FAN_OFF();
	    }
	    if (HAL_UART_Receive(&huart1, (uint8_t*) UART_Rx_Buffer, 1, 100)
		    == HAL_OK)
		{
		Key = UART_Rx_Buffer[0];
		break;
		}
	    }
	if (Key == '1')
	    {
	    Modified_Voltage = Modified_Voltage + 1;
	    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2,
	    DAC_ALIGN_12B_R, Modified_Voltage);
	    Key = 0XFF;
	    }

	if (Key == '4')
	    {
	    if(Modified_Voltage > VOLTAGE_LOW_RANGE)
		{
		Modified_Voltage = Modified_Voltage - 1;
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2,
		DAC_ALIGN_12B_R, Modified_Voltage);
		}
	    Key = 0XFF;
	    }

	if (Key == '2')
	    {
	    Modified_Voltage = Modified_Voltage + 5;
	    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2,
	    DAC_ALIGN_12B_R, Modified_Voltage);
	    Key = 0XFF;
	    }

	if (Key == '5')
	    {
	    if(Modified_Voltage > VOLTAGE_LOW_RANGE)
	    	{
		Modified_Voltage = Modified_Voltage - 5;
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2,
		DAC_ALIGN_12B_R, Modified_Voltage);
	    	}
	    Key = 0XFF;
	    }

	if (Key == '#')
	    {
	    Flash_Write();
	    Key = 0XFF;
	    UART_Rx_Buffer[0] = 0;
	    UART_Rx_Buffer[1] = 0;
	    UART_RX_Identifier = 0;
	    HAL_UART_Transmit(&huart1, (uint8_t*) "\r\nReturn\r\n",10, 10);
	    return;

	    }
	}
    }
////////////////////////////////////
void ISet_Tune()
    {
    LED_OFF();
    LEDS_80_ON();
    AC_LED_OFF();
//    HAL_UART_Transmit(&huart1, (uint8_t*) "I Set \r\n", 8, 10);
    while (1)
	{
	while (1)
	    {
	    CURRENT = CURR_SENSE(CURRENT_SENSE);
	    if (CURRENT >= CURRENT_0_5A)
	    {
	    FAN_ON();
	    }
	    else
	    {
	    FAN_OFF();
	    }
	    if (HAL_UART_Receive(&huart1, (uint8_t*) UART_Rx_Buffer, 1, 100)
		    == HAL_OK)
		{
		Key = UART_Rx_Buffer[0];
		break;
		}
	    }

	if (Key == '1')
	    {
	    Modified_Current = Modified_Current + 3;
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Modified_Current); //CURRENT
	    Key = 0XFF;
	    }

	if (Key == '4')
	    {
	     if(Modified_Current >= INITIAL_CURRENT)
		 {
		 Modified_Current = Modified_Current - 3;
		 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Modified_Current); //CURRENT
		 }
	    Key = 0XFF;
	    }
	if (Key == '2')
	    {
	    Modified_Current = Modified_Current + 30;
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Modified_Current); //CURRENT

	    Key = 0XFF;
	    }

	if (Key == '5')
	    {
	    if(Modified_Current >= INITIAL_CURRENT)
	    	{
		Modified_Current = Modified_Current - 30;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Modified_Current); //CURRENT
	    	}
	    Key = 0XFF;
	    }

	if (Key == '#')
	    {
	    Flash_Write();
	    Key = 0XFF;
	    UART_Rx_Buffer[0] = 0;
	    UART_Rx_Buffer[1] = 0;
	    UART_RX_Identifier = 0;
//	    HAL_UART_Transmit(&huart1, (uint8_t*) "\r\nReturn\r\n", 10, 10);
	    return;
	    }
	}
    }
////////////////////////////////////
void VADC_Tune()
    {
    LED_OFF();
    LEDS_50_ON();
    AC_LED_OFF();
//    HAL_UART_Transmit(&huart1, (uint8_t*) "V ADC \r\n", 8, 100);
    DELAY(10);
    while (1)
	{
	while (1)
	    {
	    if (General_timer >= 1000)
		{
		Aging_VI_Output();
		General_timer = 0;
		}
	    CURRENT = CURR_SENSE(CURRENT_SENSE);

	    if (CURRENT >= CURRENT_0_5A)
	    {
		FAN_ON();
	    }
	    else
	    {
		FAN_OFF();
	    }
	    if (HAL_UART_Receive(&huart1, (uint8_t*) UART_Rx_Buffer, 1, 100)
		    == HAL_OK)
		{
		Key = UART_Rx_Buffer[0];
		break;
		}
	    }

	if (Key == '1')
	    {
	    Modified_Voltage_Read = Modified_Voltage_Read + 1;
	    Key = 0XFF;
	    }

	if (Key == '4')
	    {
	    Modified_Voltage_Read = Modified_Voltage_Read - 1;
	    Key = 0XFF;
	    }

	if (Key == '2')
	    {
	    Modified_Voltage_Read = Modified_Voltage_Read + 5;
	    Key = 0XFF;
	    }

	if (Key == '5')
	    {
	    Modified_Voltage_Read = Modified_Voltage_Read - 5;
	    Key = 0XFF;
	    }

	if (Key == '#')
	    {
	    Flash_Write();
	    Key = 0XFF;
	    UART_Rx_Buffer[0] = 0;
	    UART_Rx_Buffer[1] = 0;
	    UART_RX_Identifier = 0;
//	    HAL_UART_Transmit(&huart1, (uint8_t*) "\r\nReturn\r\n", 10, 10);
	    return;
	    }
	}
    }
////////////////////////////////////
void IADC_Tune()
    {
    LED_OFF();
    LEDS_100_ON();
    AC_LED_OFF();
//    HAL_UART_Transmit(&huart1, (uint8_t*) "I ADC \r\n", 8, 10);
    DELAY(10);
    while (1)
	{
	while (1)
	    {
	    if (General_timer >= 1000)
		{
		Aging_VI_Output();
		General_timer = 0;
		}
	    CURRENT = CURR_SENSE(CURRENT_SENSE);

	if (CURRENT >= CURRENT_0_5A)
	    {
	    FAN_ON();
	    }
	else
	    {
	    FAN_OFF();
	    }
	    if (HAL_UART_Receive(&huart1, (uint8_t*) UART_Rx_Buffer, 1, 100)
		    == HAL_OK)
		{
		Key = UART_Rx_Buffer[0];
		break;
		}
	    }

	if (Key == '1')
	    {
	    Modified_Current_Read = Modified_Current_Read + 1;
	    Key = 0XFF;
	    }

	if (Key == '4')
	    {
	    Modified_Current_Read = Modified_Current_Read - 1;
	    Key = 0XFF;
	    }

	if (Key == '2')
	    {
	    Modified_Current_Read = Modified_Current_Read + 5;
	    Key = 0XFF;
	    }

	if (Key == '5')
	    {
	    Modified_Current_Read = Modified_Current_Read - 5;
	    Key = 0XFF;
	    }

	if (Key == '#')
	    {
	    Flash_Write();
	    Key = 0XFF;
	    UART_Rx_Buffer[0] = 0;
	    UART_Rx_Buffer[1] = 0;
	    UART_RX_Identifier = 0;
//	    HAL_UART_Transmit(&huart1, (uint8_t*) "\r\nReturn\r\n", 10, 10);
	    return;
	    }
	}
    }
/*****************************************************************/
void OVP_Check()
    {

    uint16_t OVP_Test_Voltage;

	LED_OFF();
        LEDS_30_ON();
        LEDS_50_ON();
        AC_LED_OFF();
 //       HAL_UART_Transmit(&huart1, (uint8_t*) "OVP Check \r\n", 12, 10);
        DELAY(10);
        OVP_Test_Voltage = Modified_Voltage;
        Counter100msec = 0;
        Key = 0XFF;
        while (1)
    	{
    	CURRENT = CURR_SENSE(CURRENT_SENSE);

    	if (CURRENT >= CURRENT_0_7A)
    	    {
    	    FAN_ON();
    	    }
    	else
    	    {
    	    if (Counter100msec  >= 25)
    		{
    		Counter100msec = 0;
    		OVP_Test_Voltage = OVP_Test_Voltage + 1;
    		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,OVP_Test_Voltage);
    		}
    	    }

    	if (General_timer >= 250)
    	    {
    	    Aging_VI_Output();
    	    General_timer = 0;
    	    }
    	AGING_PROTECTION_CALL();
    	if(AGING_MODE_NO == AGING_OVR_VOLT_MODE)
    	    {
    	    Aging_VI_Output();
    	    BUZZER(3);
    	    Aging_VI_Output();
    	    General_timer =0;
    	    while(General_timer <= 5000)
    		{
    		 if (HAL_UART_Receive(&huart1, (uint8_t*) UART_Rx_Buffer, 1, 100)
    				    == HAL_OK)
    		     {
    			 Key = UART_Rx_Buffer[0];
 			 break;
    		     }
    		 if(Key == '#')
    		     {
    			 Key = 0xFF;
    			 UART_Rx_Buffer[0] = 0;
    			 UART_Rx_Buffer[1] = 0;
    			 UART_RX_Identifier = 0;
 //   			 HAL_UART_Transmit(&huart1, (uint8_t*) "\r\nReturn\r\n", 10, 10);
    			 AGING_MODE_NO = AGING_IDLE_MODE;
    			 return;
    		     }
    		}
    		Key = 0xFF;
    	    	UART_Rx_Buffer[0] = 0;
    	    	UART_Rx_Buffer[1] = 0;
    	    	UART_RX_Identifier = 0;
    	 //   	HAL_UART_Transmit(&huart1, (uint8_t*) "\r\nReturn\r\n", 10, 10);
    	    	AGING_MODE_NO = AGING_IDLE_MODE;
    	    	return;
 	     }	// ovp detected
   	    }
  	}

////////////////////////////////////////////////////////////////

/* USER CODE END 4 */
//**
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
