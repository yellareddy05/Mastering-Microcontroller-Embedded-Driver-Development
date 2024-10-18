/*
 * VARIABLES.h
 *
 *  Created on: Oct 11, 2021
 *      Author: Nilakantha
 */

#include <stdint.h>
#include <stdbool.h>


#ifndef INC_VARIABLES_H_
#define INC_VARIABLES_H_

/**************Interrupt based reading and SHTCKT protections****************/

static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t FirstPage;
uint32_t NbOfPages;
uint32_t BankNumber;
uint32_t Address;
uint32_t PageError;
__IO uint32_t Data32;
uint64_t Data64;
uint64_t Page_Data[10];

uint32_t Modified_Voltage;
uint32_t Modifier_Voltage;
uint32_t Modified_Current;
uint32_t Modifier_Current;
uint32_t Modified_OVP;
uint32_t Modified_OCP;

volatile uint16_t Modified_Voltage_Read;
volatile uint16_t Modified_Current_Read;
volatile uint16_t Live_Current_Tune;
volatile uint16_t Live_Voltage_Tune;
volatile uint32_t Live_Current_ADC;
volatile uint32_t Live_Voltage_ADC;
uint8_t Key;
uint8_t Tuning_Command;

void Aging_VI_Output();
void Aging_VI_Output_UART();
void Flash_Erase_Page();
void Flash_Write();
void VSet_Tune();
void ISet_Tune();
void VADC_Tune();
void IADC_Tune();
void OVP_Check();


uint8_t UART_Tx_Buffer[100];
uint8_t UART_Rx_Buffer[100];
uint8_t Tuning_Message[20]  = "\r\nTuning Started\r\n";
uint8_t Config_Message1[20] = "\r\nDeltic:TEP_58/00\r\n";
uint8_t Config_Message2[20] = "\r\n73.00V_10A  CAN \r\n";

/***********************************************************************/
uint16_t LIMIT_0A_CURRENT;
uint16_t LIMIT_2A_CURRENT;
uint16_t LIMIT_3A_CURRENT;
uint16_t LIMIT_5A_CURRENT;
uint16_t LIMIT_6A_CURRENT;
uint16_t LIMIT_11A_CURRENT;

uint16_t LIMIT_20_VOLTS;
uint16_t LIMIT_52_VOLTS;
uint16_t LIMIT_59_2VOLTS;
uint16_t LIMIT_65_5VOLTS;
uint16_t LIMIT_66_5VOLTS;

uint16_t LIMIT_OVER_VOLTS;
uint16_t LIMIT_OVER_CURR;
uint16_t ACVOLTAGE,VOLTAGE;
uint16_t CURRENT;
uint16_t TEMPERATURE;
uint16_t CHARGER_VOLTAGE;
uint16_t OVER_ALL_TIMER;
uint16_t TIMER_ROLL;
uint16_t SEC_TIMER;

uint8_t MODE;
uint8_t LED_SENSE_VOLTAGE_TIME;
uint8_t ITARATIONS;
uint8_t NO_BAT_DETECT_TIMES;
//uint8_t AC_SENSE;
uint16_t LED_TIMER;
uint16_t PULSE_OVER;
uint16_t ACCHECK_MODE;
uint16_t EMERGENCY_SHUTDOWN_TIME;
uint16_t REON_CHECK_TIME;
uint16_t CHECK_NO_BATTERY_TIME;

uint16_t UVT_TMR;
uint16_t OVP_TMR;
uint16_t OVC_TMR;
uint16_t OVT_TMR;
uint16_t SHT_TMR;
uint16_t AC_OVP_TMR;

uint16_t OVT_REON_TMR;
uint16_t BUZZER_TIMER;
uint16_t SAFE_MOD_TIMER;
uint16_t SAFE_SEC;
uint16_t OVP_AVG_VOLTGAE;

enum MODE_NO
{
	INITIAL_MODE, CC_CV_MODE, UV_MODE, OV_MODE, FULL_CHARGE_MODE, SELF_START_MODE,
	SHORT_CKT_MODE, OVER_CURRENT_MODE, OVR_VOLT_MODE, OVR_TEMP_MODE,
	IDLE_MODE, ONE_AMP_MODE, UNDER_VOLT_MODE, CAN_TIME_OUT, CHG_OFF_MODE,
	AC_RECOVERY_MODE, PRE_FULL_CHARGE
};

enum Charger_Status  // Bhargav
{
	CHARGER_HARDWARE=0x01, CHARGER_OT=0x02, CHARGER_INPUT=0x04, CHARGER_READY = 0x08, CHARGER_START =0x10, CHARGER_CAN=0x20,
};

enum
{
    AGING_IDLE_MODE, AGING_SHORT_CKT_MODE, AGING_OVER_CURRENT_MODE,
	AGING_OVR_VOLT_MODE, AGING_UNDER_VOLT_MODE
}AGING_MODE_NO;

bool TIMER_RESUME;
bool LEDCONSTANT_BIT;
bool AGING_MODE;
bool OVER_CURRENT_BIT = 0;
bool BLINK_30_BIT;
bool BLINK_50_BIT;
bool BLINK_80_BIT;
bool BLINK_100_BIT;
bool CONST_100_BIT;
bool PULSE_MODEBIT;
bool ACCHECK_MODEBIT;
bool REON_CHECK_TIME_BIT;
bool EMERGENCY_SHUTDOWN_TIME_BIT;
bool AGING_NORMAL, aging_soft_bit;
bool OVP_BIT;
bool OVC_BIT;
bool OVT_BIT;
bool SHT_BIT;
bool AC_OVP;
bool OVT_REON_BIT;
bool UVT_BIT;
bool one_tym;
bool CHECK_NO_BATTERY_BIT;
bool SAFE_MOD_BIT;
bool ONE_TIME_EXE_BIT;
bool REVERSE_BIT;


bool TRUE = 1;
bool FALSE = 0;


#endif /* INC_VARIABLES_H_ */
