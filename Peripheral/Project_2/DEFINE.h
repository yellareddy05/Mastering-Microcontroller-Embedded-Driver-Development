/*
 * DEFINE.h
 *
 *  Created on: Oct 11, 2021
 *      Author: Nilakantha
 */

#ifndef INC_DEFINE_H_
#define INC_DEFINE_H_

#define AC_SENSE	    ADC_CHANNEL_0
#define RESERVE_ADC	    ADC_CHANNEL_1
#define BATTERY_SENSE	ADC_CHANNEL_2
#define CHARGER_SENSE	ADC_CHANNEL_3
#define CURRENT_SENSE	ADC_CHANNEL_4

#define             OVP_SAMPLE                  4
#define             OVP_DIV                     2
#define	            NO_SAMPLES		        	16
#define	            DIV_VAL			         	4
#define             LED_BLINK_SENSE_TIME        10
#define				V_SENSE_ERROR        		0
#define             SHORT_CIRCUIT_SAMPLES	3
#define             OC_SAMPLES		30
#define             OV_SAMPLES		30
#define SHORT_CIRUCIT_CURRENT_ADC	(9 * 285)
#define OVER_VOLT_VALUE_ADC			(70 * 42) 	//(ADC_EQUIVALENT- 74V)
#define UNDER_VOLT_VALUE_ADC		(16 * 42)
#define   CRC_16_POLYNOMIALS            0xA001
#define				LED30_CHANGE_POINT			30
#define				LED50_CHANGE_POINT			50
#define				LED80_CHANGE_POINT			80
#define				LED100_CHANGE_POINT			100

#define				LED30_CHANGE_POINT_AG			580
#define				LED50_CHANGE_POINT_AG			650
#define				LED80_CHANGE_POINT_AG			690
#define				LED100_CHANGE_POINT_AG			750

#define             SEC_1                       1
#define             SEC_30                      30
#define             MIN_1                       60
#define             MIN_10                      600
#define             MIN_15                      900
#define             MIN_30                      1800
#define             MIN_45                      2700
#define             HOUR_1                      3600
#define             HOUR_2                      7200
#define             HOUR_2_30                   9000
#define             HOUR_3                      3600+7200
#define             HOUR_4                      14400
#define             HOUR_6                      21600
#define             HOUR_8                      28800
#define             HOUR_8_30                   30600
#define             HOUR_10                     36000
#define             HOUR_12                     36000+7200
/**********************VOLTAGE SENSING*******************/
#define UNDER_VOLT_VALUE_ONEAMP     (50 * 10) //(ACTUAL_VOLTAGE*ADC_EQUIVALENT)
//#define TENAMP_START_VOLTAGE	    (54.5 * 10) //(ACTUAL_VOLTAGE*ADC_EQUIVALENT)
#define UNDER_VOLT_VALUE_PROTECTION (16 * 10) //(ACTUAL_VOLTAGE*ADC_EQUIVALENT)
#define OVER_VOLT_VALUE_PROTECTION  (77 * 10) //(ACTUAL_VOLTAGE*ADC_EQUIVALENT)
//#define UNDER_VOLT_VALUE_ADC        (30 * 10)
/**********************VOLTAGE SETTING*******************/
#define OVER_VOLT_VALUE		   (77 * 42)//(69 * 42) //(ACTUAL_VOLTAGE*DAC_EQUIVALENT)
#define MUL_V_FORMULA         3.5 //0.2630952//0.3242424//0.3569230
#define V_SET_ERROR           0
#define MAX_VOLTAGE           (73 * 42) //tune to 67.2 V only (ACTUAL_VOLTAGE*DAC_EQUIVALENT)

//#define BATTERY_VOLTAGE		  (83.95 * 10)
#define MIN_CELL_VOLTAGE      2500//mV
#define PRE_MAX_CELL_VOLTAGE  3600//mV
#define MAX_CELL_VOLTAGE	  3650//mV
#define VOLTAGE_LOW_RANGE     (30 * 42)
#define BUFFER_VOLTAGE        (0.5 * 42)
/************************CURRENT SENSING***************/


#define CURRENT_0_5A        (0.5 * 10)//5 = 0.5*10
#define CURRENT_0_7A        (0.7 * 10)
#define CURRENT_1A			8//10//1.3A
#define CURRENT_2A          397//2A//1000//5A//2000//1.6V//3000/2.4V//2000//1000//397
#define CURRENT_6A			60
#define CURRENT_5A          50
#define CURRENT_4A			40
/**********************CURRENT SETTING*******************/
#define  PRE_CHARGE_CURRENT             (2 * 320)
#define   DEEP_DISCHARGE_CURRENT	    (2 * 320)
#define MUL_I_FORMULA             		1.78//1.24//1.47
#define OVER_CURRENT_VALUE              (12 * 320)//(ACTUAL_CURRENT*PWM_EQUIVALENT) //80//6.9A//70//6.2A//120//11.5A//115//10.4A//50//4.6A
#define OVER_CURRENT_VALUE_PROTECTION   (12 * 10)//(ACTUAL_CURRENT*ADC_EQUIVALENT)
#define MAX_CURRENT               		(10 * 320)//(ACTUAL_CURRENT*PWM_EQUIVALENT)
#define BATTERY_CURRENT                 (0.5 * 10)
#define BUFFER_CURRENT                  (0.5 * 10)
#define INITIAL_CURRENT			        (1.5 * 320)
//#define SHORT_CIRUCIT_CURRENT_ADC       (9 * 10)
/**********************TEMPERATURE SETTING*****************///changed OK
#define	LIMIT_OVR_TEMP_VAL     	  256 //75Degrees
#define	LIMIT_REON_TEMP_VAL       236 //65Degrees
#define MAX_CELL_TEMP             65 //Degrees
#define MIN_CELL_TEMP             0  //Degrees

/**************************** MEMORY VARIABLES ****************************/
#define LOW_RANGE		30
#define	OVP_BUFFER		4
#define OCP_BUFFER		2


#define VOLATGE_MULTIPLIER	2376		//  divide by 1000 to get orignal value in mV
#define CURRENT_MULTIPLIER	350		//  divide by 1000 to get orignal value in mA

#define ADDR_FLASH_PAGE     	63		//0x0801F800

#define ADDR_FLASH_PAGE_63    ((uint32_t)0x0801F800)
#define	DEFINED_VOLTAGE		ADDR_FLASH_PAGE_63 + 8
#define DEFINED_CURRENT		DEFINED_VOLTAGE + 8
#define VOLATGE_ADC_MULTIPLIER	DEFINED_CURRENT + 8
#define CURRENT_ADC_MULTIPLIER	VOLATGE_ADC_MULTIPLIER + 8

/**********************AC VOLTAGE SETTING*****************/

//void ADC_Select_CH0 (void);
void ADC_Select_CH0(uint32_t, uint8_t);
//void ADC_Select_CH1 (void);
void ADC_Select_CH1(uint32_t, uint8_t);
//void ADC_Select_CH2 (void);
void ADC_Select_CH2(uint32_t, uint8_t);
//void ADC_Select_CH3 (void);
void ADC_Select_CH10(uint32_t, uint8_t);
//unsigned int CURR_GETADC (unsigned char);
unsigned int CURR_GETADC (uint32_t);
//unsigned int BATT_GETADC (unsigned char);
unsigned int BATT_GETADC (uint32_t);
//unsigned int CHAG_GETADC (unsigned char);
unsigned int CHAG_GETADC(uint32_t);
//unsigned int TEMP_GETADC (unsigned char);
unsigned int AC_SENSE_GETADC(uint32_t);

void CC_CV_AC_CALL(void);
void LED_BLINK(void);
void LED_BLINK_AG(void);
void SLOW_CURRENT_INCREMENT(unsigned int INIT_CURR,unsigned int END_CURRENT_VAL);
void SLOW_CURRENT_INCREMENT_ONE_AMP(unsigned int INIT_CURR,unsigned int END_CURRENT_VAL);
void VOLTAGE_INC_SLOWLY_ONE_AMP(unsigned int INIT_VOLTAG, unsigned int END_VOLT_VAL);
void VOLTAGE_INC_SLOWLY(unsigned int INIT_VOLTAG,unsigned int END_VOLT_VAL);
void VOLTAGE_INC_SLOWLY_AG(unsigned int INIT_VOLTAG,unsigned int END_VOLT_VAL);
void AGING_SLOW_CURRENT_INCREMENT(unsigned int INIT_CURR,unsigned int END_CURRENT_VAL);
void AGING_VOLTAGE_INC_SLOWLY(unsigned int INIT_VOLTAG,unsigned int END_VOLT_VAL);
void FULLCHARGE_CALL(void);
void ALL_LEDS_OFF(void);
void LEDS_OFF(void);
void LEDS_30(void);
void LEDS_50(void);
void LEDS_80(void);
void LEDS_100(void);
void LEDS_ON(void);
void OFF_CONDITION(void);
void INIT_COND_FUN_CALL(void);
void AGING_LED_INDICATION(void);
void MIN_CURR_FUN(void);
uint8_t PROTECTION_CALL(void);

//unsigned int CHAG_SENSE(unsigned char Channel);
unsigned int CHAG_SENSE(uint32_t ADC_Channel);
//unsigned int BATT_SENSE(unsigned char Channel);
unsigned int BATT_SENSE(uint32_t ADC_Channel);
//unsigned int CURR_SENSE(unsigned char Channel);
unsigned int CURR_SENSE(uint32_t ADC_Channel);
//unsigned int TEMP_SENSE(unsigned char Channel);
unsigned int AC_VOLT_SENSE(uint32_t ADC_Channel);
void SHUTDOWN_HIGH_FUN(void);
void BUZZER(unsigned char SHORT_BEEPS);
unsigned int OVP_AVG();

//uint16_t ARRAY_12A[11]={MIN_10,MIN_45,HOUR_2_30,HOUR_2_30,HOUR_2_30,HOUR_2_30,HOUR_2,HOUR_2,HOUR_2,HOUR_2,HOUR_2};
//						   0      1       2         3         4         5         6     7       8      9     10


#endif /* INC_DEFINE_H_ */
