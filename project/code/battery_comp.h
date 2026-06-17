#ifndef _BATTERY_COMP_H_
#define _BATTERY_COMP_H_

#include "zf_common_headfile.h"

/*
 * Battery voltage compensation module (minimal invasive integration)
 *
 * Workflow:
 * 1) Sample battery divider voltage from ADC
 * 2) Convert ADC count -> battery voltage (V)
 * 3) Low-pass filter battery voltage
 * 4) Build compensation scale:
 *      scale = V_nominal / V_batt
 * 5) Apply scale on throttle command in altitude loop
 *
 * Notes:
 * - Please verify ADC channel and divider ratio on your board.
 * - All parameters below are intentionally centralized for fast tuning.
 */

/* Hardware config */
#define BATT_ADC_CHANNEL              (ADC0_CH21_P07_5)
#define BATT_ADC_RESOLUTION           (ADC_12BIT)
#define BATT_ADC_REF_V                (3.3f)
#define BATT_ADC_MAX_COUNT            (4095.0f)
#define BATT_DIV_RATIO                (10.93f)   // Calibrated by DMM: 12.33V vs code ~8.46V

/* Filter/compensation config */
#define BATT_VOLT_LPF_ALPHA           (0.15f)
#define BATT_NOMINAL_V                (11.20f)   // Board-side nominal voltage after new power-cable drop
#define BATT_COMP_MIN_SCALE           (0.95f)
#define BATT_COMP_MAX_SCALE           (1.15f)
#define BATT_COMP_THR_GATE            (1600.0f)  // no compensation below this throttle

/* API */
void Battery_Comp_Init(void);
void Battery_Comp_Task_10ms(void);
float Battery_Comp_Apply(float throttle_in);

float Battery_Comp_Get_Voltage(void);
float Battery_Comp_Get_Scale(void);
uint16 Battery_Comp_Get_AdcRaw(void);

#endif
