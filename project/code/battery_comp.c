#include "battery_comp.h"

static uint8 batt_inited = 0;
static uint16 batt_adc_raw = 0;
static float batt_voltage_v = BATT_NOMINAL_V;
static float batt_scale = 1.0f;

static inline float batt_limit(float v, float min_v, float max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

void Battery_Comp_Init(void)
{
    adc_init(BATT_ADC_CHANNEL, BATT_ADC_RESOLUTION);

    /* 上电先取一次均值作为初值，避免首帧跳变 */
    batt_adc_raw = adc_mean_filter_convert(BATT_ADC_CHANNEL, 8);

    {
        float pin_v = ((float)batt_adc_raw / BATT_ADC_MAX_COUNT) * BATT_ADC_REF_V;
        float batt_v = pin_v * BATT_DIV_RATIO;
        batt_voltage_v = batt_v;
        batt_scale = batt_limit(BATT_NOMINAL_V / batt_limit(batt_voltage_v, 1.0f, 100.0f),
                                BATT_COMP_MIN_SCALE, BATT_COMP_MAX_SCALE);
    }

    batt_inited = 1;
}

void Battery_Comp_Task_10ms(void)
{
    if (!batt_inited) return;

    batt_adc_raw = adc_mean_filter_convert(BATT_ADC_CHANNEL, 8);

    {
        float pin_v = ((float)batt_adc_raw / BATT_ADC_MAX_COUNT) * BATT_ADC_REF_V;
        float batt_v_new = pin_v * BATT_DIV_RATIO;

        /* 简单异常门限：过滤明显不合理读数 */
        if (batt_v_new >= 5.0f && batt_v_new <= 26.0f)
        {
            batt_voltage_v += BATT_VOLT_LPF_ALPHA * (batt_v_new - batt_voltage_v);
        }
    }

    batt_scale = batt_limit(BATT_NOMINAL_V / batt_limit(batt_voltage_v, 1.0f, 100.0f),
                            BATT_COMP_MIN_SCALE, BATT_COMP_MAX_SCALE);
}

float Battery_Comp_Apply(float throttle_in)
{
    if (!batt_inited) return throttle_in;
    if (throttle_in < BATT_COMP_THR_GATE) return throttle_in;

    return throttle_in * batt_scale;
}

float Battery_Comp_Get_Voltage(void)
{
    return batt_voltage_v;
}

float Battery_Comp_Get_Scale(void)
{
    return batt_scale;
}

uint16 Battery_Comp_Get_AdcRaw(void)
{
    return batt_adc_raw;
}

