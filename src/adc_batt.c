#include "adc_batt.h"


// --- Настройки ---
#define BAT_ADC_PIN   GPIO_Pin_4 
#define BAT_ADC_CHANNEL    0   // канал 0 = PA4
#define ADC_SAMPLES        8   // усредняем несколько измерений
#define ADC_VREF           3.3f
#define ADC_RESOLUTION     4096.0f
#define VOLTAGE_DIVIDER    1.0f  // 1 без делителя,а если стоит делитель то (R1+R2)/R2

static int RoughCalib_Value = 0;

// --- Инициализация ---
void ADC_Batt_Init(void)
{
    PRINT("Init Battery ADC (PA5)\r\n");
    // Настраиваем пин как вход (аналог)
    GPIOA_ModeCfg(BAT_ADC_PIN, GPIO_ModeIN_Floating);
    // Инициализация внешнего одноканального режима
    // SampleFreq_3_2 ≈ 3.2M можно выбрать быстрее/медленнее
    ADC_ExtSingleChSampInit(SampleFreq_3_2, ADC_PGA_0);
    // Грубая калибровка
    RoughCalib_Value = ADC_DataCalib_Rough();
    PRINT("ADC Calib = %d\r\n", RoughCalib_Value);
    // Выбираем канал PA5
    ADC_ChannelCfg(BAT_ADC_CHANNEL);
}

// --- Чтение значения батареи в АЦП ---
uint16_t ADC_GetBatteryADC(void)
{
    int32_t avg = 0;
    for (uint8_t i = 0; i < ADC_SAMPLES; i++)
    {
        avg +=  (int32_t)ADC_ExcutSingleConver() + RoughCalib_Value;;
    }
    avg = avg / ADC_SAMPLES;
    if(avg < 0) avg = 0;
    return avg;
}

// --- Чтение значения батареи в вольтах ---
float ADC_GetBatteryVoltage(void)
{
    uint16_t adc = ADC_GetBatteryADC();
    // V = adc / 4096 * 3.3 * (R1+R2)/R2
    float voltage = (adc / ADC_RESOLUTION) * ADC_VREF * VOLTAGE_DIVIDER;
    return voltage;
}