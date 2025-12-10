#ifndef ADC_BATT_H
#define ADC_BATT_H

#include "CONFIG.h"

/* Инициализация ADC для измерения батареи (PA5) */
void ADC_Batt_Init(void);

/* Вернуть усреднённое значение АЦП (raw) */
uint16_t ADC_GetBatteryADC(void);

/* Вернуть напряжение батареи в вольтах */
float ADC_GetBatteryVoltage(void);

#endif // ADC_BATT_H