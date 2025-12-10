// Распиновка подключения DRV8833:
// PA5  - EEP (спящий режим)
// Для движения вперёд назад используется ШИМ
// PA12 - IN1   (PWM4 канал)
// PA13 - IN2   (PWM5 канал)
// Для поворота используются обычные GPIO:
// PA14 - IN3   
// PA15 - IN4

#include "drv8833.h"
#include "autooff.h" 


void DRV8833_Init()
{
    GPIOA_ModeCfg(GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, GPIO_ModeOut_PP_5mA);

    // Настройка ШИМ
    PWMX_CLKCfg(6);              // такт ШИМ = Fsys / 6 = 32 MHz / 6 = 5.33 MHz
    PWMX_CycleCfg(PWMX_Cycle_255); // период = 255 тактов  Fpwm = 5.33 MHz / 255 = ~20 kHz

    AutoOff_Init(); // инициализация авто-офф модуля
}

void DRV8833_Control(uint8_t speed, uint8_t dir, uint8_t turn)
{
    AutoOff_NotifyActivity(); // сообщаем что было действие

    if (dir == DIR_FORWARD) // вперёд
    {
        PWMX_ACTOUT(CH_PWM4, speed, High_Level, ENABLE);
        PWMX_ACTOUT(CH_PWM5, 0, High_Level, ENABLE);
    }
    else if (dir == DIR_BACKWARD) // назад
    {
        PWMX_ACTOUT(CH_PWM4, 0, High_Level, ENABLE);
        PWMX_ACTOUT(CH_PWM5, speed, High_Level, ENABLE);
    }
    else if (dir == DIR_NONE) // стоп
    {
        PWMX_ACTOUT(CH_PWM4, 0, High_Level, ENABLE);
        PWMX_ACTOUT(CH_PWM5, 0, High_Level, ENABLE);
    }

    if (turn == TURN_RIGHT) // вправо
    {
        GPIOA_SetBits(GPIO_Pin_14); 
        GPIOA_ResetBits(GPIO_Pin_15); 
    }
    else if (turn == TURN_LEFT) // влево
    {
        GPIOA_SetBits(GPIO_Pin_15); 
        GPIOA_ResetBits(GPIO_Pin_14); 
    }
    else if (turn == TURN_NONE) // прямо
    {
        GPIOA_ResetBits(GPIO_Pin_14 | GPIO_Pin_15);
    }

}

// --- DRV8833 Sleep/Wake ---
void DRV8833_Sleep(FunctionalState state)
{
    if (state == ENABLE)
    {
        GPIOA_ResetBits(GPIO_Pin_5);
        PRINT("DRV8833: SLEEP\r\n");
    }
    else
    {
        GPIOA_SetBits( GPIO_Pin_5);
        PRINT("DRV8833: WAKE\r\n");
    }
}