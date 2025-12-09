#include "autooff.h"
#include "drv8833.h"    // для DRV8833_Sleep()


// Настройки для таймаута бездействия
#define IDLE_SLEEP_TIMEOUT_S   60   // через 60 секунд (1 минута) усыпить драйвер
#define IDLE_SHUTDOWN_TIMEOUT_S 60 * 10  // через 10 минут выключить чип

#define AUTOFF_PERIODIC_EVENT   0x0001
#define AUTOFF_PERIODIC_MS   MS1_TO_SYSTEM_TIME(1000)   // 1 секунда
  
static uint16_t autoTaskID = INVALID_TASK_ID; // ID задачи tmos
static uint32_t idleCounter = 0;



/* TMOS процессор для autooff */
static uint16_t AutoOff_ProcessEvent(uint8_t task_id, uint16_t events)
{
    if (events & AUTOFF_PERIODIC_EVENT)
    {
        idleCounter++;

        if (idleCounter == IDLE_SHUTDOWN_TIMEOUT_S) 
        {
            PRINT("Entering shutdown...\r\n");
            DelayMs(1000);
            LowPower_Shutdown(0); // полностью выключить питание
        }
        else
        if (idleCounter == IDLE_SLEEP_TIMEOUT_S) 
        {                        
            PRINT("Idle timeout\r\n");
            DRV8833_Sleep(ENABLE); // усыпляем драйвер
        }

        tmos_start_task(task_id, AUTOFF_PERIODIC_EVENT, MS1_TO_SYSTEM_TIME(AUTOFF_PERIODIC_MS));
        return (events ^ AUTOFF_PERIODIC_EVENT);
    }
    return 0;
}

void AutoOff_Init()
{
    autoTaskID = TMOS_ProcessEventRegister(AutoOff_ProcessEvent);
    idleCounter = 0;
    tmos_start_task(autoTaskID, AUTOFF_PERIODIC_EVENT, AUTOFF_PERIODIC_MS);
}

void AutoOff_NotifyActivity(void)
{
    // если было усыпление — разбудим
    if(idleCounter > IDLE_SLEEP_TIMEOUT_S)
    {
        DRV8833_Sleep(DISABLE);
    }
    idleCounter = 0; // сбрасываем счётчик бездействия
}

