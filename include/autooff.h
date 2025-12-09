#ifndef AUTOOFF_H
#define AUTOOFF_H

#include "CONFIG.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Инициализация auto-off модуля. Запускает собственный TMOS таск. */
void AutoOff_Init();

/* Сообщить что было действие (reset idle counter). Вызывать из DRV8833_Control или обработчиков ввода. */
void AutoOff_NotifyActivity(void);


#ifdef __cplusplus
}
#endif

#endif // AUTOOFF_H