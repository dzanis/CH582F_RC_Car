#ifndef DRV8833_H
#define DRV8833_H

#include "CONFIG.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Direction */
#define DIR_NONE       0
#define DIR_FORWARD    1
#define DIR_BACKWARD   2

/* Turn */
#define TURN_NONE      0
#define TURN_RIGHT     1
#define TURN_LEFT      2

void DRV8833_Init(void);
void DRV8833_Control(uint8_t speed, uint8_t dir, uint8_t turn);
void DRV8833_Sleep(FunctionalState state);

#ifdef __cplusplus
}
#endif

#endif // DRV8833_H