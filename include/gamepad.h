#ifndef GAMEPAD_H
#define GAMEPAD_H

// https://github.com/dzanis/CH582F_XBOX_SX_Controller
#include "central.h"
#include "xbox_sx_controller.h"  
#include "drv8833.h" 

#ifdef __cplusplus
extern "C" {
#endif

// Инициализация/регистрация обработчика геймпада
void gamepad_init(void);

// Вспомогательные реализации перенесены сюда (статические, чтобы не давать внешнюю видимость)
static XboxControllerState_t gamepad_state = {0};

static void handle_xbox_data(uint8_t* ble_data, uint16_t data_len)
{
    if (tmos_memcmp(&gamepad_state, ble_data, data_len))
    {
        return; // показывать только изменения
    }

    uint8_t error = Xbox_Update_State(&gamepad_state, ble_data, data_len);

    if (error == 0)
    {
        uint8_t speed = 0;
        uint8_t direction = DIR_NONE;
        uint8_t turn = TURN_NONE;

        uint16_t left_stick_y = gamepad_state.leftY;
        const uint16_t CENTER = 32768;
        const uint16_t DEADZONE = 10000;

        if (left_stick_y < CENTER - DEADZONE)
        {
            uint16_t value = (CENTER - DEADZONE) - left_stick_y;
            speed = (uint8_t)((value * 255UL) / (CENTER - DEADZONE));
            direction = DIR_FORWARD;
        }
        else if (left_stick_y > CENTER + DEADZONE)
        {
            uint16_t value = left_stick_y - (CENTER + DEADZONE);
            speed = (uint8_t)((value * 255UL) / (CENTER - DEADZONE));
            direction = DIR_BACKWARD;
        }
        else
        {
            speed = 0;
            direction = DIR_NONE;
        }

        uint16_t right_stick_x = gamepad_state.rightX;
        const uint16_t TURN_DEADZONE = 10000;

        if (right_stick_x < CENTER - TURN_DEADZONE)
        {
            turn = TURN_RIGHT;
        }
        else if (right_stick_x > CENTER + TURN_DEADZONE)
        {
            turn = TURN_LEFT;
        }
        else
        {
            turn = TURN_NONE;
        }

        DRV8833_Control(speed, direction, turn);
        PRINT("%d %d %d\n", speed, direction, turn);
    }
    else
    {
        PRINT("Error update Xbox data: %u\n", error);
    }
}

void gamepad_init(void)
{
    GAPRole_CentralInit();
    Central_Init();
    Central_RegisterGamepadInputCallback(handle_xbox_data);
}

#ifdef __cplusplus
}
#endif

#endif // GAMEPAD_H