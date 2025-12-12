/*******************************************************************************
* File Name          : xbox_sx_controller.h
* Author             : dzanis
* Version            : V1.1
* Date               : 16.11.2025
* Description        : Functions and structure for parsing BLE HID reports
* with the Xbox Series X controller.
********************************************************************************
* MIT License
* Copyright (c) 2025 dzanis
*******************************************************************************/
#ifndef XBOXSXCONTROLLER_H
#define XBOXSXCONTROLLER_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define XBOX_CONTROLLER_TOTAL_PACKAGE_SIZE 16 
#define XBOX_CONTROLLER_ERROR_INVALID_LENGTH 1

#define MASK_A        (1 << 0)
#define MASK_B        (1 << 1)
#define MASK_X        (1 << 3) 
#define MASK_Y        (1 << 4)
#define MASK_LB       (1 << 6)
#define MASK_RB       (1 << 7)

#define MASK_VIEW     (1 << 2)
#define MASK_MENU     (1 << 3)
#define MASK_XBOX     (1 << 4)
#define MASK_L_STICK  (1 << 5)
#define MASK_R_STICK  (1 << 6)

#define MASK_SHARE    (1 << 0)

#define DPAD_NONE        0
#define DPAD_UP          1
#define DPAD_UP_RIGHT    2
#define DPAD_RIGHT       3
#define DPAD_DOWN_RIGHT  4
#define DPAD_DOWN        5
#define DPAD_DOWN_LEFT   6
#define DPAD_LEFT        7
#define DPAD_UP_LEFT     8


typedef struct __attribute__((packed)) {
    // Sticks
    uint16_t leftX;
    uint16_t leftY;
    uint16_t rightX;
    uint16_t rightY;
    // Triggers
    uint16_t LT;
    uint16_t RT;
    // Buttons
    uint8_t  dpad;       // +
    uint8_t  buttonsABXY;    // A, B, X, Y, LB, RB
    uint8_t  buttonsCenter;  // View, Menu, Xbox, LS, RS
    uint8_t  buttonShare;    // Share
} XboxControllerState_t;

typedef enum {
    // buttonsABXY
     A,
     B,
     X,
     Y,
     LB,
     RB,
    
    // buttonsCenter
     VIEW,
     MENU,
     XBOX,
     LEFT_STICK,
     RIGHT_STICK,
    
    // buttonShare
     SHARE
    
} XboxButton_e;


uint8_t Xbox_Update_State(XboxControllerState_t *state, const uint8_t *data, size_t length)
{
    if (length != XBOX_CONTROLLER_TOTAL_PACKAGE_SIZE)
        return XBOX_CONTROLLER_ERROR_INVALID_LENGTH;

    memcpy(state, data, sizeof(XboxControllerState_t));
    return 0;
}


void Xbox_Print_State(const XboxControllerState_t *s)
{
    static const char *dpad_names[] = {
        "Neutral", "Up", "Up-Right", "Right", "Down-Right",
        "Down", "Down-Left", "Left", "Up-Left"
    };

    PRINT("\n=== Xbox Controller State ===\n");

    PRINT("Left Stick : X=%4u  Y=%4u\n", s->leftX, s->leftY);
    PRINT("Right Stick: X=%4u  Y=%4u\n", s->rightX, s->rightY);
    PRINT("Triggers   : LT=%4u  RT=%4u\n", s->LT, s->RT);

    PRINT("D-Pad      : %s (%u)\n", dpad_names[s->dpad], s->dpad);

    PRINT("Buttons ABXY+LB/RB: [ ");
    if (s->buttonsABXY & MASK_A)  PRINT("A ");
    if (s->buttonsABXY & MASK_B)  PRINT("B ");
    if (s->buttonsABXY & MASK_X)  PRINT("X ");
    if (s->buttonsABXY & MASK_Y)  PRINT("Y ");
    if (s->buttonsABXY & MASK_LB) PRINT("LB ");
    if (s->buttonsABXY & MASK_RB) PRINT("RB ");
    PRINT("]\n");

    PRINT("Center Buttons     : [ ");
    if (s->buttonsCenter & MASK_VIEW)    PRINT("View ");
    if (s->buttonsCenter & MASK_MENU)    PRINT("Menu ");
    if (s->buttonsCenter & MASK_XBOX)    PRINT("Xbox ");
    if (s->buttonsCenter & MASK_L_STICK) PRINT("LS ");
    if (s->buttonsCenter & MASK_R_STICK) PRINT("RS ");
    PRINT("]\n");

    PRINT("Share Button       : %s\n", (s->buttonShare & MASK_SHARE) ? "Pressed" : "Released");

    PRINT("=============================\n");
}


// only for button A, B, X, Y, View, Menu, Xbox, LS, RS
uint8_t Xbox_Is_Pressed(const XboxControllerState_t *state, XboxButton_e button)
{
    switch (button)
    {
        // Поле: buttonsABXY
        case  A: return (state->buttonsABXY & MASK_A);
        case  B: return (state->buttonsABXY & MASK_B);
        case  X: return (state->buttonsABXY & MASK_X);
        case  Y: return (state->buttonsABXY & MASK_Y);
        case  LB: return (state->buttonsABXY & MASK_LB);
        case  RB: return (state->buttonsABXY & MASK_RB);

        // Поле: buttonsCenter
        case  VIEW: return (state->buttonsCenter & MASK_VIEW);
        case  MENU: return (state->buttonsCenter & MASK_MENU);
        case  XBOX: return (state->buttonsCenter & MASK_XBOX);
        case  LEFT_STICK: return (state->buttonsCenter & MASK_L_STICK);
        case  RIGHT_STICK: return (state->buttonsCenter & MASK_R_STICK);

        // Поле: buttonShare
        case  SHARE: return (state->buttonShare & MASK_SHARE);
        
        default:
            return 0;
    }
}

#ifdef __cplusplus
}
#endif

#endif