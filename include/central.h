/********************************** (C) COPYRIGHT *******************************
 * File Name          : central.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/11/12
 * Description        : �۲�Ӧ��������������ϵͳ��ʼ��
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef CENTRAL_H
#define CENTRAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
// xbox controller service and characteristic UUIDs
#define GENERAL_SERV_UUID     0x1801 
// battery service and characteristic UUIDs
#define BATTERY_SERV_UUID     0x180F
#define BATTERY_LEVEL_CHAR_UUID    0x2A19
// HID service and characteristic UUIDs
#define HID_SERV_UUID    0x1812 // HID service UUID
#define INPUT_REPORT_CHAR_UUID    0x2A4D // get xbox gamepad input report characteristic UUID (stick, button, trigger data)

// Simple BLE Observer Task Events
#define START_DEVICE_EVT              0x0001
#define START_DISCOVERY_EVT           0x0002
#define START_SCAN_EVT                0x0004
#define START_SVC_DISCOVERY_EVT       0x0008
#define START_PARAM_UPDATE_EVT        0x0010
#define START_PHY_UPDATE_EVT          0x0020
#define START_READ_OR_WRITE_EVT       0x0040
#define START_WRITE_CCCD_EVT          0x0080
#define START_READ_RSSI_EVT           0x0100
#define ESTABLISH_LINK_TIMEOUT_EVT    0x0200
#define START_LED_BLINK_EVT           0x0400

/*********************************************************************
 * TYPEDEFS
 */

// Gamepad input report callback function type    
typedef void (*gamepadInputCallback_t)(uint8_t *data, uint16_t len);



/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void Central_Init(void);

/*
 * Task Event Processor for the BLE Application
 */
extern uint16_t Central_ProcessEvent(uint8_t task_id, uint16_t events);

/*
 * Register a callback function to receive gamepad input reports
 */
extern void Central_RegisterGamepadInputCallback(gamepadInputCallback_t cb);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
