#ifndef __TWAI_PORT_H
#define __TWAI_PORT_H

#pragma once

#include <Arduino.h>
#include "driver/twai.h"
// #include <esp_io_expander.hpp>

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 19
#define TX_PIN 20

// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

#define MotorLock 0x05
#define ECU_Status 0x08
#define BMS_Status 0X09
#define TPS_Status 0X0A
#define ThrottleBrake 0x20
#define Reverse 0x22
#define BattStatus 0x24
#define VehicleSpeed 0x38

#define MOTOR_UNLOCKED 0x00
#define MOTOR_LOCKED   0x01

#define TIMEOUT_NONE 0x00
#define TIMEOUT_THROTTLE 0x03
#define TIMEOUT_STEERING 0x04
#define TIMEOUT_WHEEL_SPEED 0x05

// ECU status codes
#define ECU_ERROR 0x00
#define ECU_ONLINE 0x01
#define ECU_OFFLINE 0x0255

#define BMS_ERROR 0x00
#define BMS_ONLINE 0x01
#define BMS_OFFLINE 0x255

#define TPS_ERROR 0x00
#define TPS_ONLINE 0x01
#define TPS_OFFLINE 0x0255

#define Forward 0x00
#define Reverse1 0x01

extern uint8_t ecu_status;
extern uint8_t bms_status;
extern uint8_t tps_status;
extern uint8_t ecu_timeout_status;
extern uint8_t reverse_status;
extern float throttle_position;
extern float vehicle_speed;
extern float battery_state;
extern float battery_temp;
extern float last_battery_state;
extern bool battery_charging;

// Add this function declaration

#define POLLING_RATE_MS 1000

bool waveshare_twai_init();
void waveshare_twai_receive();
void update_i2c_reverse(uint8_t reverse_value);
void handle_tx_message();
void update_motor_lock_state(bool locked);           // <-- Add this
void handle_motor_lock_transmission();

#endif