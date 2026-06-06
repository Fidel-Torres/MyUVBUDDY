#ifndef MOTOR_BUTTON_H
#define MOTOR_BUTTON_H

#include <Arduino.h>

// ─────────────────────────────────────────────
// DEBUG
// ─────────────────────────────────────────────
#define DEBUG_MODE 1

#if DEBUG_MODE
  #define ALERT_REPEAT_MS   10000
#else
  #define ALERT_REPEAT_MS   30000
#endif

// ─────────────────────────────────────────────
// PIN CONFIG
// ─────────────────────────────────────────────
#define MOTOR_PIN      D9
#define BUTTON_PIN     D10
#define DEBOUNCE_MS    50
#define LONG_PRESS_MS  2000
#define DOUBLE_TAP_MS  400
#define SNOOZE_MS      30000

// ─────────────────────────────────────────────
// PWM LEVELS
// ─────────────────────────────────────────────
#define PWM_OFF   0
#define PWM_LOW   100
#define PWM_MED   180
#define PWM_HIGH  230
#define PWM_MAX   255

// ─────────────────────────────────────────────
// TYPES
// ─────────────────────────────────────────────
typedef struct {
    uint8_t  duty;
    uint16_t on_ms;
    uint16_t off_ms;
    uint8_t  pulses;
} MotorPattern;

typedef enum {
    SYS_IDLE,
    SYS_ALERT,
    SYS_BLE_PAIRING,
    SYS_BLE_CONNECTED
} sys_state_t;

typedef enum {
    MOTOR_IDLE,
    MOTOR_PULSE_ON,
    MOTOR_PULSE_OFF
} motor_state_t;

// ─────────────────────────────────────────────
// API FUNCTIONS USED BY MyUVBuddyMain.ino
// ─────────────────────────────────────────────
void motorSetup();
void motorLoop();

void triggerUVAlert();
void notifyBLEConnected();
void stopHaptic();

void motorSetSedPercent(float percent);
bool motorTakeBleDisconnectRequest();

#endif
