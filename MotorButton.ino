#include "MotorButton.h"

// ─────────────────────────────────────────────
// MOTOR PATTERNS
// ─────────────────────────────────────────────
static const MotorPattern PAT_UV_ALERT      = { PWM_MAX,  400, 300, 3 };
static const MotorPattern PAT_SNOOZE        = { PWM_MED,  150, 100, 2 };
static const MotorPattern PAT_DISMISS       = { PWM_LOW,  100,   0, 1 };
static const MotorPattern PAT_BLE_PAIRING   = { PWM_MED,  100, 150, 2 };
static const MotorPattern PAT_BLE_CONNECTED = { PWM_HIGH, 600,   0, 1 };
static const MotorPattern PAT_BLE_CANCELLED  = { PWM_LOW,  100,   0, 1 };

// ─────────────────────────────────────────────
// STATE VARIABLES
// ─────────────────────────────────────────────
static volatile sys_state_t systemState = SYS_IDLE;
static volatile bool btnFalling = false;
static volatile bool btnRising = false;
static volatile bool bleNotifyPending = false;

static motor_state_t motorState = MOTOR_IDLE;
static MotorPattern activePattern;

static uint8_t currentPulse = 0;
static unsigned long motorLastTime = 0;
static unsigned long pressStart = 0;
static unsigned long lastAlertTime = 0;

// ─────────────────────────────────────────────
// FORWARD DECLARATIONS (fix Arduino parsing issues)
// ─────────────────────────────────────────────
static void motorStart(const MotorPattern& p);
static void motorOff();
static void motorUpdate();
static void alertUpdate();
static void handleButton(bool longPress);
static void buttonISR();

#if DEBUG_MODE
static void debugUpdate();
static const char* getStateName(sys_state_t s);
#endif

// ─────────────────────────────────────────────
// MOTOR CONTROL
// ─────────────────────────────────────────────
static void motorOff() {
    analogWrite(MOTOR_PIN, PWM_OFF);
    motorState = MOTOR_IDLE;
    currentPulse = 0;
}

static void motorStart(const MotorPattern& p) {
    activePattern = p;
    currentPulse = 0;
    motorLastTime = millis();
    motorState = MOTOR_PULSE_ON;
    analogWrite(MOTOR_PIN, p.duty);
}

static void motorUpdate() {
    if (motorState == MOTOR_IDLE) {
        if (bleNotifyPending && systemState != SYS_ALERT) {
            bleNotifyPending = false;
            systemState = SYS_BLE_CONNECTED;
            motorStart(PAT_BLE_CONNECTED);
            Serial.println("[BLE] Connected (delayed notify)");
        }
        return;
    }

    unsigned long now = millis();
    unsigned long elapsed = now - motorLastTime;

    switch (motorState) {
        case MOTOR_PULSE_ON:
            if (elapsed >= activePattern.on_ms) {
                analogWrite(MOTOR_PIN, PWM_OFF);
                currentPulse++;

                if (currentPulse >= activePattern.pulses) {
                    motorState = MOTOR_IDLE;
                } else {
                    motorState = MOTOR_PULSE_OFF;
                    motorLastTime = now;
                }
            }
            break;

        case MOTOR_PULSE_OFF:
            if (elapsed >= activePattern.off_ms) {
                analogWrite(MOTOR_PIN, activePattern.duty);
                motorState = MOTOR_PULSE_ON;
                motorLastTime = now;
            }
            break;

        default:
            break;
    }
}

// ─────────────────────────────────────────────
// ALERT SYSTEM
// ─────────────────────────────────────────────
static void alertUpdate() {
    if (systemState != SYS_ALERT) return;
    if (motorState != MOTOR_IDLE) return;

    if (millis() - lastAlertTime >= ALERT_REPEAT_MS) {
        lastAlertTime = millis();
        motorStart(PAT_UV_ALERT);
        Serial.println("[ALERT] UV alert repeated");
    }
}

// ─────────────────────────────────────────────
// BUTTON ISR
// ─────────────────────────────────────────────
static void buttonISR() {
    if (digitalRead(BUTTON_PIN) == LOW) btnFalling = true;
    else btnRising = true;
}

// ─────────────────────────────────────────────
// BUTTON HANDLER
// ─────────────────────────────────────────────
static void handleButton(bool longPress) {
    switch (systemState) {

        case SYS_IDLE:
            if (longPress) {
                systemState = SYS_BLE_PAIRING;
                motorStart(PAT_BLE_PAIRING);
                Serial.println("[BTN] BLE pairing started");
            }
            break;

        case SYS_ALERT:
            if (!longPress) {
                motorOff();
                lastAlertTime = millis();
                motorStart(PAT_SNOOZE);
                Serial.println("[BTN] Alert snoozed");
            } else {
                bleNotifyPending = false;
                motorOff();
                systemState = SYS_IDLE;
                motorStart(PAT_DISMISS);
                Serial.println("[BTN] Alert dismissed");
            }
            break;

        case SYS_BLE_PAIRING:
            if (!longPress) {
                motorOff();
                systemState = SYS_IDLE;
                motorStart(PAT_BLE_CANCELLED);
                Serial.println("[BTN] Pairing cancelled");
            }
            break;

        case SYS_BLE_CONNECTED:
            if (longPress) {
                systemState = SYS_IDLE;
                Serial.println("[BTN] BLE disconnected");
            }
            break;
    }
}

// ─────────────────────────────────────────────
// PUBLIC API
// ─────────────────────────────────────────────
void triggerUVAlert() {
    bleNotifyPending = false;
    motorOff();

    systemState = SYS_ALERT;
    lastAlertTime = millis();

    motorStart(PAT_UV_ALERT);
    Serial.println("[UV] Alert triggered");
}

void notifyBLEConnected() {
    if (systemState == SYS_ALERT) {
        bleNotifyPending = true;
        Serial.println("[BLE] Connected pending (alert active)");
        return;
    }

    systemState = SYS_BLE_CONNECTED;
    motorStart(PAT_BLE_CONNECTED);
    Serial.println("[BLE] Connected");
}

// ─────────────────────────────────────────────
// SETUP / LOOP
// ─────────────────────────────────────────────
void motorSetup() {
    analogWriteResolution(8);
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);

    motorOff();

    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);

    Serial.println("[MOTOR] Ready");
}

void motorLoop() {
    motorUpdate();
    alertUpdate();

#if DEBUG_MODE
    debugUpdate();
#endif

    if (btnFalling) {
        btnFalling = false;
        pressStart = millis();
    }

    if (btnRising) {
        btnRising = false;

        static unsigned long lastRelease = 0;
        unsigned long now = millis();

        if (now - lastRelease >= DEBOUNCE_MS) {
            lastRelease = now;

            unsigned long duration = now - pressStart;
            bool isLong = (duration >= LONG_PRESS_MS);

            Serial.print("[BTN] ");
            Serial.print(isLong ? "Long " : "Short ");
            Serial.print(duration);
            Serial.println("ms");

            handleButton(isLong);
        }
    }
}

// ─────────────────────────────────────────────
// DEBUG
// ─────────────────────────────────────────────
#if DEBUG_MODE

static const char* getStateName(sys_state_t s) {
    switch (s) {
        case SYS_IDLE: return "SYS_IDLE";
        case SYS_ALERT: return "SYS_ALERT";
        case SYS_BLE_PAIRING: return "SYS_BLE_PAIRING";
        case SYS_BLE_CONNECTED: return "SYS_BLE_CONNECTED";
        default: return "UNKNOWN";
    }
}

static void debugUpdate() {
    if (!Serial.available()) return;

    char cmd = Serial.read();

    switch (cmd) {
        case 'A': triggerUVAlert(); break;
        case 'B': notifyBLEConnected(); break;

        case 'S':
            Serial.println(getStateName(systemState));
            break;

        case '?':
            Serial.println("A=alert B=ble S=state");
            break;
    }
}

#endif