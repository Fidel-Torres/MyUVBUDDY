#include "MotorButton.h"

// ─────────────────────────────────────────────
// MOTOR PATTERNS
// ─────────────────────────────────────────────
static const MotorPattern PAT_UV_ALERT       = { PWM_MAX,  400, 300, 3 };
static const MotorPattern PAT_SNOOZE         = { PWM_MED,  150, 100, 2 };
static const MotorPattern PAT_DISMISS        = { PWM_LOW,  100,   0, 1 };
static const MotorPattern PAT_BLE_PAIRING    = { PWM_MED,  100, 150, 2 };
static const MotorPattern PAT_BLE_CONNECTED  = { PWM_HIGH, 600,   0, 1 };
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

// New integration state
static float g_sedPercent = 0.0f;
static bool g_bleConnected = false;
static volatile bool g_bleDisconnectRequested = false;

static uint8_t tapCount = 0;
static unsigned long firstTapTime = 0;
static unsigned long snoozeUntil = 0;

// ─────────────────────────────────────────────
// FORWARD DECLARATIONS
// ─────────────────────────────────────────────
static void motorStart(const MotorPattern& p);
static void motorOff();
static void motorUpdate();
static void alertUpdate();
static void buttonISR();

static void handleSingleTap();
static void handleDoubleTap();
static void handleTripleTap();
static void handleLongPress();
static void playSedPercentPattern();

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

    // If user snoozed the alert, do not repeat until snooze expires
    if (millis() < snoozeUntil) return;

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
    if (digitalRead(BUTTON_PIN) == LOW) {
        btnFalling = true;
    } else {
        btnRising = true;
    }
}

// ─────────────────────────────────────────────
// BUTTON ACTIONS
// ─────────────────────────────────────────────
static void handleSingleTap() {
    if (systemState == SYS_ALERT) {
        motorOff();

        // Snooze means pause alert repeats temporarily
        snoozeUntil = millis() + SNOOZE_MS;
        lastAlertTime = millis();

        motorStart(PAT_SNOOZE);
        Serial.println("[BTN] Alert snoozed");
    } else {
        Serial.println("[BTN] Short tap - no active alert");
    }
}

static void handleDoubleTap() {
    if (systemState == SYS_ALERT) {
        bleNotifyPending = false;
        motorOff();

        // Stop/acknowledge means leave alert state
        if (g_bleConnected) {
            systemState = SYS_BLE_CONNECTED;
        } else {
            systemState = SYS_IDLE;
        }

        snoozeUntil = 0;
        lastAlertTime = millis();

        motorStart(PAT_DISMISS);
        Serial.println("[BTN] Alert acknowledged/stopped");
    } else {
        Serial.println("[BTN] Double tap - no active alert to stop");
    }
}

static void handleTripleTap() {
    Serial.print("[BTN] Triple tap - SED status: ");
    Serial.print(g_sedPercent, 1);
    Serial.println("%");

    motorOff();
    playSedPercentPattern();
}

static void handleLongPress() {
    if (g_bleConnected) {
        g_bleDisconnectRequested = true;
        motorOff();
        motorStart(PAT_BLE_CANCELLED);
        Serial.println("[BTN] BLE disconnect requested");
    } else {
        systemState = SYS_BLE_PAIRING;
        motorOff();
        motorStart(PAT_BLE_PAIRING);
        Serial.println("[BTN] BLE pairing mode");
    }
}

static void playSedPercentPattern() {
    MotorPattern p;

    if (g_sedPercent < 25.0f) {
        p = { PWM_LOW, 120, 150, 1 };
        Serial.println("[SED] 0-25% dose");
    } 
    else if (g_sedPercent < 50.0f) {
        p = { PWM_LOW, 120, 150, 2 };
        Serial.println("[SED] 25-50% dose");
    } 
    else if (g_sedPercent < 80.0f) {
        p = { PWM_MED, 120, 150, 3 };
        Serial.println("[SED] 50-80% dose");
    } 
    else if (g_sedPercent < 100.0f) {
        p = { PWM_HIGH, 120, 120, 4 };
        Serial.println("[SED] 80-100% dose");
    } 
    else {
        p = PAT_UV_ALERT;
        Serial.println("[SED] 100%+ dose");
    }

    motorStart(p);
}

// ─────────────────────────────────────────────
// PUBLIC API
// ─────────────────────────────────────────────
void triggerUVAlert() {
    bleNotifyPending = false;
    motorOff();

    systemState = SYS_ALERT;
    lastAlertTime = millis();

    // A new real alert cancels any previous snooze
    snoozeUntil = 0;

    motorStart(PAT_UV_ALERT);
    Serial.println("[UV] Alert triggered");
}

void notifyBLEConnected() {
    g_bleConnected = true;

    if (systemState == SYS_ALERT) {
        bleNotifyPending = true;
        Serial.println("[BLE] Connected pending (alert active)");
        return;
    }

    systemState = SYS_BLE_CONNECTED;
    motorStart(PAT_BLE_CONNECTED);
    Serial.println("[BLE] Connected");
}

void stopHaptic() {
    bleNotifyPending = false;
    g_bleConnected = false;
    motorOff();

    if (systemState == SYS_BLE_CONNECTED || systemState == SYS_BLE_PAIRING) {
        systemState = SYS_IDLE;
    }

    Serial.println("[HAPTIC] Forced off");
}

void motorSetSedPercent(float percent) {
    if (percent < 0.0f) {
        percent = 0.0f;
    }

    if (percent > 150.0f) {
        percent = 150.0f;
    }

    g_sedPercent = percent;
}

bool motorTakeBleDisconnectRequest() {
    if (g_bleDisconnectRequested) {
        g_bleDisconnectRequested = false;
        return true;
    }

    return false;
}

// ─────────────────────────────────────────────
// SETUP / LOOP
// ─────────────────────────────────────────────
void motorSetup() {
    analogWriteResolution(8);
    pinMode(MOTOR_PIN, OUTPUT);

    // Use INPUT if your circuit has external 10k pull-up.
    // Use INPUT_PULLUP if the button is wired directly D10 → button → GND.
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

            if (isLong) {
                tapCount = 0;
                handleLongPress();
            } else {
                if (tapCount == 0) {
                    firstTapTime = now;
                }

                tapCount++;

                // Cap at 3 so extra taps do not overflow behavior
                if (tapCount > 3) {
                    tapCount = 3;
                }
            }
        }
    }

    // Wait briefly to see if user is doing 1, 2, or 3 taps
    if (tapCount > 0 && (millis() - firstTapTime > DOUBLE_TAP_MS)) {
        uint8_t finalTapCount = tapCount;
        tapCount = 0;

        if (finalTapCount == 1) {
            handleSingleTap();
        } else if (finalTapCount == 2) {
            handleDoubleTap();
        } else {
            handleTripleTap();
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
        case 'A':
        case 'a':
            triggerUVAlert();
            break;

        case 'B':
        case 'b':
            notifyBLEConnected();
            break;

        case 'S':
        case 's':
            Serial.print("[DBG] systemState = ");
            Serial.println(getStateName(systemState));
            Serial.print("[DBG] SED percent = ");
            Serial.print(g_sedPercent, 1);
            Serial.println("%");
            break;

        case 'M':
        case 'm':
            Serial.print("[DBG] motorState = ");
            Serial.println(motorState == MOTOR_IDLE ? "IDLE" :
                           motorState == MOTOR_PULSE_ON ? "PULSE_ON" : "PULSE_OFF");
            break;

        case '?':
            Serial.println("A=alert B=ble S=state M=motor");
            break;

        default:
            break;
    }
}

#endif
