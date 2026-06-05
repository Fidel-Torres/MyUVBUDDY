// ============================================================
// MyUVBuddy — Button & Motor Module
// MCU    : Seeed XIAO MG24 (EFR32MG24) — Arduino IDE
// Button : Same Sky TS02 → D10 (10kΩ pull-up to 3.3V)
// Motor  : Vybronics VC1434B002U → 2N2222A BJT → D9
// ============================================================
// Integration: motorSetup() called from Bluetooth2.0 setup()
//              motorLoop()  called from Bluetooth2.0 loop()
// Public API : triggerUVAlert()    — called on SIG_UVI_HIGH / SIG_SED_HIGH
//              notifyBLEConnected() — called on SPP mode active
// ============================================================

#define DEBUG_MODE 1

#if DEBUG_MODE
    #define ALERT_REPEAT_MS   10000
#else
    #define ALERT_REPEAT_MS   30000
#endif

#define MOTOR_PIN      D9
#define BUTTON_PIN     D10
#define DEBOUNCE_MS    50
#define LONG_PRESS_MS  2000

#define PWM_OFF   0
#define PWM_LOW   100
#define PWM_MED   180
#define PWM_HIGH  230
#define PWM_MAX   255

// ─────────────────────────────────────────────────────────
// Motor pattern struct
// ─────────────────────────────────────────────────────────

typedef struct {
    uint8_t  duty;
    uint16_t on_ms;
    uint16_t off_ms;
    uint8_t  pulses;
} MotorPattern;

const MotorPattern PAT_UV_ALERT      = { PWM_MAX,  400, 300, 3 };
const MotorPattern PAT_SNOOZE        = { PWM_MED,  150, 100, 2 };
const MotorPattern PAT_DISMISS       = { PWM_LOW,  100,   0, 1 };
const MotorPattern PAT_BLE_PAIRING   = { PWM_MED,  100, 150, 2 };
const MotorPattern PAT_BLE_CONNECTED = { PWM_HIGH, 600,   0, 1 };
const MotorPattern PAT_BLE_CANCELLED = { PWM_LOW,  100,   0, 1 };

// ─────────────────────────────────────────────────────────
// State machines
// ─────────────────────────────────────────────────────────

typedef enum { SYS_IDLE, SYS_ALERT, SYS_BLE_PAIRING, SYS_BLE_CONNECTED } sys_state_t;
typedef enum { MOTOR_IDLE, MOTOR_PULSE_ON, MOTOR_PULSE_OFF }              motor_state_t;

volatile sys_state_t systemState      = SYS_IDLE;
volatile bool        btnFalling       = false;
volatile bool        btnRising        = false;
volatile bool        bleNotifyPending = false;

unsigned long  pressStart    = 0;
motor_state_t  motorState    = MOTOR_IDLE;
MotorPattern   activePattern;
uint8_t        currentPulse  = 0;
unsigned long  motorLastTime = 0;
unsigned long  lastAlertTime = 0;

// ─────────────────────────────────────────────────────────
// Motor control
// ─────────────────────────────────────────────────────────

void motorOff() {
    analogWrite(MOTOR_PIN, PWM_OFF);
    motorState   = MOTOR_IDLE;
    currentPulse = 0;
}

void motorStart(const MotorPattern& p) {
    activePattern = p;
    currentPulse  = 0;
    motorLastTime = millis();
    motorState    = MOTOR_PULSE_ON;
    analogWrite(MOTOR_PIN, p.duty);
}

void motorUpdate() {
    if (motorState == MOTOR_IDLE) {
        if (bleNotifyPending && systemState != SYS_ALERT) {
            bleNotifyPending = false;
            systemState      = SYS_BLE_CONNECTED;
            motorStart(PAT_BLE_CONNECTED);
            Serial.println("[BLE] Connected (delayed notify)");
        }
        return;
    }

    unsigned long now     = millis();
    unsigned long elapsed = now - motorLastTime;

    switch (motorState) {
        case MOTOR_PULSE_ON:
            if (elapsed >= activePattern.on_ms) {
                analogWrite(MOTOR_PIN, PWM_OFF);
                currentPulse++;
                if (currentPulse >= activePattern.pulses) {
                    motorState = MOTOR_IDLE;
                } else {
                    motorState    = MOTOR_PULSE_OFF;
                    motorLastTime = now;
                }
            }
            break;

        case MOTOR_PULSE_OFF:
            if (elapsed >= activePattern.off_ms) {
                analogWrite(MOTOR_PIN, activePattern.duty);
                motorState    = MOTOR_PULSE_ON;
                motorLastTime = now;
            }
            break;

        default: break;
    }
}

// ─────────────────────────────────────────────────────────
// Alert repeat
// ─────────────────────────────────────────────────────────

void alertUpdate() {
    if (systemState != SYS_ALERT) return;
    if (motorState  != MOTOR_IDLE) return;
    if (millis() - lastAlertTime >= ALERT_REPEAT_MS) {
        lastAlertTime = millis();
        motorStart(PAT_UV_ALERT);
        Serial.println("[ALERT] UV alert repeated");
    }
}

// ─────────────────────────────────────────────────────────
// Button ISR — only sets flags, no logic inside ISR
// ─────────────────────────────────────────────────────────

void buttonISR() {
    if (digitalRead(BUTTON_PIN) == LOW) btnFalling = true;
    else                                btnRising  = true;
}

void handleButton(bool longPress) {
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

// ─────────────────────────────────────────────────────────
// Public API — called from Bluetooth2.0.ino
// ─────────────────────────────────────────────────────────

void triggerUVAlert() {
    bleNotifyPending = false;
    if (systemState == SYS_BLE_PAIRING)
        Serial.println("[UV] Pairing interrupted by alert");
    motorOff();
    systemState   = SYS_ALERT;
    lastAlertTime = millis();
    motorStart(PAT_UV_ALERT);
    Serial.println("[UV] Alert triggered");
}

void notifyBLEConnected() {
    if (systemState == SYS_ALERT) {
        bleNotifyPending = true;
        Serial.println("[BLE] Connected — notify pending (alert active)");
        return;
    }
    systemState = SYS_BLE_CONNECTED;
    motorStart(PAT_BLE_CONNECTED);
    Serial.println("[BLE] Connected");
}

// ─────────────────────────────────────────────────────────
// Debug menu (DEBUG_MODE 1 only)
// ─────────────────────────────────────────────────────────

#if DEBUG_MODE
const char* getStateName(sys_state_t s) {
    switch (s) {
        case SYS_IDLE:          return "SYS_IDLE";
        case SYS_ALERT:         return "SYS_ALERT";
        case SYS_BLE_PAIRING:   return "SYS_BLE_PAIRING";
        case SYS_BLE_CONNECTED: return "SYS_BLE_CONNECTED";
        default:                return "UNKNOWN";
    }
}

void debugPrintMenu() {
    Serial.println("================================");
    Serial.println("  MyUVBuddy Debug Menu");
    Serial.println("================================");
    Serial.println("  A → triggerUVAlert()");
    Serial.println("  B → notifyBLEConnected()");
    Serial.println("  S → system state");
    Serial.println("  M → motor state");
    Serial.println("  ? → show menu");
    Serial.println("================================");
}

void debugUpdate() {
    if (!Serial.available()) return;
    char cmd = Serial.read();
    switch (cmd) {
        case 'A': case 'a': triggerUVAlert();     break;
        case 'B': case 'b': notifyBLEConnected(); break;
        case 'S': case 's':
            Serial.print("[DBG] systemState = ");
            Serial.println(getStateName(systemState));
            break;
        case 'M': case 'm':
            Serial.print("[DBG] motorState       = ");
            Serial.println(motorState == MOTOR_IDLE      ? "IDLE"      :
                           motorState == MOTOR_PULSE_ON  ? "PULSE_ON"  : "PULSE_OFF");
            Serial.print("[DBG] bleNotifyPending = ");
            Serial.println(bleNotifyPending ? "true" : "false");
            break;
        case '?': debugPrintMenu(); break;
        default:  break;
    }
}
#endif

// ─────────────────────────────────────────────────────────
// motorSetup / motorLoop — called from Bluetooth2.0.ino
// ─────────────────────────────────────────────────────────

void motorSetup() {
    analogWriteResolution(8);
    pinMode(MOTOR_PIN,  OUTPUT);
    pinMode(BUTTON_PIN, INPUT);
    motorOff();
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);
    Serial.println("[MOTOR] Button & motor ready");
    #if DEBUG_MODE
        debugPrintMenu();
    #endif
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
            Serial.print(isLong ? "Long press " : "Short press ");
            Serial.print(duration);
            Serial.println("ms");
            handleButton(isLong);
        }
    }
}
