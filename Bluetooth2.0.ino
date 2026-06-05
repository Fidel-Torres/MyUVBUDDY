// =============================================================================
// UV BUDDY — Production BLE SPP Firmware
// Hardware : Seeed Studio XIAO MG24 (EFR32MG24)
// Sensor   : Adafruit LTR390 UV / Ambient Light
// Stack    : Tools > Protocol Stack > BLE (Silabs)
// =============================================================================

#ifndef ARDUINO_SILABS_STACK_BLE_SILABS
  #error "Select Tools > Protocol Stack > BLE (Silabs)"
#endif

#include <Wire.h>
#include "Adafruit_LTR390.h"
#include "sl_power_manager.h"
#include "sl_sleeptimer.h"

// =============================================================================
// Configuration
// =============================================================================

static const uint8_t  DEVICE_NAME[]       = "MYUVBUDDY";
static const uint32_t SAMPLE_INTERVAL_MS  = 1000u;
static const uint8_t  SENSOR_RETRY_LIMIT  = 3u;
static const uint16_t BLE_CONN_MIN_INT    = 400u;
static const uint16_t BLE_CONN_MAX_INT    = 800u;
static const uint16_t BLE_CONN_LATENCY    = 0u;
static const uint16_t BLE_CONN_TIMEOUT    = 600u;

// UVI threshold — fires the external signal interrupt when UVI >= this value
static const float   UVI_THRESHOLD        = 3.0f;
static const uint8_t THRESHOLD_DEBOUNCE   = 2u;

// LTR390 conversion constants (gain 18, 20-bit / 400 ms integration)
// UVI  : counts / 2300  — reference sensitivity from Liteon datasheet at GAIN_18 + 20BIT
// Lux  : ALS counts × (0.6 / (gain × integration_factor))
//        For GAIN_18, res=20-bit (400 ms): factor = 0.6 / (18 × 4) ≈ 0.0083
static const uint32_t UV_SENSITIVITY      = 2300u;
static const float    ALS_LUX_FACTOR      = 0.0083f;

// External signal bitmasks — passed to sl_bt_external_signal()
// The BLE event loop receives these as sl_bt_evt_system_external_signal_id
static const uint32_t SIG_UVI_HIGH        = (1u << 0);
static const uint32_t SIG_UVI_OK          = (1u << 1);

// =============================================================================
// UUIDs
// =============================================================================

const uuid_128 SPP_SERVICE_UUID = {
  .data = {
    0x07, 0xb9, 0xf9, 0xd7,
    0x50, 0xa4,
    0x20, 0x89,
    0x77, 0x40,
    0xcb, 0xfd,
    0x2c, 0xc1, 0x80, 0x48
  }
};

const uuid_128 SPP_DATA_CHAR_UUID = {
  .data = {
    0xd6, 0x58, 0xd6, 0x21,
    0xbc, 0x55,
    0x81, 0x9f,
    0x42, 0x44,
    0x71, 0x6d,
    0xc4, 0x6e, 0xc2, 0xfe
  }
};

// =============================================================================
// BLE state machine
// =============================================================================

typedef enum {
  STATE_DISCONNECTED = 0,
  STATE_ADVERTISING,
  STATE_CONNECTED,
  STATE_SPP_MODE
} spp_state_t;

static volatile spp_state_t g_ble_state      = STATE_DISCONNECTED;
static uint8_t              g_conn_handle     = SL_BT_INVALID_CONNECTION_HANDLE;
static uint8_t              g_max_packet_size = 20u;

static uint16_t g_gattdb_session;
static uint16_t g_ga_service_handle;
static uint16_t g_device_name_char_handle;
static uint16_t g_spp_service_handle;
static uint16_t g_spp_data_char_handle;

// =============================================================================
// Sleep-timer and wakeup flag
// =============================================================================

static sl_sleeptimer_timer_handle_t g_sample_timer;
static volatile bool                g_sensor_read_pending = false;

// =============================================================================
// Threshold state
// =============================================================================

static bool    g_above_threshold     = false;
static uint8_t g_threshold_hit_count = 0u;

// =============================================================================
// Sensor
// =============================================================================

static Adafruit_LTR390 g_ltr;
static bool            g_sensor_ok = false;

// =============================================================================
// Forward declarations
// =============================================================================

static void  ble_init_gatt_db();
static void  ble_start_advertising();
static void  ble_send_chunked(const uint8_t* data, size_t len);
static void  sample_timer_callback(sl_sleeptimer_timer_handle_t* handle, void* data);
static void  start_sample_timer();
static bool  sensor_read(uint32_t& out_uvs, uint32_t& out_als);
static float uvs_to_uvi(uint32_t uvs);
static float als_to_lux(uint32_t als);
static void  check_uvi_threshold(float uvi);

// =============================================================================
// Integration hooks — implemented in MotorButton module
// Call these to activate haptic alert and signal BLE connection state
// =============================================================================
extern void triggerUVAlert();
extern void notifyBLEConnected();

// =============================================================================
// Setup
// =============================================================================

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);

  Serial.begin(115200);
  delay(1500);
  Serial.println("[UV BUDDY] Starting...");

  Wire.begin();
  g_sensor_ok = g_ltr.begin();

  if (!g_sensor_ok) {
    Serial.println("[SENSOR] LTR390 not found — check wiring!");
  } else {
    g_ltr.setGain(LTR390_GAIN_18);
    g_ltr.setResolution(LTR390_RESOLUTION_20BIT);
    Serial.print("[SENSOR] LTR390 ready — UVI threshold: ");
    Serial.println(UVI_THRESHOLD, 1);
  }

  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);
  Serial.println("[BLE] Initializing (stack boot pending)...");
}

// =============================================================================
// Main loop
// =============================================================================

void loop()
{
  sl_power_manager_sleep();

  if (!g_sensor_read_pending) return;
  g_sensor_read_pending = false;

  if (g_ble_state != STATE_SPP_MODE) return;

  uint32_t uvs = 0, als = 0;

  if (!sensor_read(uvs, als)) {
    const char* err = "ERR=SENSOR\n";
    ble_send_chunked((const uint8_t*)err, strlen(err));
    return;
  }

  // All conversions done here on the XIAO — dashboard receives final values
  float uvi = uvs_to_uvi(uvs);
  float lux = als_to_lux(als);

  char msg[64];
  int  len = snprintf(msg, sizeof(msg),
    "UVI=%.1f LUX=%.1f UV=%lu\n",
    uvi, lux, uvs
  );

  ble_send_chunked((const uint8_t*)msg, (size_t)len);

  Serial.print("[DATA] ");
  Serial.print(msg);

  // Threshold check — fires external signal into BLE event loop if crossed
  check_uvi_threshold(uvi);
}

// =============================================================================
// Conversions
// =============================================================================

static float uvs_to_uvi(uint32_t uvs)
{
  return (float)uvs / (float)UV_SENSITIVITY;
}

static float als_to_lux(uint32_t als)
{
  return (float)als * ALS_LUX_FACTOR;
}

// =============================================================================
// Threshold check with debounce
// Uses sl_bt_external_signal() to inject an interrupt into the BLE event loop
// This is the correct pattern on EFR32 — safe to call from any context
// =============================================================================

static void check_uvi_threshold(float uvi)
{
  if (uvi >= UVI_THRESHOLD) {
    g_threshold_hit_count++;

    if (!g_above_threshold && g_threshold_hit_count >= THRESHOLD_DEBOUNCE) {
      g_above_threshold = true;
      // Fire interrupt into BLE event loop
      sl_bt_external_signal(SIG_UVI_HIGH);
    }
  } else {
    g_threshold_hit_count = 0u;

    if (g_above_threshold) {
      g_above_threshold = false;
      sl_bt_external_signal(SIG_UVI_OK);
    }
  }
}

// =============================================================================
// Sensor read with retry
// =============================================================================

static bool sensor_read(uint32_t& out_uvs, uint32_t& out_als)
{
  if (!g_sensor_ok) return false;

  for (uint8_t attempt = 0; attempt < SENSOR_RETRY_LIMIT; attempt++) {

    g_ltr.setMode(LTR390_MODE_ALS);
    delay(420);
    if (!g_ltr.newDataAvailable()) delay(80);
    out_als = g_ltr.readALS();

    g_ltr.setMode(LTR390_MODE_UVS);
    delay(420);
    if (!g_ltr.newDataAvailable()) delay(80);
    out_uvs = g_ltr.readUVS();

    if (out_als != 0xFFFFFFFF && out_uvs != 0xFFFFFFFF) return true;

    Serial.print("[SENSOR] Retry ");
    Serial.println(attempt + 1);
  }

  return false;
}

// =============================================================================
// Sleep-timer ISR
// =============================================================================

static void sample_timer_callback(sl_sleeptimer_timer_handle_t* /*handle*/,
                                  void* /*data*/)
{
  g_sensor_read_pending = true;
}

// =============================================================================
// Start periodic sample timer
// =============================================================================

static void start_sample_timer()
{
  sl_sleeptimer_stop_timer(&g_sample_timer);

  uint32_t    ticks = sl_sleeptimer_ms_to_tick(SAMPLE_INTERVAL_MS);
  sl_status_t sc    = sl_sleeptimer_start_periodic_timer(
    &g_sample_timer, ticks,
    sample_timer_callback, nullptr, 0,
    SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG
  );

  if (sc != SL_STATUS_OK) {
    Serial.print("[TIMER] Failed: 0x");
    Serial.println(sc, HEX);
  } else {
    Serial.println("[TIMER] Periodic sample timer started");
  }
}

// =============================================================================
// Send data chunked to MTU
// =============================================================================

static void ble_send_chunked(const uint8_t* data, size_t len)
{
  if (g_ble_state != STATE_SPP_MODE) return;

  size_t offset = 0;
  while (offset < len) {
    size_t chunk = min((size_t)g_max_packet_size, len - offset);

    sl_status_t sc = sl_bt_gatt_server_send_notification(
      g_conn_handle,
      g_spp_data_char_handle,
      (uint16_t)chunk,
      data + offset
    );

    if (sc != SL_STATUS_OK) {
      Serial.print("[BLE] Notification error: 0x");
      Serial.println(sc, HEX);
      break;
    }

    offset += chunk;
  }
}

// =============================================================================
// BLE Stack Event Handler
// =============================================================================

void sl_bt_on_event(sl_bt_msg_t* evt)
{
  switch (SL_BT_MSG_ID(evt->header)) {

    case sl_bt_evt_system_boot_id:
      Serial.println("[BLE] Stack booted");
      ble_init_gatt_db();
      ble_start_advertising();
      start_sample_timer();
      g_ble_state = STATE_ADVERTISING;
      break;

    case sl_bt_evt_connection_opened_id:
      Serial.println("[BLE] Connected");
      g_ble_state   = STATE_CONNECTED;
      g_conn_handle = evt->data.evt_connection_opened.connection;
      sl_bt_connection_set_parameters(
        g_conn_handle,
        BLE_CONN_MIN_INT, BLE_CONN_MAX_INT,
        BLE_CONN_LATENCY, BLE_CONN_TIMEOUT,
        0, 0xFFFF
      );
      break;

    case sl_bt_evt_connection_closed_id:
      Serial.print("[BLE] Disconnected, reason: 0x");
      Serial.println(evt->data.evt_connection_closed.reason, HEX);
      g_ble_state   = STATE_ADVERTISING;
      g_conn_handle = SL_BT_INVALID_CONNECTION_HANDLE;
      digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
      ble_start_advertising();
      break;

    case sl_bt_evt_gatt_mtu_exchanged_id:
      g_max_packet_size =
        (uint8_t)(evt->data.evt_gatt_mtu_exchanged.mtu - 3u);
      Serial.print("[BLE] MTU: ");
      Serial.println(g_max_packet_size + 3);
      break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
    {
      auto& cs = evt->data.evt_gatt_server_characteristic_status;
      if (cs.characteristic == g_spp_data_char_handle) {
        if (cs.client_config_flags == sl_bt_gatt_server_notification) {
          g_ble_state = STATE_SPP_MODE;
          digitalWrite(LED_BUILTIN, LED_BUILTIN_ACTIVE);
          Serial.println("[BLE] Notifications enabled — SPP mode active");
          notifyBLEConnected();   // ← signal BLE state to MotorButton module
        } else {
          g_ble_state = STATE_CONNECTED;
          digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
          Serial.println("[BLE] Notifications disabled");
        }
      }
    }
    break;

    // -------------------------------------------------------------------------
    // External signal interrupt handler
    // Fired by check_uvi_threshold() via sl_bt_external_signal()
    // -------------------------------------------------------------------------
    case sl_bt_evt_system_external_signal_id:
    {
      uint32_t signals = evt->data.evt_system_external_signal.extsignals;

      if (signals & SIG_UVI_HIGH) {
        Serial.println("[ALERT] UVI above threshold");
        const char* alert = "ALERT=UVI_HIGH\n";
        ble_send_chunked((const uint8_t*)alert, strlen(alert));
        digitalWrite(LED_BUILTIN, LED_BUILTIN_ACTIVE);
        triggerUVAlert();   // ← motor haptic alert (MotorButton module)
      }

      if (signals & SIG_UVI_OK) {
        Serial.println("[ALERT] UVI back below threshold");
        const char* alert = "ALERT=UVI_OK\n";
        ble_send_chunked((const uint8_t*)alert, strlen(alert));
        digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
      }
    }
    break;

    case sl_bt_evt_connection_parameters_id:
    {
      auto& p = evt->data.evt_connection_parameters;
      Serial.print("[BLE] Conn params — interval: ");
      Serial.print(p.interval);
      Serial.print(" latency: ");
      Serial.print(p.latency);
      Serial.print(" timeout: ");
      Serial.println(p.timeout);
    }
    break;

    default:
      break;
  }
}

// =============================================================================
// Advertising
// =============================================================================

static void ble_start_advertising()
{
  static uint8_t adv_handle  = 0xFF;
  static bool    initialized = false;

  if (!initialized) {
    sl_bt_advertiser_create_set(&adv_handle);
    sl_bt_advertiser_set_timing(adv_handle, 400, 400, 0, 0);
    initialized = true;
  }

  sl_bt_legacy_advertiser_generate_data(
    adv_handle,
    sl_bt_advertiser_general_discoverable
  );

  sl_bt_legacy_advertiser_start(
    adv_handle,
    sl_bt_advertiser_connectable_scannable
  );

  Serial.println("[BLE] Advertising...");
}

// =============================================================================
// GATT Database
// =============================================================================

static void ble_init_gatt_db()
{
  sl_bt_gattdb_new_session(&g_gattdb_session);

  const uint8_t ga_uuid[] = { 0x00, 0x18 };
  sl_bt_gattdb_add_service(
    g_gattdb_session, sl_bt_gattdb_primary_service,
    SL_BT_GATTDB_ADVERTISED_SERVICE,
    sizeof(ga_uuid), ga_uuid,
    &g_ga_service_handle
  );

  const sl_bt_uuid_16_t dev_name_uuid = { .data = { 0x00, 0x2A } };
  sl_bt_gattdb_add_uuid16_characteristic(
    g_gattdb_session, g_ga_service_handle,
    SL_BT_GATTDB_CHARACTERISTIC_READ,
    0x00, 0x00, dev_name_uuid,
    sl_bt_gattdb_fixed_length_value,
    sizeof(DEVICE_NAME) - 1,
    sizeof(DEVICE_NAME) - 1,
    DEVICE_NAME,
    &g_device_name_char_handle
  );
  sl_bt_gattdb_start_service(g_gattdb_session, g_ga_service_handle);

  sl_bt_gattdb_add_service(
    g_gattdb_session, sl_bt_gattdb_primary_service,
    SL_BT_GATTDB_ADVERTISED_SERVICE,
    sizeof(SPP_SERVICE_UUID),
    SPP_SERVICE_UUID.data,
    &g_spp_service_handle
  );

  uint8_t init_val = 0;
  sl_bt_gattdb_add_uuid128_characteristic(
    g_gattdb_session, g_spp_service_handle,
    SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE |
    SL_BT_GATTDB_CHARACTERISTIC_NOTIFY,
    0x00, 0x00, SPP_DATA_CHAR_UUID,
    sl_bt_gattdb_fixed_length_value,
    250, sizeof(init_val), &init_val,
    &g_spp_data_char_handle
  );
  sl_bt_gattdb_start_service(g_gattdb_session, g_spp_service_handle);

  sl_bt_gattdb_commit(g_gattdb_session);
  Serial.println("[BLE] GATT DB initialized");
}