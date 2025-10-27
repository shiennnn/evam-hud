#include <lvgl.h>
#include <ui.h>

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include "lvgl_v8_port.h"
#include "waveshare_twai_port.h"

#include "Wire.h"
#include <WiFi.h>
#include "time.h"

const char* ssid = "Sodium chloride";
const char* password = "Na11Cl17=NaCl";
const char* ntpserver = "pool.ntp.org";
const long utcOffsetInSeconds = 28800;
const int daylightOffset_sec = 0;
static bool driver_installed = false;
const float speed_warn = 150.0;
const float batt_warn = 20.0;
const float batt_temp_warn = 80.0;

static unsigned long lastTimeUpdate = 0;
unsigned long lastArcUpdate = 0;
unsigned long lastUItimeupdate = 0;
unsigned long lastStateofDrive = 0;
unsigned long lastUIupdate = 0;
unsigned long lastCANupdate = 0;
unsigned long lastWarnBlink = 0;
unsigned long lastBatteryBlink = 0;
unsigned long lastBatteryTempBlink = 0;

bool warnBlinkState = false;
bool batterywarnBlinkState = false;
bool batteryTempwarnBlinkState = false;

char buffer[16];

using namespace esp_panel::drivers;
using namespace esp_panel::board;

void setup() {
  Serial.begin(115200);
  driver_installed = waveshare_twai_init();
  delay(1000);
  printf("\n=== Starting Setup ===\n");

  // 1. Initialize Wire FIRST before board
  printf("Initializing I2C (Wire)...\n");
  Wire.begin(8, 9);
  delay(100);

  // // 2. Connect WiFi early
  wifiSet();

  // 3. Start NTP sync (non-blocking)
  printf("Starting NTP sync...\n");
  configTime(utcOffsetInSeconds, daylightOffset_sec, ntpserver);
  printf("Waiting for NTP time sync");
  int ntp_attempts = 0;
  time_t now;
  while (ntp_attempts < 40) {
    now = time(nullptr);
    if (now > 100000) {
      printf("\n✓ Time synced: %ld\n", (long)now);
      break;
    }
    printf(".");
    delay(500);
    ntp_attempts++;
  }

  if (now < 100000) {
    printf("\n✗ NTP sync failed!\n");
  }

  // 4. Initialize board
  printf("Initializing board...\n");
  Board* board = new Board();
  board->init();

  // CRITICAL: Tell the board to skip I2C initialization for touch
  // This prevents conflict with Wire
  auto touch = board->getTouch();
  if (touch != nullptr) {
    auto touchBus = touch->getBus();
    if (touchBus != nullptr) {
      printf("Configuring touch to skip I2C init...\n");
      static_cast<BusI2C*>(touchBus)->configI2C_HostSkipInit();
    }
  }

  // Check if there's an IO Expander and skip its I2C init too
  auto ioExpander = board->getIO_Expander();
  if (ioExpander != nullptr) {
    printf("Configuring IO Expander to skip I2C init...\n");
    ioExpander->skipInitHost();
  }

#if LVGL_PORT_AVOID_TEARING_MODE
  auto lcd = board->getLCD();
  lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
  auto lcd_bus = lcd->getBus();
  if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
    static_cast<BusRGB*>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
  }
#endif
#endif

  printf("Starting board...\n");
  assert(board->begin());

  printf("Initializing LVGL...\n");
  lvgl_port_init(board->getLCD(), board->getTouch());

  printf("Creating UI...\n");
  lvgl_port_lock(-1);
  ui_init();
  lvgl_port_unlock();
  lv_timer_handler();

  printf("ESP32-S3 Master Started\n");
}

void loop() {
  lv_timer_handler();
  CAN();
  delay(50);
  stateofdrive();
  delay(50);
  current_time();
  delay(50);
  ui_update();
  delay(50);
  UIwarnings();
  delay(50);
}

void wifiSet() {
  printf("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    printf(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    printf("\n✓ WiFi connected\n");
    printf("IP: %s\n", WiFi.localIP().toString().c_str());
    printf("Connected to SSID: %s\n", WiFi.SSID().c_str());
  } else {
    printf("\n✗ WiFi connection failed!\n");
  }
}

void stateofdrive() {
  // Read I2C switch state
  int bytesReceived = Wire.requestFrom(0x08, 1);

  if (bytesReceived > 0) {
    byte switchState = Wire.read();
    printf("Switch State: %d ", switchState);

    if (switchState == 1) {
      printf("(Reverse)\n");
      lv_obj_set_style_text_color(ui_drive, lv_color_hex(0xFAF9F6), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_text_color(ui_reverse, lv_color_hex(0xEE2000), LV_PART_MAIN | LV_STATE_DEFAULT);
    } else {
      printf("(Drive)\n");
      lv_obj_set_style_text_color(ui_drive, lv_color_hex(0xEE2000), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_text_color(ui_reverse, lv_color_hex(0xFAF9F6), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
  } else {
    printf("No response from Arduino\n");
  }
}

void current_time() {
  if (millis() - lastTimeUpdate >= 1000) {
    time_t now = time(nullptr);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    char timeStr[6];
    strftime(timeStr, sizeof(timeStr), "%H:%M", &timeinfo);

    printf("Current time: %s\n", timeStr);
    lv_label_set_text(ui_time, timeStr);
    lv_label_set_text(ui_time1, timeStr);
    lv_label_set_text(ui_time2, timeStr);
    lv_label_set_text(ui_time3, timeStr);
    lastTimeUpdate = millis();
  }
}

void CAN() {
  if (!driver_installed) {
    // Driver not installed
    delay(1000);
    return;
  }
  waveshare_twai_receive();
}

void ui_update() {
  lvgl_port_lock(-1);

  if (millis() - lastArcUpdate > 50) {

    if (ui_throttlearc != nullptr) {
      lv_arc_set_value(ui_throttlearc, int(throttle_position));
      sprintf(buffer, "%.0f%%", throttle_position);
      lv_label_set_text(ui_throttlerealtime, buffer);
    }

    if (ui_speedarc != nullptr) {
      lv_arc_set_value(ui_speedarc, int(vehicle_speed));
      sprintf(buffer, "%.0f KM/H", vehicle_speed);
      lv_label_set_text(ui_speedrealtime, buffer);
      lv_label_set_text(ui_speedtop, buffer);
    }

    if (ui_speedtop1 != nullptr) {
      sprintf(buffer, "%.0f KM/H", vehicle_speed);
      lv_label_set_text(ui_speedtop1, buffer);
    }

    if (ui_speedtop3 != nullptr) {
      sprintf(buffer, "%.0f KM/H", vehicle_speed);
      lv_label_set_text(ui_speedtop3, buffer);
    }

    if (ui_speedarc1 != nullptr) {
      lv_arc_set_value(ui_speedarc1, int(vehicle_speed));
      sprintf(buffer, "%.0f KM/H", vehicle_speed);
      lv_label_set_text(ui_speedrealtime1, buffer);
      lv_label_set_text(ui_speedtop2, buffer);
    }

    if (ui_batttemparc != nullptr) {
      lv_arc_set_value(ui_batttemparc, int(battery_temp));
      sprintf(buffer, "%.0f C", battery_temp);
      lv_label_set_text(ui_batttemprealtime, buffer);
    }

    if (ui_battarc != nullptr) {
      lv_arc_set_value(ui_battarc, int(battery_state));
      sprintf(buffer, "%.0f%%", battery_state);
      lv_label_set_text(ui_battrealtime, buffer);
      lv_label_set_text(ui_batterytop, buffer);
    }

    if (ui_battarc1 != nullptr) {
      lv_arc_set_value(ui_battarc1, int(battery_state));
      sprintf(buffer, "%.0f%%", battery_state);
      lv_label_set_text(ui_battrealtime1, buffer);
      lv_label_set_text(ui_batterytop2, buffer);
    }

    if (ui_batterytop1 != nullptr) {
      sprintf(buffer, "%.0f%%", battery_state);
      lv_label_set_text(ui_batterytop1, buffer);
    }

    if (ui_batterytop3 != nullptr) {
      sprintf(buffer, "%.0f%%", battery_state);
      lv_label_set_text(ui_batterytop3, buffer);
    }
    if (ui_stateofbatt != nullptr) {
      if (battery_charging) {
        lv_obj_set_style_bg_img_opa(ui_stateofbatt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
      } else {
        lv_obj_set_style_bg_img_opa(ui_stateofbatt, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
      }
    }
    lastArcUpdate = millis();
  }
  lvgl_port_unlock();
}


void UIwarnings() {

  // Toggle blink state every 500ms
  if (millis() - lastWarnBlink >= 50) {
    warnBlinkState = !warnBlinkState;
    lastWarnBlink = millis();
  }
  if (millis() - lastBatteryBlink >= 50) {
    batterywarnBlinkState = !batterywarnBlinkState;
    lastBatteryBlink = millis();
  }
   if (millis() - lastBatteryTempBlink >= 50) {
    batteryTempwarnBlinkState = !batteryTempwarnBlinkState;
    lastBatteryTempBlink = millis();
  }
  lvgl_port_lock(-1);

  if (vehicle_speed >= speed_warn) {
    lv_obj_set_style_bg_opa(ui_speedwarn, warnBlinkState ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speedwarn1, warnBlinkState ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speedwarn2, warnBlinkState ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speedwarn3, warnBlinkState ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  } else {
    // Hide warning when speed is below threshold
    lv_obj_set_style_bg_opa(ui_speedwarn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speedwarn1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speedwarn2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speedwarn3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    warnBlinkState = false;  // Reset state when speed is safe
  }

  if (battery_state <= batt_warn) {
    lv_obj_set_style_bg_opa(ui_battwarn, batterywarnBlinkState ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_battwarn1, batterywarnBlinkState ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_battwarn2, batterywarnBlinkState ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_battwarn3, batterywarnBlinkState ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  } else {
    // Hide warning when speed is below threshold
    lv_obj_set_style_bg_opa(ui_battwarn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_battwarn1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_battwarn2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_battwarn3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    batterywarnBlinkState = false;  // Reset state when speed is safe
  }

  if (battery_temp >= batt_temp_warn) {
    lv_obj_set_style_bg_opa(ui_tempwarn, batteryTempwarnBlinkState ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_tempwarn1, batteryTempwarnBlinkState ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_tempwarn2, batteryTempwarnBlinkState ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_tempwarn3, batteryTempwarnBlinkState ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  } else {
    // Hide warning when speed is below threshold
    lv_obj_set_style_bg_opa(ui_tempwarn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_tempwarn1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_tempwarn2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_tempwarn3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    batteryTempwarnBlinkState = false;  // Reset state when speed is safe
  }

  lvgl_port_unlock();
}