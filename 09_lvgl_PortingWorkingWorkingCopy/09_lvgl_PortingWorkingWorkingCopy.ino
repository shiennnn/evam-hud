#include <lvgl.h>
#include <ui.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <esp_display_panel.hpp>
#include "lvgl_v8_port.h"
#include "waveshare_twai_port.h"

#include "Wire.h"
#include <WiFi.h>
#include "time.h"

#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <base64.h>

const char* ssid = "Sodium chloride";
const char* password = "Na11Cl17=NaCl";
const char* ntpserver = "pool.ntp.org";
// 

const long utcOffsetInSeconds = 28800;
const int daylightOffset_sec = 0;
static bool driver_installed = false;
const float speed_warn = 150.0;
const float batt_warn = 90.0;
const float batt_temp_warn = 20.0;

static unsigned long lastTimeUpdate = 0;
unsigned long lastArcUpdate = 0;
unsigned long lastUItimeupdate = 0;
unsigned long lastStateofDrive = 0;
unsigned long lastUIupdate = 0;
unsigned long lastCANupdate = 0;
unsigned long lastWarnBlink = 0;
unsigned long lastBatteryBlink = 0;
unsigned long lastBatteryTempBlink = 0;

double map_lat =1.342511;
double map_lon = 103.964079;
int map_zoom = 17;             // Instead of 16, download bigger tiles
const double MAP_SCALE = 1.0;  // No scaling needed
// static int16_t touch_start_x = 0;
// static int16_t touch_start_y = 0;
// static double drag_start_lat = 0;
// static double drag_start_lon = 0;

bool warnBlinkState = false;
bool batterywarnBlinkState = false;
bool batteryTempwarnBlinkState = false;

static lv_obj_t* map_img = nullptr;
static double last_map_lat = 0.0;
static double last_map_lon = 0.0;
static lv_img_dsc_t img_dsc;  // Persistent image descriptor
String spotify_access_token = "";
unsigned long token_expires_at = 3600;
unsigned long lastSpotifyUpdate = 0;
static byte lastSwitchState = 0xFF;
static byte lastStableState = 0xFF; 
extern bool motor_locked;
static bool lastMotorLockedState = false;
static unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

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
  loadMapInContainer();
  getSpotifyAccessToken();
  stateofdrive();
  lvgl_port_unlock();
  lv_timer_handler();
  printf("ESP32-S3 Master Started\n");
}

void loop() {
  lv_timer_handler();
  CAN();
  stateofdrive();
  current_time();
  ui_update();
  UIwarnings();
  sendmotor();
  delay(50);  // single short delay instead of multiple
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

    if (switchState != lastSwitchState) {
      lastSwitchState = switchState;
      lastDebounceTime = millis();
    }

    // If state has been stable long enough, accept it
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (lastStableState != lastSwitchState) {
        lastStableState = lastSwitchState;
        Serial.printf("Switch stable at %d (debounced)\n", lastStableState);

        // === Perform actions only after stable change ===
        update_i2c_reverse(lastStableState);
        handle_tx_message();

        lvgl_port_lock(-1);
        if (lastStableState == 1) {
          lv_obj_set_style_text_color(ui_drive, lv_color_hex(0xFAF9F6), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_reverse, lv_color_hex(0xEE2000), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_drive1, lv_color_hex(0xFAF9F6), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_reverse1, lv_color_hex(0xEE2000), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_drive2, lv_color_hex(0xFAF9F6), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_reverse2, lv_color_hex(0xEE2000), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_drive3, lv_color_hex(0xFAF9F6), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_reverse3, lv_color_hex(0xEE2000), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_drivelabel, lv_color_hex(0xFAF9F6), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_reverselabel, lv_color_hex(0xEE2000), LV_PART_MAIN | LV_STATE_DEFAULT);
        } else {
          lv_obj_set_style_text_color(ui_drive, lv_color_hex(0xEE2000), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_reverse, lv_color_hex(0xFAF9F6), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_drive1, lv_color_hex(0xEE2000), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_reverse1, lv_color_hex(0xFAF9F6), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_drive2, lv_color_hex(0xEE2000), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_reverse2, lv_color_hex(0xFAF9F6), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_drive3, lv_color_hex(0xEE2000), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_reverse3, lv_color_hex(0xFAF9F6), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_drivelabel, lv_color_hex(0xEE2000), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_obj_set_style_text_color(ui_reverselabel, lv_color_hex(0xFAF9F6), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        lvgl_port_unlock();
      }
    }
  } else {
    printf("No response from Arduino\n");
  }
}

void current_time() {
  if (millis() - lastTimeUpdate >= 500) {
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
  if (millis() - lastWarnBlink >= 100) {
    warnBlinkState = !warnBlinkState;
    lastWarnBlink = millis();
  }
  if (millis() - lastBatteryBlink >= 100) {
    batterywarnBlinkState = !batterywarnBlinkState;
    lastBatteryBlink = millis();
  }
  if (millis() - lastBatteryTempBlink >= 100) {
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

struct SpotifyTrack {
  String track_name;
  String artist_name;
  String album_name;
  String album_image_url;
  int duration_ms;
  int progress_ms;
  bool is_playing;
} current_track;

void spotifycredentials() {
  Serial.println("=== Spotify Credentials Debug ===");
  Serial.print("Client ID length: ");
  Serial.println(strlen(SPOTIFY_CLIENT_ID));
  Serial.print("Client Secret length: ");
  Serial.println(strlen(SPOTIFY_CLIENT_SECRET));
  Serial.print("Refresh Token length: ");
  Serial.println(strlen(SPOTIFY_REFRESH_TOKEN));
  Serial.print("Token ends with: ");
  Serial.println(String(SPOTIFY_REFRESH_TOKEN).substring(strlen(SPOTIFY_REFRESH_TOKEN) - 20));
}

bool getSpotifyAccessToken() {
  HTTPClient http;
  WiFiClientSecure client;
  client.setInsecure();

  http.begin(client, SPOTIFY_TOKEN_URL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  // Create Basic Auth header
  String auth = String(SPOTIFY_CLIENT_ID) + ":" + String(SPOTIFY_CLIENT_SECRET);
  String authHeader = "Basic " + base64::encode(auth);
  http.addHeader("Authorization", authHeader);

  // Request body
  String postData = "grant_type=refresh_token&refresh_token=" + String(SPOTIFY_REFRESH_TOKEN);

  Serial.println("Sending token request...");
  int httpCode = http.POST(postData);

  Serial.printf("HTTP Response Code: %d\n", httpCode);

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);

    spotify_access_token = doc["access_token"].as<String>();
    int expires_in = doc["expires_in"].as<int>();
    token_expires_at = millis() + (expires_in - 60) * 1000;

    Serial.println("✓ Spotify token refreshed");
    http.end();
    return true;
  } else {
    // Print the error response
    String errorResponse = http.getString();
    Serial.printf("✗ Token refresh failed: %d\n", httpCode);
    Serial.println("Error response:");
    Serial.println(errorResponse);
    http.end();
    return false;
  }
}
// void map_touch_handler(lv_event_t* e) {
//   lv_event_code_t code = lv_event_get_code(e);

//   if (code == LV_EVENT_PRESSED) {
//     lv_indev_t* indev = lv_indev_get_act();
//     lv_point_t point;
//     lv_indev_get_point(indev, &point);

//     touch_start_x = point.x;
//     touch_start_y = point.y;
//     drag_start_lat = map_lat;
//     drag_start_lon = map_lon;

//     Serial.println("Touch started");
//   }
//   else if (code == LV_EVENT_PRESSING) {
//     // Don't reload here - just calculate new position
//     lv_indev_t* indev = lv_indev_get_act();
//     lv_point_t point;
//     lv_indev_get_point(indev, &point);

//     int16_t dx = point.x - touch_start_x;
//     int16_t dy = point.y - touch_start_y;

//     double pixels_per_degree = 256.0 * pow(2.0, map_zoom) / 360.0;

//     map_lon = drag_start_lon - (dx / pixels_per_degree);
//     map_lat = drag_start_lat + (dy / pixels_per_degree);

//     if (map_lat > 85.0) map_lat = 85.0;
//     if (map_lat < -85.0) map_lat = -85.0;
//     if (map_lon > 180.0) map_lon -= 360.0;
//     if (map_lon < -180.0) map_lon += 360.0;

//     // NO loadMapInContainer() here!
//   }
//   else if (code == LV_EVENT_RELEASED) {
//     // Only reload ONCE when released
//     Serial.println("Released - reloading map");
//     loadMapInContainer();
//   }
// }

void loadMapInContainer() {
  lvgl_port_lock(-1);
  Serial.println("Starting map load...");

  if (map_img == nullptr) {
    map_img = lv_img_create(ui_gps);
    lv_obj_align(map_img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(map_img, 256, 256);
    // lv_obj_add_flag(map_img, LV_OBJ_FLAG_CLICKABLE);
    // lv_obj_clear_flag(map_img, LV_OBJ_FLAG_SCROLLABLE);  // Disable scrollable
    // lv_obj_add_event_cb(map_img, map_touch_handler, LV_EVENT_PRESSED, NULL);
    // lv_obj_add_event_cb(map_img, map_touch_handler, LV_EVENT_PRESSING, NULL);
    // lv_obj_add_event_cb(map_img, map_touch_handler, LV_EVENT_RELEASED, NULL);
    Serial.println("Map image object created.");
  }

  // Calculate tile coordinates
  int tile_x, tile_y;
  double n = pow(2.0, map_zoom);
  tile_x = (int)((map_lon + 180.0) / 360.0 * n);
  double lat_rad = map_lat * M_PI / 180.0;
  tile_y = (int)((1.0 - asinh(tan(lat_rad)) / M_PI) / 2.0 * n);

  char url[200];
  snprintf(url, sizeof(url),
           "https://tile.openstreetmap.org/%d/%d/%d.png",
           map_zoom, tile_x, tile_y);

  Serial.printf("Map URL: %s\n", url);

  HTTPClient http;
  http.begin(url);                                    // Regular HTTP, not HTTPS
  http.addHeader("User-Agent", "ESP32-Display/1.0");  // Be polite!
  http.setTimeout(15000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int httpCode = http.GET();
  Serial.printf("HTTP response code: %d\n", httpCode);

  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("Map download failed: %d\n", httpCode);
    http.end();
    lvgl_port_unlock();
    return;
  }

  size_t img_size = http.getSize();
  Serial.printf("Image size: %d bytes\n", img_size);

  if (img_size == 0 || img_size > 500000) {
    Serial.println("Invalid image size, aborting.");
    http.end();
    lvgl_port_unlock();
    return;
  }

  uint8_t* img_data = (uint8_t*)heap_caps_malloc(img_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!img_data) {
    Serial.println("Memory alloc failed");
    http.end();
    lvgl_port_unlock();
    return;
  }

  WiFiClient* stream = http.getStreamPtr();
  size_t downloaded = 0;
  uint8_t buffer[512];

  while (http.connected() && downloaded < img_size) {
    size_t avail = stream->available();
    if (avail) {
      size_t chunk = min(avail, sizeof(buffer));
      chunk = min(chunk, img_size - downloaded);
      size_t read_bytes = stream->readBytes(buffer, chunk);
      memcpy(img_data + downloaded, buffer, read_bytes);
      downloaded += read_bytes;
    }
    delay(1);
  }
  http.end();

  Serial.printf("Downloaded %d bytes\n", downloaded);

  img_dsc.header.always_zero = 0;
  img_dsc.header.w = 256;  // Tiles are 256x256
  img_dsc.header.h = 256;
  img_dsc.data_size = img_size;
  img_dsc.header.cf = LV_IMG_CF_RAW_ALPHA;
  img_dsc.data = img_data;

  lv_img_set_src(map_img, &img_dsc);

  lv_img_set_antialias(map_img, false);
 
  lv_img_set_pivot(map_img, 0, 0); // Use pivot point for cleaner scaling

  int zoom_level = (int)(MAP_SCALE * 256); // Apply zoom - convert MAP_SCALE to LVGL zoom units (256 = 1.0x)
  lv_img_set_zoom(map_img, zoom_level);

  Serial.printf("Applied zoom: %d (%.2fx)\n", zoom_level, MAP_SCALE);

  Serial.println("Image set to lv_img.");

  last_map_lat = map_lat;
  last_map_lon = map_lon;

  lvgl_port_unlock();
  Serial.println("Map load complete.");
}
void sendmotor() {
  if (motor_locked != lastMotorLockedState) {
    lastMotorLockedState = motor_locked;
    update_motor_lock_state(motor_locked);  // Update CAN state
  }

  // Send motor lock CAN message every 1 second
  handle_motor_lock_transmission();
}