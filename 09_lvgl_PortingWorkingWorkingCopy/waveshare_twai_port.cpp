#include "waveshare_twai_port.h"

uint8_t ecu_status = 0xFF;
uint8_t ecu_timeout_status = 0xFF;
uint8_t bms_status = 0xFF;
uint8_t tps_status = 0xFF;
uint8_t reverse_status = 0xFF;

float throttle_position = 0.0f;
float vehicle_speed = 0.0f;
float battery_state = 0.0f;
float battery_temp = 0.0f;
float last_battery_state = 0.0f;
bool battery_charging = false;
uint8_t i2c_reverse_data = 0;
uint8_t last_sent_reverse = 0xFF;
unsigned long lastTransmitTime = 0;
unsigned long lastMotorLockTransmitTime = 0;
static twai_message_t last_sent_message;

uint8_t motor_lock_status = MOTOR_UNLOCKED;  // Default unlocked (0x00)
uint8_t last_sent_motor_lock = 0xFF;


static void send_reverse(uint8_t state) {
  if (state == last_sent_reverse) return;
  twai_clear_transmit_queue();
  twai_message_t message = { 0 };
  message.identifier = Reverse;
  message.data_length_code = 2;
  message.data[0] = state;
  message.data[1] = 0;

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    last_sent_message = message;  // save the frame
    last_sent_reverse = state;
    Serial.printf("Queued frame: ID=0x%X Data=[%02X,%02X]\n",
                  message.identifier, message.data[0], message.data[1]);
  } else {
    printf("Failed to queue message for transmission\n");  // Print failure message
  }
  memset(message.data, 0, sizeof(message.data));
}

void update_i2c_reverse(uint8_t new_state) {
  i2c_reverse_data = new_state;
}

void handle_tx_message() {
  if (i2c_reverse_data != last_sent_reverse)
    send_reverse(i2c_reverse_data);
}

static void send_motor_lock(uint8_t motor_state) {
  twai_message_t message = { 0 };
  message.identifier = MotorLock;  // 0x05
  message.data_length_code = 2;
  message.data[0] = motor_state;  // 0x00 = unlocked, 0x01 = locked
  message.data[1] = 0x00;

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    last_sent_motor_lock = motor_state;
    Serial.printf("Motor Lock CAN: ID=0x%X Data=[%02X,%02X] - %s\n",
                  message.identifier, message.data[0], message.data[1],
                  motor_state == MOTOR_LOCKED ? "LOCKED" : "UNLOCKED");
  } else {
    printf("Failed to send motor lock message\n");
  }
}
void update_motor_lock_state(bool locked) {
  motor_lock_status = locked ? MOTOR_LOCKED : MOTOR_UNLOCKED;  // true = 0x01, false = 0x00
}

// Add this function to handle periodic transmission (call every 1 second)
void handle_motor_lock_transmission() {
  unsigned long currentTime = millis();

  // Send every 1000ms (1 second)
  if (currentTime - lastMotorLockTransmitTime >= 1000) {
    send_motor_lock(motor_lock_status);
    lastMotorLockTransmitTime = currentTime;
  }
}
// Function to handle received messages
static void handle_rx_message(twai_message_t &message) {
  uint32_t id = message.identifier;

  switch (id) {
    case ECU_Status:
    case BMS_Status:
    case TPS_Status:
    case Reverse:
    case ThrottleBrake:
    case BattStatus:
    case VehicleSpeed:
      if (message.extd)  // Process received message
      {
        Serial.println("Message is in Extended Format");  // Print if the message is in extended format
      } else {
        Serial.println("Message is in Standard Format");  // Print if the message is in standard format
      }
      Serial.printf("ID: %x\nByte:", message.identifier);  // Print message ID
      if (!(message.rtr)) {                                // Check if it is not a remote transmission request
        for (int i = 0; i < message.data_length_code; i++) {
          Serial.printf(" %d = %02x,", i, message.data[i]);  // Print each byte of the message data
        }
        Serial.println("");  // Print a new line
      }
      switch (id) {
        case ECU_Status:
          if (message.data_length_code >= 2) {
            ecu_status = message.data[0];
            ecu_timeout_status = message.data[1];

            switch (ecu_status) {
              case ECU_ERROR:
                Serial.println("ECU: ERROR");
                break;
              case ECU_ONLINE:
                Serial.println("ECU: ONLINE");
                break;
              case ECU_OFFLINE:
                Serial.println("ECU: OFFLINE");
                break;
              default:
                Serial.printf("ECU: UNKNOWN (0x%02X)\n", ecu_status);
                break;
            }

            switch (ecu_timeout_status) {
              case TIMEOUT_NONE:
                Serial.println("Timeout: None");
                break;
              case TIMEOUT_THROTTLE:
                Serial.println("Timeout: THROTTLE TIMED OUT");
                break;
              case TIMEOUT_STEERING:
                Serial.println("Timeout: STEERING TIMED OUT");
                break;
              case TIMEOUT_WHEEL_SPEED:
                Serial.println("Timeout: WHEEL SPEED TIMED OUT");
                break;
              default:
                Serial.printf("Timeout: UNKNOWN (0x%02X)\n", ecu_timeout_status);
                break;
            }
          }
          break;

        case BMS_Status:
          if (message.data_length_code >= 1) {
            bms_status = message.data[0];

            switch (bms_status) {
              case BMS_ERROR:
                Serial.println("BMS: ERROR");
                break;
              case BMS_ONLINE:
                Serial.println("BMS: ONLINE");
                break;
              case BMS_OFFLINE:
                Serial.println("BMS: OFFLINE");
                break;
              default:
                Serial.printf("BMS: UNKNOWN (0x%02X)\n", bms_status);
                break;
            }
          }
          break;

        case TPS_Status:
          if (message.data_length_code >= 1) {
            tps_status = message.data[0];

            switch (tps_status) {
              case TPS_ERROR:
                Serial.println("TPS: ERROR");
                break;
              case TPS_ONLINE:
                Serial.println("TPS: ONLINE");
                break;
              case TPS_OFFLINE:
                Serial.println("TPS: OFFLINE");
                break;
              default:
                Serial.printf("TPS: UNKNOWN (0x%02X)\n", tps_status);
                break;
            }
          }
          break;

        case Reverse:

          if (message.data_length_code >= 2) {
            reverse_status = message.data[0];
            switch (reverse_status) {
              case Forward:
                Serial.println("State: Forward");
                break;

              case Reverse1:
                Serial.println("State: Reverse");
                break;
            }
          }
          break;

        case ThrottleBrake:  // CAN ID 0x20 (32 decimal)
          if (message.data_length_code >= 5) {
            uint8_t throttle_raw = message.data[0];
            throttle_position = throttle_raw * 0.4f;
            Serial.printf("Throttle Position: %.0f%%\n", throttle_position);
          }
          break;

        case BattStatus:  // CAN ID 0x24 (32 decimal)
          if (message.data_length_code >= 8) {
            uint8_t batt_raw = message.data[6];
            uint8_t batt_raw2 = message.data[7];
            battery_state = batt_raw;
            battery_temp = batt_raw2 - 40;

            battery_charging = battery_state > last_battery_state;
            last_battery_state = battery_state;

            Serial.printf("Battery SOC: %.0f %%\n", battery_state);
            Serial.printf("Battery Temp: %.0f°C\n", battery_temp);
          }
          break;
        case VehicleSpeed:
          if (message.data_length_code >= 2) {
            uint8_t speed_raw = message.data[0];
            uint8_t speed_raw2 = message.data[1];
            vehicle_speed = ((speed_raw2 * 256.0f) + speed_raw) / 256.0f;
            Serial.printf("Speed: %.0f  KM/H%\n", vehicle_speed);
          }
          break;
      }
      break;
  }  // closes switch(id)
}

bool waveshare_twai_init() {
  // Use NORMAL mode for bidirectional communication
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)TX_PIN,
    (gpio_num_t)RX_PIN,
    TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 1;
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install driver");
    return false;
  }
  Serial.println("Driver installed");

  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start driver");
    return false;
  }
  Serial.println("Driver started");

  // Enable alerts for both TX and RX
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;

  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return false;
  }

  return true;
}

void waveshare_twai_receive() {
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
    Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
    Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.println("Alert: The Transmission failed.");
    Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
    Serial.printf("TX error: %d\t", twaistatus.tx_error_counter);
    Serial.printf("TX failed: %d\n", twaistatus.tx_failed_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
    Serial.println("Alert: The Transmission was successful.");
    Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
  }

  // Handle received messages
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
    }
  }
}