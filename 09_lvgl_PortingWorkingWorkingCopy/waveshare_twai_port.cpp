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
  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();    // set 500Kbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();  // Accept all messages

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install driver");  // Print error message
    return false;                                // Return false if driver installation fails
  }
  Serial.println("Driver installed");  // Print success message

  // Start TWAI driver
  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start driver");  // Print error message
    return false;                              // Return false if starting the driver fails
  }
  Serial.println("Driver started");  // Print success message

  // Reconfigure alerts to detect frame receive, Bus-Off error, and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");  // Print success message
  } else {
    Serial.println("Failed to reconfigure alerts");  // Print error message
    return false;                                    // Return false if alert reconfiguration fails
  }

  // TWAI driver is now successfully installed and started
  return true;  // Return true on success
}

// Function to receive messages via TWAI
void waveshare_twai_receive() {
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));  // Read triggered alerts
  twai_status_info_t twaistatus;                                        // Create status info structure
  twai_get_status_info(&twaistatus);                                    // Get status information

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");  // Print passive error alert
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");  // Print bus error alert
    Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);                      // Print bus error count
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("Alert: The RX queue is full causing a received frame to be lost.");  // Print RX queue full alert
    Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);                           // Print buffered RX messages
    Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);                        // Print missed RX count
    Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);                       // Print RX overrun count
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    // One or more messages received. Handle all.
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {  // Receive messages
      handle_rx_message(message);                  // Handle each received message
    }
  }
}
