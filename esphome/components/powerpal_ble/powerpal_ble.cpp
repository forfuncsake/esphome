#include "powerpal_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32

namespace esphome {
namespace powerpal_ble {

static const char *const TAG = "powerpal_ble";

void Powerpal::dump_config() {
  ESP_LOGCONFIG(TAG, "POWERPAL");
  LOG_SENSOR(" ", "Battery", this->battery_);
  LOG_SENSOR(" ", "Power", this->power_sensor_);
  LOG_SENSOR(" ", "Daily Energy", this->daily_energy_sensor_);
  LOG_SENSOR(" ", "Total Energy", this->energy_sensor_);
}

void Powerpal::setup() {
  this->authenticated_ = false;
  this->batch_size_set_ - false;
  this->collecting_ = false;
  this->pulse_multiplier_ = ((seconds_in_minute * this->reading_batch_size_[0]) / (this->pulses_per_kwh_ / kw_to_w_conversion));
  ESP_LOGD(TAG, "pulse_multiplier_: %f", this->pulse_multiplier_ );

#ifdef USE_HTTP_REQUEST
    this->stored_measurements_.resize(15); //TODO dynamic
    this->ingesting_history_ = false;
    this->recent_ts_ = 0;
    this->requested_ts_ = 0;
    this->uploaded_ts_ = 0;
#endif
}

std::string Powerpal::pkt_to_hex_(const uint8_t *data, uint16_t len) {
  char buf[64];
  memset(buf, 0, 64);
  for (int i = 0; i < len; i++)
    sprintf(&buf[i * 2], "%02x", data[i]);
  std::string ret = buf;
  return ret;
}

void Powerpal::decode_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
}

void Powerpal::parse_battery_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "Battery: DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
  if (length == 1) {
    this->battery_->publish_state(data[0]);
  }
}

void Powerpal::parse_measurement_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "Meaurement: DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
  if (length >= 6) {
    time_t unix_time = data[0];
    unix_time += (data[1] << 8);
    unix_time += (data[2] << 16);
    unix_time += (data[3] << 24);

#ifdef USE_HTTP_REQUEST
    if (this->ingesting_history_ && unix_time > this->recent_ts_) {
      // Drop this record for now, we'll get it again later once caught up.
      ESP_LOGD(TAG, "Dropping measurement for %ld while we ingest history", unix_time);
      ESP_LOGD(TAG, "Requested: %ld, Recent: %ld", this->requested_ts_, this->recent_ts_);
      this->recent_ts_ = unix_time;
      return;
    }
    if (unix_time >= this->recent_ts_) {
      // we're all caught up
      this->ingesting_history_ = false;
    }
#endif

    uint16_t pulses_within_interval = data[4];
    pulses_within_interval += data[5] << 8;

    // float total_kwh_within_interval = pulses_within_interval / this->pulses_per_kwh_;
    float avg_watts_within_interval = pulses_within_interval * this->pulse_multiplier_;

    ESP_LOGI(TAG, "Timestamp: %ld, Pulses: %d, Average Watts within interval: %f W", unix_time, pulses_within_interval,
             avg_watts_within_interval);

    if (this->power_sensor_ != nullptr) {
      this->power_sensor_->publish_state(avg_watts_within_interval);
    }

    if (this->energy_sensor_ != nullptr) {
      this->total_pulses_ += pulses_within_interval;
      float energy = this->total_pulses_ / this->pulses_per_kwh_;
      this->energy_sensor_->publish_state(energy);
    }

    if (this->daily_energy_sensor_ != nullptr) {
      // even if new day, publish last measurement window before resetting
      this->daily_pulses_ += pulses_within_interval;
      float energy = this->daily_pulses_ / this->pulses_per_kwh_;
      this->daily_energy_sensor_->publish_state(energy);

      // if esphome device has a valid time component set up, use that (preferred)
      // else, use the powerpal measurement timestamps
#ifdef USE_TIME
      auto *time_ = *this->time_;
      ESPTime date_of_measurement = time_->now();
      if (date_of_measurement.is_valid()) {
        if (this->day_of_last_measurement_ == 0) { this->day_of_last_measurement_ = date_of_measurement.day_of_year;}
        else if (this->day_of_last_measurement_ != date_of_measurement.day_of_year) {
          this->daily_pulses_ = 0;
          this->day_of_last_measurement_ = date_of_measurement.day_of_year;
        }
      } else {
        // if !date_of_measurement.is_valid(), user may have a bare "time:" in their yaml without a specific platform selected, so fallback to date of powerpal measurement
#else
        // avoid using ESPTime here so we don't need a time component in the config
        struct tm *date_of_measurement = ::localtime(&unix_time);
        // date_of_measurement.tm_yday + 1 because we are matching ESPTime day of year (1-366 instead of 0-365), which lets us catch a day_of_last_measurement_ of 0 as uninitialised
        if (this->day_of_last_measurement_ == 0) { this->day_of_last_measurement_ = date_of_measurement->tm_yday + 1 ;}
        else if (this->day_of_last_measurement_ != date_of_measurement->tm_yday + 1) {
          this->daily_pulses_ = 0;
          this->day_of_last_measurement_ = date_of_measurement->tm_yday + 1;
        }
#endif
#ifdef USE_TIME
      }
#endif
    }

#ifdef USE_HTTP_REQUEST
    if(this->cloud_uploader_ != nullptr) {
      this->store_measurement_(
        pulses_within_interval,
        unix_time,
        (uint32_t)roundf(pulses_within_interval * (this->pulses_per_kwh_ / kw_to_w_conversion)),
        (pulses_within_interval / this->pulses_per_kwh_) * this->energy_cost_
      );
      if (this->stored_measurements_count_ == 15) {
        this->upload_data_to_cloud_();
      }
      if (this->ingesting_history_ && unix_time == this->requested_ts_) {
        // give the system half a sec to breathe
        //std::this_thread::sleep_for(std::chrono::milliseconds(800));

        // request next batch of history
        this->requested_ts_ += 60;
        uint8_t payload[8];
        payload[0] = (this->requested_ts_ & 0xff);
        payload[1] = ((this->requested_ts_ >> 8) & 0xff);
        payload[2] = ((this->requested_ts_ >> 16) & 0xff);
        payload[3] = ((this->requested_ts_ >> 24) & 0xff);

        this->requested_ts_ += 840;
        if (this->requested_ts_ > this->recent_ts_) {
          this->requested_ts_ = this->recent_ts_;
        }
        payload[4] = (this->requested_ts_ & 0xff);
        payload[5] = ((this->requested_ts_ >> 8) & 0xff);
        payload[6] = ((this->requested_ts_ >> 16) & 0xff);
        payload[7] = ((this->requested_ts_ >> 24) & 0xff);

        ESP_LOGD(TAG, "Requesting MeasurementAccess: DEC(%d): 0x%s", length, this->pkt_to_hex_(payload, 8).c_str());

        auto maStatus =
                esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                        this->measurement_access_char_handle_, 8,
                                        payload, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (maStatus) {
          ESP_LOGW(TAG, "Error sending write request for measurementAccess, status=%d", maStatus);
        }
      }
    }
#endif
  }
}

std::string Powerpal::uuid_to_device_id_(const uint8_t *data, uint16_t length) {
  const char* hexmap[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "a", "b", "c", "d", "e", "f"};
  std::string device_id;
  for (int i = length-1; i >= 0; i--) {
    device_id.append(hexmap[(data[i] & 0xF0) >> 4]);
    device_id.append(hexmap[data[i] & 0x0F]);
  }
  return device_id;
}

std::string Powerpal::serial_to_apikey_(const uint8_t *data, uint16_t length) {
  const char* hexmap[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "a", "b", "c", "d", "e", "f"};
  std::string api_key;
  for (int i = 0; i < length; i++) {
    if ( i == 4 || i == 6 || i == 8 || i == 10 ) {
      api_key.append("-");
    }
    api_key.append(hexmap[(data[i] & 0xF0) >> 4]);
    api_key.append(hexmap[data[i] & 0x0F]);
  }
  return api_key;
}

#ifdef USE_HTTP_REQUEST
void Powerpal::process_first_rec_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "First Record: DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
  if (length < 8) {
    return;
  }

  this->recent_ts_ = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | (data[4]);
  this->requested_ts_ = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | (data[0]);
  ESP_LOGI(TAG, "Powerpal has records stored from %ld to %ld", this->requested_ts_, this->recent_ts_);

  if (this->requested_ts_ < this->uploaded_ts_) {
    this->requested_ts_ = this->uploaded_ts_ + 60;
  }

  uint8_t payload[8];
  payload[0] = (this->requested_ts_ & 0xff);
  payload[1] = ((this->requested_ts_ >> 8) & 0xff);
  payload[2] = ((this->requested_ts_ >> 16) & 0xff);
  payload[3] = ((this->requested_ts_ >> 24) & 0xff);

  // Request history in 15-measurement batches.
  this->requested_ts_ += 840;
  if (this->requested_ts_ > this->recent_ts_) {
    this->requested_ts_ = this->recent_ts_;
  }
  payload[4] = (this->requested_ts_ & 0xff);
  payload[5] = ((this->requested_ts_ >> 8) & 0xff);
  payload[6] = ((this->requested_ts_ >> 16) & 0xff);
  payload[7] = ((this->requested_ts_ >> 24) & 0xff);

  ESP_LOGD(TAG, "Requesting MeasurementAccess: DEC(%d): 0x%s", length, this->pkt_to_hex_(payload, 8).c_str());

  auto maStatus =
          esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                   this->measurement_access_char_handle_, 8,
                                   payload, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (maStatus) {
    ESP_LOGW(TAG, "Error sending write request for measurementAccess, status=%d", maStatus);
  }
}

void Powerpal::store_measurement_(uint16_t pulses, time_t timestamp, uint32_t watt_hours, float cost) {
  this->stored_measurements_[this->stored_measurements_count_].pulses = pulses;
  this->stored_measurements_[this->stored_measurements_count_].timestamp = timestamp;
  this->stored_measurements_[this->stored_measurements_count_].watt_hours = watt_hours;
  this->stored_measurements_[this->stored_measurements_count_].cost = cost;
  this->stored_measurements_count_++;
}

void Powerpal::upload_data_to_cloud_() {
  this->stored_measurements_count_ = 0;
  if (this->powerpal_device_id_.length() && this->powerpal_apikey_.length()) {
    StaticJsonDocument<2048> doc; // 768 bytes, each entry may take up 15 bytes (uint16_t + uint32_t + uint32_t + float + bool)
    JsonArray array = doc.to<JsonArray>();
    for (int i = 0; i < 15; i++) {
      if (this->stored_measurements_[i].timestamp > 0) {
        JsonObject nested = array.createNestedObject();
        nested["timestamp"] = this->stored_measurements_[i].timestamp;
        nested["pulses"] = this->stored_measurements_[i].pulses;
        nested["watt_hours"] = this->stored_measurements_[i].watt_hours;
        nested["cost"] = this->stored_measurements_[i].cost;
        nested["is_peak"] = false;
      }
    }
    std::string body;
    serializeJson(doc, body);

    this->cloud_uploader_->post(this->powerpal_api_root_ + this->powerpal_api_meter_, body, this->powerpal_headers_);
  } else {
    // apikey or device missing
  }
}
#endif

void Powerpal::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                   esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_DISCONNECT_EVT: {
      this->authenticated_ = false;
      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      break;
    }
    case ESP_GATTC_READ_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_READ_CHAR_EVT (Received READ)", this->parent()->address_str().c_str());
      if (param->read.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error reading char at handle %d, status=%d", param->read.handle, param->read.status);
        break;
      }
      // reading batch size
      if (param->read.handle == this->reading_batch_size_char_handle_) {
        ESP_LOGD(TAG, "Recieved reading_batch_size read event");
        this->decode_(param->read.value, param->read.value_len);
        if (param->read.value_len == 4) {
          if (param->read.value[0] != this->reading_batch_size_[0]) {
            // reading batch size needs changing, so write
            auto status =
                esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                         this->reading_batch_size_char_handle_, sizeof(this->reading_batch_size_),
                                         this->reading_batch_size_, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
            if (status) {
              ESP_LOGW(TAG, "Error sending write request for batch_size, status=%d", status);
            }
          } else {
            this->batch_size_set_ = true;
          }
        } else {
          // error, length should be 4
        }
        break;
      }

      // battery
      if (param->read.handle == this->battery_char_handle_) {
        ESP_LOGD(TAG, "Recieved battery read event");
        this->parse_battery_(param->read.value, param->read.value_len);
        break;
      }

      // firmware
      if (param->read.handle == this->firmware_char_handle_) {
        ESP_LOGD(TAG, "Recieved firmware read event");
        this->decode_(param->read.value, param->read.value_len);
        break;
      }

#ifdef USE_HTTP_REQUEST
      // first record
      if (param->read.handle == this->first_rec_char_handle_) {
        ESP_LOGD(TAG, "Recieved first record read event");
        this->process_first_rec_(param->read.value, param->read.value_len);
        break;
      }
#endif

      // led sensitivity
      if (param->read.handle == this->led_sensitivity_char_handle_) {
        ESP_LOGD(TAG, "Recieved led sensitivity read event");
        this->decode_(param->read.value, param->read.value_len);
        break;
      }

      // serialNumber
      if (param->read.handle == this->serial_number_char_handle_) {
        ESP_LOGI(TAG, "Recieved uuid read event");
        this->powerpal_device_id_ = this->uuid_to_device_id_(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "Powerpal device id: %s", this->powerpal_device_id_.c_str());
#ifdef USE_HTTP_REQUEST
        this->powerpal_api_device_.append(this->powerpal_device_id_);
        this->powerpal_api_meter_.append(this->powerpal_device_id_);
#endif
        break;
      }

      // uuid
      if (param->read.handle == this->uuid_char_handle_) {
        ESP_LOGI(TAG, "Recieved serial_number read event");
        this->powerpal_apikey_ = this->serial_to_apikey_(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "Powerpal apikey: %s", this->powerpal_apikey_.c_str());
#ifdef USE_HTTP_REQUEST
        http_request::Header acceptheader;
        acceptheader.name = "Accept";
        acceptheader.value = "application/json";
        http_request::Header contentheader;
        contentheader.name = "Content-Type";
        contentheader.value = "application/json";
        http_request::Header authheader;
        authheader.name = "Authorization";
        authheader.value = this->powerpal_apikey_.c_str();
        this->powerpal_headers_.push_back(acceptheader);
        this->powerpal_headers_.push_back(contentheader);
        this->powerpal_headers_.push_back(authheader);
#endif
        break;
      }

      break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_WRITE_CHAR_EVT (Write confirmed)", this->parent()->address_str().c_str());
      if (param->write.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error writing value to char at handle %d, status=%d", param->write.handle, param->write.status);
        break;
      }

      if (param->write.handle == this->pairing_code_char_handle_ && !this->authenticated_) {
        this->authenticated_ = true;

        // read measurement notification batch size
        auto read_reading_batch_size_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->reading_batch_size_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_reading_batch_size_status) {
          ESP_LOGW(TAG, "Error sending read request for reading batch size, status=%d", read_reading_batch_size_status);
        }

        if (!this->powerpal_apikey_.length()) {
          // read uuid (apikey)
          auto read_uuid_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                            this->uuid_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_uuid_status) {
            ESP_LOGW(TAG, "Error sending read request for powerpal uuid, status=%d", read_uuid_status);
          }
        }

        if (!this->powerpal_device_id_.length()) {
          // read serial number (device id)
          auto read_serial_number_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                            this->serial_number_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_serial_number_status) {
            ESP_LOGW(TAG, "Error sending read request for powerpal serial number, status=%d", read_serial_number_status);
          }
        }

        if (this->battery_ != nullptr) {
          // read battery
          auto read_battery_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                             this->battery_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_battery_status) {
            ESP_LOGW(TAG, "Error sending read request for battery, status=%d", read_battery_status);
          }
          // Enable notifications for battery
          auto notify_battery_status = esp_ble_gattc_register_for_notify(
              this->parent()->get_gattc_if(), this->parent()->get_remote_bda(), this->battery_char_handle_);
          if (notify_battery_status) {
            ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
                     this->parent()->address_str().c_str(), notify_battery_status);
          }
        }

        // read firmware version
        auto read_firmware_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->firmware_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_firmware_status) {
          ESP_LOGW(TAG, "Error sending read request for led sensitivity, status=%d", read_firmware_status);
        }

        // read led sensitivity
        auto read_led_sensitivity_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->led_sensitivity_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_led_sensitivity_status) {
          ESP_LOGW(TAG, "Error sending read request for led sensitivity, status=%d", read_led_sensitivity_status);
        }

        break;
      }
      if (param->write.handle == this->reading_batch_size_char_handle_) {
        // reading batch size is now set correctly so subscribe to measurement notifications
        this->batch_size_set_ = true;
        break;
      }
      if (param->write.handle == this->first_rec_char_handle_) {
        break;
      }
      if (param->write.handle == this->measurement_access_char_handle_) {
        break;
      }

      ESP_LOGW(TAG, "[%s] Missed all handle matches: %d",
               this->parent()->address_str().c_str(), param->write.handle);
      break;
    }  // ESP_GATTC_WRITE_CHAR_EVT

    case ESP_GATTC_NOTIFY_EVT: {
      ESP_LOGD(TAG, "[%s] Received Notification", this->parent()->address_str().c_str());

      // battery
      if (param->notify.handle == this->battery_char_handle_) {
        ESP_LOGD(TAG, "Recieved battery notify event");
        this->parse_battery_(param->notify.value, param->notify.value_len);
        break;
      }

      // measurement
      if (param->notify.handle == this->measurement_char_handle_) {
        ESP_LOGD(TAG, "Recieved measurement notify event");
        this->parse_measurement_(param->notify.value, param->notify.value_len);
        break;
      }
      break;  // registerForNotify
    }
    default:
      break;
  }

  if (this->powerpal_device_id_.length() && this->powerpal_apikey_.length() && this->batch_size_set_) {
    this->start_collection();
  }
}

void Powerpal::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    // This event is sent once authentication has completed
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
      if (param->ble_security.auth_cmpl.success) {
        ESP_LOGI(TAG, "[%s] Writing pairing code to Powerpal", this->parent()->address_str().c_str());
        auto status = esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                               this->pairing_code_char_handle_, sizeof(this->pairing_code_),
                                               this->pairing_code_, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (status) {
          ESP_LOGW(TAG, "Error sending write request for pairing_code, status=%d", status);
        }
      }
      break;
    }
    default:
      break;
  }
}

void Powerpal::start_collection() {
  if (this->collecting_) {
    return;
  }
  this->collecting_ = true;

#ifdef USE_HTTP_REQUEST
  // If we can contact the API to get a last uploaded time, request history.
  if (this->cloud_uploader_ != nullptr) {
    std::string url = this->powerpal_api_root_ + this->powerpal_api_device_;
    ESP_LOGI(TAG, "Powerpal api URL: %s", url.c_str());
    std::shared_ptr<http_request::HttpContainer> cont = this->cloud_uploader_->get(url, this->powerpal_headers_);
    ESP_LOGD(TAG, "Got http api bytes: %d", cont->content_length);
    ExternalRAMAllocator<uint8_t> allocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);
    uint8_t *buf = allocator.allocate(cont->content_length);
    cont->read(buf, cont->content_length);
    StaticJsonDocument<360> doc;
    deserializeJson(doc, buf);
    allocator.deallocate(buf, cont->content_length);
    time_t last_reading = doc["last_reading_timestamp"];
    if (last_reading > 0) {
      ESP_LOGI(TAG, "Found last uploaded reading: %ld", last_reading);
      this->uploaded_ts_ = last_reading;
      this->ingesting_history_ = true;
    }
  }
#endif
  auto status = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(), this->parent()->get_remote_bda(),
                                                  this->measurement_char_handle_);
  if (status) {
    ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
              this->parent()->address_str().c_str(), status);
  }

#ifdef USE_HTTP_REQUEST
  if (this->ingesting_history_) {
    // read first record to fill in missing history
    auto read_first_rec_status =
        esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                this->first_rec_char_handle_, ESP_GATT_AUTH_REQ_NONE);
    if (read_first_rec_status) {
      ESP_LOGW(TAG, "Error sending read request for first record, status=%d", read_first_rec_status);
    }
  }
#endif 
}

}  // namespace powerpal_ble
}  // namespace esphome

#endif
