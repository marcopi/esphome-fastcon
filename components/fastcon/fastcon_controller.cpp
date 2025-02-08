#include "esphome/core/component_iterator.h"
#include "esphome/core/log.h"
#include "fastcon_controller.h"
#include "protocol.h"

namespace esphome
{
    namespace fastcon
    {
        static const char *const TAG = "fastcon.controller";

        void FastconController::queueCommand(uint32_t light_id_, const std::vector<uint8_t> &data)
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (queue_.size() >= max_queue_size_)
            {
                ESP_LOGW(TAG, "Command queue full (size=%d), dropping command for light %d",
                         queue_.size(), light_id_);
                return;
            }

            Command cmd;
            cmd.data = data;
            cmd.timestamp = millis();
            cmd.retries = 0;

            queue_.push(cmd);
            ESP_LOGV(TAG, "Command queued, queue size: %d", queue_.size());
        }

        void FastconController::clear_queue()
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            std::queue<Command> empty;
            std::swap(queue_, empty);
        }

        void FastconController::setup()
        {
            ESP_LOGCONFIG(TAG, "Setting up Fastcon BLE Controller...");
            ESP_LOGCONFIG(TAG, "  Advertisement interval: %d-%d", this->adv_interval_min_, this->adv_interval_max_);
            ESP_LOGCONFIG(TAG, "  Advertisement duration: %dms", this->adv_duration_);
            ESP_LOGCONFIG(TAG, "  Advertisement gap: %dms", this->adv_gap_);
        }

        void FastconController::loop()
        {
            const uint32_t now = millis();

            switch (adv_state_)
            {
            case AdvertiseState::IDLE:
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (queue_.empty())
                    return;

                Command cmd = queue_.front();
                queue_.pop();

                esp_ble_adv_params_t adv_params = {
                    .adv_int_min = adv_interval_min_,
                    .adv_int_max = adv_interval_max_,
                    .adv_type = ADV_TYPE_NONCONN_IND,
                    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                    .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
                    .channel_map = ADV_CHNL_ALL,
                    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
                };

                uint8_t adv_data_raw[31] = {0};
                uint8_t adv_data_len = 0;

                // Add flags
                adv_data_raw[adv_data_len++] = 2;
                adv_data_raw[adv_data_len++] = ESP_BLE_AD_TYPE_FLAG;
                adv_data_raw[adv_data_len++] = ESP_BLE_ADV_FLAG_BREDR_NOT_SPT | ESP_BLE_ADV_FLAG_GEN_DISC;

                // Add manufacturer data
                adv_data_raw[adv_data_len++] = cmd.data.size() + 2;
                adv_data_raw[adv_data_len++] = ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE;
                adv_data_raw[adv_data_len++] = MANUFACTURER_DATA_ID & 0xFF;
                adv_data_raw[adv_data_len++] = (MANUFACTURER_DATA_ID >> 8) & 0xFF;

                memcpy(&adv_data_raw[adv_data_len], cmd.data.data(), cmd.data.size());
                adv_data_len += cmd.data.size();

                esp_err_t err = esp_ble_gap_config_adv_data_raw(adv_data_raw, adv_data_len);
                if (err != ESP_OK)
                {
                    ESP_LOGW(TAG, "Error setting raw advertisement data (err=%d): %s", err, esp_err_to_name(err));
                    return;
                }

                err = esp_ble_gap_start_advertising(&adv_params);
                if (err != ESP_OK)
                {
                    ESP_LOGW(TAG, "Error starting advertisement (err=%d): %s", err, esp_err_to_name(err));
                    return;
                }

                adv_state_ = AdvertiseState::ADVERTISING;
                state_start_time_ = now;
                ESP_LOGV(TAG, "Started advertising");
                break;
            }

            case AdvertiseState::ADVERTISING:
            {
                if (now - state_start_time_ >= adv_duration_)
                {
                    esp_ble_gap_stop_advertising();
                    adv_state_ = AdvertiseState::GAP;
                    state_start_time_ = now;
                    ESP_LOGV(TAG, "Stopped advertising, entering gap period");
                }
                break;
            }

            case AdvertiseState::GAP:
            {
                if (now - state_start_time_ >= adv_gap_)
                {
                    adv_state_ = AdvertiseState::IDLE;
                    ESP_LOGV(TAG, "Gap period complete");
                }
                break;
            }
            }
        }

        std::vector<uint8_t> FastconController::get_advertisement(uint32_t light_id_, bool is_on, float brightness, float red, float green, float blue)
        {
            std::vector<uint8_t> light_data;

            // Convert brightness to 0-127 range
            uint8_t bright = static_cast<uint8_t>(std::min(brightness * 127.0f, 127.0f));

            if (!is_on)
            {
                // Off state
                light_data = {static_cast<uint8_t>(0)}; // Just the off command
            }
            else if (red == 0 && green == 0 && blue == 0)
            {
                // Warm white mode
                light_data = std::vector<uint8_t>{
                    static_cast<uint8_t>(128 + bright), // On bit (128) + brightness
                    0, 0, 0,                            // RGB values
                    127, 127                            // Warm/cold values
                };
            }
            else
            {
                // RGB mode
                uint8_t r = static_cast<uint8_t>(red * 255.0f);
                uint8_t g = static_cast<uint8_t>(green * 255.0f);
                uint8_t b = static_cast<uint8_t>(blue * 255.0f);

                light_data = std::vector<uint8_t>{
                    static_cast<uint8_t>(128 + bright), // On bit (128) + brightness
                    b, r, g,                            // RGB values (in BRG order per protocol)
                    0, 0                                // No warm/cold values in RGB mode
                };
            }

            return this->single_control(light_id_, light_data);
        }

        std::vector<uint8_t> FastconController::single_control(uint32_t light_id_, const std::vector<uint8_t> &data)
        {
            std::vector<uint8_t> result_data(12);

            result_data[0] = 2 | (((0xfffffff & (data.size() + 1)) << 4));
            result_data[1] = light_id_;
            std::copy(data.begin(), data.end(), result_data.begin() + 2);

            return this->generate_command(5, light_id_, result_data, true);
        }

        std::vector<uint8_t> FastconController::generate_command(uint8_t n, uint32_t light_id_, const std::vector<uint8_t> &data, bool forward)
        {
            static uint8_t sequence = 0;

            // Create command body with header
            std::vector<uint8_t> body(data.size() + 4);
            uint8_t i2 = (light_id_ / 256);

            // Construct header
            body[0] = (i2 & 0b1111) | ((n & 0b111) << 4) | (forward ? 0x80 : 0);
            body[1] = sequence++; // Use and increment sequence number
            if (sequence >= 255)
                sequence = 1;

            body[2] = this->mesh_key_[3]; // Safe key

            // Copy data
            std::copy(data.begin(), data.end(), body.begin() + 4);

            // Calculate checksum
            uint8_t checksum = 0;
            for (size_t i = 0; i < body.size(); i++)
            {
                if (i != 3)
                {
                    checksum = checksum + body[i];
                }
            }
            body[3] = checksum;

            // Encrypt header and data
            for (size_t i = 0; i < 4; i++)
            {
                body[i] = DEFAULT_ENCRYPT_KEY[i & 3] ^ body[i];
            }

            for (size_t i = 0; i < data.size(); i++)
            {
                body[4 + i] = this->mesh_key_[i & 3] ^ body[4 + i];
            }

            // Prepare the final payload with RF protocol formatting
            std::vector<uint8_t> addr = {DEFAULT_BLE_FASTCON_ADDRESS.begin(), DEFAULT_BLE_FASTCON_ADDRESS.end()};
            return prepare_payload(addr, body);
        }
    } // namespace fastcon
} // namespace esphome
