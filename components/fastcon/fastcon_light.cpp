#include <algorithm>
#include "esphome/core/log.h"
#include "fastcon_light.h"
#include "fastcon_controller.h"

namespace esphome
{
    namespace fastcon
    {
        static const char *const TAG = "fastcon.light";

        void FastconLight::setup()
        {
            if (this->controller_ == nullptr)
            {
                ESP_LOGE(TAG, "Controller not set for light %d!", this->light_id_);
                this->mark_failed();
                return;
            }
            ESP_LOGCONFIG(TAG, "Setting up Fastcon BLE light (ID: %d)...", this->light_id_);
        }

        void FastconLight::set_controller(FastconController *controller)
        {
            this->controller_ = controller;
        }

        light::LightTraits FastconLight::get_traits()
        {
            auto traits = light::LightTraits();
            traits.set_supported_color_modes({light::ColorMode::RGB, light::ColorMode::WHITE, light::ColorMode::BRIGHTNESS});
            traits.set_min_mireds(153);
            traits.set_max_mireds(500);
            return traits;
        }

        void FastconLight::write_state(light::LightState *state)
        {
            float red = 0.0f, green = 0.0f, blue = 0.0f;

            // Get the light state values
            float brightness = state->current_values.get_brightness() * 127.0f; // Scale to 0-127
            bool is_on = state->current_values.is_on();
            auto color_mode = state->current_values.get_color_mode();

            if (!is_on)
            {
                brightness = 0.0f;
            }

            if (color_mode == light::ColorMode::RGB)
            {
                state->current_values_as_rgb(&red, &green, &blue);
            }

            // Convert to protocol values
            auto r = static_cast<uint8_t>(red * 255.0f);
            auto g = static_cast<uint8_t>(green * 255.0f);
            auto b = static_cast<uint8_t>(blue * 255.0f);

            ESP_LOGD(TAG, "Writing state: light_id=%d, on=%d, brightness=%.1f%%, rgb=(%d,%d,%d)", light_id_, is_on, (brightness / 127.0f) * 100.0f, r, g, b);

            // Get the advertisement data
            auto adv_data = this->controller_->get_advertisement(this->light_id_, is_on, brightness, red, green, blue);

            // Debug output - print payload as hex
            char hex_str[adv_data.size() * 2 + 1]; // Each byte needs 2 chars + null terminator
            for (size_t i = 0; i < adv_data.size(); i++)
            {
                sprintf(hex_str + (i * 2), "%02X", adv_data[i]);
            }
            hex_str[adv_data.size() * 2] = '\0'; // Ensure null termination
            ESP_LOGD(TAG, "Advertisement Payload (%d bytes): %s", adv_data.size(), hex_str);

            // Send the advertisement
            this->controller_->queueCommand(this->light_id_, adv_data);
        }
    } // namespace fastcon
} // namespace esphome
