#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include "utils.h"

namespace esphome
{
    namespace fastcon
    {
        static const std::array<uint8_t, 4> DEFAULT_ENCRYPT_KEY = {0x5e, 0x36, 0x7b, 0xc4};
        static const std::array<uint8_t, 3> DEFAULT_BLE_FASTCON_ADDRESS = {0xC1, 0xC2, 0xC3};

        std::vector<uint8_t> get_rf_payload(const std::vector<uint8_t> &addr, const std::vector<uint8_t> &data);
        std::vector<uint8_t> prepare_payload(const std::vector<uint8_t> &addr, const std::vector<uint8_t> &data);
    } // namespace fastcon
} // namespace esphome