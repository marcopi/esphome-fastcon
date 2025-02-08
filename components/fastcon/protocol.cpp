#include <algorithm>
#include "protocol.h"

namespace esphome
{
    namespace fastcon
    {
        std::vector<uint8_t> get_rf_payload(const std::vector<uint8_t> &addr, const std::vector<uint8_t> &data)
        {
            const size_t data_offset = 0x12;
            const size_t inverse_offset = 0x0f;
            const size_t result_data_size = data_offset + addr.size() + data.size();

            // Create result buffer including space for checksum
            std::vector<uint8_t> resultbuf(result_data_size + 2, 0);

            // Set hardcoded values
            resultbuf[0x0f] = 0x71;
            resultbuf[0x10] = 0x0f;
            resultbuf[0x11] = 0x55;

            // Copy address in reverse
            for (size_t i = 0; i < addr.size(); i++)
            {
                resultbuf[data_offset + addr.size() - i - 1] = addr[i];
            }

            // Copy data
            std::copy(data.begin(), data.end(), resultbuf.begin() + data_offset + addr.size());

            // Reverse bytes in specified range
            for (size_t i = inverse_offset; i < inverse_offset + addr.size() + 3; i++)
            {
                resultbuf[i] = reverse_8(resultbuf[i]);
            }

            // Add CRC
            uint16_t crc = crc16(addr, data);
            resultbuf[result_data_size] = crc & 0xFF;
            resultbuf[result_data_size + 1] = (crc >> 8) & 0xFF;

            return resultbuf;
        }

        std::vector<uint8_t> prepare_payload(const std::vector<uint8_t> &addr, const std::vector<uint8_t> &data)
        {
            auto payload = get_rf_payload(addr, data);

            // Initialize whitening
            WhiteningContext context;
            whitening_init(0x25, context);

            // Apply whitening to the payload
            whitening_encode(payload, context);

            // Return only the portion after 0xf bytes
            return std::vector<uint8_t>(payload.begin() + 0xf, payload.end());
        }
    } // namespace fastcon
} // namespace esphome