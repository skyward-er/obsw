/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <stdint.h>

#include <iostream>
#include <string>

/**
 * @brief Logging struct for the main radio informations
 *
 */

namespace RotatingPlatform
{
struct MainRadioLog
{
    uint64_t timestamp                    = 0;
    uint16_t main_packet_tx_error_count   = 0;
    uint32_t main_tx_bitrate              = 0;
    uint16_t main_packet_rx_success_count = 0;
    uint16_t main_packet_rx_drop_count    = 0;
    uint32_t main_rx_bitrate              = 0;
    float main_rx_rssi                    = 0;

    static std::string header()
    {
        return "timestamp,main_packet_tx_error_count,main_tx_bitrate,main_"
               "packet_rx_success_count,main_packet_rx_drop_count,main_rx_"
               "bitrate,main_rx_rssi\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << main_packet_tx_error_count << ","
           << main_tx_bitrate << "," << main_tx_bitrate << ","
           << main_packet_rx_success_count << "," << main_packet_rx_drop_count
           << "," << main_rx_bitrate << "," << main_rx_rssi << "\n";
    }
};
}  // namespace RotatingPlatform
