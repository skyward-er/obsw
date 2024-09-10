/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <radio/SX1278/SX1278Fsk.h>
#include <radio/SX1278/SX1278Lora.h>

namespace Common
{

static const Boardcore::SX1278Fsk::Config MAIN_RADIO_CONFIG = {
    .freq_rf    = 439000000,
    .freq_dev   = 50000,
    .bitrate    = 48000,
    .rx_bw      = Boardcore::SX1278Fsk::Config::RxBw::HZ_125000,
    .afc_bw     = Boardcore::SX1278Fsk::Config::RxBw::HZ_125000,
    .ocp        = 120,
    .power      = 13,
    .shaping    = Boardcore::SX1278Fsk::Config::Shaping::GAUSSIAN_BT_1_0,
    .dc_free    = Boardcore::SX1278Fsk::Config::DcFree::WHITENING,
    .enable_crc = false};

static constexpr Boardcore::SX1278Fsk::Config PAYLOAD_RADIO_CONFIG = {
    .freq_rf    = 868000000,
    .freq_dev   = 50000,
    .bitrate    = 48000,
    .rx_bw      = Boardcore::SX1278Fsk::Config::RxBw::HZ_125000,
    .afc_bw     = Boardcore::SX1278Fsk::Config::RxBw::HZ_125000,
    .ocp        = 120,
    .power      = 13,
    .shaping    = Boardcore::SX1278Fsk::Config::Shaping::GAUSSIAN_BT_1_0,
    .dc_free    = Boardcore::SX1278Fsk::Config::DcFree::WHITENING,
    .enable_crc = false};

static constexpr Boardcore::SX1278Lora::Config RIG_RADIO_CONFIG = {
    .bandwidth              = Boardcore::SX1278Lora::Config::Bw::HZ_125000,
    .coding_rate            = Boardcore::SX1278Lora::Config::Cr::CR_1,
    .spreading_factor       = Boardcore::SX1278Lora::Config::Sf::SF_7,
    .low_data_rate_optimize = false,
    .freq_rf                = 434000000,
    .ocp                    = 120,
    .power                  = 2,
    .enable_crc             = false};

}  // namespace Common