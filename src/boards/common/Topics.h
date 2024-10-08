/* Copyright (c) 2018-2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <cstdint>
#include <string>
#include <vector>

namespace Common
{

enum Topics : uint8_t
{
    TOPIC_ABK,
    TOPIC_ADA,
    TOPIC_MEA,
    TOPIC_ARP,
    TOPIC_DPL,
    TOPIC_CAN,
    TOPIC_FLIGHT,
    TOPIC_FMM,
    TOPIC_FSR,
    TOPIC_NAS,
    TOPIC_TMTC,
    TOPIC_MOTOR,
    TOPIC_TARS,
    TOPIC_ALT,
    TOPIC_WING,
};

const std::vector<uint8_t> TOPICS_LIST{
    TOPIC_ABK,  TOPIC_ADA,    TOPIC_MEA,  TOPIC_ARP, TOPIC_DPL,
    TOPIC_CAN,  TOPIC_FLIGHT, TOPIC_FMM,  TOPIC_FSR, TOPIC_NAS,
    TOPIC_TMTC, TOPIC_MOTOR,  TOPIC_TARS, TOPIC_ALT, TOPIC_WING};

}  // namespace Common
