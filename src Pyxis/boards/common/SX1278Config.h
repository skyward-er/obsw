/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include <radio/SX1278/SX1278.h>

namespace Common
{

Boardcore::SX1278::Config SX1278_CONFIG = {
    .freq_rf  = 422075000,
    .freq_dev = 25000,
    .bitrate  = 19200,
    .rx_bw    = Boardcore::SX1278::RxBw::HZ_83300,
    .afc_bw   = Boardcore::SX1278::RxBw::HZ_125000,
    .ocp      = 120,
    .power    = 17,
    .shaping  = Boardcore::SX1278::Shaping::GAUSSIAN_BT_1_0,
    .dc_free  = Boardcore::SX1278::DcFree::WHITENING};

}  // namespace Common
