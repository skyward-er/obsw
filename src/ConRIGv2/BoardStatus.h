/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Raul Radu
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

#include <ActiveObject.h>
#include <ConRIGv2/Hub/Hub.h>
#include <ConRIGv2/Radio/Radio.h>
#include <common/MavlinkOrion.h>
#include <drivers/timer/TimestampTimer.h>
#include <utils/DependencyManager/DependencyManager.h>
#include <utils/collections/CircularBuffer.h>

namespace ConRIGv2
{
template <unsigned int WINDOW_SIZE, unsigned int PERIOD>
class BitrateCalculator
{
public:
    BitrateCalculator() {}

    /**
     * @brief Update the calculator, should be called every PERIOD ms
     */
    uint16_t update(uint32_t sample)
    {
        if (window.isFull())
        {
            uint32_t last = window.pop();
            window.put(sample);

            uint16_t delta = sample - last;

            // window size is in ms, we want the result in s
            return delta * 1000 / WINDOW_SIZE;
        }
        else
        {
            window.put(sample);
            return 0;
        }
    }

private:
    Boardcore::CircularBuffer<uint32_t, WINDOW_SIZE / PERIOD> window;
};

class BoardStatus : public Boardcore::InjectableWithDeps<Hub, Radio>,
                    private Boardcore::ActiveObject
{
public:
    explicit BoardStatus() {};

    bool start();

private:
    void run() override;

    void sendRadioLinkInfoTm();

    BitrateCalculator<Config::Radio::RADIO_BITRATE_WINDOW_SIZE,
                      Config::Radio::RADIO_STATUS_PERIOD>
        radioBitrate;
};
}  // namespace ConRIGv2
