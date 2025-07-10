/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include "PinObserver.h"

#include <Biliquid/Control/Events.h>
#include <events/EventBroker.h>

using namespace Boardcore;

namespace Biliquid
{
namespace Pins
{
void registerPins(Boardcore::PinObserver& pinObserver)
{
    constexpr auto DETECTION_THRESHOLD = 5;

    pinObserver.registerPinCallback(
        hwmapping::DewesoftInterrupt1::getPin(),
        [&broker = EventBroker::getInstance()](PinTransition transition)
        {
            switch (transition)
            {
                case PinTransition::RISING_EDGE:
                    broker.post(Events::START_SEQUENCE_1,
                                Topics::CONTROL_SEQUENCE);
                    break;
                case PinTransition::FALLING_EDGE:
                    broker.post(Events::STOP_SEQUENCE_1,
                                Topics::CONTROL_SEQUENCE);
                    break;
            }
        },
        DETECTION_THRESHOLD);

    pinObserver.registerPinCallback(
        hwmapping::DewesoftInterrupt2::getPin(),
        [&broker = EventBroker::getInstance()](PinTransition transition)
        {
            switch (transition)
            {
                case PinTransition::RISING_EDGE:
                    broker.post(Events::START_SEQUENCE_2,
                                Topics::CONTROL_SEQUENCE);
                    break;
                case PinTransition::FALLING_EDGE:
                    broker.post(Events::STOP_SEQUENCE_2,
                                Topics::CONTROL_SEQUENCE);
                    break;
            }
        },
        DETECTION_THRESHOLD);

    pinObserver.registerPinCallback(
        hwmapping::DewesoftInterrupt3::getPin(),
        [&broker = EventBroker::getInstance()](PinTransition transition)
        {
            switch (transition)
            {
                case PinTransition::RISING_EDGE:
                    broker.post(Events::START_SEQUENCE_3,
                                Topics::CONTROL_SEQUENCE);
                    break;
                case PinTransition::FALLING_EDGE:
                    broker.post(Events::STOP_SEQUENCE_3,
                                Topics::CONTROL_SEQUENCE);
                    break;
            }
        },
        DETECTION_THRESHOLD);
}
}  // namespace Pins
}  // namespace Biliquid
