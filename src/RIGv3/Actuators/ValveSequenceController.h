/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Riccardo Sironi
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
#include <common/Events.h>
#include <common/canbus/MotorStatus.h>
#include <events/EventBroker.h>
#include <events/EventHandler.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace RIGv3
{
class Actuators;
class CanHandler;
class Radio;
class Sensors;

class ValveSequenceController
    : public Boardcore::EventHandler,
      public Boardcore::InjectableWithDeps<Actuators, Sensors, Radio,
                                           CanHandler, Common::MotorStatus>
{
public:
    ValveSequenceController(
        unsigned int stacksize    = miosix::STACK_DEFAULT_FOR_PTHREAD,
        miosix::Priority priority = miosix::MAIN_PRIORITY);

    uint16_t wiggleValves();
    void closeValves();

protected:
    void handleEvent(const Boardcore::Event& ev) override;
};
}  // namespace RIGv3
