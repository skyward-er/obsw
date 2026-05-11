/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#include <Main/BoardScheduler.h>
#include <Main/GpioExpander.h>
#include <drivers/MCP23S17/MCP23S17.h>
#include <utils/DependencyManager/DependencyManager.h>
#include <utils/PinObserver/ExternalPinObserver.h>
#include <utils/PinObserver/PinObserver.h>

#include <memory>

namespace Main
{

class PinHandler
    : public Boardcore::InjectableWithDeps<BoardScheduler, GpioExpander>
{
public:
    enum class PinList : uint8_t
    {
        RAMP_PIN = 0,
        DETACH_PAYLOAD_PIN,
        EXPULSION_SENSE,
        RELEASER_SENSE,
    };

    PinHandler() {}

    [[nodiscard]] bool start();

    bool isStarted();

    Boardcore::PinData getPinData(PinList pin);

    std::chrono::milliseconds getRampPinDetectionDelay()
    {
        return rampPinDetectionDelay.load();
    }

private:
    void logPin(PinList pin, const Boardcore::PinData& data);

    void onRampPinTransition(Boardcore::PinTransition transition,
                             const Boardcore::PinData& data);

    void onDetachPayloadTransition(Boardcore::PinTransition transition,
                                   const Boardcore::PinData& data);
    void onExpulsionSenseTransition(Boardcore::PinTransition transition,
                                    const Boardcore::PinData& data);
    void onReleaserSenseTransition(Boardcore::PinTransition transition,
                                   const Boardcore::PinData& data);

    std::atomic<std::chrono::milliseconds> rampPinDetectionDelay = {};

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("pinhandler");

    std::atomic<bool> started{false};

    std::unique_ptr<Boardcore::PinObserver> pinObserver;
    std::unique_ptr<Boardcore::ExternalPinObserver> externalPinObserver;
    Boardcore::MCP23S17* expander;
};

}  // namespace Main
