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
#include <utils/DependencyManager/DependencyManager.h>
#include <utils/PinObserver/PinObserver.h>

#include <memory>

namespace Main
{

class PinHandler : public Boardcore::InjectableWithDeps<BoardScheduler>
{
public:
    enum class PinList : uint8_t
    {
        RAMP_PIN = 0,
        DETACH_MAIN_PIN,
        DETACH_PAYLOAD_PIN,
        EXPULSION_SENSE,
        CUTTER_SENSE,
    };

    PinHandler() {}

    [[nodiscard]] bool start();

    bool isStarted();

    Boardcore::PinData getPinData(PinList pin);

private:
    void logPin(PinList pin);

    void onRampPinTransition(Boardcore::PinTransition transition);
    void onDetachMainTransition(Boardcore::PinTransition transition);
    void onDetachPayloadTransition(Boardcore::PinTransition transition);
    void onExpulsionSenseTransition(Boardcore::PinTransition transition);
    void onCutterSenseTransition(Boardcore::PinTransition transition);

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("pinhandler");

    std::atomic<bool> started{false};

    std::unique_ptr<Boardcore::PinObserver> pinObserver;
};

}  // namespace Main
