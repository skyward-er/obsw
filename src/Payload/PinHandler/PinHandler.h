/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#pragma once

#include <common/MavlinkOrion.h>
#include <diagnostic/PrintLogger.h>
#include <utils/DependencyManager/DependencyManager.h>
#include <utils/PinObserver/PinObserver.h>

namespace Payload
{
class BoardScheduler;

enum class PinList : uint8_t
{
    RAMP_DETACH_PIN     = 0,
    NOSECONE_DETACH_PIN = 1,
};

/**
 * @brief This class contains the handlers for the detach pins on the payload.
 *
 * It uses Boardcore's PinObserver to bind these functions to the GPIO pins.
 * The handlers post an event on the EventBroker.
 */
class PinHandler : public Boardcore::InjectableWithDeps<BoardScheduler>
{
public:
    static const std::array<PinList, 2> PIN_LIST;

    [[nodiscard]] bool start();

    bool isStarted();

    /**
     * @brief Returns information about the specified pin.
     */
    Boardcore::PinData getPinData(PinList pin);

private:
    void logPin(PinList pin, const Boardcore::PinData& data);

    void onRampDetachTransition(Boardcore::PinTransition transition,
                                const Boardcore::PinData& data);
    void onNoseconeDetachTransition(Boardcore::PinTransition transition,
                                    const Boardcore::PinData& data);

    std::unique_ptr<Boardcore::PinObserver> pinObserver;

    std::atomic<bool> started{false};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("PinHandler");
};

}  // namespace Payload
