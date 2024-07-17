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

#include <common/Mavlink.h>
#include <diagnostic/PrintLogger.h>
#include <utils/DependencyManager/DependencyManager.h>
#include <utils/PinObserver/PinObserver.h>

namespace Payload
{
class BoardScheduler;

/**
 * @brief This class contains the handlers for the detach pins on the rocket.
 *
 * It uses Boardcore's PinObserver to bind these functions to the GPIO pins.
 * The handlers post an event on the EventBroker.
 */
class PinHandler : public Boardcore::InjectableWithDeps<BoardScheduler>
{
public:
    bool start();

    bool isStarted();

    /**
     * @brief Returns information about all pins handled by this class
     */
    std::map<PinsList, Boardcore::PinData> getPinData();

private:
    /**
     * @brief Detach pin transition handler.
     */
    void onDetachPinTransition(Boardcore::PinTransition transition);

    std::unique_ptr<Boardcore::PinObserver> pinObserver;

    bool started = false;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("PinHandler");
};

}  // namespace Payload
