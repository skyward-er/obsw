/* Copyright (c) 2019-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio
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
#include <utils/PinObserver/PinObserver.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Parafoil
{

/**
 * @brief This class contains the handlers for the detach pins on the rocket.
 *
 * It uses Boardcore's PinObserver to bind these functions to the GPIO pins.
 * The handlers post an event on the EventBroker.
 */
class PinHandler : public Boardcore::Module
{
public:
    PinHandler();

    /**
     * @brief Called when the deployment servo actuation is detected via the
     * optical sensor.
     */
    void onExpulsionPinTransition(Boardcore::PinTransition transition);

    /**
     * @brief Returns a vector with all the pins data.
     */
    std::map<PinsList, Boardcore::PinData> getPinsData();

private:
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("pinhandler");
};

}  // namespace Parafoil
