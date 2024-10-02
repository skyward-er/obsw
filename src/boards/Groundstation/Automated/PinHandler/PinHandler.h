/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Nicolò Caruso
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

#include <Groundstation/Automated/PinHandler/PinData.h>
#include <diagnostic/PrintLogger.h>
#include <scheduler/TaskScheduler.h>
#include <utils/PinObserver/PinObserver.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Antennas
{
class PinHandler : public Boardcore::Module
{
public:
    enum PinList : uint8_t
    {
        ARM_SWITCH,
        ACTIVE_SWITCH
    };

    PinHandler();

    /**
     * @brief Starts the PinObserver module thread
     */
    bool start();

    /**
     * @brief Checks if the module has started correctly
     */
    bool isStarted();

    /**
     * @brief Called when the arm switch has been flipped
     */
    void onArmTransition(Boardcore::PinTransition transition);

    /**
     * @brief Called when the active switch has been flipped
     */
    void onActiveTransition(Boardcore::PinTransition transition);

    /**
     * @brief Returns the status of the requested pin
     */
    Boardcore::PinData getPinData(PinList pin);

private:
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("PinHandler");

    Boardcore::TaskScheduler scheduler;
    Boardcore::PinObserver pin_observer;
};
}  // namespace Antennas