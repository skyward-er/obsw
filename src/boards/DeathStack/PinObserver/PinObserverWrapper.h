/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <PinObserver.h>
#include "DeathStack/configs/PinObserverConfig.h"
#include "PinObserverData.h"

namespace DeathStackBoard
{

//Forward dec
class LoggerService;

class PinObserverWrapper
{
public:
    PinObserverWrapper();

    /**
     * @brief Starts the pin observer
     * 
     */
    bool start()
    {
        return pin_obs.start();
    }

    /**
     * @brief Stops the pin observer
     * 
     */
    void stop()
    {
        pin_obs.stop();
    }

    /**
     * @brief Function called by the pinobserver when a launch pin detachment is
     * detected.
     *
     * @param p
     * @param n
     */
    void onLaunchPinTransition(unsigned int p, unsigned char n);

    /**
     * @brief Function called by the pinobserver when a nosecone pin detachment
     * is detected.
     *
     * @param p
     * @param n
     */
    void onNCPinTransition(unsigned int p, unsigned char n);


    void onLaunchPinStateChange(unsigned int p, unsigned char n, int state);
    void onNCPinStateChange(unsigned int p, unsigned char n, int state);


private:
    PinStatus status_pin_launch{ObservedPin::LAUNCH};
    PinStatus status_pin_nosecone{ObservedPin::NOSECONE};

    PinObserver pin_obs;

    LoggerService* logger;
};

}  // namespace DeathStackBoard