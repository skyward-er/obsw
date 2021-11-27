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

#include <PinHandler/PinHandlerData.h>
#include <configs/PinObserverConfig.h>
#include <diagnostic/PrintLogger.h>
#include <utils/PinObserver.h>

using namespace Boardcore;

namespace DeathStackBoard
{

/**
 * @brief Forward dec.
 */
class LoggerService;

/**
 * @brief This class contains the handlers for both the launch pin (umbilical)
 * and the nosecone detachment pin.
 *
 * It uses boardcore's PinObserver to bind these functions to the GPIO pins.
 * The handlers post an event on the EventBroker.
 */
class PinHandler
{
public:
    PinHandler();

    /**
     * @brief Starts the pin observer.
     *
     */
    bool start() { return pin_obs.start(); }

    /**
     * @brief Stops the pin observer.
     *
     */
    void stop() { pin_obs.stop(); }

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

    /**
     * @brief Function called by the pinobserver when a deployment servo
     * actuation is detected via the optical sensor.
     *
     * @param p
     * @param n
     */
    void onDPLServoPinTransition(unsigned int p, unsigned char n);

    void onLaunchPinStateChange(unsigned int p, unsigned char n, int state);
    void onNCPinStateChange(unsigned int p, unsigned char n, int state);
    void onDPLServoPinStateChange(unsigned int p, unsigned char n, int state);

private:
    PinStatus status_pin_launch{ObservedPin::LAUNCH};
    PinStatus status_pin_nosecone{ObservedPin::NOSECONE};
    PinStatus status_pin_dpl_servo{ObservedPin::DPL_SERVO};

    PinObserver pin_obs;

    LoggerService* logger;
    PrintLogger log = Logging::getLogger("deathstack.pinhandler");
};

}  // namespace DeathStackBoard