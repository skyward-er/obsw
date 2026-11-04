/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Tommaso Domenicali
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
#include <Groundstation/Automated/Actuators/Actuators.h>
#include <Groundstation/Automated/SMA/SMA.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <Groundstation/Common/HubBase.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Antennas
{

/**
 * @brief ARP-only module that periodically sends the ARP telemetry.
 *
 * Every RADIO_STATUS_PERIOD it collects the antenna state (VN300 attitude and
 * GPS, stepper positions/speeds, SMA target and state), packs it into a
 * MAVLink ARP_TM message and dispatches it through the Hub.
 *
 * It lives apart from BoardStatus on purpose: its dependencies (Actuators, SMA,
 * Sensors) exist only in the ARP binary, so it must only be inserted by
 * arp-gs-entry. Putting them in a module shared with gs-entry would make
 * DependencyManager::inject() fail on the plain GS.
 */
class ArpStatus : public Boardcore::InjectableWithDeps<Actuators, SMA, Sensors,
                                                       Groundstation::HubBase>,
                  private Boardcore::ActiveObject
{
public:
    /**
     * @brief Starts the thread that sends the ARP telemetry.
     *
     * Call it after Actuators, Sensors and SMA have been started, since the
     * telemetry reads their state.
     *
     * @return false if the thread could not be started.
     */
    bool start() { return ActiveObject::start(); }

private:
    /**
     * @brief Builds one ARP_TM message from the current state of the ARP
     * modules and dispatches it to the Hub.
     */
    void sendArpTm();

    /**
     * @brief Thread body: calls sendArpTm() every RADIO_STATUS_PERIOD until the
     * thread is asked to stop.
     */
    void run() override;
};

}  // namespace Antennas
