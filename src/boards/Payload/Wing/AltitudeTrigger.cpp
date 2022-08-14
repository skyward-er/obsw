/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/WingConfig.h>
#include <Payload/FlightModeManager/FlightModeManager.h>
#include <Payload/NASController/NASController.h>
#include <Payload/Wing/AltitudeTrigger.h>
#include <algorithms/NAS/NASState.h>
#include <common/events/Events.h>
#include <events/EventBroker.h>

#include <functional>

using namespace std;
using namespace Payload::WingConfig;
using namespace Boardcore;
using namespace Common;

namespace Payload
{
AltitudeTrigger::AltitudeTrigger()
{
    BoardScheduler::getInstance().getScheduler().addTask(
        bind(&AltitudeTrigger::update, this), WING_ALTITUDE_CHECKER_PERIOD,
        WING_ALTITUDE_CHECKER_TASK_ID);

    // Set also the altitude to the default one
    altitude = WING_ALTITUDE_REFERENCE;
}

void AltitudeTrigger::setDeploymentAltitude(float altitude)
{
    this->altitude = altitude;
}

void AltitudeTrigger::update()
{
    if (FlightModeManager::getInstance().getStatus().state ==
        FlightModeManagerState::DROGUE_DESCENT)
    {
        NASState state = NASController::getInstance().getNasState();

        if (-state.d < altitude)
            EventBroker::getInstance().post(FLIGHT_WING_ALT_REACHED,
                                            TOPIC_FLIGHT);
    }
}
}  // namespace Payload