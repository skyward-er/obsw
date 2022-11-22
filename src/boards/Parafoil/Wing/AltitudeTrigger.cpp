/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Federico Mandelli
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

#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/Wing/AltitudeTrigger.h>
#include <algorithms/NAS/NASState.h>
#include <common/events/Events.h>
#include <events/EventBroker.h>

#include <functional>

using namespace std;
using namespace Parafoil::WingConfig;
using namespace Boardcore;
using namespace Common;

namespace Parafoil
{

AltitudeTrigger::AltitudeTrigger()
{
    BoardScheduler::getInstance().getScheduler().addTask(
        bind(&AltitudeTrigger::update, this), WING_ALTITUDE_CHECKER_PERIOD,
        WING_ALTITUDE_CHECKER_TASK_ID);

    // Set the altitude to the default one
    altitude        = WING_ALTITUDE_REFERENCE;
    confidence      = 0;
    fallingAltitude = 0;
}

void AltitudeTrigger::setDeploymentAltitude(float alt)
{
    miosix::PauseKernelLock lock;
    this->altitude = alt;
}

void AltitudeTrigger::update()
{
    FlightModeManagerState status =
        FlightModeManager::getInstance().getStatus().state;

    if (status == FlightModeManagerState::ASCENDING)
    {
        NASState state = NASController::getInstance().getNasState();

        if (-state.d > altitude)
            confidence++;

        // When we are sure that the altitude is below the set one we trigger
        // the cutters
        if (confidence >= WING_ALTITUDE_TRIGGER_CONFIDENCE)
        {
            confidence = 0;
            EventBroker::getInstance().post(FLIGHT_WING_ALT_REACHED,
                                            TOPIC_FLIGHT);
        }
    }
    else if (status == FlightModeManagerState::DROGUE_DESCENT)
    {
        NASState state = NASController::getInstance().getNasState();

        if (-state.d < altitude)
            confidence++;

        // When we are sure that the altitude is below the set one we trigger
        // the cutters
        if (confidence >= WING_ALTITUDE_TRIGGER_CONFIDENCE)
        {
            confidence = 0;
            EventBroker::getInstance().post(FLIGHT_WING_ALT_REACHED,
                                            TOPIC_FLIGHT);
            /*BoardScheduler::getInstance().getScheduler().removeTask(
                WING_ALTITUDE_CHECKER_TASK_ID);*/
        }
    }
    else if (status == FlightModeManagerState::CONTROLLED_DESCENT)
    {
        float height = -NASController::getInstance().getNasState().d;
        if (fallingAltitude == 0)
        {
            fallingAltitude = height;
        }
        else if (fallingAltitude - height > WING_ALTITUDE_DESCEND_CONTROL)
        {
            fallingAltitude = 0;
            EventBroker::getInstance().post(FLIGHT_TWIRL, TOPIC_FLIGHT);
        }
    }
}

}  // namespace Parafoil
