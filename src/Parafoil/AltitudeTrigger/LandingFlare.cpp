/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Leonardo Montecchi
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

#include "LandingFlare.h"

#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <common/Events.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Common;

namespace Parafoil
{

bool LandingFlare::start()
{
    auto& scheduler = getModule<BoardScheduler>()->altitudeTrigger();

    auto task = scheduler.addTask([this] { update(); }, updateRate);

    if (task == 0)
        return false;

    if (!map.init())
        return false;

    started = true;
    return true;
}

void LandingFlare::setTargetGEO(Eigen::Vector2f targetGEO)
{
    this->targetGEO = targetGEO;
}

void LandingFlare::enable()
{
    AltitudeTrigger::enable();
    flareAltitudeDetected = false;
    detectionAltitude     = 0.0;
}

Eigen::Vector2f LandingFlare::findCurrentPositionNED()
{
    auto gps = getModule<Sensors>()->getUBXGPSLastSample();

    // NED in the target's reference frame
    Eigen::Vector2f currentPositionNED = Aeroutils::geodetic2NED(
        {gps.latitude, gps.longitude}, {targetGEO[0], targetGEO[1]});

    return currentPositionNED;
}

float LandingFlare::calculateAboveGroundAltitude(LandingFlareData& data)
{
    auto altitude = getModule<NASController>()->getAltitude();
    Eigen::Vector2f currentPositionNED = findCurrentPositionNED();

    float currentGroundAltitude = map.getClosestGroundAltitude(
        currentPositionNED[0], currentPositionNED[1]);

    data.map_n = currentPositionNED[0];
    data.map_e = currentPositionNED[1];
    data.map_u = currentGroundAltitude;

    float aboveGroundAltitude = altitude.value() - currentGroundAltitude;

    return aboveGroundAltitude;
}

void LandingFlare::update()
{
    LandingFlareData data{
        .timestamp          = TimestampTimer::getTimestamp(),
        .flare_detected     = false,
        .detection_altitude = 0,
        .estimated_agl_u    = 0,
        .map_n              = 0,
        .map_e              = 0,
        .map_u              = 0,
    };

    float AGLAltitude = calculateAboveGroundAltitude(data);

    data.estimated_agl_u    = AGLAltitude;
    data.flare_detected     = flareAltitudeDetected;
    data.detection_altitude = detectionAltitude;

    if (!running)
        return;

    if (AGLAltitude < thresholdAltitude)
        confidence++;
    else
        confidence = 0;

    if (confidence >= confidenceThreshold)
    {
        detectionAltitude       = AGLAltitude;
        flareAltitudeDetected   = true;
        data.flare_detected     = flareAltitudeDetected;
        data.detection_altitude = detectionAltitude;
        confidence              = 0;
        EventBroker::getInstance().post(ALTITUDE_TRIGGER_ALTITUDE_REACHED,
                                        TOPIC_ALT);
        running = false;
    }
}

}  // namespace Parafoil
