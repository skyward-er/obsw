/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Niccolò Betto
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

#include "AutomaticWingAlgorithm.h"

#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/StateMachines/WingController/WingController.h>
#include <drivers/timer/TimestampTimer.h>
#include <math.h>
#include <utils/AeroUtils/AeroUtils.h>
#include <utils/Constants.h>

using namespace Boardcore;
using namespace Eigen;
using namespace Parafoil::Config::Wing;

namespace Parafoil
{
AutomaticWingAlgorithm::AutomaticWingAlgorithm(float Kp, float Ki,
                                               ServosList servo1,
                                               ServosList servo2,
                                               GuidanceAlgorithm& guidance)
    : Super(servo1, servo2), guidance(guidance)
{
    // PIController needs the sample period in floating point seconds
    auto samplePeriod = 1.0f / Hertz{UPDATE_RATE}.value();

    controller = std::make_unique<PIController>(Kp, Ki, samplePeriod,
                                                PI::SATURATION_MIN_LIMIT,
                                                PI::SATURATION_MAX_LIMIT);
}

void AutomaticWingAlgorithm::step()
{
    auto gps = getModule<Sensors>()->getUBXGPSLastSample();

    if (gps.fix == 3)
    {
        auto nas = getModule<NASController>();
        // The PI calculated result
        Degree result = algorithmStep(nas->getReferenceValues(), gps);

        // Actuate the result
        // To see how to interpret the PI output
        // https://www.geogebra.org/calculator/xrhwarpz
        //
        // Reference system
        // N ^
        //   |
        //   |
        //   ----> E
        if (result > Degree(0))
        {
            // Activate the servo2 and reset servo1
            getModule<Actuators>()->setServoAngle(servo1, 0.0_rad);
            getModule<Actuators>()->setServoAngle(servo2, -Radian(result));
        }
        else
        {
            // Activate the servo1 and reset servo2
            getModule<Actuators>()->setServoAngle(servo1, -Radian(result));
            getModule<Actuators>()->setServoAngle(servo2, 0.0_rad);
        }

        // Log the servo positions
        {
            miosix::Lock<FastMutex> l(mutex);

            data.timestamp   = TimestampTimer::getTimestamp();
            data.servo1Angle = result > Degree(0) ? 0 : -result.value();
            data.servo2Angle = result > Degree(0) ? -result.value() : 0;
            SDlogger->log(data);
        }
    }
    else
    {
        // If we loose fix we set both servo at 0
        getModule<Actuators>()->setServoAngle(servo1, 0.0_rad);
        getModule<Actuators>()->setServoAngle(servo2, 0.0_rad);
    }
}

Degree AutomaticWingAlgorithm::algorithmStep(const ReferenceValues& ref,
                                             const GPSData& gps)
{
    Vector2f heading;  // used for logging purposes

    auto currentPosition = Aeroutils::geodetic2NED(
        {gps.latitude, gps.longitude}, {ref.refLatitude, ref.refLongitude});
    Radian targetAngle = guidance.calculateTargetAngle(
        {currentPosition.x(), currentPosition.y(), gps.height}, heading);

    Vector2f relativeVelocity(gps.velocityNorth, gps.velocityEast);

    // Compute the angle of the current velocity
    // All angle are computed as angle from the north direction
    Radian velocityAngle =
        Radian{atan2(relativeVelocity[1], relativeVelocity[0])};

    // Compute the angle difference
    float error = angleDiff(targetAngle, velocityAngle);

    // Call the PI with the just calculated error. The result is in RADIANS,
    // if positive we activate one servo, if negative the other
    float result = controller->update(error);

    // Convert the result from radians back to degrees
    auto resultDeg = Degree(Radian(result));

    // Logs the outputs
    {
        miosix::Lock<FastMutex> l(mutex);
        data.targetX       = heading[0];
        data.targetY       = heading[1];
        data.targetAngle   = targetAngle.value();
        data.velocityAngle = velocityAngle.value();
        data.error         = error;
        data.pidOutput     = resultDeg.value();
    }

    return resultDeg;
}

float AutomaticWingAlgorithm::angleDiff(Radian a, Radian b)
{
    float diff = (a - b).value();

    // Angle difference
    if (diff < -Constants::PI || Constants::PI < diff)
    {
        diff += Constants::PI;
        bool positiveInput = diff > 0;

        diff = diff - floor(diff / (2 * Constants::PI)) * (2 * Constants::PI);

        // diff = fmod(diff, 2 * Constants::PI);
        if (diff == 0 && positiveInput)
            diff = 2 * Constants::PI;

        diff -= Constants::PI;
    }

    return diff;
}

}  // namespace Parafoil
