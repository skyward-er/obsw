#pragma once

#include "Valve.h"

#include <RIGv2/BoardScheduler.h>
#include <RIGv2/CanHandler/CanHandler.h>
#include <RIGv2/Registry/Registry.h>
#include <actuators/Servo/Servo.h>
#include <miosix.h>
#include <scheduler/SignaledDeadlineTask.h>
#include <scheduler/TaskScheduler.h>

#include <chrono>
#include <memory>
#include <optional>
#include <variant>

#include "ValveServoPCA.h"
#include "ValveSolenoid.h"
#include "ValveTimed.h"

namespace RIGv2
{
void Valve::unsafeSetServoPosition(float position)
{
    if (!servo)
        return;

    if (!(servo->getType() == ValveType::SOLENOID))
    {
        position *= config.limit;

        if (config.flipped)
            position = 1.0f - position;

        servo->setPosition(position);
    }
    else
    {
        servo->setPosition(position);
    }
}

bool Valve::isServoOpen()
{
    if (servo->getType() == ValveType::TIMED)
        return closeTs != ValveClosed;
}

float Valve::getServoPosition()
{
    if (!servo)
        return 0.0f;

    float position = servo->getPosition();

    if (servo->getType() == ValveType::TIMED)
    {
        if (config.flipped)
            position = 1.0f - position;

        position /= config.limit;
    }
}

float Valve::getMaxAperture()
{
    return getModule<Registry>()->getOrSetDefaultUnsafe(
        config.maxApertureRegKey, config.defaultMaxAperture);
}

uint32_t Valve::getOpeningTime()
{
    return getModule<Registry>()->getOrSetDefaultUnsafe(
        config.openingTimeRegKey, config.defaultOpeningTime);
}

bool Valve::setMaxAperture(float aperture)
{
    if (aperture >= 0.0 && aperture <= 1.0)
    {
        getModule<Registry>()->setUnsafe(config.maxApertureRegKey, aperture);
        return true;
    }
    else
    {
        // What? Who would ever set this to above 100%?
        return false;
    }
}

bool Valve::setOpeningTime(uint32_t time)
{
    getModule<Registry>()->setUnsafe(config.openingTimeRegKey, time);
    return true;
}

}  // namespace RIGv2
