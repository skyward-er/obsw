/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Leonardo Montecchi
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
#include "HeatingPadController.h"

#include <Pitot/BoardScheduler.h>
#include <Pitot/Configs/HeatingPadControllerConfig.h>
#include <Pitot/Sensors/Sensors.h>
#include <interfaces-impl/hwmapping.h>

namespace Pitot
{
HeatingPadController::HeatingPadController(HeatingPadConfig config)
    : updateRate(config.updateRate)
{
    schmittTrigger.setThresholds(config.thresholdLow, config.thresholdHigh);
    schmittTrigger.setTargetState(config.targetTemperature);
}

HeatingPadController::HeatingPadController()
    : updateRate(Config::HeatingPadController::UPDATE_RATE)
{
    schmittTrigger.setThresholds(Config::HeatingPadController::THRESHOLD,
                                 Config::HeatingPadController::THRESHOLD);
    schmittTrigger.setTargetState(
        Config::HeatingPadController::TARGET_TEMPERATURE);
}

bool HeatingPadController::start()
{
    if (started)
        return false;

    if (!heatingPadSense())
    {
        LOG_ERR(logger, "Heating pad not detected!");
        return false;
    }

    if (!schmittTrigger.init())
    {
        LOG_ERR(logger, "Failed to initialize Schmitt trigger!");
        return false;
    }

    schmittTrigger.begin();

    auto& scheduler = getModule<BoardScheduler>()->heatingPadController();
    auto task       = scheduler.addTask([this] { update(); }, updateRate);

    if (task == 0)
        return false;

    started = true;
    enable();
    return true;
}

bool HeatingPadController::isStarted() { return started; }

void HeatingPadController::enable()
{
    if (running)
        return;

    lowConfidence  = 0;
    highConfidence = 0;
    running        = true;
}

void HeatingPadController::disable() { running = false; }

bool HeatingPadController::isEnabled() { return running; }

void HeatingPadController::setTargetTemperature(float temperature)
{
    miosix::Lock<FastMutex> lock(heatingPadMutex);
    schmittTrigger.setTargetState(temperature);
}

bool HeatingPadController::getPinEnabled() { return pinEnabled; }

uint8_t HeatingPadController::getState()
{
    return static_cast<uint8_t>(heatingPadSense());
}

bool HeatingPadController::heatingPadSense()
{
    if (miosix::HeatingPad::sense::value() ==
        Config::HeatingPadController::SENSE_ACTIVE)
        return true;
    else
        return false;
}

void HeatingPadController::enableHeatingPad()
{
    pinEnabled = true;
    miosix::HeatingPad::enable::high();
}
void HeatingPadController::disableHeatingPad()
{
    pinEnabled = false;
    miosix::HeatingPad::enable::low();
}

void HeatingPadController::update()
{
    if (!running)
        return;

    if (!heatingPadSense() != pinEnabled)
    {
        LOG_WARN(logger, "Heating pad sense mismatch: SENSE:{}, ENABLED:{} ",
                 heatingPadSense(), pinEnabled);
    }

    float temperature =
        getModule<Sensors>()->getHeatingPadNTCLastSample().temperature;  // K
    schmittTrigger.setCurrentState(temperature);
    Boardcore::SchmittTrigger::Activation activation;

    {
        miosix::Lock<miosix::FastMutex> lock(heatingPadMutex);
        schmittTrigger.update();
        activation = schmittTrigger.getOutput();
    }

    switch (activation)
    {
        case Boardcore::SchmittTrigger::Activation::HIGH:
            lowConfidence = 0;

            if (!pinEnabled)
            {
                highConfidence++;
                if (highConfidence >=
                    Config::HeatingPadController::CONFIDENCE_THRESHOLD)
                {
                    enableHeatingPad();
                    highConfidence = 0;
                }
            }
            else
                highConfidence = 0;

            break;

        case Boardcore::SchmittTrigger::Activation::LOW:
            highConfidence = 0;

            if (pinEnabled)
            {
                lowConfidence++;
                if (lowConfidence >=
                    Config::HeatingPadController::CONFIDENCE_THRESHOLD)
                {
                    disableHeatingPad();
                    lowConfidence = 0;
                }
            }
            else
                lowConfidence = 0;

            break;

        case Boardcore::SchmittTrigger::Activation::STOP:
            lowConfidence  = 0;
            highConfidence = 0;
            break;

            auto data = HeatingPadData{
                .timestamp  = Boardcore::TimestampTimer::getTimestamp(),
                .pinEnabled = pinEnabled,
                .schmittTriggerActivation = static_cast<uint8_t>(activation),
                .lowConfidence            = static_cast<uint8_t>(lowConfidence),
                .highConfidence = static_cast<uint8_t>(highConfidence),
                .temperature    = temperature};

            sdLogger.log(data);
    }
}

}  // namespace Pitot
