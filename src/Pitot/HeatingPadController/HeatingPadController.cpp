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
#include <Pitot/Configs/HeatingPadControllerConfig.h>
#include <Pitot/BoardScheduler.h>
#include <Pitot/Sensors/Sensors.h>
#include <interfaces-impl/hwmapping.h>


#include "HeatingPadController.h"

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
        schmittTrigger.setThresholds(Config::HeatingPadController::THRESHOLD_LOW, Config::HeatingPadController::THRESHOLD_HIGH);
        schmittTrigger.setTargetState(Config::HeatingPadController::TARGET_TEMPERATURE);
    }

    bool HeatingPadController::start()
    {
        if (started)
            return false;
        
        if(!heatingPadSense()){
            LOG_ERR(logger, "Heating pad not detected!");
            return false;
        }

        if(!schmittTrigger.init()){
            LOG_ERR(logger, "Failed to initialize Schmitt trigger!");
            return false;
        }

        schmittTrigger.begin();

        auto& scheduler = getModule<BoardScheduler>()->heatingPadController();
        auto task = scheduler.addTask([this] { update(); }, updateRate);

        if (task == 0)
            return false;

        started = true;
        return true;
    }

    bool HeatingPadController::isStarted() { return started; }

    void HeatingPadController::enable()
    {
        if (running)
            return;

        running    = true;
    }

    void HeatingPadController::disable() { running = false; }

    bool HeatingPadController::isEnabled() { return running; }

    void HeatingPadController::setTargetTemperature(float temperature)
    {
        schmittTrigger.setTargetState(temperature);
    }

    bool HeatingPadController::heatingPadSense()
    {
        if(miosix::HeatingPad::sense::value() == 1)
            return true;
        else
            return false;
    }

    void HeatingPadController::enableHeatingPad()
    {
        miosix::HeatingPad::enable::high();
    }
    void HeatingPadController::disableHeatingPad()
    {
        miosix::HeatingPad::enable::low();
    }

    void HeatingPadController::update()
    {
        if (!running)
            return;
        
        if(!heatingPadSense()){
            LOG_ERR(logger, "Heating pad not detected!");
            disableHeatingPad();
            return;
        }
        
        float temperature = getModule<Sensors>()->getHeatingPadNTCTemperatureLastSample().temperature; //K
        schmittTrigger.setCurrentState(temperature);

        schmittTrigger.update();
        auto activation = schmittTrigger.getOutput();

        switch (activation)
        {       
            case Boardcore::SchmittTrigger::Activation::HIGH:
                enableHeatingPad();
                break;
            case Boardcore::SchmittTrigger::Activation::LOW:
                disableHeatingPad();
                break;
            case Boardcore::SchmittTrigger::Activation::STOP:
                break;
        }
    }

}  // namespace Pitot
