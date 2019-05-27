/**
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

#include <miosix.h>

#include "ActiveObject.h"
#include "CutterData.h"
#include "DeathStack/configs/CutterConfig.h"
#include "drivers/pwm/pwm.h"

using miosix::GpioPin;
using miosix::Thread;

namespace DeathStackBoard
{
class PidCutter : public ActiveObject
{
public:
    PidCutter()
        : pwm(CUTTER_TIM, CUTTER_PWM_FREQUENCY),
          pin_enable_drogue(DrogueCutterEna::getPin()),
          pin_enable_main_chute(MainChuteCutterEna::getPin())
    {
        pin_enable_drogue.low();
        pin_enable_main_chute.low();

        // Start PWM with 0 duty cycle to keep IN pins low
        pwm.enableChannel(CUTTER_CHANNEL_DROGUE, 0.0f);
        pwm.enableChannel(CUTTER_CHANNEL_MAIN_CHUTE, 0.0f);

        pwm.start();
    }

    ~PidCutter()
    {
        stopCutDrogue();
        stopCutMainChute();
        pwm.stop();
    }

    void startCutDrogue() { cut_drogue = true; }
    void stopCutDrogue() { cut_drogue = false; }
    void startCutMainChute() { cut_main = true; }
    void stopCutMainChute() { cut_main = false; }

    void updateCut1Current(float current) { cut_1_curr = current; }
    void updateCut2Current(float current) { cut_2_curr = current; }

protected:
    void run() override
    {
        for (;;)
        {
            if (cut_drogue)
            {
                float err = cut_1_curr - CUT_1_TARGET_CURR;
                pwm.setDutyCycle(CUTTER_CHANNEL_DROGUE, err * PID_K);
            }
            else if (cut_main)
            {
                float err = cut_2_curr - CUT_2_TARGET_CURR;
                pwm.setDutyCycle(CUTTER_CHANNEL_MAIN_CHUTE, err * PID_K);
            }
            else
            {
                pwm.setDutyCycle(CUTTER_CHANNEL_DROGUE, 0.0f);
                pwm.setDutyCycle(CUTTER_CHANNEL_MAIN_CHUTE, 0.0f);
            }

            Thread::sleep(50);
        }
    }

private:
    float cut_1_curr = 0;
    float cut_2_curr = 0;
    bool cut_drogue  = false;
    bool cut_main    = false;

    PWM pwm;
    
    GpioPin pin_enable_drogue;
    GpioPin pin_enable_main_chute;

    CutterStatus status;
};
}  // namespace DeathStackBoard