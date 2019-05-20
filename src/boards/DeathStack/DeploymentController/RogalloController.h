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

#include <drivers/servo/servo.h>
#include <events/Event.h>
#include "DeathStack/Events.h"
#include "DeathStack/configs/DeploymentConfig.h"

namespace DeathStackBoard
{
class RogalloController : public EventHandler
{
public:
    RogalloController() : servo_lr(TIM4_DATA), servo_back(TIM8_DATA)
    {
        sEventBroker->subscribe(this, TOPIC_DEPLOYMENT);
        sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    }

    virtual ~RogalloController() {}

protected:
    void handleEvent(const Event& ev) override
    {
        if (ev.sig == EV_START_ROGALLO_CONTROL)
        {
            configureServos();

            // Start rogallo control sequence
            servo_lr.start();
            servo_back.start();

            servo_lr.setPosition(SERVO_LEFT_CH, 0);
            servo_lr.setPosition(SERVO_RIGHT_CH, 0);
            servo_back.setPosition(SERVO_BACK_CH, 0);

            Thread::sleep(1000);

            servo_lr.setPosition(SERVO_LEFT_CH, 0.4);
            servo_lr.setPosition(SERVO_RIGHT_CH, 0.4);
            servo_back.setPosition(SERVO_BACK_CH, 0.4);

            Thread::sleep(7000);

            servo_lr.setPosition(SERVO_LEFT_CH, 0);
            servo_lr.setPosition(SERVO_RIGHT_CH, 0);
            servo_back.setPosition(SERVO_BACK_CH, 0);

            servo_lr.stop();
            servo_back.stop();
        }
        else if (ev.sig == EV_DPL_ALTITUDE)
        {
            Thread::sleep(SERVO_POSITION_RESET_DELAY);
            configureServos();

            // Start rogallo control sequence
            servo_lr.start();
            servo_back.start();

            servo_lr.setPosition(SERVO_LEFT_CH, 0);
            servo_lr.setPosition(SERVO_RIGHT_CH, 0);
            servo_back.setPosition(SERVO_BACK_CH, 0);

            servo_lr.stop();
            servo_back.stop();
        }
    }

private:
    void configureServos()
    {
        miosix::FastInterruptDisableLock dLock;
        miosix::nosecone::rogP1::mode(miosix::Mode::ALTERNATE);
        miosix::nosecone::rogP2::mode(miosix::Mode::ALTERNATE);
        miosix::nosecone::motP1::mode(miosix::Mode::ALTERNATE);

        miosix::nosecone::rogP1::alternateFunction(2);
        miosix::nosecone::rogP2::alternateFunction(2);
        miosix::nosecone::motP1::alternateFunction(3);

        servo_lr.enable(SERVO_LEFT_CH);
        servo_lr.enable(SERVO_RIGHT_CH);
        servo_back.enable(SERVO_BACK_CH);
    }

    Servo servo_lr;
    Servo servo_back;
};
}  // namespace DeathStackBoard