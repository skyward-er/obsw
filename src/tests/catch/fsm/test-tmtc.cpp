/*
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#ifdef STANDALONE_CATCH1_TEST
#include "../catch-tests-entry.cpp"
#endif

// We need access to the handleEvent(...) function in state machines in order to
// test them synchronously

#include <miosix.h>

#include <catch2/catch.hpp>

#include <Eigen/Dense>

#define private public
#define protected public

#include <TelemetriesTelecommands/TMTCController.h>

#include "events/Events.h"
#include "utils/testutils/TestHelper.h"

using miosix::Thread;
using namespace DeathStackBoard;

class TMTCFixture
{
public:
    // This is called at the beginning of each test / section
    TMTCFixture()
    {
        sEventBroker->start();
        fsm = new TMTCController();
        fsm->start();
    }

    // This is called at the end of each test / section
    ~TMTCFixture()
    {
        fsm->stop();
        sEventBroker->unsubscribe(fsm);
        sEventBroker->clearDelayedEvents();
        delete fsm;
    }

protected:
    TMTCController* fsm;
};

TEST_CASE_METHOD(TMTCFixture, "Testing transitions from stateGroundTM")
{
    SECTION("EV_TC_START_SENSOR_TM -> stateSensorTM")
    {
        REQUIRE(testFSMTransition(*fsm, Event{EV_TC_START_SENSOR_TM},
                                  &TMTCController::stateSensorTM));
    }

    SECTION("EV_ARMED -> stateSensorTM")
    {
        REQUIRE(testFSMTransition(*fsm, Event{EV_ARMED},
                                  &TMTCController::stateFlightTM));
    }

    SECTION("EV_LIFTOFF -> stateFlightTM")
    {
        REQUIRE(testFSMTransition(*fsm, Event{EV_LIFTOFF},
                                  &TMTCController::stateFlightTM));
    }

    SECTION("EV_TC_SERIAL_TM -> stateSerialDebugTM")
    {
        REQUIRE(testFSMTransition(*fsm, Event{EV_TC_SERIAL_TM},
                                  &TMTCController::stateSerialDebugTM));
    }
}

TEST_CASE_METHOD(TMTCFixture, "Testing transitions from stateSensorTM")
{
    fsm->transition(&TMTCController::stateSensorTM);
    Thread::sleep(100);

    SECTION("EV_TC_STOP_SENSOR_TM -> stateGroundTM")
    {
        REQUIRE(testFSMTransition(*fsm, Event{EV_TC_STOP_SENSOR_TM},
                                  &TMTCController::stateGroundTM));
    }
}

TEST_CASE_METHOD(TMTCFixture, "Testing transitions from stateFlightTM")
{
    fsm->transition(&TMTCController::stateFlightTM);
    Thread::sleep(100);

    SECTION("EV_DISARMED -> stateGroundTM")
    {
        REQUIRE(testFSMTransition(*fsm, Event{EV_DISARMED},
                                  &TMTCController::stateGroundTM));
    }
}

TEST_CASE_METHOD(TMTCFixture, "Testing transitions from stateSerialDebugTM")
{
    fsm->transition(&TMTCController::stateSerialDebugTM);
    Thread::sleep(100);

    SECTION("EV_TC_RADIO_TM -> stateSerialDebugTM")
    {
        REQUIRE(testFSMTransition(*fsm, Event{EV_TC_RADIO_TM},
                                  &TMTCController::stateGroundTM));
    }
}