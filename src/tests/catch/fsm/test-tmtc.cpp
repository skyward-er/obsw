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
#include "catch/catch-tests-entry.cpp"
#endif

// We need access to the handleEvent(...) function in state machines in order to
// test them synchronously

#include <miosix.h>


#include <utils/testutils/catch.hpp>

#define private public
#define protected public

#include "events/Events.h"
#include "utils/testutils/TestHelper.h"
#include "Radio/TMTCManager.h"

using miosix::Thread;
using namespace DeathStackBoard;

class TMTCFixture
{
public:
    // This is called at the beginning of each test / section
    TMTCFixture()
    {
        sEventBroker->start();
        fsm = new TMTCManager();
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

    TMTCManager* fsm;
};

TEST_CASE_METHOD(TMTCFixture, "Testing transitions from stateGroundTM")
{
    SECTION("EV_TC_START_SENSOR_TM -> stateSensorTM")
    {
        REQUIRE(testFSMTransition(*fsm, Event{EV_TC_START_SENSOR_TM},
                                  &TMTCManager::stateSensorTM));
    }

    SECTION("EV_ARMED -> stateSensorTM")
    {
        REQUIRE(testFSMTransition(*fsm, Event{EV_ARMED},
                                  &TMTCManager::stateFlightTM));
    }

    SECTION("EV_LIFTOFF -> stateFlightTM")
    {
        REQUIRE(testFSMTransition(*fsm, Event{EV_LIFTOFF},
                                  &TMTCManager::stateFlightTM));
    }
}

TEST_CASE_METHOD(TMTCFixture, "Testing transitions from stateSensorTM")
{
    fsm->transition(&TMTCManager::stateSensorTM);
    Thread::sleep(100);

    SECTION("EV_TC_STOP_SENSOR_TM -> stateGroundTM")
    {
        REQUIRE(testFSMTransition(*fsm, Event{EV_TC_STOP_SENSOR_TM},
                                  &TMTCManager::stateGroundTM));
    }
}

TEST_CASE_METHOD(TMTCFixture, "Testing transitions from stateFlightTM")
{
    fsm->transition(&TMTCManager::stateFlightTM);
    Thread::sleep(100);

    SECTION("EV_DISARMED -> stateGroundTM")
    {
        REQUIRE(testFSMTransition(*fsm, Event{EV_DISARMED},
                                  &TMTCManager::stateGroundTM));
    }
}