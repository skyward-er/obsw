/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#ifdef STANDALONE_CATCH1_TEST
#include "../catch-tests-entry.cpp"
#endif

// We need access to the handleEvent(...) function in state machines in order to
// test them synchronously

#include <miosix.h>

#include <Eigen/Dense>
#include <catch2/catch.hpp>

#define private public
#define protected public

#include <FlightStatsRecorder/FSRController.h>

#include "events/Events.h"
#include "utils/testutils/TestHelper.h"

using miosix::Thread;
using namespace DeathStackBoard;

class FlightStatsFixture
{
public:
    // This is called at the beginning of each test / section
    FlightStatsFixture()
    {
        sEventBroker.start();
        // cppcheck-suppress noCopyConstructor
        // cppcheck-suppress noOperatorEq
        fsm = new FlightStatsRecorder();
        fsm->start();
    }

    // This is called at the end of each test / section
    ~FlightStatsFixture()
    {
        fsm->stop();
        sEventBroker.unsubscribe(fsm);
        sEventBroker.clearDelayedEvents();
        delete fsm;
    }

protected:
    FlightStatsRecorder* fsm;
};

TEST_CASE_METHOD(FlightStatsFixture, "Testing transitions from idle")
{
    SECTION("EV_LIFTOFF -> liftoff")
    {
        REQUIRE(testFSMTransition(*fsm, Boardcore::Event{EV_LIFTOFF},
                                  &FlightStatsRecorder::state_liftOff));
    }

    SECTION("EV_DPL_ALTITUDE -> mainDPL")
    {
        REQUIRE(testFSMTransition(*fsm, Boardcore::Event{EV_DPL_ALTITUDE},
                                  &FlightStatsRecorder::state_mainDeployment));
    }
}

TEST_CASE_METHOD(FlightStatsFixture, "Testing transitions from liftoff")
{
    fsm->transition(&FlightStatsRecorder::state_liftOff);
    Thread::sleep(100);

    SECTION("EV_STATS_TIMEOUT -> ascending")
    {
        REQUIRE(testFSMTransition(*fsm, Boardcore::Event{EV_STATS_TIMEOUT},
                                  &FlightStatsRecorder::state_ascending));
    }
}

TEST_CASE_METHOD(FlightStatsFixture, "Testing transitions from ascending")
{
    fsm->transition(&FlightStatsRecorder::state_ascending);
    Thread::sleep(100);

    SECTION("EV_APOGEE -> drogueDPL")
    {
        REQUIRE(
            testFSMTransition(*fsm, Boardcore::Event{EV_STATS_TIMEOUT},
                              &FlightStatsRecorder::state_drogueDeployment));
    }
}

TEST_CASE_METHOD(FlightStatsFixture, "Testing transitions from drogueDPL")
{
    fsm->transition(&FlightStatsRecorder::state_drogueDeployment);
    Thread::sleep(100);

    SECTION("EV_STATS_TIMEOUT -> idle")
    {
        REQUIRE(testFSMTransition(*fsm, Boardcore::Event{EV_STATS_TIMEOUT},
                                  &FlightStatsRecorder::state_idle));
    }
}

TEST_CASE_METHOD(FlightStatsFixture, "Testing transitions from mainDPL")
{
    fsm->transition(&FlightStatsRecorder::state_mainDeployment);
    Thread::sleep(100);

    SECTION("EV_STATS_TIMEOUT -> idle")
    {
        REQUIRE(testFSMTransition(*fsm, Boardcore::Event{EV_STATS_TIMEOUT},
                                  &FlightStatsRecorder::state_idle));
    }
}
