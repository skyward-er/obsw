/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

// We need access to the handleEvent(...) function in state machines in order to
// test them synchronously
#define protected public

#include <Main/StateMachines/FlightStatsRecorder/FlightStatsRecorder.h>
#include <common/events/Events.h>
#include <miosix.h>
#include <utils/TestUtils/TestHelper.h>

#include <catch2/catch.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Main;

class FSRControllerFixture
{
public:
    // This is called at the beginning of each test / section
    FSRControllerFixture()
    {
        controller = &FlightStatsRecorder::getInstance();
        EventBroker::getInstance().start();
        controller->start();
    }

    // This is called at the end of each test / section
    ~FSRControllerFixture()
    {
        controller->stop();
        EventBroker::getInstance().unsubscribe(controller);
        EventBroker::getInstance().clearDelayedEvents();
    }

protected:
    FlightStatsRecorder* controller;
};

TEST_CASE_METHOD(FSRControllerFixture, "FSR - Testing transitions from idle")
{
    controller->transition(&FlightStatsRecorder::state_idle);

    SECTION("FLIGHT_LIFTOFF -> LIFTOFF")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_LIFTOFF},
                                  &FlightStatsRecorder::state_liftoff));
    }

    SECTION("FLIGHT_DPL_ALT_DETECTED -> MAIN_DEPLOYMENT")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_DPL_ALT_DETECTED},
                                  &FlightStatsRecorder::state_main_deployment));
    }
}

TEST_CASE_METHOD(FSRControllerFixture, "FSR - Testing transitions from liftoff")
{
    controller->transition(&FlightStatsRecorder::state_liftoff);

    SECTION("FSR_STATS_TIMEOUT -> ASCENDING")
    {
        REQUIRE(testFSMTransition(*controller, Event{FSR_STATS_TIMEOUT},
                                  &FlightStatsRecorder::state_ascending));
    }
}

TEST_CASE_METHOD(FSRControllerFixture,
                 "FSR - Testing transitions from ascending")
{
    controller->transition(&FlightStatsRecorder::state_ascending);

    SECTION("FLIGHT_APOGEE_DETECTED -> ASCENDING")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_APOGEE_DETECTED},
                                  &FlightStatsRecorder::state_ascending));
    }

    SECTION("FSR_STATS_TIMEOUT -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{FSR_STATS_TIMEOUT},
                                  &FlightStatsRecorder::state_idle));
    }
}

TEST_CASE_METHOD(FSRControllerFixture,
                 "FSR - Testing transitions from main_deployment")
{
    controller->transition(&FlightStatsRecorder::state_main_deployment);

    SECTION("FSR_STATS_TIMEOUT -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{FSR_STATS_TIMEOUT},
                                  &FlightStatsRecorder::state_idle));
    }
}
