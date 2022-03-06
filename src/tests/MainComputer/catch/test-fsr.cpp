/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

#include <MainComputer/FlightStatsRecorder/FSRController.h>
#include <MainComputer/events/Events.h>
#include <miosix.h>
#include <utils/testutils/TestHelper.h>

#include <catch2/catch.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace MainComputer;

class FSRControllerFixture
{
public:
    // This is called at the beginning of each test / section
    FSRControllerFixture()
    {
        // cppcheck-suppress noCopyConstructor
        // cppcheck-suppress noOperatorEq
        controller = new FSRController();
        sEventBroker.start();
        controller->start();
    }

    // This is called at the end of each test / section
    ~FSRControllerFixture()
    {
        controller->stop();
        sEventBroker.unsubscribe(controller);
        sEventBroker.clearDelayedEvents();
        delete controller;
    }

protected:
    FSRController* controller;
};

TEST_CASE_METHOD(FSRControllerFixture, "Testing transitions from idle")
{
    controller->transition(&FSRController::state_idle);

    SECTION("FLIGHT_LIFTOFF_DETECTED -> LIFTOFF")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_LIFTOFF_DETECTED},
                                  &FSRController::state_liftoff));
    }

    SECTION("FLIGHT_DPL_ALT_DETECTED -> MAIN_DEPLOYMENT")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_DPL_ALT_DETECTED},
                                  &FSRController::state_main_deployment));
    }
}

TEST_CASE_METHOD(FSRControllerFixture, "Testing transitions from liftoff")
{
    controller->transition(&FSRController::state_liftoff);

    SECTION("FSR_STATS_TIMEOUT -> ASCENDING")
    {
        REQUIRE(testFSMTransition(*controller, Event{FSR_STATS_TIMEOUT},
                                  &FSRController::state_ascending));
    }
}

TEST_CASE_METHOD(FSRControllerFixture, "Testing transitions from ascending")
{
    controller->transition(&FSRController::state_ascending);

    SECTION("FLIGHT_APOGEE_DETECTED -> ASCENDING")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_APOGEE_DETECTED},
                                  &FSRController::state_ascending));
    }

    SECTION("FSR_STATS_TIMEOUT -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{FSR_STATS_TIMEOUT},
                                  &FSRController::state_idle));
    }
}

TEST_CASE_METHOD(FSRControllerFixture,
                 "Testing transitions from main_deployment")
{
    controller->transition(&FSRController::state_main_deployment);

    SECTION("FSR_STATS_TIMEOUT -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{FSR_STATS_TIMEOUT},
                                  &FSRController::state_idle));
    }
}
