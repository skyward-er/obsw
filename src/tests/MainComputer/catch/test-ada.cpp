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

#include <MainComputer/StateMachines/ApogeeDetectionAlgorithm/ADAController.h>
#include <MainComputer/events/Events.h>
#include <miosix.h>
#include <utils/TestUtils/TestHelper.h>

#include <catch2/catch.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace MainComputer;

class ADAControllerFixture
{
public:
    // This is called at the beginning of each test / section
    ADAControllerFixture()
    {
        // cppcheck-suppress noCopyConstructor
        // cppcheck-suppress noOperatorEq
        controller = new ADAController();
        EventBroker::getInstance().start();
        controller->start();
    }

    // This is called at the end of each test / section
    ~ADAControllerFixture()
    {
        controller->stop();
        EventBroker::getInstance().unsubscribe(controller);
        EventBroker::getInstance().clearDelayedEvents();
        delete controller;
    }

protected:
    ADAController* controller;
};

TEST_CASE_METHOD(ADAControllerFixture, "ADA - Testing transitions from idle")
{
    controller->transition(&ADAController::state_idle);

    SECTION("ADA_CALIBRATE -> CALIBRATING")
    {
        REQUIRE(testFSMTransition(*controller, Event{ADA_CALIBRATE},
                                  &ADAController::state_calibrating));
    }
}

TEST_CASE_METHOD(ADAControllerFixture,
                 "ADA - Testing transitions from calibrating")
{
    controller->transition(&ADAController::state_calibrating);

    SECTION("ADA_READY -> READY")
    {
        REQUIRE(testFSMTransition(*controller, Event{ADA_READY},
                                  &ADAController::state_ready));
    }
}

TEST_CASE_METHOD(ADAControllerFixture, "ADA - Testing transitions from ready")
{
    controller->transition(&ADAController::state_ready);

    SECTION("ADA_CALIBRATE -> CALIBRATING")
    {
        REQUIRE(testFSMTransition(*controller, Event{ADA_CALIBRATE},
                                  &ADAController::state_calibrating));
    }

    SECTION("ADA_EV_LIFTOFF -> SHADOW_MODE")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_LIFTOFF_DETECTED},
                                  &ADAController::state_shadow_mode));
    }
}

TEST_CASE_METHOD(ADAControllerFixture,
                 "ADA - Testing transitions from shadow_mode")
{
    controller->transition(&ADAController::state_shadow_mode);

    SECTION("ADA_SHADOW_MODE_TIMEOUT -> ACTIVE")
    {
        REQUIRE(testFSMTransition(*controller, Event{ADA_SHADOW_MODE_TIMEOUT},
                                  &ADAController::state_active));
    }
}

TEST_CASE_METHOD(ADAControllerFixture, "ADA - Testing transitions from active")
{
    controller->transition(&ADAController::state_active);

    SECTION("FLIGHT_APOGEE_DETECTED -> PRESSURE_STABILIZATION")
    {
        REQUIRE(
            testFSMTransition(*controller, Event{FLIGHT_APOGEE_DETECTED},
                              &ADAController::state_pressure_stabilization));
    }
}

TEST_CASE_METHOD(ADAControllerFixture,
                 "ADA - Testing transitions from pressure_stabilization")
{
    controller->transition(&ADAController::state_pressure_stabilization);

    SECTION("ADA_PRESS_STAB_TIMEOUT -> DROGUE_DESCENT")
    {
        REQUIRE(testFSMTransition(*controller, Event{ADA_PRESS_STAB_TIMEOUT},
                                  &ADAController::state_drogue_descent));
    }
}

TEST_CASE_METHOD(ADAControllerFixture,
                 "ADA - Testing transitions from drogue_descent")
{
    controller->transition(&ADAController::state_drogue_descent);

    SECTION("FLIGHT_DPL_ALT_DETECTED -> TERMINAL_DESCENT")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_DPL_ALT_DETECTED},
                                  &ADAController::state_terminal_descent));
    }
}

TEST_CASE_METHOD(ADAControllerFixture,
                 "ADA - Testing transitions from terminal_descent")
{
    controller->transition(&ADAController::state_terminal_descent);

    SECTION("FLIGHT_LANDING_DETECTED -> ")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_LANDING_DETECTED},
                                  &ADAController::state_landed));
    }
}

TEST_CASE_METHOD(ADAControllerFixture, "ADA - Testing transitions from landed")
{
    controller->transition(&ADAController::state_landed);
}
