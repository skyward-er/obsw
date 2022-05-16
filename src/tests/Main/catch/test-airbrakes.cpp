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

#include <Main/StateMachines/AirBrakes/AirBrakesController.h>
#include <Main/events/Events.h>
#include <miosix.h>
#include <utils/TestUtils/TestHelper.h>

#include <catch2/catch.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Main;

class AirBrakesControllerFixture
{
public:
    // This is called at the beginning of each test / section
    AirBrakesControllerFixture()
    {
        // cppcheck-suppress noCopyConstructor
        // cppcheck-suppress noOperatorEq
        controller = new AirBrakesController();
        EventBroker::getInstance().start();
        controller->start();
    }

    // This is called at the end of each test / section
    ~AirBrakesControllerFixture()
    {
        controller->stop();
        EventBroker::getInstance().unsubscribe(controller);
        EventBroker::getInstance().clearDelayedEvents();
        delete controller;
    }

protected:
    AirBrakesController* controller;
};

TEST_CASE_METHOD(AirBrakesControllerFixture,
                 "AirBrakes - Testing transitions from init")
{
    controller->transition(&AirBrakesController::state_init);
}

TEST_CASE_METHOD(AirBrakesControllerFixture,
                 "AirBrakes - Testing transitions from idle")
{
    controller->transition(&AirBrakesController::state_idle);

    SECTION("ABK_WIGGLE -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{ABK_WIGGLE},
                                  &AirBrakesController::state_idle));
    }

    SECTION("ABK_OPEN -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{ABK_OPEN},
                                  &AirBrakesController::state_idle));
    }

    SECTION("ABK_RESET -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{ABK_RESET},
                                  &AirBrakesController::state_idle));
    }

    SECTION("FLIGHT_LIFTOFF_DETECTED -> SHADOW_MODE")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_LIFTOFF_DETECTED},
                                  &AirBrakesController::state_shadow_mode));
    }
}

TEST_CASE_METHOD(AirBrakesControllerFixture,
                 "AirBrakes - Testing transitions from shadow_mode")
{
    controller->transition(&AirBrakesController::state_shadow_mode);

    SECTION("ABK_SHADOW_MODE_TIMEOUT -> ACTIVE")
    {
        REQUIRE(testFSMTransition(*controller, Event{ABK_SHADOW_MODE_TIMEOUT},
                                  &AirBrakesController::state_active));
    }
}

TEST_CASE_METHOD(AirBrakesControllerFixture,
                 "AirBrakes - Testing transitions from active")
{
    controller->transition(&AirBrakesController::state_active);

    SECTION("FLIGHT_APOGEE_DETECTED -> END")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_APOGEE_DETECTED},
                                  &AirBrakesController::state_end));
    }

    SECTION("ABK_DISABLE -> END")
    {
        REQUIRE(testFSMTransition(*controller, Event{ABK_DISABLE},
                                  &AirBrakesController::state_end));
    }
}

TEST_CASE_METHOD(AirBrakesControllerFixture,
                 "AirBrakes - Testing transitions from end")
{
    controller->transition(&AirBrakesController::state_end);
}
