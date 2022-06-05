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

#include <Main/StateMachines/NavigationAttitudeSystem/NASController.h>
#include <Main/events/Events.h>
#include <miosix.h>
#include <utils/TestUtils/TestHelper.h>

#include <catch2/catch.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Main;

class NASControllerFixture
{
public:
    // This is called at the beginning of each test / section
    NASControllerFixture()
    {
        controller = &NASController::getInstance();
        EventBroker::getInstance().start();
        controller->start();
    }

    // This is called at the end of each test / section
    ~NASControllerFixture()
    {
        controller->stop();
        EventBroker::getInstance().unsubscribe(controller);
        EventBroker::getInstance().clearDelayedEvents();
    }

protected:
    NASController* controller;
};

TEST_CASE_METHOD(NASControllerFixture, "NAS - Testing transitions from idle")
{
    controller->transition(&NASController::state_idle);

    SECTION("NAS_CALIBRATE -> CALIBRATING")
    {
        REQUIRE(testFSMTransition(*controller, Event{NAS_CALIBRATE},
                                  &NASController::state_calibrating));
    }
}

TEST_CASE_METHOD(NASControllerFixture,
                 "NAS - Testing transitions from calibrating")
{
    controller->transition(&NASController::state_calibrating);

    SECTION("NAS_READY -> READY")
    {
        REQUIRE(testFSMTransition(*controller, Event{NAS_READY},
                                  &NASController::state_ready));
    }
}

TEST_CASE_METHOD(NASControllerFixture, "NAS - Testing transitions from ready")
{
    controller->transition(&NASController::state_ready);

    SECTION("NAS_CALIBRATE -> CALIBRATING")
    {
        REQUIRE(testFSMTransition(*controller, Event{NAS_CALIBRATE},
                                  &NASController::state_calibrating));
    }

    SECTION("FLIGHT_LIFTOFF_DETECTED -> ACTIVE")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_LIFTOFF_DETECTED},
                                  &NASController::state_active));
    }
}

TEST_CASE_METHOD(NASControllerFixture, "NAS - Testing transitions from active")
{
    controller->transition(&NASController::state_active);

    SECTION("FLIGHT_LANDING_DETECTED -> END")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_LANDING_DETECTED},
                                  &NASController::state_end));
    }
}

TEST_CASE_METHOD(NASControllerFixture, "NAS - Testing transitions from end")
{
    controller->transition(&NASController::state_end);
}
