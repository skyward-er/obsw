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

#include <Main/StateMachines/Deployment/DeploymentController.h>
#include <Main/events/Events.h>
#include <miosix.h>
#include <utils/TestUtils/TestHelper.h>

#include <catch2/catch.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Main;

class DeploymentControllerFixture
{
public:
    // This is called at the beginning of each test / section
    DeploymentControllerFixture()
    {
        // cppcheck-suppress noCopyConstructor
        // cppcheck-suppress noOperatorEq
        controller = new DeploymentController();
        EventBroker::getInstance().start();
        controller->start();
    }

    // This is called at the end of each test / section
    ~DeploymentControllerFixture()
    {
        controller->stop();
        EventBroker::getInstance().unsubscribe(controller);
        EventBroker::getInstance().clearDelayedEvents();
        delete controller;
    }

protected:
    DeploymentController* controller;
};

TEST_CASE_METHOD(DeploymentControllerFixture,
                 "Deployment - Testing transitions from init")
{
    controller->transition(&DeploymentController::state_init);
}

TEST_CASE_METHOD(DeploymentControllerFixture,
                 "Deployment - Testing transitions from idle")
{
    controller->transition(&DeploymentController::state_idle);

    SECTION("WIGGLE -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{DPL_WIGGLE},
                                  &DeploymentController::state_idle));
    }

    SECTION("OPEN -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{DPL_OPEN},
                                  &DeploymentController::state_idle));
    }

    SECTION("RESET -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{DPL_RESET},
                                  &DeploymentController::state_idle));
    }

    SECTION("OPEN_NC -> NOSECONE_EJECTION")
    {
        REQUIRE(
            testFSMTransition(*controller, Event{DPL_OPEN_NC},
                              &DeploymentController::state_nosecone_ejection));
    }

    SECTION("CUT_DROGUE -> CUTTING")
    {
        REQUIRE(testFSMTransition(*controller, Event{DPL_CUT_DROGUE},
                                  &DeploymentController::state_cutting));
    }
}

TEST_CASE_METHOD(DeploymentControllerFixture,
                 "Deployment - Testing transitions from nosecone_ejection")
{
    controller->transition(&DeploymentController::state_nosecone_ejection);

    SECTION("OPEN_NC_TIMEOUT -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{DPL_OPEN_NC_TIMEOUT},
                                  &DeploymentController::state_idle));
    }

    SECTION("NC_DETACHED -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{FLIGHT_NC_DETACHED},
                                  &DeploymentController::state_idle));
    }
}

TEST_CASE_METHOD(DeploymentControllerFixture,
                 "Deployment - Testing transitions from cutting")
{
    controller->transition(&DeploymentController::state_cutting);

    SECTION("CUT_TIMEOUT -> IDLE")
    {
        REQUIRE(testFSMTransition(*controller, Event{DPL_CUT_TIMEOUT},
                                  &DeploymentController::state_idle));
    }
}
