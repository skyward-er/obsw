/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron, Luca Erbetta, Luca Mozzarelli
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
#define protected public

#include <Deployment/DeploymentController.h>
#include <drivers/servo/Servo.h>
#include <miosix.h>

#include <catch2/catch.hpp>

#include "configs/DeploymentConfig.h"
#include "events/Events.h"
#include "utils/testutils/TestHelper.h"

using miosix::Thread;
using namespace DeathStackBoard;

class DeploymentControllerFixture
{
public:
    // This is called at the beginning of each test / section
    DeploymentControllerFixture()
    {
        controller = new DeploymentController(&ejection_servo);
        sEventBroker.start();
        controller->start();
    }

    // This is called at the end of each test / section
    ~DeploymentControllerFixture()
    {
        controller->stop();
        sEventBroker.unsubscribe(controller);
        sEventBroker.clearDelayedEvents();
        delete controller;
    }

protected:
    DeploymentController* controller;
    DeploymentServo ejection_servo;
};

TEST_CASE_METHOD(DeploymentControllerFixture, "Testing transitions from idle")
{
    controller->transition(&DeploymentController::state_idle);

    SECTION("DPL_IDLE -> EV_RESET_SERVO")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_RESET_SERVO},
                                  &DeploymentController::state_idle));
    }

    SECTION("DPL_IDLE -> EV_WIGGLE_SERVO")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_WIGGLE_SERVO},
                                  &DeploymentController::state_idle));
    }

    SECTION("DPL_IDLE -> EV_NC_OPEN")
    {
        REQUIRE(
            testFSMTransition(*controller, Event{EV_NC_OPEN},
                              &DeploymentController::state_noseconeEjection));
    }

    SECTION("DPL_IDLE -> EV_CUT_DROGUE")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_CUT_DROGUE},
                                  &DeploymentController::state_cutting));
    }
}

TEST_CASE_METHOD(DeploymentControllerFixture,
                 "Testing transitions from nosecone_ejection")
{
    controller->transition(&DeploymentController::state_noseconeEjection);

    SECTION("DPL_NOSECONE_EJECTION -> EV_NC_DETACHED")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_NC_DETACHED},
                                  &DeploymentController::state_idle));
    }

    SECTION("DPL_NOSECONE_EJECTION -> EV_NC_OPEN_TIMEOUT")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_NC_OPEN_TIMEOUT},
                                  &DeploymentController::state_idle));
    }
}

TEST_CASE_METHOD(DeploymentControllerFixture,
                 "Testing transitions from cutting")
{
    controller->transition(&DeploymentController::state_cutting);

    SECTION("DPL_CUTTING -> EV_CUTTING_TIMEOUT")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_CUTTING_TIMEOUT},
                                  &DeploymentController::state_idle));
    }
}
