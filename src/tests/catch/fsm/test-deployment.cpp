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
#include "catch/catch-tests-entry.cpp"
#endif

// We need access to the handleEvent(...) function in state machines in order to
// test them synchronously
#define protected public

#include <drivers/servo/servo.h>
#include <miosix.h>

#include <utils/testutils/catch.hpp>

#include "DeploymentController/DeploymentController.h"
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
        controller = new DeploymentController(&primaryCutter, &backupCutter,
                                              &ejection_servo);
        sEventBroker->start();
        controller->start();
    }

    // This is called at the end of each test / section
    ~DeploymentControllerFixture()
    {
        controller->stop();
        sEventBroker->unsubscribe(controller);
        sEventBroker->clearDelayedEvents();
        delete controller;
    }

protected:
    DeploymentController* controller;

    HBridge primaryCutter{PrimaryCutterEna::getPin(), CUTTER_TIM,
                          CUTTER_CHANNEL_PRIMARY, PRIMARY_CUTTER_PWM_FREQUENCY,
                          PRIMARY_CUTTER_PWM_DUTY_CYCLE};
    HBridge backupCutter{BackupCutterEna::getPin(), CUTTER_TIM,
                         CUTTER_CHANNEL_BACKUP, BACKUP_CUTTER_PWM_FREQUENCY,
                         BACKUP_CUTTER_PWM_DUTY_CYCLE};
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
                                  &DeploymentController::state_cuttingPrimary));
    }

    SECTION("DPL_IDLE -> EV_TEST_CUT_PRIMARY")
    {
        REQUIRE(
            testFSMTransition(*controller, Event{EV_TEST_CUT_PRIMARY},
                              &DeploymentController::state_testCuttingPrimary));
    }

    SECTION("DPL_IDLE -> EV_TEST_CUT_BACKUP")
    {
        REQUIRE(
            testFSMTransition(*controller, Event{EV_TEST_CUT_BACKUP},
                              &DeploymentController::state_testCuttingBackup));
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
                 "Testing transitions from cutting_primary")
{
    controller->transition(&DeploymentController::state_cuttingPrimary);

    SECTION("DPL_CUTTING_PRIMARY -> EV_CUTTING_TIMEOUT")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_CUTTING_TIMEOUT},
                                  &DeploymentController::state_cuttingBackup));
    }
}

TEST_CASE_METHOD(DeploymentControllerFixture,
                 "Testing transitions from cutting_backup")
{
    controller->transition(&DeploymentController::state_cuttingBackup);

    SECTION("DPL_CUTTING_BACKUP -> EV_CUTTING_TIMEOUT")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_CUTTING_TIMEOUT},
                                  &DeploymentController::state_idle));
    }
}

TEST_CASE_METHOD(DeploymentControllerFixture,
                 "Testing transitions from test_cutting_primary")
{
    controller->transition(&DeploymentController::state_testCuttingPrimary);

    SECTION("DPL_TEST_CUTTING_PRIMARY -> EV_CUTTING_TIMEOUT")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_CUTTING_TIMEOUT},
                                  &DeploymentController::state_idle));
    }
}

TEST_CASE_METHOD(DeploymentControllerFixture,
                 "Testing transitions from test_cutting_backup")
{
    controller->transition(&DeploymentController::state_testCuttingBackup);

    SECTION("DPL_TEST_CUTTING_BACKUP -> EV_CUTTING_TIMEOUT")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_CUTTING_TIMEOUT},
                                  &DeploymentController::state_idle));
    }
}
