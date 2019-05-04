/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
#define protected public

#include <miosix.h>
#include <utils/catch.hpp>

#include "DeathStack/DeploymentController/Deployment.h"
#include "DeathStack/Events.h"
#include "PinObserver.h"
#include "utils/testutils/TestHelper.h"

using miosix::Thread;
using namespace DeathStackBoard;

/**
 * @brief Ensure cleanup in every test using RAIII
 *
 */
class DeploymentControllerFixture
{
public:
    // This is called at the beginning of each test / section
    DeploymentControllerFixture()
    { 
        dpl = new DeploymentController();
        sEventBroker->start();
        dpl->start();
    }

    // This is called at the end of each test / section
    ~DeploymentControllerFixture()
    {
        dpl->stop();
        sEventBroker->unsubscribe(dpl);
        sEventBroker->clearDelayedEvents();
        delete dpl;
    }

protected:
    DeploymentController* dpl;
};

/**
 * TEST_CASE_METHOD(Foo, "...") can access all protected members of Foo. See the
 * catch framework reference on Github.
 */
TEST_CASE_METHOD(DeploymentControllerFixture, "Testing transitions from IDLE")
{
    SECTION("IDLE -> Opening Nosecone")
    {
        REQUIRE(
            testHSMTransition(*dpl, Event{EV_NC_OPEN},
                              &DeploymentController::state_spinning));
    }

    SECTION("IDLE -> Cutting main")
    {
        REQUIRE(
            testHSMTransition(*dpl, Event{EV_CUT_MAIN},
                            &DeploymentController::state_cuttingMain));
    }

    SECTION("IDLE -> Cutting drogue")
    {
        REQUIRE(
            testHSMTransition(*dpl, Event{EV_CUT_DROGUE},
                              &DeploymentController::state_cuttingDrogue));
    }

}

TEST_CASE_METHOD(DeploymentControllerFixture, "Testing transitions from CUTTING MAIN")
{
    REQUIRE(
        testHSMTransition(*dpl, Event{EV_CUT_MAIN},
                            &DeploymentController::state_cuttingMain));

    SECTION(" Cutting Main -> Idle")
    {
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_CUTTING},
                            &DeploymentController::state_idle));
    }

    SECTION("Deferred event: EV_CUT_DROGUE")
    {
        // Send CUT_DROGUE: nothing should happen
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_CUT_DROGUE},
                            &DeploymentController::state_cuttingMain));
        // Send TIMEOUT_CUTTING: back in idle
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_CUTTING},
                            &DeploymentController::state_idle));
        // Wait a bit to allow EV_ENTRY handling
        Thread::sleep(10);
        // CUT_DROGUE should now be processed
        REQUIRE( dpl->testState(&DeploymentController::state_cuttingDrogue ));
    }
}

TEST_CASE_METHOD(DeploymentControllerFixture, "Testing transitions from CUTTING DROGUE")
{
    REQUIRE(
        testHSMTransition(*dpl, Event{EV_CUT_DROGUE},
                            &DeploymentController::state_cuttingDrogue));

    SECTION("Cutting Drogue -> Idle")
    {
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_CUTTING},
                            &DeploymentController::state_idle));
    }
    SECTION("Deferred event: EV_CUT_MAIN")
    {
        // Send CUT_MAIN: nothing should happen
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_CUT_MAIN},
                            &DeploymentController::state_cuttingDrogue));
        // Send TIMEOUT_CUTTING: back in idle
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_CUTTING},
                            &DeploymentController::state_idle));
        // Wait a bit to allow EV_ENTRY handling
        Thread::sleep(10);
        // CUT_MAIN should now be processed
        REQUIRE( dpl->testState(&DeploymentController::state_cuttingMain ));
    }
}

TEST_CASE_METHOD(DeploymentControllerFixture, "Testing transitions from SPINNING")
{
    REQUIRE(
    testHSMTransition(*dpl, Event{EV_NC_OPEN},
                      &DeploymentController::state_spinning));
    SECTION("SPINNING -> Waiting Detachment")
    {
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_MOT_MIN_OPEN_TIME},
                          &DeploymentController::state_awaitingDetachment));
    }
    SECTION("SPINNING -> Waiting Min Open Time")
    {
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_NC_DETACHED},
                          &DeploymentController::state_awaitingOpenTime));
    }
    SECTION("OPENING NOSECONE -> Idle")
    {
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_MOT_OPEN},
                          &DeploymentController::state_idle));
    }
    SECTION("Deferred event: EV_CUT_MAIN")
    {
        // Send CUT_MAIN: nothing should happen
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_CUT_MAIN},
                            &DeploymentController::state_spinning));
        // Go back in IDLE
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_MOT_OPEN},
                          &DeploymentController::state_idle));
        // Wait a bit to allow EV_ENTRY handling
        Thread::sleep(10);
        // CUT_MAIN should now be processed
        REQUIRE( dpl->testState(&DeploymentController::state_cuttingMain ));
    }
    SECTION("Deferred event: EV_CUT_DROGUE")
    {
        // Send CUT_DROGUE: nothing should happen
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_CUT_DROGUE},
                            &DeploymentController::state_spinning));
        // Go back in IDLE
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_MOT_OPEN},
                          &DeploymentController::state_idle));
        // Wait a bit to allow EV_ENTRY handling
        Thread::sleep(10);
        // CUT_DROGUE should now be processed
        REQUIRE( dpl->testState(&DeploymentController::state_cuttingDrogue ));
    }
}

TEST_CASE_METHOD(DeploymentControllerFixture, "Testing transitions from WAITING MIN OPEN TIME")
{
    REQUIRE(
    testHSMTransition(*dpl, Event{EV_NC_OPEN},
                      &DeploymentController::state_spinning));
    REQUIRE(
    testHSMTransition(*dpl, Event{EV_NC_DETACHED},
                      &DeploymentController::state_awaitingOpenTime));
    
    SECTION("WAITING MIN OPEN TIME -> Idle")
    {
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_MOT_MIN_OPEN_TIME},
                          &DeploymentController::state_idle));
    }
    SECTION("OPENING NOSECONE -> Idle")
    {
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_MOT_OPEN},
                          &DeploymentController::state_idle));
    }
    SECTION("Deferred event: EV_CUT_MAIN")
    {
        // Send CUT_MAIN: nothing should happen
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_CUT_MAIN},
                            &DeploymentController::state_awaitingOpenTime));
        // Go back in IDLE
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_MOT_OPEN},
                          &DeploymentController::state_idle));
        // Wait a bit to allow EV_ENTRY handling
        Thread::sleep(10);
        // CUT_MAIN should now be processed
        REQUIRE( dpl->testState(&DeploymentController::state_cuttingMain ));
    }
    SECTION("Deferred event: EV_CUT_DROGUE")
    {
        // Send CUT_DROGUE: nothing should happen
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_CUT_DROGUE},
                            &DeploymentController::state_awaitingOpenTime));
        // Go back in IDLE
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_MOT_OPEN},
                          &DeploymentController::state_idle));
        // Wait a bit to allow EV_ENTRY handling
        Thread::sleep(10);
        // CUT_DROGUE should now be processed
        REQUIRE( dpl->testState(&DeploymentController::state_cuttingDrogue ));
    }
}

TEST_CASE_METHOD(DeploymentControllerFixture, "Testing transitions from WAITING DETACHEMENT")
{
    REQUIRE(
    testHSMTransition(*dpl, Event{EV_NC_OPEN},
                      &DeploymentController::state_spinning));
    REQUIRE(
    testHSMTransition(*dpl, Event{EV_MOT_MIN_OPEN_TIME},
                      &DeploymentController::state_awaitingDetachment));
    
    SECTION("WAITING DETACHEMENT -> Idle")
    {
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_NC_DETACHED},
                          &DeploymentController::state_idle));
    }
    SECTION("OPENING NOSECONE -> Idle")
    {
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_MOT_OPEN},
                          &DeploymentController::state_idle));
    }
    SECTION("Deferred event: EV_CUT_MAIN")
    {
        // Send CUT_MAIN: nothing should happen
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_CUT_MAIN},
                            &DeploymentController::state_awaitingDetachment));
        // Go back in IDLE
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_MOT_OPEN},
                          &DeploymentController::state_idle));
        // Wait a bit to allow EV_ENTRY handling
        Thread::sleep(10);
        // CUT_MAIN should now be processed
        REQUIRE( dpl->testState(&DeploymentController::state_cuttingMain ));
    }
    SECTION("Deferred event: EV_CUT_DROGUE")
    {
        // Send CUT_DROGUE: nothing should happen
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_CUT_DROGUE},
                            &DeploymentController::state_awaitingDetachment));
        // Go back in IDLE
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_MOT_OPEN},
                          &DeploymentController::state_idle));
        // Wait a bit to allow EV_ENTRY handling
        Thread::sleep(10);
        // CUT_DROGUE should now be processed
        REQUIRE( dpl->testState(&DeploymentController::state_cuttingDrogue ));
    }
}