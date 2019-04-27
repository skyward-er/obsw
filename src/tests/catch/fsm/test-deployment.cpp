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
#include "utils/TestHelper.h"

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
TEST_CASE_METHOD(DeploymentControllerFixture, "Testing transitions from Idle")
{
    
    SECTION("IDLE -> Opening Nosecone")
    {
        REQUIRE(
            testHSMTransition(*dpl, Event{EV_NC_OPEN},
                              &DeploymentController::state_openingNosecone));
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

TEST_CASE_METHOD(DeploymentControllerFixture, "Testing transitions from Cutting Main")
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
}

TEST_CASE_METHOD(DeploymentControllerFixture, "Testing transitions from Cutting Drogue")
{
    REQUIRE(
        testHSMTransition(*dpl, Event{EV_CUT_DROGUE},
                            &DeploymentController::state_cuttingDrogue));

    SECTION(" Cutting Main -> Idle")
    {
        REQUIRE(
        testHSMTransition(*dpl, Event{EV_TIMEOUT_CUTTING},
                            &DeploymentController::state_idle));
    }
}
