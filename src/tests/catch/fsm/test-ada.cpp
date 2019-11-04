/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Benedetta Margrethe Cattani & Luca Mozzarelli
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
#define private public

#include <miosix.h>
#include <utils/testutils/catch.hpp>

#include <boards/CanInterfaces.h>
#include <boards/DeathStack/ADA/ADAController.h>
#include <boards/DeathStack/events/Events.h>
#include <boards/DeathStack/configs/ADA_config.h>
#include "utils/testutils/TestHelper.h"

using miosix::Thread;
using namespace DeathStackBoard;
using namespace CanInterfaces;

class ADATestFixture
{
public:
    ADATestFixture() { ada = new ADAController(); }
    ~ADATestFixture()
    {
        sEventBroker->unsubscribe(ada);
        sEventBroker->clearDelayedEvents();
        delete ada;
    }

protected:
    ADAController* ada;
};

TEST_CASE_METHOD(ADATestFixture, "Testing all transitions")
{
    SECTION("Testing CALIBRATION transitions")
    {
        REQUIRE(testFSMTransition(*ada, Event{EV_ADA_READY}, &ADAController::stateIdle));
    }

    SECTION("Testing IDLE transitions")
    {
        REQUIRE(testFSMTransition(*ada, Event{EV_ADA_READY}, &ADAController::stateIdle));

        SECTION("IDLE->CALIBRATION")
        {
            REQUIRE(testFSMTransition(*ada, Event{EV_TC_CALIBRATE_ADA},
                                      &ADAController::stateCalibrating));
        }

        SECTION("IDLE->SHADOW_MODE")
        {
            REQUIRE(testFSMTransition(*ada, Event{EV_LIFTOFF},
                                      &ADAController::stateShadowMode));
        }
    }

    SECTION("Testing all the transition from SHADOW_MODE")
    {
        REQUIRE(testFSMTransition(*ada, Event{EV_ADA_READY}, &ADAController::stateIdle));
        REQUIRE(
            testFSMTransition(*ada, Event{EV_LIFTOFF}, &ADAController::stateShadowMode));
        REQUIRE(testFSMTransition(*ada, Event{EV_TIMEOUT_SHADOW_MODE},
                                  &ADAController::stateActive));
        REQUIRE(testFSMTransition(*ada, Event{EV_APOGEE},
                                  &ADAController::stateFirstDescentPhase));
        REQUIRE(
            testFSMTransition(*ada, Event{EV_DPL_ALTITUDE}, &ADAController::stateEnd));
    }
}