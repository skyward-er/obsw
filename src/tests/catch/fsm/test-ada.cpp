/*
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#include <miosix.h>

#include <utils/testutils/catch.hpp>

#define private public
#define protected public

#include "ADA/ADAController.h"
#include "MockSensors/MockGPS.h"
#include "MockSensors/MockPressureSensor.h"
#include "events/Events.h"
#include "utils/testutils/TestHelper.h"

using miosix::Thread;
using namespace DeathStackBoard;

using ADACtrl = ADAController<PressureData, GPSData>;

class ADAControllerFixture
{
public:
    // This is called at the beginning of each test / section
    ADAControllerFixture()
    {
        sEventBroker->start();
        controller = new ADACtrl(mock_baro, mock_gps);
        controller->start();
    }

    // This is called at the end of each test / section
    ~ADAControllerFixture()
    {
        controller->stop();
        sEventBroker->unsubscribe(controller);
        sEventBroker->clearDelayedEvents();
        delete controller;
    }

protected:
    MockPressureSensor mock_baro;
    MockGPS mock_gps;

    ADACtrl* controller;
};

TEST_CASE_METHOD(ADAControllerFixture, "Testing transitions from idle")
{
    controller->transition(&ADACtrl::state_idle);

    SECTION("EV_CALIBRATE_ADA -> CALIBRATING")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_CALIBRATE_ADA},
                                  &ADACtrl::state_calibrating));
    }
}

TEST_CASE_METHOD(ADAControllerFixture, "Testing transitions from calibrating")
{
    controller->transition(&ADACtrl::state_calibrating);

    SECTION("EV_CALIBRATE_ADA -> CALIBRATING")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_CALIBRATE_ADA},
                                  &ADACtrl::state_calibrating));
    }
}

TEST_CASE_METHOD(ADAControllerFixture,
                 "Testing transitions from drogue_descent")
{
    controller->transition(
        &ADAController<PressureData, GPSData>::state_drogueDescent);

    SECTION("EV_ADA_READY -> READY")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_ADA_READY},
                                  &ADACtrl::state_ready));
    }
}

TEST_CASE_METHOD(ADAControllerFixture, "Testing transitions from ready")
{
    controller->transition(&ADACtrl::state_ready);

    SECTION("EV_ADA_READY -> READY")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_ADA_READY},
                                  &ADACtrl::state_ready));
    }
}

TEST_CASE_METHOD(ADAControllerFixture, "Testing transitions from ready")
{
    controller->transition(&ADACtrl::state_ready);

    SECTION("EV_LIFTOFF -> SHADOW_MODE")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_LIFTOFF},
                                  &ADACtrl::state_shadowMode));
    }
}

TEST_CASE_METHOD(ADAControllerFixture, "Testing transitions from shadow_mode")
{
    controller->transition(&ADACtrl::state_shadowMode);

    SECTION("EV_SHADOW_MODE_TIMEOUT -> ACTIVE")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_SHADOW_MODE_TIMEOUT},
                                  &ADACtrl::state_active));
    }
}

TEST_CASE_METHOD(ADAControllerFixture, "Testing transitions from active")
{
    controller->transition(&ADACtrl::state_active);

    SECTION("EV_ADA_APOGEE_DETECTED -> PRESSURE_STABILIZATION")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_ADA_APOGEE_DETECTED},
                                  &ADACtrl::state_pressureStabilization));
    }
}

TEST_CASE_METHOD(ADAControllerFixture,
                 "Testing transitions from pressure_stabilization")
{
    controller->transition(&ADACtrl::state_pressureStabilization);

    SECTION("EV_TIMEOUT_PRESS_STABILIZATION -> DROGUE_DESCENT")
    {
        REQUIRE(testFSMTransition(*controller,
                                  Event{EV_TIMEOUT_PRESS_STABILIZATION},
                                  &ADACtrl::state_drogueDescent));
    }
}

TEST_CASE_METHOD(ADAControllerFixture,
                 "Testing transitions from drogue_descent")
{
    controller->transition(&ADACtrl::state_drogueDescent);

    SECTION("EV_ADA_DPL_ALT_DETECTED -> END")
    {
        REQUIRE(testFSMTransition(*controller, Event{EV_ADA_DPL_ALT_DETECTED},
                                  &ADACtrl::state_end));
    }
}

TEST_CASE_METHOD(ADAControllerFixture, "Testing transitions from end")
{
    controller->transition(&ADACtrl::state_end);
}