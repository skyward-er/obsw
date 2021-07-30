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
#define private public
#define protected public

#include <utils/testutils/catch.hpp>

#include "MockSensors/MockSensors.h"
#include "NavigationSystem/NASController.h"
#include "events/Events.h"
#include "utils/testutils/TestHelper.h"

using namespace DeathStackBoard;

using NASCtrl = NASController<MockIMUData, PressureData, GPSData>;

class NASControllerFixture
{
public:
    // This is called at the beginning of each test / section
    NASControllerFixture() : controller(mock_imu, mock_baro, mock_gps)
    {
        sEventBroker->start();
        controller.start();
    }

    // This is called at the end of each test / section
    ~NASControllerFixture()
    {
        controller.stop();
        sEventBroker->unsubscribe(&controller);
        sEventBroker->clearDelayedEvents();
    }

protected:
    MockIMU mock_imu;
    MockPressureSensor mock_baro;
    MockGPS mock_gps;

    NASCtrl controller;
};

TEST_CASE_METHOD(NASControllerFixture, "Testing transitions from idle")
{
    // controller.transition(&NASCtrl::state_idle);

    SECTION("EV_CALIBRATE_NAS -> CALIBRATING")
    {
        REQUIRE(testFSMTransition(controller, Event{EV_CALIBRATE_NAS},
                                  &NASCtrl::state_calibrating));
    }
}

TEST_CASE_METHOD(NASControllerFixture, "Testing transitions from calibrating")
{
    controller.transition(&NASCtrl::state_calibrating);

    SECTION("EV_CALIBRATE_NAS -> CALIBRATING")
    {
        REQUIRE(testFSMTransition(controller, Event{EV_CALIBRATE_NAS},
                                  &NASCtrl::state_calibrating));
    }

    SECTION("EV_NAS_READY -> READY")
    {
        REQUIRE(testFSMTransition(controller, Event{EV_NAS_READY},
                                  &NASCtrl::state_ready));
    }
}

TEST_CASE_METHOD(NASControllerFixture, "Testing transitions from ready")
{
    controller.transition(&NASCtrl::state_ready);

    SECTION("EV_LIFTOFF -> ACTIVE")
    {
        REQUIRE(testFSMTransition(controller, Event{EV_LIFTOFF},
                                  &NASCtrl::state_active));
    }

    SECTION("EV_CALIBRATE_NAS -> CALIBRATING")
    {
        REQUIRE(testFSMTransition(controller, Event{EV_CALIBRATE_NAS},
                                  &NASCtrl::state_calibrating));
    }
}

TEST_CASE_METHOD(NASControllerFixture, "Testing transitions from active")
{
    controller.transition(&NASCtrl::state_active);

    SECTION("EV_LANDED -> END")
    {
        REQUIRE(testFSMTransition(controller, Event{EV_LANDED},
                                  &NASCtrl::state_end));
    }
}

TEST_CASE_METHOD(NASControllerFixture, "Testing transitions from end")
{
    controller.transition(&NASCtrl::state_end);
}