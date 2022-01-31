/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#ifdef STANDALONE_CATCH1_TEST
#include "../catch-tests-entry.cpp"
#endif

// We need access to the handleEvent(...) function in state machines in order to
// test them synchronously
#define protected public

#include <AirBrakes/AirBrakesController.h>
#include <AirBrakes/AirBrakesServo.h>
#include <miosix.h>

#include <catch2/catch.hpp>

#include "configs/AirBrakesConfig.h"
#include "events/Events.h"
#include "utils/testutils/TestHelper.h"

using miosix::Thread;
using namespace DeathStackBoard;

class InputClass
{
public:
    float z;
    float vz;
    float vMod;
    uint64_t timestamp;
};

template <typename T>
class MockSensor : public Sensor<T>
{
private:
    int ts = 0;

protected:
    T sampleImpl() override
    {
        T data;
        return data;
    }

    bool init() override { return true; }

    bool selfTest() override { return true; }
};

class AirBrakesControllerFixture
{
public:
    // This is called at the beginning of each test / section
    AirBrakesControllerFixture()
    {
        controller = new AirBrakesController<InputClass>(sensor);
        sEventBroker.start();
        controller->start();
    }

    // This is called at the end of each test / section
    ~AirBrakesControllerFixture()
    {
        controller->stop();
        sEventBroker.unsubscribe(controller);
        sEventBroker.clearDelayedEvents();
        delete controller;
    }

protected:
    AirBrakesController<InputClass>* controller;

    MockSensor<InputClass> sensor;
};

TEST_CASE_METHOD(AirBrakesControllerFixture,
                 "Testing transitions from initialization")
{
    controller->transition(
        &AirBrakesController<InputClass>::state_initialization);

    REQUIRE(
        controller->testState(&AirBrakesController<InputClass>::state_idle));
}

TEST_CASE_METHOD(AirBrakesControllerFixture, "Testing transitions from idle")
{
    controller->transition(&AirBrakesController<InputClass>::state_idle);

    SECTION("EV_LIFTOFF -> SHADOW_MODE")
    {
        REQUIRE(testFSMTransition(
            *controller, {EV_LIFTOFF},
            &AirBrakesController<InputClass>::state_shadowMode));
    }

    SECTION("EV_WIGGLE_SERVO -> IDLE")
    {
        REQUIRE(
            testFSMTransition(*controller, {EV_WIGGLE_SERVO},
                              &AirBrakesController<InputClass>::state_idle));
    }

    SECTION("EV_RESET_SERVO -> IDLE")
    {
        REQUIRE(
            testFSMTransition(*controller, {EV_RESET_SERVO},
                              &AirBrakesController<InputClass>::state_idle));
    }

    SECTION("EV_TEST_ABK -> TEST_AEROBRAKES")
    {
        REQUIRE(testFSMTransition(
            *controller, {EV_TEST_ABK},
            &AirBrakesController<InputClass>::state_testAirbrakes));
    }
}

TEST_CASE_METHOD(AirBrakesControllerFixture,
                 "Testing transitions from shadow_mode")
{
    controller->transition(&AirBrakesController<InputClass>::state_shadowMode);

    SECTION("EV_SHADOW_MODE_TIMEOUT -> ENABLED")
    {
        REQUIRE(
            testFSMTransition(*controller, {EV_SHADOW_MODE_TIMEOUT},
                              &AirBrakesController<InputClass>::state_enabled));
    }

    SECTION("EV_DISABLE_ABK -> DISABLED")
    {
        REQUIRE(testFSMTransition(
            *controller, {EV_DISABLE_ABK},
            &AirBrakesController<InputClass>::state_disabled));
    }
}

TEST_CASE_METHOD(AirBrakesControllerFixture, "Testing transitions from enabled")
{
    controller->transition(&AirBrakesController<InputClass>::state_enabled);

    SECTION("EV_APOGEE -> END")
    {
        REQUIRE(testFSMTransition(*controller, {EV_APOGEE},
                                  &AirBrakesController<InputClass>::state_end));
    }

    SECTION("EV_DISABLE_ABK -> DISABLED")
    {
        REQUIRE(testFSMTransition(
            *controller, {EV_DISABLE_ABK},
            &AirBrakesController<InputClass>::state_disabled));
    }
}

TEST_CASE_METHOD(AirBrakesControllerFixture, "Testing transitions from end")
{
    controller->transition(&AirBrakesController<InputClass>::state_end);
}

TEST_CASE_METHOD(AirBrakesControllerFixture,
                 "Testing transitions from disabled")
{
    controller->transition(&AirBrakesController<InputClass>::state_disabled);
}

TEST_CASE_METHOD(AirBrakesControllerFixture,
                 "Testing transitions from test_airbrakes")
{
    controller->transition(
        &AirBrakesController<InputClass>::state_testAirbrakes);

    SECTION("EV_TEST_TIMEOUT -> IDLE")
    {
        REQUIRE(
            testFSMTransition(*controller, {EV_TEST_TIMEOUT},
                              &AirBrakesController<InputClass>::state_idle));
    }
}