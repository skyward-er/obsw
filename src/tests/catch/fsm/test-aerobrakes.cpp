/*
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

#include <utils/testutils/catch.hpp>

#include "AeroBrakesController/AeroBrakesController.h"
#include "AeroBrakesController/AeroBrakesServo.h"
#include "configs/AeroBrakesConfig.h"
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

class AeroBrakesControllerFixture
{
public:
    // This is called at the beginning of each test / section
    AeroBrakesControllerFixture()
    {
        controller = new AeroBrakesController<InputClass>(sensor);
        sEventBroker->start();
        controller->start();
    }

    // This is called at the end of each test / section
    ~AeroBrakesControllerFixture()
    {
        controller->stop();
        sEventBroker->unsubscribe(controller);
        sEventBroker->clearDelayedEvents();
        delete controller;
    }

protected:
    AeroBrakesController<InputClass>* controller;

    MockSensor<InputClass> sensor;
};

TEST_CASE_METHOD(AeroBrakesControllerFixture,
                 "Testing transitions from initialization")
{
    controller->transition(
        &AeroBrakesController<InputClass>::state_initialization);

    REQUIRE(
        controller->testState(&AeroBrakesController<InputClass>::state_idle));
}

TEST_CASE_METHOD(AeroBrakesControllerFixture, "Testing transitions from idle")
{
    controller->transition(&AeroBrakesController<InputClass>::state_idle);

    SECTION("EV_LIFTOFF -> SHADOW_MODE")
    {
        REQUIRE(testFSMTransition(
            *controller, {EV_LIFTOFF},
            &AeroBrakesController<InputClass>::state_shadowMode));
    }

    SECTION("EV_WIGGLE_SERVO -> IDLE")
    {
        REQUIRE(
            testFSMTransition(*controller, {EV_WIGGLE_SERVO},
                              &AeroBrakesController<InputClass>::state_idle));
    }

    SECTION("EV_RESET_SERVO -> IDLE")
    {
        REQUIRE(
            testFSMTransition(*controller, {EV_RESET_SERVO},
                              &AeroBrakesController<InputClass>::state_idle));
    }

    SECTION("EV_TEST_ABK -> TEST_AEROBRAKES")
    {
        REQUIRE(testFSMTransition(
            *controller, {EV_TEST_ABK},
            &AeroBrakesController<InputClass>::state_testAerobrakes));
    }
}

TEST_CASE_METHOD(AeroBrakesControllerFixture,
                 "Testing transitions from shadow_mode")
{
    controller->transition(
        &AeroBrakesController<InputClass>::state_shadowMode);

    SECTION("EV_SHADOW_MODE_TIMEOUT -> ENABLED")
    {
        REQUIRE(testFSMTransition(
            *controller, {EV_SHADOW_MODE_TIMEOUT},
            &AeroBrakesController<InputClass>::state_enabled));
    }

    SECTION("EV_DISABLE_ABK -> DISABLED")
    {
        REQUIRE(testFSMTransition(
            *controller, {EV_DISABLE_ABK},
            &AeroBrakesController<InputClass>::state_disabled));
    }
}

TEST_CASE_METHOD(AeroBrakesControllerFixture,
                 "Testing transitions from enabled")
{
    controller->transition(&AeroBrakesController<InputClass>::state_enabled);

    SECTION("EV_APOGEE -> END")
    {
        REQUIRE(
            testFSMTransition(*controller, {EV_APOGEE},
                              &AeroBrakesController<InputClass>::state_end));
    }

    SECTION("EV_DISABLE_ABK -> DISABLED")
    {
        REQUIRE(testFSMTransition(
            *controller, {EV_DISABLE_ABK},
            &AeroBrakesController<InputClass>::state_disabled));
    }
}

TEST_CASE_METHOD(AeroBrakesControllerFixture, "Testing transitions from end")
{
    controller->transition(&AeroBrakesController<InputClass>::state_end);
}

TEST_CASE_METHOD(AeroBrakesControllerFixture,
                 "Testing transitions from disabled")
{
    controller->transition(&AeroBrakesController<InputClass>::state_disabled);
}

TEST_CASE_METHOD(AeroBrakesControllerFixture,
                 "Testing transitions from test_aerobrakes")
{
    controller->transition(
        &AeroBrakesController<InputClass>::state_testAerobrakes);

    SECTION("EV_TEST_TIMEOUT -> IDLE")
    {
        REQUIRE(
            testFSMTransition(*controller, {EV_TEST_TIMEOUT},
                              &AeroBrakesController<InputClass>::state_idle));
    }
}