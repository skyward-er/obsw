/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include <miosix.h>

#include <utils/testutils/catch.hpp>

#include "events/Events.h"

#define private public
#define protected public

#include "FlightModeManager/FlightModeManager.h"
#include "utils/testutils/TestHelper.h"

using miosix::Thread;
using namespace DeathStackBoard;

class FMMFixture
{
public:
    // This is called at the beginning of each test / section
    FMMFixture()
    {
        sEventBroker->start();
        fsm = new FlightModeManager();
        fsm->start();
    }

    // This is called at the end of each test / section
    ~FMMFixture()
    {
        fsm->stop();
        sEventBroker->unsubscribe(fsm);
        sEventBroker->clearDelayedEvents();
        delete fsm;
    }

protected:
    FlightModeManager* fsm;
};

TEST_CASE_METHOD(FMMFixture, "Testing transitions from state_onGround")
{
    SECTION("EV_TC_RESET_BOARD -> ON_GROUND (INIT)")
    {
        // If you decomment this the board will reset continuously,
        // since the FMM receives the event that triggers a reboot...
        // I needed one hour of debugging to figure it out :')
        /*
            REQUIRE(testHSMTransition(
                *fsm, Event{EV_TC_RESET_BOARD},
                &FlightModeManager::state_init));  // initial state of
                                                   // state_onGround
        */
    }

    SECTION("EV_TC_LAUNCH -> FLYING (ASCENDING)")
    {
        REQUIRE(testHSMTransition(
            *fsm, Event{EV_TC_LAUNCH},
            &FlightModeManager::state_ascending));  // initial state of
                                                    // state_flying
    }
}

TEST_CASE_METHOD(FMMFixture, "Testing transitions from state_flying")
{
    // move to state_flying (state_ascending)
    fsm->postEvent(Event{EV_INIT_OK});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_TC_CALIBRATE_SENSORS});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_SENSORS_READY});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_CALIBRATION_OK});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_TC_ARM});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_UMBILICAL_DETACHED});
    Thread::sleep(100);

    SECTION("EV_TC_END_MISSION -> LANDED")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_TC_END_MISSION},
                                  &FlightModeManager::state_landed));
    }

    SECTION("EV_TIMEOUT_END_MISSION -> LANDED")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_TIMEOUT_END_MISSION},
                                  &FlightModeManager::state_landed));
    }
}

TEST_CASE_METHOD(FMMFixture, "Testing transitions from state_init")
{
    SECTION("EV_INIT_OK -> INIT_DONE")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_INIT_OK},
                                  &FlightModeManager::state_initDone));
    }

    SECTION("EV_INIT_ERROR -> INIT_ERROR")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_INIT_ERROR},
                                  &FlightModeManager::state_initError));
    }
}

TEST_CASE_METHOD(FMMFixture, "Testing transitions from state_initError")
{
    // move to state_initError
    fsm->postEvent(Event{EV_INIT_ERROR});
    Thread::sleep(50);

    SECTION("EV_TC_FORCE_INIT -> INIT_DONE")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_TC_FORCE_INIT},
                                  &FlightModeManager::state_initDone));
    }
}

TEST_CASE_METHOD(FMMFixture, "Testing transitions from state_initDone")
{
    // move to state_initDone
    fsm->postEvent(Event{EV_INIT_OK});
    Thread::sleep(50);

    SECTION("EV_TC_TEST_MODE -> TEST_MODE")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_TC_TEST_MODE},
                                  &FlightModeManager::state_testMode));
    }

    SECTION("EV_TC_CALIBRATE_SENSORS -> SENSORS_CALIBRATION")
    {
        REQUIRE(
            testHSMTransition(*fsm, Event{EV_TC_CALIBRATE_SENSORS},
                              &FlightModeManager::state_sensorsCalibration));
    }
}

/*
TEST_CASE_METHOD(FMMFixture, "Testing transitions from state_testMode")
{

}
*/

TEST_CASE_METHOD(FMMFixture,
                 "Testing transitions from state_sensorsCalibration")
{
    // move to state_calibrating
    fsm->postEvent(Event{EV_INIT_OK});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_TC_CALIBRATE_SENSORS});
    Thread::sleep(100);

    SECTION("EV_TC_CALIBRATE_SENSORS -> SENSORS_CALIBRATION")
    {
        REQUIRE(
            testHSMTransition(*fsm, Event{EV_TC_CALIBRATE_SENSORS},
                              &FlightModeManager::state_sensorsCalibration));
    }

    SECTION("EV_SENSORS_READY -> ALGOS_CALIBRATION")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_SENSORS_READY},
                                  &FlightModeManager::state_algosCalibration));
    }
}

TEST_CASE_METHOD(FMMFixture, "Testing transitions from state_algosCalibration")
{
    // move to state_calibrating
    fsm->postEvent(Event{EV_INIT_OK});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_TC_CALIBRATE_SENSORS});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_SENSORS_READY});
    Thread::sleep(100);

    SECTION("EV_TC_CALIBRATE_ALGOS -> ALGOS_CALIBRATION")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_TC_CALIBRATE_ALGOS},
                                  &FlightModeManager::state_algosCalibration));
    }

    SECTION("EV_CALIBRATION_OK -> DISARMED")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_CALIBRATION_OK},
                                  &FlightModeManager::state_disarmed));
    }
}

TEST_CASE_METHOD(FMMFixture, "Testing transitions from state_disarmed")
{
    // move to state_disarmed
    fsm->postEvent(Event{EV_INIT_OK});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_TC_CALIBRATE_SENSORS});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_SENSORS_READY});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_CALIBRATION_OK});
    Thread::sleep(100);

    SECTION("EV_TC_CALIBRATE_ALGOS -> ALGOS_CALIBRATION")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_TC_CALIBRATE_ALGOS},
                                  &FlightModeManager::state_algosCalibration));
    }

    SECTION("EV_TC_CALIBRATE_SENSORS -> SENSORS_CALIBRATION")
    {
        REQUIRE(
            testHSMTransition(*fsm, Event{EV_TC_CALIBRATE_SENSORS},
                              &FlightModeManager::state_sensorsCalibration));
    }

    SECTION("EV_TC_ARM -> ARMED")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_TC_ARM},
                                  &FlightModeManager::state_armed));
    }
}

TEST_CASE_METHOD(FMMFixture, "Testing transitions from state_armed")
{
    // move to state_armed
    fsm->postEvent(Event{EV_INIT_OK});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_TC_CALIBRATE_SENSORS});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_SENSORS_READY});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_CALIBRATION_OK});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_TC_ARM});
    Thread::sleep(100);

    SECTION("EV_TC_DISARM -> DISARMED")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_TC_DISARM},
                                  &FlightModeManager::state_disarmed));
    }

    SECTION("EV_TC_LAUNCH -> ASCENDING")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_TC_LAUNCH},
                                  &FlightModeManager::state_ascending));
    }

    SECTION("EV_UMBILICAL_DETACHED -> ASCENDING")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_UMBILICAL_DETACHED},
                                  &FlightModeManager::state_ascending));
    }
}

TEST_CASE_METHOD(FMMFixture, "Testing transitions from state_ascending")
{
    // move to state_ascending
    fsm->postEvent(Event{EV_INIT_OK});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_TC_CALIBRATE_SENSORS});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_SENSORS_READY});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_CALIBRATION_OK});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_TC_ARM});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_UMBILICAL_DETACHED});
    Thread::sleep(100);

    SECTION("EV_ADA_APOGEE_DETECTED -> DROGUE_DESCENT")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_ADA_APOGEE_DETECTED},
                                  &FlightModeManager::state_drogueDescent));
    }

    SECTION("EV_ADA_DISABLE_ABK -> ASCENDING")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_ADA_DISABLE_ABK},
                                  &FlightModeManager::state_ascending));
    }

    SECTION("EV_TC_NC_OPEN -> DROGUE_DESCENT")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_TC_NC_OPEN},
                                  &FlightModeManager::state_drogueDescent));
    }
}

TEST_CASE_METHOD(FMMFixture, "Testing transitions from state_drogueDescent")
{
    // move to state_drogueDescent
    fsm->postEvent(Event{EV_INIT_OK});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_TC_CALIBRATE_SENSORS});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_SENSORS_READY});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_CALIBRATION_OK});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_TC_ARM});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_UMBILICAL_DETACHED});
    Thread::sleep(50);
    fsm->postEvent(Event{EV_ADA_APOGEE_DETECTED});
    Thread::sleep(100);

    SECTION("EV_ADA_DPL_ALT_DETECTED -> TERMINAL_DESCENT")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_ADA_DPL_ALT_DETECTED},
                                  &FlightModeManager::state_terminalDescent));
    }

    SECTION("EV_TC_CUT_DROGUE -> TERMINAL_DESCENT")
    {
        REQUIRE(testHSMTransition(*fsm, Event{EV_TC_CUT_DROGUE},
                                  &FlightModeManager::state_terminalDescent));
    }
}