/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Benedetta Margrethe Cattani
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
#include <utils/catch.hpp>

#include <boards/CanInterfaces.h>
#include <boards/DeathStack/EventClasses.h>
#include <boards/DeathStack/Events.h>
#include <boards/DeathStack/IgnitionController/IgnitionController.h>
#include <boards/DeathStack/configs/IgnitionConfig.h>
#include "utils/TestHelper.h"

using miosix::Thread;
using namespace DeathStackBoard;
using namespace CanInterfaces;

class IgnitionTestFixture
{
public:
    IgnitionTestFixture()
    {
        can_mgr = new CanManager(CAN1);
        can     = new CanProxy(can_mgr);
        ign     = new IgnitionController(can);
    }
    ~IgnitionTestFixture()
    {
        sEventBroker->unsubscribe(ign);
        sEventBroker->clearDelayedEvents();
        delete ign;
        delete can;
        delete can_mgr;
    }

protected:
    CanProxy* can;
    CanManager* can_mgr;
    IgnitionController* ign;
};

TEST_CASE_METHOD(IgnitionTestFixture, "Testing IDLE transitions")
{

    SECTION("IDLE -> END")
    {
        REQUIRE(testFSMTransition(*ign, Event{EV_LIFTOFF},
                                  &IgnitionController::stateEnd));
    }

    SECTION("IDLE -> ABORT")
    {
        CanbusEvent ce;
        ce.canTopic = CanInterfaces::CAN_TOPIC_IGNITION;
        ce.sig      = EV_NEW_CAN_MSG;

        // In order to test the transition from idle to abort, the three abort
        // cases must be analyzed separately (for each processor).

        SECTION("STM32 ABORTED CMD")
        {
            IgnitionBoardStatus ibs;
            memset(&ibs, 0, sizeof(IgnitionBoardStatus));
            ibs.stm32_abortCmd = 1;
            memcpy(ce.payload, &ibs, sizeof(IgnitionBoardStatus));
            ce.len = sizeof(IgnitionBoardStatus);
            REQUIRE(
                testFSMTransition(*ign, ce, &IgnitionController::stateAborted));
        }

        SECTION("STM32 ABORTED TIMEOUT")
        {
            IgnitionBoardStatus ibs;
            memset(&ibs, 0, sizeof(IgnitionBoardStatus));
            ibs.stm32_abortTimeout = 1;
            memcpy(ce.payload, &ibs, sizeof(IgnitionBoardStatus));
            ce.len = sizeof(IgnitionBoardStatus);
            REQUIRE(
                testFSMTransition(*ign, ce, &IgnitionController::stateAborted));
        }

        SECTION("STM32 ABORTED WRONG CODE")
        {
            IgnitionBoardStatus ibs;
            memset(&ibs, 0, sizeof(IgnitionBoardStatus));
            ibs.stm32_abortWrongCode = 1;
            memcpy(ce.payload, &ibs, sizeof(IgnitionBoardStatus));
            ce.len = sizeof(IgnitionBoardStatus);
            REQUIRE(
                testFSMTransition(*ign, ce, &IgnitionController::stateAborted));
        }

        SECTION("AVR ABORTED CMD")
        {
            IgnitionBoardStatus ibs;
            memset(&ibs, 0, sizeof(IgnitionBoardStatus));
            ibs.avr_abortCmd = 1;
            memcpy(ce.payload, &ibs, sizeof(IgnitionBoardStatus));
            ce.len = sizeof(IgnitionBoardStatus);
            REQUIRE(
                testFSMTransition(*ign, ce, &IgnitionController::stateAborted));
        }

        SECTION("AVR ABORTED TIMEOUT")
        {
            IgnitionBoardStatus ibs;
            memset(&ibs, 0, sizeof(IgnitionBoardStatus));
            ibs.avr_abortTimeout = 1;
            memcpy(ce.payload, &ibs, sizeof(IgnitionBoardStatus));
            ce.len = sizeof(IgnitionBoardStatus);
            REQUIRE(
                testFSMTransition(*ign, ce, &IgnitionController::stateAborted));
        }

        SECTION("AVR ABORTED WRONG CODE")
        {
            IgnitionBoardStatus ibs;
            memset(&ibs, 0, sizeof(IgnitionBoardStatus));
            ibs.avr_abortWrongCode = 1;
            memcpy(ce.payload, &ibs, sizeof(IgnitionBoardStatus));
            ce.len = sizeof(IgnitionBoardStatus);
            REQUIRE(
                testFSMTransition(*ign, ce, &IgnitionController::stateAborted));
        }
    }
}

class IgnitionTestFixture2
{
public:
    IgnitionTestFixture2()
    {
        sEventBroker->start();
        can_mgr = new CanManager(CAN1);
        can     = new CanProxy(can_mgr);
        ign     = new IgnitionController(can);
        ign->start();
    }
    ~IgnitionTestFixture2()
    {
        ign->stop();
        sEventBroker->unsubscribe(ign);
        sEventBroker->clearDelayedEvents();
        delete ign;
        delete can;
        delete can_mgr;
    }

protected:
    CanProxy* can;
    CanManager* can_mgr;
    IgnitionController* ign;
};

TEST_CASE_METHOD(IgnitionTestFixture2, "Testing IDLE functions")
{
    SECTION("IGNITION OFFLINE")
    {
        SECTION("IGNITION OFFLINE 1")  // Wait for EV_IGN_OFFLINE when we do not
                                       // post any ign status
        {
            EventCounter counter{*sEventBroker};
            counter.subscribe(TOPIC_FLIGHT_EVENTS);

            Thread::sleep(TIMEOUT_IGN_OFFLINE - 5);

            REQUIRE(counter.getCount(EV_IGN_OFFLINE) == 0);

            Thread::sleep(5500);

            REQUIRE(ign->testState(&IgnitionController::stateIdle));

            REQUIRE(counter.getCount(EV_IGN_OFFLINE) == 1);
        }

        SECTION("IGNITION OFFLINE 2")  // Wait for EV_IGN_OFFLINE after we post
                                       // a NEW_CAN_MSG
        {
            TRACE("Beginning of section\n");

            EventCounter counter{*sEventBroker};
            counter.subscribe(TOPIC_FLIGHT_EVENTS);

            // Sending ign status
            CanbusEvent ce;
            ce.canTopic = CanInterfaces::CAN_TOPIC_IGNITION;
            ce.sig      = EV_NEW_CAN_MSG;

            IgnitionBoardStatus ibs;
            memset(&ibs, 0, sizeof(IgnitionBoardStatus));
            memcpy(ce.payload, &ibs, sizeof(IgnitionBoardStatus));
            ce.len = sizeof(IgnitionBoardStatus);

            Thread::sleep(TIMEOUT_IGN_OFFLINE / 2);

            REQUIRE(
                testFSMTransition(*ign, ce, &IgnitionController::stateIdle));

            Thread::sleep(TIMEOUT_IGN_OFFLINE / 2 + 5);

            REQUIRE(counter.getCount(EV_IGN_OFFLINE) == 0);

            Thread::sleep(TIMEOUT_IGN_OFFLINE - 10);

            REQUIRE(ign->testState(&IgnitionController::stateIdle));

            Thread::sleep(10);

            REQUIRE(counter.getCount(EV_IGN_OFFLINE) == 1);
        }
    }

    SECTION("TESTING GET STATUS") //
    {
        long long start = miosix::getTick();

        REQUIRE(expectEvent(EV_IGN_GETSTATUS, TOPIC_IGNITION, start, 5));

        REQUIRE(expectEvent(EV_IGN_GETSTATUS, TOPIC_IGNITION,
                            start + INTERVAL_IGN_GET_STATUS, 5));

        REQUIRE(expectEvent(EV_IGN_GETSTATUS, TOPIC_IGNITION,
                            start + INTERVAL_IGN_GET_STATUS * 2, 5));
    }

    SECTION(
        "TESTING REMOVE DELAYED EV_IGN_OFFLINE")  // sending a liftoff command,
                                                  // the delayed ev_ign_offline
                                                  // should be deleted
    {
        EventCounter counter{*sEventBroker};
        counter.subscribe(TOPIC_FLIGHT_EVENTS);

        Thread::sleep(TIMEOUT_IGN_OFFLINE / 2);

        REQUIRE(testFSMTransition(*ign, Event{EV_LIFTOFF},
                                  &IgnitionController::stateEnd));

        Thread::sleep(TIMEOUT_IGN_OFFLINE / 2 + 5);

        REQUIRE(counter.getCount(EV_IGN_OFFLINE) == 0);
    }
}

TEST_CASE_METHOD(IgnitionTestFixture2, "Testing ABORT functions")
{

    CanbusEvent ce;
    ce.canTopic = CanInterfaces::CAN_TOPIC_IGNITION;
    ce.sig      = EV_NEW_CAN_MSG;

    SECTION("TESTING IGN_ABORTED EVENT")
    {

        IgnitionBoardStatus ibs;
        memset(&ibs, 0, sizeof(IgnitionBoardStatus));
        ibs.stm32_abortCmd = 1;
        memcpy(ce.payload, &ibs, sizeof(IgnitionBoardStatus));
        ce.len = sizeof(IgnitionBoardStatus);

        long long start = miosix::getTick();

        REQUIRE(testFSMTransition(*ign, ce, &IgnitionController::stateAborted));

        REQUIRE(expectEvent(EV_IGN_ABORTED, TOPIC_FLIGHT_EVENTS, start, 5));
    }
}
