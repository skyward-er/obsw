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
#include <boards/DeathStack/TMTCManager/TMTCManager.h>
#include <boards/DeathStack/configs/TMTCConfig.h>
#include "utils/TestHelper.h"

using miosix::Thread;
using namespace DeathStackBoard;
using namespace CanInterfaces;

class tmtcmanagerTestFixture
{
public:
    tmtcmanagerTestFixture()
    {
        tmtcm = new TMTCManager();
        sEventBroker->start();
        tmtcm->start();
    }
    ~tmtcmanagerTestFixture()
    {
        tmtcm->stop();
        sEventBroker->clearDelayedEvents();
        delete tmtcm;
    }

protected:
    TMTCManager* tmtcm;
};

TEST_CASE_METHOD(tmtcmanagerTestFixture, "Testing all the transitions")
{

    SECTION("IDLE -> HIGH RATE TELEMETRY -> LOW RATE TELEMETRY -> LANDED")
    {

        TRACE("test case 1");
        REQUIRE(testFSMTransition(*tmtcm, Event{EV_LIFTOFF},
                                  &TMTCManager::stateHighRateTM));

        REQUIRE(testFSMTransition(*tmtcm, Event{EV_APOGEE},
                                  &TMTCManager::stateLowRateTM));

        REQUIRE(testFSMTransition(*tmtcm, Event{EV_LANDED},
                                  &TMTCManager::stateLanded));
    }
}

TEST_CASE_METHOD(tmtcmanagerTestFixture, "Testing IDLE functions")
{
    TRACE("test case 2");
    SECTION("testing EV_GS_OFFLINE")  // since the GS_OFFLINE_TIMEOUT equals to
                                      // 30 minutes this variable should be
                                      // changed in order to try these tests
    {
        SECTION("creating EV_GS_OFFLINE")  // creating delayed EV_GS_OFFLINE
        {
            TRACE("section 3");
            long long start = miosix::getTick();
            REQUIRE(expectEvent(EV_GS_OFFLINE, TOPIC_FLIGHT_EVENTS,
                                start + GS_OFFLINE_TIMEOUT));
        }

        SECTION("deleting EV_GS_OFFLINE")  // deleting delayed EV_GS_OFFLINE
                                           // after exiting IDLE status
        {
            EventCounter counter{*sEventBroker};
            counter.subscribe(TOPIC_FLIGHT_EVENTS);

            long long start = miosix::getTick();
            REQUIRE(testFSMTransition(*tmtcm, Event{EV_LIFTOFF},
                                      &TMTCManager::stateHighRateTM));

            Thread::sleep(GS_OFFLINE_TIMEOUT);
            REQUIRE(counter.getCount(EV_GS_OFFLINE) == 0);
        }
    }
}

TEST_CASE_METHOD(tmtcmanagerTestFixture, "Testing HighRateTM functions")
{

    SECTION("testing EV_SEND_HR_TM")
    {
        SECTION("creating EV_SEND_HR_TM")  // creating delayed EV_SEND_HR_TM and
                                           // cheking how it is reposted 3 times
        {
            long long start = miosix::getTick();
            REQUIRE(testFSMTransition(*tmtcm, Event{EV_LIFTOFF},
                                      &TMTCManager::stateHighRateTM));

            REQUIRE(
                expectEvent(EV_SEND_HR_TM, TOPIC_TMTC, start + HR_TM_TIMEOUT));
            REQUIRE(expectEvent(EV_SEND_HR_TM, TOPIC_TMTC,
                                start + HR_TM_TIMEOUT * 2));
            REQUIRE(expectEvent(EV_SEND_HR_TM, TOPIC_TMTC,
                                start + HR_TM_TIMEOUT * 3));
        }

        SECTION("deleting EV_SEND_HR_TM")  // deleting delayed EV_SEND_HR_TM
                                           // after exiting HIGH_RATE_TM status
        {

            long long start = miosix::getTick();

            REQUIRE(testFSMTransition(*tmtcm, Event{EV_LIFTOFF},
                                      &TMTCManager::stateHighRateTM));

            REQUIRE(testFSMTransition(*tmtcm, Event{EV_APOGEE},
                                      &TMTCManager::stateLowRateTM));

            EventCounter counter{*sEventBroker};
            counter.subscribe(TOPIC_TMTC);

            Thread::sleep(HR_TM_TIMEOUT);
            REQUIRE(counter.getCount(EV_SEND_HR_TM) == 0);
        }
    }
}

TEST_CASE_METHOD(tmtcmanagerTestFixture, "Testing LowRateTM functions")
{

    SECTION("testing EV_SEND_LR_TM")
    {

        REQUIRE(testFSMTransition(*tmtcm, Event{EV_LIFTOFF},
                                  &TMTCManager::stateHighRateTM));
        REQUIRE(testFSMTransition(*tmtcm, Event{EV_APOGEE},
                                  &TMTCManager::stateLowRateTM));

        SECTION("creating EV_SEND_LR_TM")  // creating delayed EV_SEND_LR_TM and
                                           // cheking how it is reposted 3 times
        {
            long long start = miosix::getTick();
            REQUIRE(
                expectEvent(EV_SEND_LR_TM, TOPIC_TMTC, start + LR_TM_TIMEOUT));
            REQUIRE(expectEvent(EV_SEND_LR_TM, TOPIC_TMTC,
                                start + LR_TM_TIMEOUT * 2));
            REQUIRE(expectEvent(EV_SEND_LR_TM, TOPIC_TMTC,
                                start + LR_TM_TIMEOUT * 3));
        }

        SECTION("deleting EV_SEND_LR_TM")  // deleting delayed EV_SEND_LR_TM
                                           // after exiting LOW_RATE_TM status
        {

            REQUIRE(testFSMTransition(*tmtcm, Event{EV_LANDED},
                                      &TMTCManager::stateLanded));
            
            EventCounter counter{*sEventBroker};
            counter.subscribe(TOPIC_TMTC);

            Thread::sleep(LR_TM_TIMEOUT*2);
            REQUIRE(counter.getCount(EV_SEND_LR_TM) == 0); 
        }
    }
}

TEST_CASE_METHOD(tmtcmanagerTestFixture, "Testing LANDED functions")
{

    SECTION("testing EV_SEND_POS_TM")
    {

        REQUIRE(testFSMTransition(*tmtcm, Event{EV_LIFTOFF},
                                  &TMTCManager::stateHighRateTM));
        REQUIRE(testFSMTransition(*tmtcm, Event{EV_APOGEE},
                                  &TMTCManager::stateLowRateTM));
        REQUIRE(testFSMTransition(*tmtcm, Event{EV_LANDED},
                                      &TMTCManager::stateLanded));
        

        SECTION("creating EV_SEND_POS_TM")  // creating delayed EV_SEND_POS_TM and
                                           // cheking how it is reposted 3 times
        {
            long long start = miosix::getTick();
            REQUIRE(
                expectEvent(EV_SEND_POS_TM, TOPIC_TMTC, start + POS_TM_TIMEOUT));
            REQUIRE(expectEvent(EV_SEND_POS_TM, TOPIC_TMTC,
                                start + POS_TM_TIMEOUT * 2));
            REQUIRE(expectEvent(EV_SEND_POS_TM, TOPIC_TMTC,
                                start + POS_TM_TIMEOUT * 3));
        }

    }
}
