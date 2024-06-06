/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#include <Groundstation/Automated/SMA/SMA.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <events/EventBroker.h>
#include <utils/Debug.h>

#include <memory>
#include <unordered_set>

#define ARP_EVENTS                                                   \
    {                                                                \
        ARP_INIT_OK, ARP_INIT_ERROR, ARP_CAL_DONE, ARP_FIX_ANTENNAS, \
            ARP_FIX_ROCKET                                           \
    }

#define TMTC_EVENTS                                                    \
    {                                                                  \
        TMTC_ARP_FORCE_INIT, TMTC_ARP_RESET_ALGORITHM,                 \
            TMTC_ARP_FORCE_NO_FEEDBACK, TMTC_ARP_ARM, TMTC_ARP_DISARM, \
            TMTC_ARP_CALIBRATE, TMTC_ARP_ENTER_TEST_MODE,              \
            TMTC_ARP_EXIT_TEST_MODE                                    \
    }

#define TEST_STATE(INITIAL_STATE, EVENT, TOPIC_SM, FINAL_STATE)           \
    sm->forceState(&SMA::INITIAL_STATE);                                  \
    EventBroker::getInstance().post(EVENT, TOPIC_SM);                     \
    Thread::sleep(20);                                                    \
    TRACE("Testing %-26s in " #INITIAL_STATE " -> " #FINAL_STATE " ... ", \
          getEventString(EVENT).c_str());                                 \
    correct = sm->testState(&SMA::FINAL_STATE);                           \
    printf(correct ? "OK\n" : "FAIL\n");                                  \
    ok &= correct;

#define TEST_NO_TRANSITION(INITIAL_STATE, EVENT, TOPIC_SM) \
    sm->forceState(&SMA::INITIAL_STATE);                   \
    EventBroker::getInstance().post(EVENT, TOPIC_SM);      \
    Thread::sleep(20);                                     \
    TRACE("Testing %-26s in " #INITIAL_STATE " -X ... ",   \
          getEventString(EVENT).c_str());                  \
    correct = sm->testState(&SMA::INITIAL_STATE);          \
    printf(correct ? "OK\n" : "FAIL\n");                   \
    ok &= correct;

#define TEST_ALL_OTHER(INITIAL_STATE, ...)                     \
    to_exclude = {__VA_ARGS__};                                \
    for (Events ev : arp_events)                               \
    {                                                          \
        if (to_exclude.find(ev) == to_exclude.end())           \
        {                                                      \
            TEST_NO_TRANSITION(INITIAL_STATE, ev, TOPIC_ARP);  \
        }                                                      \
    }                                                          \
    for (Events ev : tmtc_events)                              \
    {                                                          \
        if (to_exclude.find(ev) == to_exclude.end())           \
        {                                                      \
            TEST_NO_TRANSITION(INITIAL_STATE, ev, TOPIC_TMTC); \
        }                                                      \
    }

// for (auto state : {__VA_ARGS__})
// {

using namespace Boardcore;
using namespace Common;
using namespace Antennas;

SMA* sm = new SMA();

int main()
{
    bool correct, ok = true;
    std::unordered_set<Events> to_exclude;
    std::vector<Events> arp_events  = ARP_EVENTS;
    std::vector<Events> tmtc_events = TMTC_EVENTS;

    sm->start();
    TRACE("SMA started\n");

    // TEST STATE: INIT
    TEST_STATE(state_init, ARP_INIT_OK, TOPIC_ARP, state_init_done);
    TEST_STATE(state_init, ARP_INIT_ERROR, TOPIC_ARP, state_init_error);
    TEST_ALL_OTHER(state_init, ARP_INIT_OK, ARP_INIT_ERROR);

    // TEST STATE: INIT_DONE
    TEST_STATE(state_init_done, TMTC_ARP_ARM, TOPIC_TMTC, state_armed);
    TEST_STATE(state_init_done, TMTC_ARP_FORCE_NO_FEEDBACK, TOPIC_TMTC,
               state_insert_info);
    TEST_ALL_OTHER(state_init_done, TMTC_ARP_ARM, TMTC_ARP_FORCE_NO_FEEDBACK);

    // TEST STATE: INIT_ERROR
    TEST_STATE(state_init_error, TMTC_ARP_FORCE_INIT, TOPIC_TMTC,
               state_init_done);
    TEST_STATE(state_init_error, TMTC_ARP_FORCE_NO_FEEDBACK, TOPIC_TMTC,
               state_insert_info);
    TEST_ALL_OTHER(state_init_error, TMTC_ARP_FORCE_INIT,
                   TMTC_ARP_FORCE_NO_FEEDBACK);

    // TEST STATE: INSERT_INFO
    TEST_STATE(state_insert_info, TMTC_ARP_ARM, TOPIC_TMTC, state_armed_nf);
    TEST_ALL_OTHER(state_insert_info, TMTC_ARP_ARM);

    // TEST STATE: ARMED
    TEST_STATE(state_armed, TMTC_ARP_DISARM, TOPIC_TMTC, state_init_done);
    TEST_STATE(state_armed, TMTC_ARP_CALIBRATE, TOPIC_TMTC, state_calibrate);
    TEST_STATE(state_armed, TMTC_ARP_ENTER_TEST_MODE, TOPIC_TMTC, state_test);
    TEST_ALL_OTHER(state_armed, TMTC_ARP_DISARM, TMTC_ARP_CALIBRATE,
                   TMTC_ARP_ENTER_TEST_MODE);

    // TEST STATE: TEST
    TEST_STATE(state_test, TMTC_ARP_EXIT_TEST_MODE, TOPIC_TMTC, state_armed);
    TEST_STATE(state_test, TMTC_ARP_DISARM, TOPIC_TMTC, state_init_done);
    TEST_ALL_OTHER(state_test, TMTC_ARP_EXIT_TEST_MODE, TMTC_ARP_DISARM);

    // TEST STATE: CALIBRATE
    TEST_STATE(state_calibrate, ARP_CAL_DONE, TOPIC_ARP, state_fix_antennas);
    TEST_STATE(state_calibrate, TMTC_ARP_DISARM, TOPIC_TMTC, state_init_done);
    TEST_STATE(state_calibrate, TMTC_ARP_RESET_ALGORITHM, TOPIC_TMTC,
               state_armed);
    TEST_ALL_OTHER(state_calibrate, ARP_CAL_DONE, TMTC_ARP_DISARM,
                   TMTC_ARP_RESET_ALGORITHM);

    // TEST STATE: FIX_ANTENNAS
    TEST_STATE(state_fix_antennas, ARP_FIX_ANTENNAS, TOPIC_ARP,
               state_fix_rocket);
    TEST_STATE(state_fix_antennas, TMTC_ARP_DISARM, TOPIC_TMTC,
               state_init_done);
    TEST_STATE(state_fix_antennas, TMTC_ARP_RESET_ALGORITHM, TOPIC_TMTC,
               state_armed);
    TEST_ALL_OTHER(state_fix_antennas, ARP_FIX_ANTENNAS, TMTC_ARP_DISARM,
                   TMTC_ARP_RESET_ALGORITHM);

    // TEST STATE: FIX_ROCKET
    TEST_STATE(state_fix_rocket, ARP_FIX_ROCKET, TOPIC_ARP, state_active);
    TEST_STATE(state_fix_rocket, TMTC_ARP_DISARM, TOPIC_TMTC, state_init_done);
    TEST_STATE(state_fix_rocket, TMTC_ARP_RESET_ALGORITHM, TOPIC_TMTC,
               state_armed);
    TEST_ALL_OTHER(state_fix_rocket, ARP_FIX_ROCKET, TMTC_ARP_DISARM,
                   TMTC_ARP_RESET_ALGORITHM);

    // TEST STATE: ACTIVE
    TEST_STATE(state_active, TMTC_ARP_DISARM, TOPIC_TMTC, state_init_done);
    TEST_STATE(state_active, TMTC_ARP_RESET_ALGORITHM, TOPIC_TMTC, state_armed);
    TEST_ALL_OTHER(state_active, TMTC_ARP_DISARM, TMTC_ARP_RESET_ALGORITHM);

    // TEST STATE: ARMED_NO_FEEDBACK
    TEST_STATE(state_armed_nf, TMTC_ARP_DISARM, TOPIC_TMTC, state_insert_info);
    TEST_STATE(state_armed_nf, TMTC_ARP_CALIBRATE, TOPIC_TMTC,
               state_fix_rocket_nf);
    TEST_STATE(state_armed_nf, TMTC_ARP_ENTER_TEST_MODE, TOPIC_TMTC,
               state_test_nf);
    TEST_ALL_OTHER(state_armed_nf, TMTC_ARP_DISARM, TMTC_ARP_CALIBRATE,
                   TMTC_ARP_ENTER_TEST_MODE);

    // TEST STATE: TEST_NO_FEEDBACK
    TEST_STATE(state_test_nf, TMTC_ARP_EXIT_TEST_MODE, TOPIC_TMTC,
               state_armed_nf);
    TEST_STATE(state_test_nf, TMTC_ARP_DISARM, TOPIC_TMTC, state_insert_info);
    TEST_ALL_OTHER(state_test_nf, TMTC_ARP_EXIT_TEST_MODE, TMTC_ARP_DISARM);

    // TEST STATE: FIX_ROCKET_NO_FEEDBACK
    TEST_STATE(state_fix_rocket_nf, ARP_FIX_ROCKET, TOPIC_ARP, state_active_nf);
    TEST_STATE(state_fix_rocket_nf, TMTC_ARP_DISARM, TOPIC_TMTC,
               state_insert_info);
    TEST_STATE(state_fix_rocket_nf, TMTC_ARP_RESET_ALGORITHM, TOPIC_TMTC,
               state_armed_nf);
    TEST_ALL_OTHER(state_fix_rocket_nf, ARP_FIX_ROCKET, TMTC_ARP_DISARM,
                   TMTC_ARP_RESET_ALGORITHM);

    // TEST STATE: ACTIVE_NO_FEEDBACK
    TEST_STATE(state_active_nf, TMTC_ARP_DISARM, TOPIC_TMTC, state_insert_info);
    TEST_STATE(state_active_nf, TMTC_ARP_RESET_ALGORITHM, TOPIC_TMTC,
               state_armed_nf);
    TEST_ALL_OTHER(state_active_nf, TMTC_ARP_DISARM, TMTC_ARP_RESET_ALGORITHM);

    TRACE("Testing SMA ... ");
    ok ? printf("OK\n") : printf("FAIL\n");
    return 0;
}
