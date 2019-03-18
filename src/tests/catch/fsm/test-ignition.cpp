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
#include "catch1-tests-entry.cpp"
#endif

// We need access to the handleEvent(...) function in state machines in order to
// test them synchronously
#define protected public


#include <miosix.h>
#include <catch.hpp>

#include "DeathStack/configs/IgnitionConfig.h"
#include "DeathStack/IgnitionController/IgnitionController.h"
#include "state_machine_test_helper.h"
#include "DeathStack/Events.h
#include "DeathStack/EventClasses.h"
#include "CanInterfaces.h"

using miosix::Thread;
using namespace DeathStackBoard;

TEST_CASE("Testing IDLE transitions")
{
    IgnitionController ign;

    SECTION("IDLE -> END")
    {
        REQUIRE(testFSMTransition(ign, Event{EV_LIFTOFF}, &IgnitionController::stateIdle));
    }

    SECTION("IDLE -> ABORT")
    {
      CanbusEvent ce;


      SECTION("STM32 ABORTED CMD")
      {
        IgnitionBoardStatus ibs;
        ibs.stm32_abortCmd=1;
        memcpy(ce.payload, &ibs, sizeof(ibs));
        ce.len=sizeof(ibs);
        REQUIRE(testFSMTransition(ign, ce, &IgnitionController::stateAborted));
      }

      SECTION("STM32 ABORTED TIMEOUT")
      {
        IgnitionBoardStatus ibs;
        ibs.stm32_abort=1;
      }

      SECTION("STM32 ABORTED WRONG CODE")
      {
        IgnitionBoardStatus ibs;
        ibs.stm32_abortCmd=1;
      }

      SECTION("AVR ABORTED")
      {
        IgnitionBoardStatus ibs;
      }

      ce.cantopic=0;
      ce.
      REQUIRE(testFSMTransition(ign, Event{EV_LIFTOFF}, &IgnitionControler::stateIdle));
    }
