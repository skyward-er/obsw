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
#define private public


#include <miosix.h>
#include <catch.hpp>

#include <boards/DeathStack/configs/IgnitionConfig.h>
#include <boards/DeathStack/IgnitionController/IgnitionController.h>
#include "state_machine_test_helper.h"
#include <boards/DeathStack/Events.h>
#include <boards/DeathStack/EventClasses.h>
#include <boards/CanInterfaces.h>

using miosix::Thread;
using namespace DeathStackBoard;
using namespace CanInterfaces;

class IgnitionTestFicture {
    public:
      IgnitionTestFicture() {
              can_mgr= new CanManager(CAN1);
              can = new CanProxy(can_mgr);
              ign = new IgnitionController(can);
      }
      ~IgnitionTestFicture(){
              sEventBroker->unsubscribe(ign);
              delete ign;
              delete can;
              delete can_mgr;
      }
    protected:
      CanProxy* can;
      CanManager* can_mgr;
      IgnitionController* ign;

};

TEST_CASE_METHOD(IgnitionTestFicture, "Testing IDLE transitions")
{
    //printf("Sta compilando\n");

    SECTION("IDLE -> END")
    {
        REQUIRE(testFSMTransition(*ign, Event{EV_LIFTOFF}, &IgnitionController::stateIdle));
    }

    SECTION("IDLE -> ABORT")
    {
      CanbusEvent ce;

// In order to test the transition from idle to abort, the three abort cases must be
// analyzed separately.

      SECTION("STM32 ABORTED CMD")
      {
        IgnitionBoardStatus ibs;
        ibs.stm32_abortCmd=1;
        memcpy(ce.payload, &ibs, sizeof(ibs));
        ce.len=sizeof(ibs);
        REQUIRE(testFSMTransition(*ign, ce, &IgnitionController::stateAborted));
      }

      SECTION("STM32 ABORTED TIMEOUT")
      {
        IgnitionBoardStatus ibs;
        ibs.stm32_abortTimeout=1;
        memcpy(ce.payload, &ibs, sizeof(ibs));
        ce.len=sizeof(ibs);
        REQUIRE(testFSMTransition(*ign, ce, &IgnitionController::stateAborted));

      }

      SECTION("STM32 ABORTED WRONG CODE")
      {
        IgnitionBoardStatus ibs;
        ibs.stm32_abortWrongCode=1;
        memcpy(ce.payload, &ibs, sizeof(ibs));
        ce.len=sizeof(ibs);
        REQUIRE(testFSMTransition(*ign, ce, &IgnitionController::stateAborted));
      }




    }

  /* TEST_CASE("Testing IDLE functions")
    {
        IgnitionController ign;

        SECTION("GET STATUS")
        {

        }
        */
      }
