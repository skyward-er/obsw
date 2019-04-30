/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include <events/EventBroker.h>
#include <miosix.h>
#include <utils/testutils/TestHelper.h>
#include <utils/catch.hpp>

// We need access to the handleEvent(...) function in state machines in order to
// test them synchronously
#define protected public

#include "DeathStack/FlightModeManager/FlightModeManager.h"

using namespace DeathStackBoard;

/**
 * @brief Ensure cleanup in every test using RAII
 */
class FMMTestFixture
{
public:
    // This is called at the beginning of each test / section
    FMMTestFixture()
    {
        fmm.start();
        sEventBroker->start();
    }

    // This is called at the end of each test / section
    ~FMMTestFixture()
    {
        sEventBroker->clearDelayedEvents();
        fmm.stop();
    }

protected:
    FlightModeManager fmm{};
};

TEST_CASE_METHOD(FMMTestFixture, "Test starting state")
{
    REQUIRE(fmm.testState(&FlightModeManager::state_startup));
}