/**
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

#pragma once

// The StackLogger is an active object!
#include <diagnostic/StackLogger.h>

namespace DeathStackBoard
{

// define thread ids to be used by the StackLogger
enum LynxThreadIds : uint8_t
{
    THID_ENTRYPOINT = THID_FIRST_AVAILABLE_ID,
    THID_DPL_FSM,
    THID_FMM_FSM,
    THID_TMTC_FSM,
    THID_STATS_FSM,
    THID_ADA_FSM,
    THID_ABK_FSM,
    THID_NAS_FSM,
    THID_TASK_SCHEDULER,
    THID_CAN_BUS
};
}