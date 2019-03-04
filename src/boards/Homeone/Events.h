/* Copyright (c) 2018 Skyward Experimental Rocketry
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

/*
 ******************************************************************************
 *                  THIS FILE IS AUTOGENERATED. DO NOT EDIT.                  *
 ******************************************************************************
 */

// Generated from:  https://docs.google.com/spreadsheets/d/12TecOmDd7Uot-MvXkCbhDJRU48-XO6s5ChKDlr4AOvI
// Autogen date:    2018-12-13 19:38:35.317781


#ifndef SRC_SHARED_BOARDS_HOMEONE_EVENTS_H
#define SRC_SHARED_BOARDS_HOMEONE_EVENTS_H

#include <cstdint>
#include <string>

#include "events/Event.h"
#include "events/EventBroker.h"
#include "EventClasses.h"
#include "Topics.h"

using std::string;
using std::map;

namespace HomeoneBoard
{
/**
 * Definition of all events in the Homeone Board software
 * Refer to section 5.1.1 of the Software Design Document.
 */
enum Events : uint8_t
{
    EV_ADA_APOGEE_DETECTED = EV_FIRST_SIGNAL,
    EV_ADA_CALIBRATION_COMPLETE,
    EV_ADA_DPL_ALT_DETECTED,
    EV_APOGEE,
    EV_ARMED,
    EV_CUT_DROGUE,
    EV_DPL_ALTITUDE,
    EV_GS_OFFLINE,
    EV_IGN_ABORTED,
    EV_IGN_GETSTATUS,
    EV_IGN_OFFLINE,
    EV_INIT_ERROR,
    EV_INIT_OK,
    EV_LANDED,
    EV_LAUNCH,
    EV_LIFTOFF,
    EV_NC_OPEN,
    EV_NC_DETACHED,
    EV_NC_GETSTATUS,
    EV_NC_OFFLINE,
    EV_NEW_CAN_MSG,
    EV_SEND_HR_TM,
    EV_SEND_LR_TM,
    EV_SEND_POS_TM,
    EV_TC_ABORT_LAUNCH,
    EV_TC_ARM,
    EV_TC_SET_DPL_PRESSURE,
    EV_TC_BOARD_RESET,
    EV_TC_CUT_MAIN,
    EV_TC_CUT_ALL,
    EV_TC_CUT_FIRST_DROGUE,
    EV_TC_DISARM,
    EV_TC_END_MISSION,
    EV_TC_LAUNCH,
    EV_TC_MANUAL_MODE,
    EV_TC_NC_CLOSE,
    EV_TC_NC_OPEN,
    EV_TC_START_LOGGING,
    EV_TC_STOP_LOGGING,
    EV_TC_RESET_CALIBRATION,
    EV_TC_TEST_MODE,
    EV_TIMEOUT_APOGEE,
    EV_TIMEOUT_ARM,
    EV_TIMEOUT_CUTTING,
    EV_TIMEOUT_DPL_ALT,
    EV_TIMEOUT_END_MISSION,
    EV_TIMEOUT_SHADOW_MODE,
    EV_UMBILICAL_DETACHED,
    EV_TIMEOUT_ADA_CALIBRATION
};

/**
 * @brief Returns the name of the provided event
 * 
 * @param event 
 * @return string 
 */
string getEventString(uint8_t event);

}

#endif /* SRC_SHARED_BOARDS_HOMEONE_EVENTS_H */
