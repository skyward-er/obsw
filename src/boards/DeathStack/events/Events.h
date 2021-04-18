/* Copyright (c) 2018-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alvise de' Faveri Tron
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

// Autogen date: 2021-04-17 23:09:11.522877

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "Topics.h"
#include "events/Event.h"
#include "events/EventBroker.h"

using std::string;

/**
 * Definition of all events in the Homeone Board software
 * Refer to section 5.1.1 of the Software Design Document.
 */
enum Events : uint8_t
{
    EV_CALIBRATE = EV_FIRST_SIGNAL,
    EV_CALIBRATE_SENSORS,
    EV_ARMED,
    EV_TC_START_SENSOR_LOGGING,
    EV_SM_READY,
    EV_LANDED,
    EV_TC_STOP_SENSOR_LOGGING,
    EV_RESET_SERVO,
    EV_WIGGLE_SERVO,
    EV_NC_OPEN,
    EV_CUT_DROGUE,
    EV_TEST_CUT_PRIMARY,
    EV_TEST_CUT_BACKUP,
    EV_NC_DETACHED,
    EV_NC_OPEN_TIMEOUT,
    EV_CUTTING_TIMEOUT,
    EV_LIFTOFF,
    EV_TEST_ABK,
    EV_SHADOW_MODE_TIMEOUT,
    EV_DISABLE_ABK,
    EV_APOGEE,
    EV_TEST_TIMEOUT,
    EV_CALIBRATE_NAS,
    EV_NAS_READY,
    EV_SEND_TEST_TM,
    EV_SEND_HR_TM,
    EV_SEND_LR_TM,
    EV_DISARMED,
    EV_DPL_ALTITUDE,
    EV_STATS_TIMEOUT,
    EV_INIT_ERROR,
    EV_INIT_OK,
    EV_CUT_BACKUP,
    EV_CUT_PRIMARY,
    EV_TC_ARM,
    EV_TC_BOARD_RESET,
    EV_TC_CALIBRATE_ADA,
    EV_TC_CLOSE_LOG,
    EV_TC_CUT_BACKUP,
    EV_TC_CUT_DROGUE,
    EV_TC_CUT_PRIMARY,
    EV_TC_DISARM,
    EV_TC_END_MISSION,
    EV_TC_FORCE_INIT,
    EV_TC_LAUNCH,
    EV_TC_NC_OPEN,
    EV_TC_RESET_SERVO,
    EV_TC_SET_DPL_ALTITUDE,
    EV_TC_SET_REFERENCE_ALTITUDE,
    EV_TC_SET_REFERENCE_TEMP,
    EV_TC_TEST_CUTTER_BACKUP,
    EV_TC_TEST_CUTTER_PRIMARY,
    EV_TC_TEST_MODE,
    EV_TC_WIGGLE_SERVO,
    EV_TEST_CUTTER_BACKUP,
    EV_TEST_CUTTER_PRIMARY,
    EV_TEST_MODE,
    EV_TIMEOUT_END_MISSION,
    EV_UMBILICAL_DETACHED,
    EV_CALIBRATE_ADA,
    EV_ADA_READY,
    EV_TIMEOUT_SHADOW_MODE,
    EV_ADA_APOGEE_DETECTED,
    EV_TIMEOUT_PRESS_STABILIZATION,
    EV_ADA_DPL_ALT_DETECTED,
};

const std::vector<uint8_t> EVENT_LIST{
    EV_CALIBRATE,
    EV_CALIBRATE_SENSORS,
    EV_ARMED,
    EV_TC_START_SENSOR_LOGGING,
    EV_SM_READY,
    EV_LANDED,
    EV_TC_STOP_SENSOR_LOGGING,
    EV_RESET_SERVO,
    EV_WIGGLE_SERVO,
    EV_NC_OPEN,
    EV_CUT_DROGUE,
    EV_TEST_CUT_PRIMARY,
    EV_TEST_CUT_BACKUP,
    EV_NC_DETACHED,
    EV_NC_OPEN_TIMEOUT,
    EV_CUTTING_TIMEOUT,
    EV_LIFTOFF,
    EV_TEST_ABK,
    EV_SHADOW_MODE_TIMEOUT,
    EV_DISABLE_ABK,
    EV_APOGEE,
    EV_TEST_TIMEOUT,
    EV_CALIBRATE_NAS,
    EV_NAS_READY,
    EV_SEND_TEST_TM,
    EV_SEND_HR_TM,
    EV_SEND_LR_TM,
    EV_DISARMED,
    EV_DPL_ALTITUDE,
    EV_STATS_TIMEOUT,
    EV_CALIBRATE_ADA,
    EV_ADA_READY,
    EV_TIMEOUT_SHADOW_MODE,
    EV_ADA_APOGEE_DETECTED,
    EV_TIMEOUT_PRESS_STABILIZATION,
    EV_ADA_DPL_ALT_DETECTED,
};

/**
 * @brief Returns the name of the provided event
 *
 * @param event
 * @return string
 */
string getEventString(uint8_t event);