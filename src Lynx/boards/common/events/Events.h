/* Copyright (c) 2018-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alvise de'Faveri Tron
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

/*
 ******************************************************************************
 *                  THIS FILE IS AUTOGENERATED. DO NOT EDIT.                  *
 ******************************************************************************
 */

// Autogen date: 2021-09-08 23:46:23.104837

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "Topics.h"
#include "events/Event.h"
#include "events/EventBroker.h"

/**
 * Definition of all events in the Board software.
 * Refer to the Software Design Document.
 */
enum Events : uint8_t
{
    EV_ADA_APOGEE_DETECTED = Boardcore::EV_FIRST_SIGNAL,
    EV_ADA_DISABLE_ABK,
    EV_ADA_DPL_ALT_DETECTED,
    EV_ADA_READY,
    EV_APOGEE,
    EV_ARMED,
    EV_CALIBRATE,
    EV_CALIBRATE_ADA,
    EV_CALIBRATE_NAS,
    EV_CALIBRATION_OK,
    EV_CUTTING_TIMEOUT,
    EV_CUT_DROGUE,
    EV_DISABLE_ABK,
    EV_DISARMED,
    EV_DPL_ALTITUDE,
    EV_INIT_ERROR,
    EV_INIT_OK,
    EV_LANDED,
    EV_LIFTOFF,
    EV_NAS_READY,
    EV_NC_DETACHED,
    EV_NC_OPEN,
    EV_NC_OPEN_TIMEOUT,
    EV_RESET_SERVO,
    EV_SEND_HR_TM,
    EV_SEND_HR_TM_OVER_SERIAL,
    EV_SEND_LR_TM,
    EV_SEND_SENS_TM,
    EV_SEND_TEST_TM,
    EV_SENSORS_READY,
    EV_SHADOW_MODE_TIMEOUT,
    EV_STATS_TIMEOUT,
    EV_TC_ABK_DISABLE,
    EV_TC_ABK_RESET_SERVO,
    EV_TC_ABK_WIGGLE_SERVO,
    EV_TC_ARM,
    EV_TC_CALIBRATE_ALGOS,
    EV_TC_CALIBRATE_SENSORS,
    EV_TC_CLOSE_LOG,
    EV_TC_CUT_DROGUE,
    EV_TC_DISARM,
    EV_TC_DPL_RESET_SERVO,
    EV_TC_DPL_WIGGLE_SERVO,
    EV_TC_END_MISSION,
    EV_TC_FORCE_INIT,
    EV_TC_LAUNCH,
    EV_TC_NC_OPEN,
    EV_TC_RADIO_TM,
    EV_TC_RESET_BOARD,
    EV_TC_SERIAL_TM,
    EV_TC_START_SENSOR_TM,
    EV_TC_START_LOG,
    EV_TC_STOP_SENSOR_TM,
    EV_TC_TEST_ABK,
    EV_TC_TEST_MODE,
    EV_TC_EXIT_TEST_MODE,
    EV_TEST_ABK,
    EV_TEST_TIMEOUT,
    EV_TIMEOUT_END_MISSION,
    EV_TIMEOUT_PRESS_STABILIZATION,
    EV_TIMEOUT_SHADOW_MODE,
    EV_UMBILICAL_DETACHED,
    EV_WIGGLE_SERVO,
};

const std::vector<uint8_t> EVENT_LIST{
    EV_ADA_APOGEE_DETECTED,
    EV_ADA_DISABLE_ABK,
    EV_ADA_DPL_ALT_DETECTED,
    EV_ADA_READY,
    EV_APOGEE,
    EV_ARMED,
    EV_CALIBRATE,
    EV_CALIBRATE_ADA,
    EV_CALIBRATE_NAS,
    EV_CALIBRATION_OK,
    EV_CUTTING_TIMEOUT,
    EV_CUT_DROGUE,
    EV_DISABLE_ABK,
    EV_DISARMED,
    EV_DPL_ALTITUDE,
    EV_INIT_ERROR,
    EV_INIT_OK,
    EV_LANDED,
    EV_LIFTOFF,
    EV_NAS_READY,
    EV_NC_DETACHED,
    EV_NC_OPEN,
    EV_NC_OPEN_TIMEOUT,
    EV_RESET_SERVO,
    EV_SEND_HR_TM,
    EV_SEND_HR_TM_OVER_SERIAL,
    EV_SEND_LR_TM,
    EV_SEND_SENS_TM,
    EV_SEND_TEST_TM,
    EV_SENSORS_READY,
    EV_SHADOW_MODE_TIMEOUT,
    EV_STATS_TIMEOUT,
    EV_TC_ABK_DISABLE,
    EV_TC_ABK_RESET_SERVO,
    EV_TC_ABK_WIGGLE_SERVO,
    EV_TC_ARM,
    EV_TC_CALIBRATE_ALGOS,
    EV_TC_CALIBRATE_SENSORS,
    EV_TC_CLOSE_LOG,
    EV_TC_CUT_DROGUE,
    EV_TC_DISARM,
    EV_TC_DPL_RESET_SERVO,
    EV_TC_DPL_WIGGLE_SERVO,
    EV_TC_END_MISSION,
    EV_TC_FORCE_INIT,
    EV_TC_LAUNCH,
    EV_TC_NC_OPEN,
    EV_TC_RADIO_TM,
    EV_TC_RESET_BOARD,
    EV_TC_SERIAL_TM,
    EV_TC_START_SENSOR_TM,
    EV_TC_START_LOG,
    EV_TC_STOP_SENSOR_TM,
    EV_TC_TEST_ABK,
    EV_TC_TEST_MODE,
    EV_TC_EXIT_TEST_MODE,
    EV_TEST_ABK,
    EV_TEST_TIMEOUT,
    EV_TIMEOUT_END_MISSION,
    EV_TIMEOUT_PRESS_STABILIZATION,
    EV_TIMEOUT_SHADOW_MODE,
    EV_UMBILICAL_DETACHED,
    EV_WIGGLE_SERVO,
};

/**
 * @brief Returns the name of the provided event
 *
 * @param event
 * @return string
 */
std::string getEventString(uint8_t event);
