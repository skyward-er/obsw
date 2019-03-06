/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#include <mavlink.h>

#include "logger/Logger.h"
#include "Singleton.h"
#include "sensors/MPU9250/MPU9250Data.h"

namespace DeathStackBoard 
{

class LoggerProxy : public Singleton<LoggerProxy>
{
    friend class Singleton<LoggerProxy>;

public:

    LoggerProxy() : logger(Logger::instance()) {}

    template <typename T>
    inline LogResult log(const T& t)
    {
        return logger.log(t);
    }

    /**
     * Returns the last logged struct corresponding to a given telemetry
     * @req_tm          required telemetry
     * @return          packed mavlink telemetry
     */
    mavlink_message_t getTM(MavTMList req_tm)
    {
        // TODO search map
        return m;
    }

private:
    // TODO remove m
    mavlink_message_t m;

    Logger& logger;

    const std::map<uint8_t, mavlink_message_t> status_map =
{
        { MAV_HM1_TM_ID,    {0} },
        { MAV_IGN_TM_ID,    {0} },
        { MAV_HR_TM_ID,     {0} },
        { MAV_LR_TM_ID,     {0} },
        { MAV_POS_TM_ID,    {0} },
        { MAV_LOGGER_TM_ID, {0} },
        { MAV_TMTC_TM_ID,   {0} },
        { MAV_SM_TM_ID,     {0} },
        { MAV_IGN_CTRL_TM_ID, {0} },
        { MAV_DPL_CTRL_TM_ID, {0} },
        { MAV_ADA_TM_ID,    {0} },
        { MAV_CAN_TM_ID,    {0} },
        { MAV_AD7994_TM_ID, {0} },
        { MAV_ADC_TM_ID,    {0} },
        { MAV_ADIS_TM_ID,   {0} },
        { MAV_MPU_TM_ID,    {0} },
        { MAV_GPS_TM_ID,    {0} }
};

}
