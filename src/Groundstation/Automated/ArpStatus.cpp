/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Tommaso Domenicali
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

#include "Groundstation/Automated/ArpStatus.h"

#include <Groundstation/Common/Config/GeneralConfig.h>
#include <Groundstation/Common/Config/RadioConfig.h>
#include <common/MavlinkHydra.h>
#include <drivers/timer/TimestampTimer.h>
#include <logger/Logger.h>

namespace Antennas
{

void ArpStatus::sendArpTm()
{
    Actuators* actuators                  = getModule<Actuators>();
    SMA* sma                              = getModule<SMA>();
    Boardcore::AntennaAngles targetAngles = sma->getTargetAngles();
    auto vn300 = getModule<Sensors>()->getVN300LastSample();

    mavlink_arp_tm_t tm = {};

    // Common GS fields.
    tm.timestamp       = Boardcore::TimestampTimer::getTimestamp();
    tm.log_number      = Boardcore::Logger::getInstance().getCurrentLogNumber();
    tm.battery_voltage = -420.0;  /// TODO: measured?

    // ARP-specific fields
    tm.yaw          = vn300.yaw;
    tm.pitch        = vn300.pitch;
    tm.roll         = vn300.roll;
    tm.stepperX_pos = actuators->getCurrentDegPosition(StepperList::STEPPER_X);
    tm.stepperX_delta = actuators->getDeltaAngleDeg(StepperList::STEPPER_X);
    tm.stepperX_speed = actuators->getSpeed(StepperList::STEPPER_X);
    tm.stepperY_pos = actuators->getCurrentDegPosition(StepperList::STEPPER_Y);
    tm.stepperY_delta = actuators->getDeltaAngleDeg(StepperList::STEPPER_Y);
    tm.stepperY_speed = actuators->getSpeed(StepperList::STEPPER_Y);
    tm.gps_latitude   = vn300.latitude;
    tm.gps_longitude  = vn300.longitude;
    tm.gps_height     = vn300.altitude;
    tm.gps_fix        = vn300.gpsFix;
    tm.target_yaw     = targetAngles.yaw;
    tm.target_pitch   = targetAngles.pitch;
    tm.state          = static_cast<uint8_t>(sma->getStatus());

    mavlink_message_t msg;
    mavlink_msg_arp_tm_encode(Groundstation::ARP_SYSTEM_ID,
                              Groundstation::MAV_DEFAULT_REQUEST_ID, &msg, &tm);

    getModule<Groundstation::HubBase>()->dispatchIncomingMsg(msg);
}

void ArpStatus::run()
{
    while (!shouldStop())
    {
        sendArpTm();
        miosix::Thread::sleep(Groundstation::RADIO_STATUS_PERIOD);
    }
}

}  // namespace Antennas
