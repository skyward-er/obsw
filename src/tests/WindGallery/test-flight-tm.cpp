/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Main/Configs/RadioConfig.h>
#include <Main/Radio/Mavlink.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SerialTransceiver/SerialTransceiver.h>
#include <scheduler/TaskScheduler.h>

using namespace miosix;
using namespace Boardcore;
using namespace Main;
using namespace RadioConfig;

using MavDriver =
    Boardcore::MavlinkDriver<RADIO_PKT_LENGTH, RADIO_OUT_QUEUE_SIZE,
                             RADIO_MAV_MSG_LENGTH>;

Transceiver* transceiver;
MavDriver* mavDriver;

void sendFlightTm()
{
    mavlink_rocket_flight_tm_t tm;

    float random = 100 * sinf(TimestampTimer::getTimestamp() / 1e6);

    tm.timestamp       = TimestampTimer::getTimestamp();
    tm.ada_state       = random;
    tm.fmm_state       = random;
    tm.dpl_state       = random;
    tm.ab_state        = random;
    tm.nas_state       = random;
    tm.pressure_ada    = random;
    tm.pressure_digi   = random;
    tm.pressure_static = random;
    tm.pressure_dpl    = random;
    tm.airspeed_pitot  = random;
    tm.msl_altitude    = random;
    tm.ada_vert_speed  = random;
    tm.ada_vert_accel  = random;
    tm.acc_x           = random;
    tm.acc_y           = random;
    tm.acc_z           = random;
    tm.gyro_x          = random;
    tm.gyro_y          = random;
    tm.gyro_z          = random;
    tm.mag_x           = random;
    tm.mag_y           = random;
    tm.mag_z           = random;
    tm.gps_fix         = random;
    tm.gps_lat         = random;
    tm.gps_lon         = random;
    tm.gps_alt         = random;
    tm.vbat            = random;
    tm.vsupply_5v      = random;
    tm.temperature     = random;
    tm.pin_launch      = random;
    tm.pin_nosecone    = random;
    tm.servo_sensor    = random;
    tm.ab_angle        = random;
    tm.ab_estimated_cd = random;
    tm.nas_x           = random;
    tm.nas_y           = random;
    tm.nas_z           = random;
    tm.nas_vx          = random;
    tm.nas_vy          = random;
    tm.nas_vz          = random;
    tm.nas_roll        = random;
    tm.nas_pitch       = random;
    tm.nas_yaw         = random;
    tm.nas_bias0       = random;
    tm.nas_bias1       = random;
    tm.nas_bias2       = random;
    tm.logger_error    = random;

    mavlink_message_t msg;
    mavlink_msg_rocket_flight_tm_encode(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg,
                                        &tm);

    mavDriver->enqueueMsg(msg);
}

int main()
{
    USART uart(USART1, Boardcore::USARTInterface::Baudrate::B115200);
    uart.init();
    transceiver = new SerialTransceiver(uart);
    mavDriver = new MavDriver(transceiver, nullptr, 0, MAV_OUT_BUFFER_MAX_AGE);

    TaskScheduler scheduler;
    scheduler.addTask(sendFlightTm, 50);

    mavDriver->start();
    scheduler.start();

    while (true)
        Thread::sleep(1000);
}
