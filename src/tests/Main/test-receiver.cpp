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

#include <Main/Configs/RadioConfigs.h>
#include <Main/Radio/Mavlink.h>
#include <miosix.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SerialTransceiver/SerialTransceiver.h>

using namespace miosix;
using namespace Boardcore;
using namespace Main;

using MavDriver = Boardcore::MavlinkDriver<RadioConfigs::RADIO_PKT_LENGTH,
                                           RadioConfigs::RADIO_OUT_QUEUE_SIZE,
                                           RadioConfigs::RADIO_MAV_MSG_LENGTH>;

void handleMavlinkMessage(MavDriver* driver, const mavlink_message_t& msg);

int main()
{
    // Pin A2, A3
    u2rx1::mode(Mode::ALTERNATE);
    u2rx1::alternateFunction(7);
    u2tx1::mode(Mode::ALTERNATE);
    u2tx1::alternateFunction(7);

    USART uart(USART2, USARTInterface::Baudrate::B115200);
    SerialTransceiver transceiver(uart);
    MavDriver mavlink(&transceiver, handleMavlinkMessage);

    uart.init();
    mavlink.start();

    while (true)
        Thread::sleep(1000);
}

void handleMavlinkMessage(MavDriver* driver, const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_ROCKET_FLIGHT_TM:
            mavlink_rocket_flight_tm_t tm;
            mavlink_msg_rocket_flight_tm_decode(&msg, &tm);

            printf("[%.2f] ", tm.timestamp / 1e6);
            printf("%d ", tm.ada_state);
            printf("%d ", tm.fmm_state);
            printf("%d ", tm.dpl_state);
            printf("%d ", tm.ab_state);
            printf("%d ", tm.nas_state);
            printf("%f ", tm.pressure_ada);
            printf("%f ", tm.pressure_digi);
            printf("%f ", tm.pressure_static);
            printf("%f ", tm.pressure_dpl);
            printf("%f ", tm.airspeed_pitot);
            printf("%f ", tm.msl_altitude);
            printf("%f ", tm.ada_vert_speed);
            printf("%f ", tm.ada_vert_accel);
            printf("%f ", tm.acc_x);
            printf("%f ", tm.acc_y);
            printf("%f ", tm.acc_z);
            printf("%f ", tm.gyro_x);
            printf("%f ", tm.gyro_y);
            printf("%f ", tm.gyro_z);
            printf("%f ", tm.mag_x);
            printf("%f ", tm.mag_y);
            printf("%f ", tm.mag_z);
            printf("%d ", tm.gps_fix);
            printf("%f ", tm.gps_lat);
            printf("%f ", tm.gps_lon);
            printf("%f ", tm.gps_alt);
            printf("%f ", tm.vbat);
            printf("%f ", tm.vsupply_5v);
            printf("%f ", tm.temperature);
            printf("%d ", tm.pin_launch);
            printf("%d ", tm.pin_nosecone);
            printf("%d ", tm.servo_sensor);
            printf("%f ", tm.ab_angle);
            printf("%f ", tm.ab_estimated_cd);
            printf("%f ", tm.nas_x);
            printf("%f ", tm.nas_y);
            printf("%f ", tm.nas_vx);
            printf("%f ", tm.nas_vy);
            printf("%f ", tm.nas_vz);
            printf("%f ", tm.nas_roll);
            printf("%f ", tm.nas_pitch);
            printf("%f ", tm.nas_yaw);
            printf("%f ", tm.nas_bias0);
            printf("%f ", tm.nas_bias1);
            printf("%f ", tm.nas_bias2);
            printf("%d\n", tm.logger_error);
            break;

        default:
            break;
    }
}
