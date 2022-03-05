/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <ActiveObject.h>
#include <miosix.h>

#include <cstdio>

#define _USE_MATH_DEFINES
#include <cmath>

#include "Radio/Mavlink.h"

using namespace miosix;

FastMutex mutex;

/**
 * @brief Generate a sinusoidal signal
 *
 * @param offs Y axis offset
 * @param amp Amplittude of the wave
 * @param freq Frequency [Hz]
 * @param phase Phase [deg]
 * @return float
 */
float sine(float offs, float amp, float freq, float phase)
{
    float t = getTick() / 1000.0f;

    return offs + amp * sin(2 * M_PI * freq * t + phase * M_PI / 180.0f);
}

void packSensorsTM(mavlink_message_t* out)
{
    mavlink_sensors_tm_t tm;
    tm.timestamp    = getTick();
    tm.bmx160_acc_x = sine(0.2, 2, 0.1, 45);
    tm.bmx160_acc_y = sine(-0.2, 1.7, 0.05, 123);
    tm.bmx160_acc_z = sine(9.81, 3.3, 0.11, 0);

    tm.bmx160_gyro_x = sine(0.2, 2, 0.1, 0);
    tm.bmx160_gyro_y = sine(-0.2, 1.7, 0.17, 66);
    tm.bmx160_gyro_z = sine(0.1, 3.3, 0.4, 145);

    tm.bmx160_mag_x = sine(30, 7, 0.1, 87);
    tm.bmx160_mag_y = sine(20, 3, 0.17, 33);
    tm.bmx160_mag_z = sine(-25, 5, 0.4, 117);

    tm.bmx160_temp  = sine(25, 5, 0.1, 117);
    tm.ms5803_temp  = sine(27, 3, 0.04, 12);
    tm.lis3mdl_temp = sine(21, 4, 0.2, 85);

    tm.ms5803_press = sine(99000, 500, 0.06, 33);
    tm.dpl_press    = sine(101000, 1500, 0.1, 67);
    tm.static_press = sine(95000, 1500, 0.2, 200);
    tm.pitot_press  = sine(0, 700, 0.4, 176);

    tm.lis3mdl_mag_x = sine(30, 7, 0.1, 11);
    tm.lis3mdl_mag_y = sine(20, 3, 0.17, 85);
    tm.lis3mdl_mag_z = sine(-25, 5, 0.4, 222);

    tm.gps_lat = sine(45, 0.001, 0.1, 123);
    tm.gps_lon = sine(8, 0.0012, 0.08, 1);
    tm.gps_alt = sine(1234, 40, 0.1, 0);
    tm.gps_fix = sine(0.5, 1, 0.03, 0) > 0;

    tm.c_sense_1 = sine(2, 0.3, 1, 0);
    tm.c_sense_2 = sine(1.3, 0.4, 2, 77);

    tm.vbat    = sine(11, 1, 0.4, 0);
    tm.vbat_5v = sine(11, 1, 0.4, 0);

    mavlink_msg_sensors_tm_encode(0, 0, out, &tm);
}

void writeMessage(mavlink_message_t& msg)
{
    uint8_t buf[256];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    {
        Lock<FastMutex> l(mutex);
        fwrite(buf, sizeof(uint8_t), len, stdout);
        fflush(stdout);
    }
}

void handleMessage(mavlink_message_t& const msg)
{
    mavlink_message_t ack_msg;
    mavlink_ack_tm_t ack;
    ack.seq_ack    = msg.seq;
    ack.recv_msgid = msg.msgid;
    mavlink_msg_ack_tm_encode(0, 0, &ack_msg, &ack);

    writeMessage(ack_msg);
}

class SerialReceiver : public ActiveObject
{

protected:
    void run() override
    {
        mavlink_status_t status;
        while (!shouldStop())
        {
            uint8_t byte;
            fread(&byte, sizeof(uint8_t), 1, stdin);
            uint8_t res = mavlink_parse_char(0, byte, &parsing, &status);
            if (res == 1)
            {
                handleMessage(parsing);
            }
        }
    }

private:
    mavlink_message_t parsing;
};

int main()
{
    mavlink_message_t buf;

    SerialReceiver receiver;
    receiver.start();

    for (;;)
    {
        auto t = getTick();
        packSensorsTM(&buf);
        writeMessage(buf);
        Thread::sleepUntil(t + 100);
    }
}
