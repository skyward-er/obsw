/* Copyright (c) 2019 Skyward Experimental Rocketry
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

#include <drivers/piksi/piksi.h>
#include <interfaces-impl/hwmapping.h>
#include <sensors/ADIS16405/ADIS16405.h>
#include <sensors/LM75B.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/SensorSampling.h>

#include "SensorManager/Sensors/AD7994Wrapper.h"
#include "SensorManager/Sensors/ADCWrapper.h"
#include "configs/SensorManagerConfig.h"
#include "sensors/MS580301BA07/MS580301BA07.h"

using namespace DeathStackBoard;
using namespace miosix;

typedef LM75B<i2c1> LM75BType;
typedef MS580301BA07<spiMS5803> MS580301BA07Type;

AD7994Wrapper* adc_ad7994;

LM75BType* temp_lm75b_analog;
LM75BType* temp_lm75b_imu;
MS580301BA07Type* pressure_ms5803;

Piksi* piksi;

void init()
{
    i2c1::init();

    adc_ad7994        = new AD7994Wrapper(sensors::ad7994::addr, AD7994_V_REF);
    temp_lm75b_analog = new LM75BType(sensors::lm75b_analog::addr);
    temp_lm75b_imu    = new LM75BType(sensors::lm75b_imu::addr);

    pressure_ms5803 = new MS580301BA07Type();

    Thread::sleep(1000);

    printf("Testing temp_lm75b_analog... %s\n",
           temp_lm75b_analog->init() ? "Ok" : "Failed");
    printf("Testing temp_lm75b_imu... %s\n",
           temp_lm75b_imu->init() ? "Ok" : "Failed");
    printf("Testing adc_ad7994... %s\n", adc_ad7994->init() ? "Ok" : "Failed");
    printf("Testing digital pressure sensor... %s\n",
           pressure_ms5803->init() ? "Ok" : "Failed");

    printf(
        "imu_temp,analog_temp,digital_temp,hw_pressure,nxp_pressure,digital_"
        "pressure\n");
    printf("\n\n");
}

void update()
{
    temp_lm75b_analog->onSimpleUpdate();
    temp_lm75b_imu->onSimpleUpdate();
    adc_ad7994->onSimpleUpdate();
    pressure_ms5803->onSimpleUpdate();
}

void print()
{
    printf("%d,%f,%f,%f,%f,%f,%f\n", (int)miosix::getTick(),
           temp_lm75b_imu->getTemp(), temp_lm75b_analog->getTemp(),
           pressure_ms5803->getData().temp,
           adc_ad7994->getDataPtr()->honeywell_baro_pressure,
           adc_ad7994->getDataPtr()->nxp_baro_pressure,
           pressure_ms5803->getData().pressure);
}

int main()
{
    init();

    printf("Press enter to start\n");
    (void)getchar();

    for (;;)
    {
        update();
        print();
        Thread::sleep(500);
    }

    return 0;
}