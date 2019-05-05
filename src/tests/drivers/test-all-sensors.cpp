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

#include "DeathStack/configs/SensorManagerConfig.h"

#include "DeathStack/SensorManager/Sensors/AD7994Wrapper.h"
#include "DeathStack/SensorManager/Sensors/ADCWrapper.h"

#include <drivers/piksi/piksi.h>
#include <sensors/ADIS16405/ADIS16405.h>
#include <sensors/LM75B.h>
#include <sensors/MPU9250/MPU9250.h>

#include <interfaces-impl/hwmapping.h>

using namespace DeathStackBoard;
using namespace miosix;

typedef MPU9250<spiMPU9250> MPU9250Type;
typedef ADIS16405<spiADIS16405> ADIS16405Type;
typedef LM75B<i2c1> LM75BType;

AD7994Wrapper* adc_ad7994;
ADCWrapper* adc_internal;

MPU9250Type* imu_mpu9250;
ADIS16405Type* imu_adis16405;

LM75BType* temp_lm75b;

Piksi* piksi;

void init()
{
    adc_ad7994 = new AD7994Wrapper(sensors::ad7994::addr);
    temp_lm75b = new LM75BType(sensors::lm75b_analog::addr);

    imu_mpu9250 =
        new MPU9250Type(0, 0);  // TODO: Update with correct parameters

    imu_adis16405 = new ADIS16405Type();
    adc_internal  = new ADCWrapper();

    piksi = new Piksi("/dev/gps");

    // Some sensors dont have init or self tests

    // Initialization
    if (!imu_mpu9250->init())
    {
        printf("MPU9250 Init Failed!\n");
    }

    if (!imu_adis16405->init())
    {
        printf("ADIS Init Failed!\n");
    }
    if (!temp_lm75b->init())
    {
        printf("LM75B Init Failed!\n");
    }

    if (!adc_ad7994->init())
    {
        printf("AD7994 Init Failed!\n");
    }
    if (!adc_internal->getBatterySensorPtr()->init())
    {
        printf("Battery voltage sensor Init Failed!\n");
    }
    if (!adc_internal->getCurrentSensorPtr()->init())
    {
        printf("Current sensor Init Failed!\n");
    }
}

void update()
{
    imu_mpu9250->onSimpleUpdate();
    imu_adis16405->onSimpleUpdate();
    temp_lm75b->onSimpleUpdate();
    adc_ad7994->onSimpleUpdate();
    adc_internal->getBatterySensorPtr()->onSimpleUpdate();
    adc_internal->getCurrentSensorPtr()->onSimpleUpdate();
}

void print()
{
    printf("MPU9255 Compass:\tX: %.3f\n",
           imu_mpu9250->compassDataPtr()->getX());
    printf("ADIS Acc:       \tZ: %.3f\n",
           imu_adis16405->accelDataPtr()->getZ());
    printf("LM75B Temp:     \tT: %.3f\n", temp_lm75b->getTemp());
    printf("HW Pressure:    \tP: %d\n", adc_ad7994->getDataPtr()->honeywell_baro_volt);
    printf("NXP Pressure:   \tP: %d\n", adc_ad7994->getDataPtr()->nxp_baro_volt);    
    printf("Battery tension:\tV: %d\n", adc_internal->getBatterySensorPtr()->getBatteryDataPtr()->battery_voltage_value);    
    printf("Current sens:   \tC: %d\n", adc_internal->getCurrentSensorPtr()->getCurrentDataPtr()->current_1_value);    
}

int main()
{
    init();
    printf("Initialization completed\n");

    for (;;)
    {
        update();
        print();
        Thread::sleep(2000);
    }
}