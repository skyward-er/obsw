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
#include <sensors/SensorSampling.h>

#include <interfaces-impl/hwmapping.h>

using namespace DeathStackBoard;
using namespace miosix;

typedef MPU9250<spiMPU9250> MPU9250Type;
//typedef ADIS16405<spiADIS16405, sensors::adis16405::rst> ADIS16405Type;
typedef LM75B<i2c1> LM75BType;

AD7994Wrapper* adc_ad7994;
ADCWrapper* adc_internal;

MPU9250Type* imu_mpu9250;
//ADIS16405Type* imu_adis16405;

LM75BType* temp_lm75b_analog;
LM75BType* temp_lm75b_imu;

Piksi* piksi;

void init()
{
    i2c1::init();
    spiMPU9250::init();
    spiADIS16405::init();

    adc_ad7994        = new AD7994Wrapper(sensors::ad7994::addr);
    temp_lm75b_analog = new LM75BType(sensors::lm75b_analog::addr);
    temp_lm75b_imu    = new LM75BType(sensors::lm75b_imu::addr);
    //imu_adis16405 = new ADIS16405Type(ADIS16405Type::GYRO_FS_300);
    adc_internal  = new ADCWrapper();

    imu_mpu9250 =
        new MPU9250Type(MPU9250Type::ACC_FS_16G, MPU9250Type::GYRO_FS_2000);

    piksi = new Piksi("/dev/gps");

    Thread::sleep(1000);

    printf("\nTesting imu_mpu9250... %s\n", imu_mpu9250->init() ? "Ok" : "Failed");
    printf("Testing imu_adis16405... RIP\n"); // \n", imu_adis16405->init() ? "Ok" : "Failed");
    printf("Testing temp_lm75b_analog... %s\n", temp_lm75b_analog->init() ? "Ok" : "Failed");
    printf("Testing temp_lm75b_imu... %s\n", temp_lm75b_imu->init() ? "Ok" : "Failed");
    printf("Testing adc_ad7994... %s\n", adc_ad7994->init() ? "Ok" : "Failed");
    printf("Testing battery sensor ... %s\n", adc_internal->getBatterySensorPtr()->init() ? "Ok" : "Failed");
    printf("Testing current sensor... %s\n", adc_internal->getCurrentSensorPtr()->init() ? "Ok" : "Failed");
    printf("\n\n");
}

void update()
{
    imu_mpu9250->onSimpleUpdate();
    // imu_adis16405->onSimpleUpdate();
    temp_lm75b_analog->onSimpleUpdate();
    temp_lm75b_imu->onSimpleUpdate();
    adc_ad7994->onSimpleUpdate();
    adc_internal->getBatterySensorPtr()->onSimpleUpdate();
    adc_internal->getCurrentSensorPtr()->onSimpleUpdate();
}

void print()
{
    printf("MPU9250 Accel:  \tZ: %.3f\n",
           imu_mpu9250->accelDataPtr()->getZ());
    /* printf("ADIS Acc:       \tZ: %.3f\n",
           imu_adis16405->accelDataPtr()->getZ()); */
    printf("LM75B imu Temp:     \tT: %.3f\n", temp_lm75b_imu->getTemp());
    printf("LM75B analog Temp:  \tT: %.3f\n", temp_lm75b_analog->getTemp());
    printf("HW Pressure:    \tP: %f\n", adc_ad7994->getDataPtr()->honeywell_baro_pressure);
    printf("NXP Pressure:   \tP: %f\n", adc_ad7994->getDataPtr()->nxp_baro_pressure);    
    printf("Battery voltage:\tV: %f\n", 
        adc_internal->getBatterySensorPtr()->getBatteryDataPtr()->volt);    
    printf("Current sens 1: \tC: %f\n", 
        adc_internal->getCurrentSensorPtr()->getCurrentDataPtr()->current_1); 
    printf("Current sens 2: \tC: %f\n", 
        adc_internal->getCurrentSensorPtr()->getCurrentDataPtr()->current_2);
    printf("Pins: LP: %d, MC: %d\n", inputs::lp_dtch::value(), nosecone::nc_dtch::value());
    try
    {
        auto gps = piksi->getGpsData();
        printf("GPS Data:       \tLAT:   %f\n", gps.latitude);
        printf("                \tLON:   %f\n", gps.longitude);
        printf("                \tN_SAT: %d\n", gps.numSatellites);
        printf("                \tALT:   %f\n", gps.height);
    }
    catch (...)
    {
        printf("GPS Data:      \tNO FIX!\n");
    }
    printf("\n");
}

int main()
{
    init();

    printf("Press enter to start\n");
    (void) getchar();

    for (;;)
    {
        update();
        print();
        Thread::sleep(1000);
    }

    return 0;
}