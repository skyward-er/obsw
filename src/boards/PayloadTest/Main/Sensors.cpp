/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Configs/SensorsConfig.h>
#include "Sensors.h"

using std::bind;
//using std::function;

using namespace Boardcore;

namespace ParafoilTest
{
    /**
     * PRIVATE METHODS
     */
    void Sensors::initLIS3DSH()
    {
        //CS PIN which we use to enable the dialgue with the sensor
        GpioPin cs(GPIOE_BASE, 3);
        cs.mode(miosix::Mode::OUTPUT);

        accel_sensor = new LIS3DSH(spiInterface,
                                   cs,
                                   ACCEL_LIS3DSH_ORD,
                                   ACCEL_LIS3DSH_BDU,
                                   ACCEL_LIS3DSH_FULL_SCALE);

        accel_sensor -> init();

        //Create the sensor description binding it to the sample period,
        //its callback function and if it is DMA access and enabled.
        SensorInfo info("LIS3DSH", SAMPLE_PERIOD_ACCEL_LIS3DSH,
                        bind(&Sensors::LIS3DSHCallback, this), false, true);

        //Adding the sensor to the sensors map
        sensors_map.emplace(std::make_pair(accel_sensor, info));
    }   

    void Sensors::LIS3DSHCallback()
    {
        LoggerService::getInstance() -> log(accel_sensor -> getLastSample());
    }

    /**
     * PUBLIC METHODS
     */
    Sensors::Sensors(SPIBusInterface& spi, TaskScheduler* scheduler)
        : spiInterface(spi)
    {
        //Sensor init
        initLIS3DSH();

        //Sensor manager instance
        //At this point sensors_map contains all the initialized sensors
        manager = new SensorManager(scheduler, sensors_map);
    }

    Sensors::~Sensors()
    {
        //Delete the sensors
        delete accel_sensor;

        //Sensor manager stop and delete
        manager -> stop();
        delete manager;
    }

    bool Sensors::start()
    {
        return manager -> start();
    }

    void Sensors::calibrate()
    {

    }
}