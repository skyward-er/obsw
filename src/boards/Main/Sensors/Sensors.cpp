/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include "Sensors.h"

#include <Main/Buses.h>
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
using namespace std;
namespace Main
{
LPS22DFData Sensors::getLPS22DFLastSample() { return LPS22DFData{}; }
LPS28DFWData Sensors::getLPS28DFW_1LastSample() { return LPS28DFWData{}; }
LPS28DFWData Sensors::getLPS28DFW_2LastSample() { return LPS28DFWData{}; }
H3LIS331DLData Sensors::getH3LIS331DLLastSample() { return H3LIS331DLData{}; }
LIS2MDLData Sensors::getLIS2MDLLastSample() { return LIS2MDLData{}; }

Sensors::Sensors(TaskScheduler* sched) : scheduler(sched) {}

bool Sensors::start()
{
    // Init all the sensors
    lps22dfInit();
    // lps28dfw_1Init();
    // lps28dfw_2Init();
    h3lis331dlInit();
    lis2mdlInit();

    // Create sensor manager with populated map and configured scheduler
    manager = new SensorManager(sensorMap, scheduler);
    return manager->start();
}

void Sensors::stop() { manager->stop(); }

bool Sensors::isStarted() { return manager->areAllSensorsInitialized(); }

void Sensors::calibrate() {}

void Sensors::lps22dfInit()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Get the correct SPI configuration
    SPIBusConfig config = LPS22DF::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Configure the device
    LPS22DF::Config sensorConfig;
    sensorConfig.avg = LPS22DF::AVG_4;
    sensorConfig.odr = LPS22DF::ODR_50;

    // Create sensor instance with configured parameters
    lps22df = new LPS22DF(modules.get<Buses>()->spi3,
                          miosix::sensors::LPS22DF::cs::getPin(), config,
                          sensorConfig);

    // Emplace the sensor inside the map
    SensorInfo info("LPS22DF", 20, bind(&Sensors::lps22dfCallback, this));
    sensorMap.emplace(make_pair(lps22df, info));
}
void Sensors::lps28dfw_1Init()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // TODO insert bus speed
    I2C i2c(modules.get<Buses>()->i2c1, miosix::interfaces::i2c1::scl::getPin(),
            miosix::interfaces::i2c1::sda::getPin());

    // Configure the sensor
    LPS28DFW::SensorConfig config{false, LPS28DFW::FullScaleRange::FS_1260,
                                  LPS28DFW::AVG_4, LPS28DFW::ODR::ODR_50,
                                  false};

    // Create sensor instance with configured parameters
    lps28dfw_1 = new LPS28DFW(i2c, config);

    // Emplace the sensor inside the map
    SensorInfo info("LPS28DFW_1", 20, bind(&Sensors::lps28dfw_1Callback, this));
    sensorMap.emplace(make_pair(lps28dfw_1, info));
}
void Sensors::lps28dfw_2Init()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // TODO insert bus speed
    I2C i2c(modules.get<Buses>()->i2c1, miosix::interfaces::i2c1::scl::getPin(),
            miosix::interfaces::i2c1::sda::getPin());

    // Configure the sensor
    LPS28DFW::SensorConfig config{true, LPS28DFW::FullScaleRange::FS_1260,
                                  LPS28DFW::AVG_4, LPS28DFW::ODR::ODR_50,
                                  false};

    // Create sensor instance with configured parameters
    lps28dfw_2 = new LPS28DFW(i2c, config);

    // Emplace the sensor inside the map
    SensorInfo info("LPS28DFW_2", 20, bind(&Sensors::lps28dfw_1Callback, this));
    sensorMap.emplace(make_pair(lps28dfw_2, info));
}
void Sensors::h3lis331dlInit()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Get the correct SPI configuration
    SPIBusConfig config = H3LIS331DL::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Create sensor instance with configured parameters
    h3lis331dl = new H3LIS331DL(
        modules.get<Buses>()->spi3, miosix::sensors::H3LIS331DL::cs::getPin(),
        config, H3LIS331DLDefs::OutputDataRate::ODR_100,
        H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE,
        H3LIS331DLDefs::FullScaleRange::FS_100);

    // Emplace the sensor inside the map
    SensorInfo info("H3LIS331DL", 10, bind(&Sensors::h3lis331dlCallback, this));
    sensorMap.emplace(make_pair(h3lis331dl, info));
}
void Sensors::lis2mdlInit() {}

void Sensors::lps22dfCallback() {}
void Sensors::lps28dfw_1Callback() {}
void Sensors::lps28dfw_2Callback() {}
void Sensors::h3lis331dlCallback()
{
    H3LIS331DLData sample = h3lis331dl->getLastSample();
    printf("%f %f %f\n", sample.accelerationX, sample.accelerationY,
           sample.accelerationZ);
}
void Sensors::lis2mdlCallback() {}

}  // namespace Main