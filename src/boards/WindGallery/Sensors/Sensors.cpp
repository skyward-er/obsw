/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

#include <WindGallery/Buses.h>
#include <WindGallery/Configs/SensorsConfig.h>
#include <WindGallery/Configs/config.h>
#include <drivers/timer/TimestampTimer.h>

using namespace std;
using namespace Boardcore;
using namespace WindGallery::SensorsConfigs;

namespace WindGallery
{

bool Sensors::start() { return sensorManager->start(); }

Sensors::Sensors()
{
    // Initialize all the sensors
    ads1118Init();
    pitotPressureInit();

    // Create the sensor manager
    sensorManager = new SensorManager(sensorsMap);
}

void Sensors::ads1118Init()
{
    SPIBusConfig spiConfig = ADS1118::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ADS1118::ADS1118Config config = ADS1118::ADS1118_DEFAULT_CONFIG;
    config.bits.mode              = ADS1118::ADS1118Mode::CONTINUOUS_CONV_MODE;

    ads1118 =
        new ADS1118(Buses::getInstance().spi1,
                    miosix::sensors::ads1118::cs::getPin(), config, spiConfig);

    ads1118->enableInput(ADC_CH_PITOT_PORT, ADC_DR_PITOT_PORT,
                         ADC_PGA_PITOT_PORT);

    SensorInfo info("ADS1118", SAMPLE_PERIOD_ADC_ADS1118,
                    [&]()
                    { Logger::getInstance().log(ads1118->getLastSample()); });

    sensorsMap.emplace(std::make_pair(ads1118, info));

    LOG_INFO(logger, "ADS1118 setup done!");
}

void Sensors::pitotPressureInit()
{
    function<ADCData()> getVoltage(
        bind(&ADS1118::getVoltage, ads1118, ADC_CH_PITOT_PORT));
    pitotPressure = new SSCDRRN015PDA(getVoltage, ADC_REFERENCE_VOLTAGE);

    SensorInfo info("PITOT", SAMPLE_PERIOD_ADC_ADS1118,
                    bind(&Sensors::pitotPressureCallback, this));

    sensorsMap.emplace(std::make_pair(pitotPressure, info));

    LOG_INFO(logger, "Pitot pressure sensor setup done!");
}

void Sensors::pitotPressureCallback()
{
    SSCDRRN015PDAData pressureData = pitotPressure->getLastSample();
    Logger::getInstance().log(pressureData);

    // in the reference code used, the first pressure parameter was misured by a
    // different digital pressure sensor (MS5803). Here I used the value that
    // has just been sampled
    float rel_density = Aeroutils::relDensity(
        pressureData.pressure, DEFAULT_REFERENCE_PRESSURE,
        DEFAULT_REFERENCE_ALTITUDE, DEFAULT_REFERENCE_TEMPERATURE);

    if (rel_density != 0.0f)
    {
        float airspeed = sqrtf(2 * fabs(pressureData.pressure) / rel_density);

        aSpeedData = AirSpeedPitot{TimestampTimer::getTimestamp(), airspeed};

        Logger::getInstance().log(aSpeedData);
    }
}

}  // namespace WindGallery
