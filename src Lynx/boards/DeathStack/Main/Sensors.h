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

#pragma once

#include <Main/SensorsData.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/adc/InternalADC.h>
#include <drivers/spi/SPIBusInterface.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/ADS1118/ADS1118.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/BMX160/BMX160WithCorrection.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/MS5803/MS5803.h>
#include <sensors/SensorData.h>
#include <sensors/SensorManager.h>
#include <sensors/UbloxGPS/UbloxGPS.h>
#include <sensors/analog/battery/BatteryVoltageSensor.h>
#include <sensors/analog/current/CurrentSensor.h>
#include <sensors/analog/pressure/MPXHZ6130A/MPXHZ6130A.h>
#include <sensors/analog/pressure/honeywell/SSCDANN030PAA.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDA.h>

#include <map>

#ifdef HARDWARE_IN_THE_LOOP
#include <HIL.h>
#include <HIL_sensors/HILSensors.h>
#elif defined(USE_MOCK_SENSORS)
#include <mocksensors/MockSensors.h>
#endif

namespace DeathStackBoard
{

/**
 * @brief Initializes all the sensors on the death stack
 *
 */
class Sensors
{
public:
    Boardcore::SensorManager* sensor_manager = nullptr;

    Boardcore::InternalADC* internal_adc             = nullptr;
    Boardcore::BatteryVoltageSensor* battery_voltage = nullptr;

    Boardcore::MS5803* press_digital = nullptr;

    Boardcore::ADS1118* adc_ads1118          = nullptr;
    Boardcore::SSCDANN030PAA* press_dpl_vane = nullptr;
    Boardcore::MPXHZ6130A* press_static_port = nullptr;
    Boardcore::SSCDRRN015PDA* press_pitot    = nullptr;

    Boardcore::BMX160* imu_bmx160                               = nullptr;
    Boardcore::BMX160WithCorrection* imu_bmx160_with_correction = nullptr;
    Boardcore::LIS3MDL* mag_lis3mdl                             = nullptr;
    Boardcore::UbloxGPS* gps_ublox                              = nullptr;

#ifdef HARDWARE_IN_THE_LOOP
    HILImu* hil_imu        = nullptr;
    HILBarometer* hil_baro = nullptr;
    HILGps* hil_gps        = nullptr;
#elif defined(USE_MOCK_SENSORS)
    MockIMU* mock_imu             = nullptr;
    MockPressureSensor* mock_baro = nullptr;
    MockGPS* mock_gps             = nullptr;
#endif

    Sensors(Boardcore::SPIBusInterface& spi1_bus,
            Boardcore::TaskScheduler* scheduler);

    ~Sensors();

    bool start();

    void calibrate();

#ifdef USE_MOCK_SENSORS
    void signalLiftoff();
#endif

private:
    void internalAdcInit();
    void internalAdcCallback();

    void batteryVoltageInit();
    void batteryVoltageCallback();

    void pressDigiInit();
    void pressDigiCallback();

    void ADS1118Init();
    void ADS1118Callback();

    void pressPitotInit();
    void pressPitotCallback();

    void pressDPLVaneInit();
    void pressDPLVaneCallback();

    void pressStaticInit();
    void pressStaticCallback();

    void imuBMXInit();
    void imuBMXCallback();

    void imuBMXWithCorrectionInit();
    void imuBMXWithCorrectionCallback();

    void magLISinit();
    void magLISCallback();

    void gpsUbloxInit();
    void gpsUbloxCallback();

#ifdef HARDWARE_IN_THE_LOOP
    void hilBarometerInit();
    void hilImuInit();
    void hilGpsInit();
    void hilIMUCallback();
    void hilBaroCallback();
    void hilGPSCallback();
#elif defined(USE_MOCK_SENSORS)
    void mockSensorsInit();
    void mockBaroCallback();
    void mockImuCallback();
    void mockGpsCallback();
#endif

    void updateSensorsStatus();

    Boardcore::SPIBusInterface& spi1_bus;

    Boardcore::SensorManager::SensorMap_t sensors_map;

    SensorsStatus status;

    Boardcore::PrintLogger log = Boardcore::Logging::getLogger("sensors");

    unsigned int battery_critical_counter = 0;
};

}  // namespace DeathStackBoard
