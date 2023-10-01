/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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

#include <sensors/ADS131M04/ADS131M04.h>
#include <sensors/HX711/HX711.h>
#include <sensors/MAX31855/MAX31855.h>
#include <sensors/SensorManager.h>
#include <sensors/analog/BatteryVoltageSensorData.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace RIG
{
class Sensors : public Boardcore::Module
{
public:
    /**
     * @brief Construct a new Sensors object
     *
     * @param sched The scheduler to be used from the board scheduler
     * @note The scheduler determines the priority
     */
    Sensors(Boardcore::TaskScheduler* sched);

    [[nodiscard]] bool start();

    /**
     * @brief Stops the sensors manager
     * @warning Stops the passed scheduler
     */
    void stop();

    /**
     * @brief Returns if the module is correctly started
     */
    bool isStarted();

    /**
     * @brief Calibrates the load cells with an offset
     */
    void calibrate();

    // Getters methods, they pause the kernel
    Boardcore::ADS131M04Data getADC1LastSample();
    Boardcore::ADS131M04Data getADC2LastSample();
    Boardcore::HX711Data getTankWeight();
    Boardcore::HX711Data getVesselWeight();
    Boardcore::TemperatureData getThermocoupleLastSample();

    // Processed getters
    Boardcore::PressureData getFillingPress();
    Boardcore::PressureData getTankTopPress();
    Boardcore::PressureData getTankBottomPress();
    Boardcore::PressureData getVesselPress();
    Boardcore::BatteryVoltageSensorData getBatteryVoltage();
    Boardcore::CurrentData getCurrent();
    Boardcore::PressureData getCCPress();

    // CAN sensors setters
    void setCCPressure(Boardcore::PressureData data);
    void setBottomTankPressure(Boardcore::PressureData data);
    void setTopTankPressure(Boardcore::PressureData data);
    void setTankTemperature(Boardcore::TemperatureData data);
    void setMotorCurrent(Boardcore::CurrentData data);
    void setMainCurrent(Boardcore::CurrentData data);
    void setPayloadCurrent(Boardcore::CurrentData data);
    void setMotorVoltage(Boardcore::BatteryVoltageSensorData data);

    // Can sensors getters
    Boardcore::CurrentData getMotorCurrent();
    Boardcore::CurrentData getMainCurrent();
    Boardcore::CurrentData getPayloadCurrent();
    Boardcore::BatteryVoltageSensorData getMotorBatteryVoltage();

private:
    // Init and callback methods
    void adc1Init();
    void adc1Callback();

    void adc2Init();
    void adc2Callback();

    void loadCell1Init();
    void loadCell1Callback();

    void loadCell2Init();
    void loadCell2Callback();

    void thermocoupleInit();
    void thermocoupleCallback();

    // SD logger
    Boardcore::Logger& SDlogger = Boardcore::Logger::getInstance();

    // Sensors
    Boardcore::ADS131M04* adc1 = nullptr;
    Boardcore::ADS131M04* adc2 = nullptr;

    Boardcore::HX711* loadCell1 = nullptr;
    Boardcore::HX711* loadCell2 = nullptr;

    Boardcore::MAX31855* thermocouple = nullptr;

    // Manager components
    Boardcore::SensorManager* manager = nullptr;
    Boardcore::SensorManager::SensorMap_t sensorMap;
    Boardcore::TaskScheduler* scheduler = nullptr;

    // CAN Sensors flag. By default the RIG outputs the sensors from the on
    // board ADC, but as soon as a CAN sensor arrives, the sensors are switched
    // from the CAN
    bool canSensorFlag = false;

    // CAN fake sensors data
    Boardcore::PressureData canCCPressure;
    Boardcore::PressureData canBottomTankPressure;
    Boardcore::PressureData canTopTankPressure;
    Boardcore::TemperatureData canTankTemperature;
    Boardcore::CurrentData canMotorCurrent;
    Boardcore::CurrentData canMainCurrent;
    Boardcore::CurrentData canPayloadCurrent;
    Boardcore::BatteryVoltageSensorData canMotorVoltage;

    // Offsets (thread safe)
    std::atomic<float> offsetLoadCell1{0.0};
    std::atomic<float> offsetLoadCell2{0.0};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Sensors");
};
}  // namespace RIG