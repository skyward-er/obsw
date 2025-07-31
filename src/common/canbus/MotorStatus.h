/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <common/CanConfig.h>
#include <drivers/canbus/CanProtocol/CanProtocol.h>
#include <logger/Logger.h>
#include <miosix.h>
#include <sensors/SensorData.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <chrono>
#include <mutex>

namespace Common
{
/**
 * @brief State and data about the motor board.
 *
 * Collects information about the motor board state and sensor data received
 * from the CAN bus.
 */
struct MotorStatus : public Boardcore::Injectable
{
    struct Data
    {
        Boardcore::DeviceStatus device;

        Boardcore::PressureData n2TankPressure;
        Boardcore::PressureData regulatorOutPressure;
        Boardcore::PressureData oxTankTopPressure;
        Boardcore::PressureData oxTankBottom0Pressure;
        Boardcore::PressureData oxTankBottom1Pressure;
        Boardcore::PressureData combustionChamberPressure;
        Boardcore::TemperatureData thermocoupleTemperature;
        Boardcore::VoltageData batteryVoltage;
        Boardcore::CurrentData actuatorsCurrent;

        bool oxVentingValveOpen   = false;
        bool nitrogenValveOpen    = false;
        bool mainValveOpen        = false;
        bool n2QuenchingValveOpen = false;
    };

    /**
     * @brief Proxy object that provides locked access to MotorStatus::Data
     */
    class LockedData
    {
    public:
        LockedData(Data& data, miosix::FastMutex& mutex)
            : data(data), lock(mutex)
        {
        }

        Data& operator*() { return data; }
        Data* operator->() { return &data; }

    private:
        Data& data;
        std::unique_lock<miosix::FastMutex> lock;
    };

    /**
     * @brief Locks motor status data and returns a proxy object to access it in
     * a thread-safe manner.
     */
    LockedData lockData() { return LockedData(data, mutex); }

    bool connected() const
    {
        return Clock::now() <=
               lastStatus.load() + Common::CanConfig::STATUS_TIMEOUT;
    }

    uint8_t getState()
    {
        std::unique_lock<miosix::FastMutex> lock(mutex);
        return data.device.state;
    }

    /**
     * @brief Handles a CAN message from the motor board.
     *
     * @return True if the message was handled, false otherwise.
     */
    void handleCanMessage(const Boardcore::Canbus::CanMessage& msg);

private:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    std::atomic<TimePoint> lastStatus = {TimePoint{}};
    Data data;

    void handleSensors(const Boardcore::Canbus::CanMessage& msg);
    void handleActuators(const Boardcore::Canbus::CanMessage& msg);

    miosix::FastMutex mutex;  ///< Mutex to protect access to the status data

    Boardcore::Logger& sdLogger = Boardcore::Logger::getInstance();
};
}  // namespace Common
