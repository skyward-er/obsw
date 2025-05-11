/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Niccolò Betto, Davide Basso
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

#include <Parafoil/Configs/RadioConfig.h>
#include <common/MavlinkOrion.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SX1278/SX1278Fsk.h>
#include <radio/SerialTransceiver/SerialTransceiver.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Parafoil
{
using MavDriver =
    Boardcore::MavlinkDriver<Boardcore::SX1278Fsk::MTU,
                             Config::Radio::MavlinkDriver::PKT_QUEUE_SIZE,
                             Config::Radio::MavlinkDriver::MSG_LENGTH>;

class BoardScheduler;
class Sensors;
class Buses;
class FlightModeManager;
class Actuators;
class NASController;
class WingController;
class LandingFlare;
class PinHandler;
class FlightStatsRecorder;
class WindEstimation;

class Radio : public Boardcore::InjectableWithDeps<
                  BoardScheduler, Sensors, Buses, FlightModeManager, Actuators,
                  NASController, WingController, LandingFlare, PinHandler,
                  FlightStatsRecorder, WindEstimation>
{
public:
    /**
     * @brief Unsets the static instance for handling radio interrupts, if the
     * current one was set by this radio instance.
     */
    ~Radio();

    /**
     * @brief Initializes the radio and Mavlink driver, and sets the static
     * instance for handling radio interrupts.
     */
    [[nodiscard]] bool start();

    bool isStarted();

private:
    struct MavlinkBackend
    {
        Radio& parent;  // Reference to the parent Radio instance

        std::unique_ptr<MavDriver> driver;

        std::array<mavlink_message_t, Config::Radio::MESSAGE_QUEUE_SIZE> queue;
        size_t index = 0;
        miosix::FastMutex mutex;

        void handleMessage(const mavlink_message_t& msg);
        void handleCommand(const mavlink_message_t& msg);

        void enqueueAck(const mavlink_message_t& msg);
        void enqueueNack(const mavlink_message_t& msg);
        void enqueueWack(const mavlink_message_t& msg);

        bool enqueueSystemTm(SystemTMList tmId);
        bool enqueueSensorsTm(SensorsTMList sensorId);

        /**
         * @brief Enqueues a message in the message queue.
         */
        void enqueueMessage(const mavlink_message_t& msg);

        /**
         * @brief Flushes the message queue to the driver.
         */
        void flushQueue();

        /**
         * @brief Logs the status of MavlinkDriver and the transceiver
         */
        void logStatus();
    };

    void initMavlinkOverSerial();

    void handleRadioMessage(const mavlink_message_t& msg);
    void handleSerialMessage(const mavlink_message_t& msg);

    void enqueueHighRateTelemetry();
    void enqueueLowRateTelemetry();

    std::unique_ptr<Boardcore::SX1278Fsk> transceiver;
    MavlinkBackend radioMavlink{.parent = *this};

    std::unique_ptr<Boardcore::SerialTransceiver> serialTransceiver;
    MavlinkBackend serialMavlink{.parent = *this};

    std::atomic<bool> started{false};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Radio");
};

}  // namespace Parafoil
