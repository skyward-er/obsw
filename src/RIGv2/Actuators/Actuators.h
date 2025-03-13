/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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

#include <RIGv2/BoardScheduler.h>
#include <RIGv2/CanHandler/CanHandler.h>
#include <RIGv2/Registry/Registry.h>
#include <actuators/Servo/Servo.h>
#include <common/MavlinkOrion.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>

#include <memory>

namespace RIGv2
{

class Actuators
    : public Boardcore::InjectableWithDeps<BoardScheduler, CanHandler>
{
private:
    struct ServoInfo : public Boardcore::InjectableWithDeps<Registry>
    {
        struct ServoConfig
        {
            float limit  = 1.0;    ///< Movement range limit
            bool flipped = false;  ///< Whether the servo is flipped
            uint32_t defaultOpeningTime = 1000;  // Default opening time [ms]
            float defaultMaxAperture    = 1.0;   // Max aperture

            uint8_t openingEvent = 0;  ///< Event to fire after opening
            uint8_t closingEvent = 0;  ///< Event to fire after closing
            uint32_t openingTimeRegKey =
                CONFIG_ID_DEFAULT_OPENING_TIME;  ///< Registry key for opening
                                                 ///< time
            uint32_t maxApertureRegKey =
                CONFIG_ID_DEFAULT_MAX_APERTURE;  ///< Registry key for max
                                                 ///< aperture
        };

        ServoInfo(std::unique_ptr<Boardcore::Servo> servo,
                  const ServoConfig& config)
            : servo(std::move(servo)), config(config)
        {
        }

        std::unique_ptr<Boardcore::Servo> servo;
        ServoConfig config;

        long long closeTs = 0;  ///< Timestamp to close the servo (0 if closed)
        long long lastActionTs = 0;  ///< Timestamp of last servo action

        void openServoWithTime(uint32_t time);
        void closeServo();
        void unsafeSetServoPosition(float position);
        float getServoPosition();
        float getMaxAperture();
        uint32_t getOpeningTime();
        bool setMaxAperture(float aperture);
        bool setOpeningTime(uint32_t time);
    };

public:
    Actuators();

    [[nodiscard]] bool start();

    bool isStarted();

    bool wiggleServo(ServosList servo);
    bool toggleServo(ServosList servo);
    bool openServo(ServosList servo);
    bool openServoWithTime(ServosList servo, uint32_t time);
    bool closeServo(ServosList servo);
    void closeAllServos();
    bool setMaxAperture(ServosList servo, float aperture);
    bool setOpeningTime(ServosList servo, uint32_t time);
    bool isServoOpen(ServosList servo);
    bool isCanServoOpen(ServosList servo);

    // N2 3-way valve control
    void set3wayValveState(bool state);
    bool get3wayValveState();

    // Chamber valve control
    void openChamberWithTime(uint32_t time);
    void closeChamber();
    bool isChamberOpen();

    uint32_t getServoOpeningTime(ServosList servo);
    float getServoMaxAperture(ServosList servo);

    void armLightOn();
    void armLightOff();

    void igniterOn();
    void igniterOff();

    void setCanServoOpen(ServosList servo, bool open);

    void inject(Boardcore::DependencyInjector& injector) override;

private:
    ServoInfo* getServo(ServosList servo);

    void unsafeSetServoPosition(uint8_t idx, float position);
    void unsafeOpenChamber();
    void unsafeCloseChamber();

    void updatePositionsTask();

    std::atomic<bool> started{false};

    miosix::FastMutex infosMutex;
    std::array<ServoInfo, 10> infos;
    ServoInfo n2_3wayValveInfo;

    long long chamberCloseTs =
        0;  ///< Timestamp to close the chamber (0 if closed)
    long long chamberLastActionTs = 0;  ///< Timestamp of last chamber action

    bool canMainOpen        = false;
    bool canNitrogenOpen    = false;
    bool canOxVentingOpen   = false;
    bool canN2QuenchingOpen = false;

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("actuators");
};

}  // namespace RIGv2
