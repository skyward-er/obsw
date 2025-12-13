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
#include <common/MavlinkHydra.h>
#include <miosix.h>
#include <scheduler/SignaledDeadlineTask.h>
#include <scheduler/TaskScheduler.h>

#include <memory>

namespace RIGv2
{

class Actuators
    : public Boardcore::InjectableWithDeps<BoardScheduler, CanHandler>,
      public Boardcore::SignaledDeadlineTask
{
private:
    // Sentinel value for the valve closed state
    static const TimePoint ValveClosed;

    struct ServoInfo : public Boardcore::InjectableWithDeps<Registry>
    {
        struct ServoConfig
        {
            float limit  = 1.0f;   ///< Movement range limit
            bool flipped = false;  ///< Whether the servo is flipped
            uint32_t defaultOpeningTime = 1000;  // Default opening time [ms]
            float defaultMaxAperture    = 1.0f;  // Max aperture

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

        float currentPosition = 0.0f;  ///< Current position in range [0, 1]

        enum class Direction
        {
            CLOSE,
            OPEN,
        } direction = Direction::CLOSE;  ///< Direction of the last valve move

        float animationStep      = 0.0f;  ///< Amount of one animation step
        TimePoint animationEndTs = ValveClosed;  ///< End time of last animation

        // Time when the valve should be moved next during an animation
        TimePoint updateTs = ValveClosed;
        // Time when to backstep the valve to avoid straining the servo
        TimePoint backstepTs = ValveClosed;
        // Time when the vavle should close
        TimePoint closeTs = ValveClosed;

        void openServoWithTime(float position, uint32_t time);
        void openServoWithTime(uint32_t time);
        void animateServo(float position, uint32_t time);
        void closeServo();

        void unsafeSetServoPosition(float position);
        void move();
        void backstep();

        float scalePosition(float position);
        bool isServoOpen();
        float getServoPosition();
        float getMaxAperture();
        uint32_t getOpeningTime();

        bool setMaxAperture(float aperture);
        bool setOpeningTime(uint32_t time);
    };

    struct ValveInfo
    {
        bool valid = false;  ///< Whether the data in this struct is valid
        bool state = false;  ///< Whether the valve is open or closed
        std::chrono::milliseconds timing      = {};  ///< Opening time
        std::chrono::milliseconds timeToClose = {};  ///< Time until valve close
        float aperture                        = 0.0f;  ///< Max valve aperture
        float position                        = 0.0f;  ///< Current position
    };

public:
    Actuators();

    [[nodiscard]] bool start();

    bool isStarted();

    bool wiggleServo(ServosList servo);
    bool toggleServo(ServosList servo);
    bool openServo(ServosList servo);
    bool moveServo(ServosList servo, float position);
    bool moveServoWithTime(ServosList servo, float position, uint32_t time);
    bool openServoWithTime(ServosList servo, uint32_t time);
    bool animateServo(ServosList servo, float position, uint32_t time);

    bool closeServo(ServosList servo);
    void closeAllServos();
    bool setMaxAperture(ServosList servo, float aperture);
    bool setOpeningTime(ServosList servo, uint32_t time);
    bool isServoOpen(ServosList servo);
    uint32_t getServoOpeningTime(ServosList servo);
    float getServoMaxAperture(ServosList servo);

    ValveInfo getValveInfo(ServosList servo);

    // N2 3-way valve control
    void set3wayValveState(bool state);
    bool get3wayValveState();

    void armLightOn();
    void armLightOff();

    void igniterOn();
    void igniterOff();

    void clacsonOn();
    void clacsonOff();

    void inject(Boardcore::DependencyInjector& injector) override;

private:
    ServoInfo* getServo(ServosList servo);

    void unsafeSetServoPosition(uint8_t idx, float position);

    TimePoint nextTaskDeadline() override;
    void task() override;

    std::atomic<bool> started{false};

    miosix::FastMutex infosMutex;
    std::array<ServoInfo, 10> infos;

    // PRZ 3-way valve info
    ServoInfo prz_3wayValveInfo;
    std::atomic<bool> prz_3wayValveState{false};
    std::atomic<bool> prz_3wayValveStateChanged{true};

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("actuators");
};

}  // namespace RIGv2
