/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Riccardo Sironi, Pietro Bortolus
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

#include <RIGv3/BoardScheduler.h>
#include <RIGv3/CanHandler/CanHandler.h>
#include <RIGv3/Registry/Registry.h>
#include <Valve/Valve.h>
#include <actuators/Servo/Servo.h>
#include <actuators/SparkPlug.h>
#include <common/MavlinkHydra.h>
#include <drivers/PCA9685/PCA9685.h>
#include <miosix.h>
#include <scheduler/SignaledDeadlineTask.h>
#include <scheduler/TaskScheduler.h>

namespace RIGv3
{

class Actuators
    : public Boardcore::InjectableWithDeps<Buses, BoardScheduler, CanHandler>,
      public Boardcore::SignaledDeadlineTask
{
private:
    // Sentinel value for the valve closed state
    static const TimePoint ValveClosed;

    struct ValveInfo : Boardcore::InjectableWithDeps<Registry>
    {
        ValveInfo(std::unique_ptr<Boardcore::Valve>&& valve)
            : valve(std::move(valve))
        {
        }

        // Move-only
        ValveInfo(ValveInfo&& other)            = default;
        ValveInfo& operator=(ValveInfo&& other) = default;

        // Disable Copy
        ValveInfo(const ValveInfo&)            = delete;
        ValveInfo& operator=(const ValveInfo&) = delete;

        std::unique_ptr<Boardcore::Valve> valve;

        float animationStep      = 0.0f;  ///< Amount of one animation step
        TimePoint animationEndTs = ValveClosed;  ///< End time of last animation

        // Time when the valve should be moved next during an animation
        TimePoint updateTs = ValveClosed;
        // Time when to backstep the valve to avoid straining the servo
        TimePoint backstepTs = ValveClosed;
        // Time when the valve should close
        TimePoint closeTs = ValveClosed;

        void openServoWithTime(float position, uint32_t time);
        void openServoWithTime(uint32_t time);
        void animateServo(float position, uint32_t time);
        void closeServo();

        void backstep();
        void advanceAnimation();

        float getMaxAperture();
        uint32_t getOpeningTime();
        bool setMaxAperture(float aperture);
        bool setOpeningTime(uint32_t time);

        bool isServoOpen();
    };

    struct ValveState
    {
        bool valid = false;  ///< Whether the data in this struct is valid
        bool state = false;  ///< Whether the valve is open or closed
        std::chrono::milliseconds timing      = {};  ///< Opening time
        std::chrono::milliseconds timeToClose = {};  ///< Time until valve close
        float aperture                        = 0;   ///< Max valve aperture
        float position                        = 0;
    };

public:
    Actuators();

    bool isStarted();

    [[nodiscard]] bool start();

    bool wiggleServo(ServosList servo);
    bool toggleServo(ServosList servo);
    bool openServo(ServosList servo);
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

    ValveState getValveInfo(ServosList servo);

    // N2 3-way valve control
    void set3wayValveState(bool state);
    bool get3wayValveState();

    void startSparkPlugWithTime(uint32_t time);
    void stopSparkPlug();
    void toggleSparkPlug();
    bool isSparkSparking();

    void armLightOn();
    void armLightOff();

    void igniterOn();
    void igniterOff();

    void clacsonOn();
    void clacsonOff();

    void inject(Boardcore::DependencyInjector& injector) override;

private:
    ValveInfo* getServo(ServosList servo);

    TimePoint nextTaskDeadline() override;
    void task() override;

    std::atomic<bool> started{false};

    miosix::FastMutex infosMutex;
    std::vector<ValveInfo> infos;

    void unsafeStartSparkPlug();
    void unsafeStopSparkPlug();

    Boardcore::PCA9685 expander0;
    Boardcore::PCA9685 expander1;

    // PRZ 3-way valve info
    ValveInfo prz_3wayValveInfo;
    std::atomic<bool> prz_3wayValveState{false};
    std::atomic<bool> prz_3wayValveStateChanged{true};

    std::unique_ptr<Boardcore::SparkPlug> spark;

    TimePoint fuelSolenoidCloseTs = ValveClosed;
    TimePoint oxSolenoidCloseTs   = ValveClosed;
    TimePoint sparkPlugCloseTs    = ValveClosed;

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("actuators");
};

}  // namespace RIGv3
