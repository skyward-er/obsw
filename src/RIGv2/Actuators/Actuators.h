/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto, Riccardo Sironi
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
#include <scheduler/SignaledDeadlineTask.h>
#include <scheduler/TaskScheduler.h>

#include "Valve.h"

namespace RIGv2
{

class Actuators
    : public Boardcore::InjectableWithDeps<BoardScheduler, CanHandler>,
      public Boardcore::SignaledDeadlineTask
{
private:
    // Sentinel value for the valve closed state
    static const TimePoint ValveClosed;

    struct ServoInfo
    {
        ServoInfo(std::unique_ptr<Valve>&& valve) : valve(std::move(valve)) {}

        // Move-only
        ServoInfo(ServoInfo&& other)            = default;
        ServoInfo& operator=(ServoInfo&& other) = default;

        // Disable Copy
        ServoInfo(const ServoInfo&)            = delete;
        ServoInfo& operator=(const ServoInfo&) = delete;

        std::unique_ptr<Valve> valve;

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
        void backstep();
        void move();

        bool isServoOpen();
    };

    struct ValveInfo
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

    void openOxSolenoidWithTime(uint32_t time);
    void closeOxSolenoid();
    void toggleOxSolenoid();
    bool isOxSolenoidOpen();

    void openFuelSolenoidWithTime(uint32_t time);
    void closeFuelSolenoid();
    void toggleFuelSolenoid();
    bool isFuelSolenoidOpen();

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
    ServoInfo* getServo(ServosList servo);

    void unsafeSetServoPosition(uint8_t idx, float position);

    TimePoint nextTaskDeadline() override;
    void task() override;

    std::atomic<bool> started{false};

    miosix::FastMutex infosMutex;
    std::array<ServoInfo, 10> infos;
    std::vector<ServoInfo> infos2;

    void unsafeOpenOxSolenoid();
    void unsafeCloseOxSolenoid();

    void unsafeOpenFuelSolenoid();
    void unsafeCloseFuelSolenoid();

    void unsafeStartSparkPlug();
    void unsafeStopSparkPlug();

    // N2 3-way valve info
    ServoInfo n2_3wayValveInfo;
    std::atomic<bool> n2_3wayValveState{false};
    std::atomic<bool> n2_3wayValveStateChanged{true};

    // Time when the chamber valve should close, 0 if currently closed
    TimePoint chamberCloseTs = ValveClosed;

    // PRZ 3-way valve info
    ServoInfo prz_3wayValveInfo;
    std::atomic<bool> prz_3wayValveState{false};
    std::atomic<bool> prz_3wayValveStateChanged{true};

    std::unique_ptr<Boardcore::SparkPlug> spark;

    TimePoint fuelSolenoidCloseTs = ValveClosed;
    TimePoint oxSolenoidCloseTs   = ValveClosed;
    TimePoint sparkPlugCloseTs    = ValveClosed;

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("actuators");
};

}  // namespace RIGv2
