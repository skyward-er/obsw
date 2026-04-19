/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto, Riccardo Sironi, Pietro Bortolus
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

#include <Motor/BoardScheduler.h>
#include <Motor/CanHandler/CanHandler.h>
#include <Motor/Registry/Registry.h>
#include <Valve/Valve.h>
#include <actuators/Servo/Servo.h>
#include <actuators/SparkPlug.h>
#include <common/MavlinkHydra.h>
#include <scheduler/SignaledDeadlineTask.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Motor
{

class Actuators : public Boardcore::InjectableWithDeps<Buses, BoardScheduler,
                                                       CanHandler, Registry>,
                  public Boardcore::SignaledDeadlineTask
{
private:
    // Sentinel value for the valve closed state
    static const TimePoint noActionNeeded;

    struct ValveInfo
    {
        ValveInfo(std::unique_ptr<Boardcore::Valve>&& valve)
            : valve(std::move(valve))
        {
        }

        // The default constructor is needed since PCA valves require the
        // expanders to be created which, in turn require the I2C bus form the
        // Buses module which is injected in the Actuators module.
        ValveInfo() = default;

        // Move-only
        ValveInfo(ValveInfo&& other)            = default;
        ValveInfo& operator=(ValveInfo&& other) = default;

        // Disable Copy
        ValveInfo(const ValveInfo&)            = delete;
        ValveInfo& operator=(const ValveInfo&) = delete;

        std::unique_ptr<Boardcore::Valve> valve{};

        // Time when to backstep the valve to avoid straining the servo
        TimePoint backstepTs = noActionNeeded;
        // Time when the valve should close
        TimePoint closeTs = noActionNeeded;

        void openValve();
        void closeValve();

        void backstep();
        virtual void resetAnimation() {};

        bool isValveOpen();
    };

    struct ManualValveInfo : public ValveInfo
    {
        ManualValveInfo(std::unique_ptr<Boardcore::Valve>&& valve)
            : ValveInfo(std::move(valve))
        {
        }

        float stepCount  = 0;     ///< Number of steps for the current animation
        float stepAmount = 0.0f;  ///< Amount of one animation step
        // Time when the valve should be moved next during an animation
        TimePoint updateTs = noActionNeeded;

        void animateValve(float position, uint32_t time);
        void advanceAnimation();
        void resetAnimation() override;
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

    bool wiggleValve(ServosList servo);
    bool toggleValve(ServosList servo);
    bool openValve(ServosList servo);
    bool openValveWithTime(ServosList servo, uint32_t time);
    bool moveValve(ServosList servo, float position);
    bool animateValve(ServosList servo, float position, uint32_t time);

    bool closeValve(ServosList servo);
    void closeAllValves();
    bool setMaxAperture(ServosList servo, float aperture);
    bool setOpeningTime(ServosList servo, uint32_t time);
    bool isValveOpen(ServosList servo);
    uint32_t getServoOpeningTime(ServosList servo);
    float getServoMaxAperture(ServosList servo);
    float getValvePosition(ServosList servo);

    ValveState getValveState(ServosList servo);
    inline void logValveMovement(int idx, float position);

    void startSparkPlugWithTime(uint32_t time);
    void stopSparkPlug();
    void toggleSparkPlug();
    bool isSparkSparking();

    void armLightOn();
    void armLightOff();

private:
    ValveInfo* getValve(ServosList servo);
    ManualValveInfo* getManualValve(ServosList servo);

    void unsafeStartSparkPlug();
    void unsafeStopSparkPlug();

    TimePoint nextTaskDeadline() override;
    void task() override;

    std::atomic<bool> started{false};

    miosix::FastMutex infosMutex;
    std::vector<ValveInfo> valveInfos;
    std::vector<ManualValveInfo> manualValveInfos;

    void initializeValves();

    std::unique_ptr<Boardcore::SparkPlug> spark;

    // Timestamp for automatic venting after inactivity for safety reasons
    TimePoint safetyVentingTs;

    TimePoint fuelSolenoidCloseTs = noActionNeeded;
    TimePoint oxSolenoidCloseTs   = noActionNeeded;
    TimePoint sparkPlugCloseTs    = noActionNeeded;

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("actuators");
};

}  // namespace Motor
