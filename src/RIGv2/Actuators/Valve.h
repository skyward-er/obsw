#pragma once

#include <RIGv2/BoardScheduler.h>
#include <RIGv2/CanHandler/CanHandler.h>
#include <RIGv2/Registry/Registry.h>
#include <actuators/Servo/Servo.h>
#include <miosix.h>
#include <scheduler/SignaledDeadlineTask.h>
#include <scheduler/TaskScheduler.h>

#include <chrono>
#include <memory>
#include <optional>
#include <variant>

#include "ValveServoPCA.h"
#include "ValveSolenoid.h"
#include "ValveTimed.h"

namespace RIGv2
{
class Valve : public Boardcore::InjectableWithDeps<Registry>,
              public Boardcore::SignaledDeadlineTask
{
public:
    static const TimePoint ValveClosed;

    struct ValveConfig
    {
        float limit                 = 1.0;    ///< Movement range limit
        bool flipped                = false;  ///< Whether the servo is flipped
        uint32_t defaultOpeningTime = 1000;   // Default opening time [ms]
        float defaultMaxAperture    = 1.0;    // Max aperture

        uint8_t openingEvent = 0;  ///< Event to fire after opening
        uint8_t closingEvent = 0;  ///< Event to fire after closing
        uint32_t openingTimeRegKey =
            CONFIG_ID_DEFAULT_OPENING_TIME;  ///< Registry key for opening
                                             ///< time
        uint32_t maxApertureRegKey =
            CONFIG_ID_DEFAULT_MAX_APERTURE;  ///< Registry key for max
                                             ///< aperture
    };

    Valve(std::unique_ptr<ValveInterface> interface,
          const ValveConfig& config = {})
        : servo(std::move(interface)), config(config)
    {
    }
    ~Valve() {};

    // Move-only
    Valve(Valve&&)                 = default;
    Valve& operator=(Valve&&)      = default;
    Valve(const Valve&)            = delete;
    Valve& operator=(const Valve&) = delete;

    void unsafeSetServoPosition(float position);

    bool isServoOpen();

    float getServoPosition();
    float getMaxAperture();
    uint32_t getOpeningTime();

    bool setMaxAperture(float aperture);
    bool setOpeningTime(uint32_t time);

private:
    std::unique_ptr<ValveInterface> servo;
    ValveConfig config;

    // Time when the valve should close, 0 if currently closed
    TimePoint closeTs = ValveClosed;
    // Time when to backstep the valve to avoid straining the servo
    TimePoint backstepTs = ValveClosed;
};
}  // namespace RIGv2
