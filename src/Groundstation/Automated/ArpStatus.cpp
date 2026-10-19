#include "Groundstation/Automated/ArpStatus.h"

#include <drivers/timer/TimestampTimer.h>
#include <logger/Logger.h>
#include <iostream>

#include "common/MavlinkHydra.h"
#include "Groundstation/Common/Config/RadioConfig.h"

namespace Antennas
{

    void ArpStatus::sendArpTm()
    {

        mavlink_arp_tm_t tm = {};

        // Common gs tm fields
        tm.timestamp       = Boardcore::TimestampTimer::getTimestamp();
        tm.log_number      = Boardcore::Logger::getInstance().getCurrentLogNumber();
        tm.battery_voltage = -420.0;

    #ifdef ARP_GS_ONLY
        // ARP GS specific fields
        Actuators* actuators       = getModule<Actuators>();
        SMA* sm                    = getModule<SMA>();
        AntennaAngles targetAngles = sm->getTargetAngles();

        std::cout << "RUNNING ARP GS!" << std::endl;

        auto vn300      = getModule<Sensors>()->getVN300LastSample();
        tm.yaw          = vn300.yaw;
        tm.pitch        = vn300.pitch;
        tm.roll         = vn300.roll;
        tm.stepperX_pos = actuators->getCurrentDegPosition(StepperList::STEPPER_X);
        tm.stepperX_delta = actuators->getDeltaAngleDeg(StepperList::STEPPER_X);
        tm.stepperX_speed = actuators->getSpeed(StepperList::STEPPER_X);
        tm.stepperY_pos = actuators->getCurrentDegPosition(StepperList::STEPPER_Y);
        tm.stepperY_delta = actuators->getDeltaAngleDeg(StepperList::STEPPER_Y);
        tm.stepperY_speed = actuators->getSpeed(StepperList::STEPPER_Y);
        tm.gps_latitude   = vn300.latitude;
        tm.gps_longitude  = vn300.longitude;
        tm.gps_height     = vn300.altitude;
        tm.gps_fix        = vn300.gpsFix;
        tm.target_yaw     = targetAngles.yaw;
        tm.target_pitch   = targetAngles.pitch;
        tm.state          = static_cast<uint8_t>(sm->getStatus());
    #endif

        mavlink_message_t msg;
        mavlink_msg_arp_tm_encode(systemId, componentId, &msg, &tm);
        getModule<Groundstation::HubBase>()->dispatchIncomingMsg(msg);    }

    void ArpStatus::run()
    {
        while (!shouldStop()){
            sendArpTm();
            miosix::Thread::sleep(Groundstation::RADIO_STATUS_PERIOD);
        }
    }
} 