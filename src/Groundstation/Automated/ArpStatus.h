#pragma once

#include <utils/DependencyManager/DependencyManager.h>
#include <ActiveObject.h>

#include "Groundstation/Automated/Actuators/Actuators.h"
#include "Groundstation/Automated/SMA/SMA.h"
#include "Groundstation/Automated/Sensors/Sensors.h"
#include "Groundstation/Common/HubBase.h"

namespace Antennas
{

    class ArpStatus : public Boardcore::InjectableWithDeps<Actuators, SMA, Sensors, Groundstation::HubBase>,
                    public Boardcore::ActiveObject
    {
    public:
        bool start() { return ActiveObject::start(); }

        void sendArpTm();

    private:
        void run() override;

        uint8_t systemId    = 1;
        uint8_t componentId = 1;
    };
}