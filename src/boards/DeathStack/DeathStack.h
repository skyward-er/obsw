/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#pragma once

#include <Common.h>

#include <events/EventBroker.h>
#include <boards/CanInterfaces.h>
#include <DeathStack/Events.h>
#include <DeathStack/Topics.h>
#include <DeathStack/Status.h>

#include <DeathStack/LogProxy/LogProxy.h>
#include <DeathStack/Canbus/CanProxy.h>
#include <DeathStack/PinObserver/PinObserverWrapper.h>

#include <DeathStack/ADA/ADA.h>
#include <DeathStack/SensorManager/SensorManager.h>
#include <DeathStack/TMTCManager/TMTCManager.h>
#include <DeathStack/PinObserver/PinObserverWrapper.h>
#include <DeathStack/FlightModeManager/FlightModeManager.h>
#include <DeathStack/IgnitionController/IgnitionController.h>
#include <DeathStack/DeploymentController/Deployment.h>

namespace DeathStackBoard
{

/**
 * This file provides a simplified way to initialize and monitor all
 * the components of the DeathStack.
 */
class DeathStack
{

public:
    // Shared Components
    EventBroker* broker;
    LoggerProxy* logger;
    CanManager can_mgr;
    CanProxy can;
    //PinObserverWrapper pinObs;

    // FSMs
    ADA ada;
    SensorManager sensors;
    TMTCManager tmtc;
    FlightModeManager* fmm;
    IgnitionController ign;
    DeploymentController dpl;

    /**
     * Initialize Everything
     */
    DeathStack() : can_mgr(CAN1), can(&can_mgr), ign(&can)
    {
        /* Shared components */
        TRACE("Init shared components...");
        broker = sEventBroker;
        logger = Singleton<LoggerProxy>::getInstance();
        broker->start();
        //pinObs.start();
        TRACE(" Done\n");

        /* State Machines */
        TRACE("Init state machines...");
        fmm = Singleton<FlightModeManager>::getInstance();
        fmm->start();
        sensors.start();
        ada.start();
        tmtc.start();
        ign.start();
        dpl.start();
        TRACE(" Done\n");

        TRACE("Init finished\n");
    }

    ~DeathStack()
    {
        sEventBroker->stop();
        //pinObs.stop();

        /* State Machines */
        fmm->stop();
        sensors.stop();
        ada.stop();
        tmtc.stop();
        ign.stop();
        dpl.stop();
    }

    /**
     * Helpers for debugging purposes
     */ 
    inline void postEvent(Event ev, uint8_t topic) 
    {
        broker->post(ev, topic);
    }

    inline Status::tm_repo_t getStatus()
    {
        miosix::PauseKernelLock kLock;
        return Status::tm_repository;
    }
};

} /* namespace DeathStackBoard */