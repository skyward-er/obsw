/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Federico Lolli
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

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/AltitudeTrigger/AltitudeTrigger.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Buses.h>
#include <Parafoil/PinHandler/PinHandler.h>
#include <Parafoil/Radio/Radio.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/StateMachines/WingController/WingController.h>
#include <Parafoil/TMRepository/TMRepository.h>
#include <Parafoil/WindEstimationScheme/WindEstimation.h>
#include <Singleton.h>
#include <utils/Debug.h>

#include <utils/ModuleManager/ModuleManager.hpp>

// Dependencies
// legend:
// - PH: PinHandler
// - AC: Actuators
// - BS: BoardScheduler
// - WES: WindEstimationScheme
// - BU: Buses
// - AT: AltitudeTrigger
// - SE: Sensors
// - TM: TMRepository
// - RA: Radio
// - WC: WingController
// - NAS: NASController
//
// PH  | AC  | BS  | WES | BU  | AT  | SE  | TM  | RA  | WC  | NAS
// PH :  .     .     .     .     .     .     .     .     .     .     .
// AC :  .     .     .     .     .     .     .     .     .     .     .
// BS :  .     .     .     .     .     .     .     .     .     .     .
// WES:  .     .     x     .     .     .     x     .     .     .     .
// BU :  .     .     .     .     .     .     .     .     .     .     .
// AT :  .     .     x     .     .     .     .     .     .     .     x
// SE :  .     .     .     .     x     .     x     .     .     .     .
// TM :  x     x     x     .     .     .     x     .     x     .     x
// RA :  x     x     x     .     x     x     x     x     x     x     x
// WC :  .     x     x     x     .     x     .     .     .     .     .
// NAS:  .     .     x     .     .     .     x     .     .     .     .
namespace Parafoil
{
enum class ModuleType
{
    BoardScheduler,
    PinHandler,
    Buses,
    Sensors,
    Radio,
    Actuators,
    AltitudeTrigger,
    NASController,
    WingController,
    WindEstimation,
    TMRepository,
};

/**
 * @brief Helper class to set up the modules
 *
 * @note This class is a singleton
 */
class ModuleHelper : public Boardcore::Singleton<ModuleHelper>
{
    friend class Boardcore::Singleton<ModuleHelper>;

public:
    ModuleHelper() : modules(Boardcore::ModuleManager::getInstance()) {}

    virtual ~ModuleHelper() { delete &modules; }

    Boardcore::ModuleManager& getModules() { return modules; }

    void setUpBoardScheduler()
    {
        // set up BoardScheduler
        insert(new BoardScheduler(), ModuleType::BoardScheduler);
    }

    void setUpPinHandler()
    {
        // set up PinHandler
        insert(new PinHandler(), ModuleType::PinHandler);
    }

    void setUpBuses()
    {
        // set up Buses
        insert(new Buses(), ModuleType::Buses);
    }

    void setUpActuators()
    {
        // set up Actuators
        insert(new Actuators(), ModuleType::Actuators);
    }

    void setUpSensors()
    {
        // dependencies: Buses
        setUpBuses();

        // set up Sensors
        insert(new Sensors(), ModuleType::Sensors);
    }

    void setUpNASController()
    {
        // dependencies: BoardScheduler, Sensors
        setUpBoardScheduler();
        setUpSensors();

        // set up NASController
        insert(new NASController(), ModuleType::NASController);
    }

    void setUpWindEstimation()
    {
        // dependencies: BoardScheduler, Sensors
        setUpBoardScheduler();
        setUpSensors();

        // set up WindEstimation
        insert(new WindEstimation(), ModuleType::WindEstimation);
    }

    void setUpAltitudeTrigger()
    {
        // dependencies: BoardScheduler, NASController
        setUpBoardScheduler();
        setUpNASController();

        // set up AltitudeTrigger
        insert(new AltitudeTrigger(), ModuleType::AltitudeTrigger);
    }

    void setUpWingController()
    {
        // dependencies: Actuators, BoardScheduler, WindEstimationScheme,
        // AltitudeTrigger
        setUpActuators();
        setUpBoardScheduler();
        setUpWindEstimation();
        setUpAltitudeTrigger();

        // set up WingController
        insert(new WingController(), ModuleType::WingController);
    }

    void setUpRadio()
    {
        // dependencies: PinHandler, Actuators, BoardScheduler, Buses,
        // AltitudeTrigger, Sensors, TMRepository, WingController, NASController
        setUpPinHandler();
        setUpActuators();
        setUpBoardScheduler();
        setUpBuses();
        setUpAltitudeTrigger();
        setUpSensors();
        setUpTMRepository();
        setUpWingController();
        setUpNASController();

        // set up Radio
        insert(new Radio(), ModuleType::Radio);
    }

    void setUpTMRepository()
    {
        // dependencies: PinHandler, Actuators, BoardScheduler, Sensors, Radio,
        // NASController
        setUpPinHandler();
        setUpActuators();
        setUpBoardScheduler();
        setUpSensors();
        setUpRadio();
        setUpNASController();

        // set up TMRepository
        insert(new TMRepository(), ModuleType::TMRepository);
    }

    /**
     * @brief Mock up a module for testing purposes
     *
     * @tparam T type of the module to be mocked up
     * @param module pointer to the module to be mocked up
     * @param type type of the module
     * @return true if the module was successfully mocked up
     * @return false if the module was already set up
     *
     * @note This function must be called before the module is set up
     * by other setup procedures
     */
    template <class T>
    [[nodiscard]] bool mockUp(T* module, ModuleType type)
    {
        return insert<T>(module, type);
    }

private:
    Boardcore::ModuleManager& modules;

    // keep a list of all modules set up
    std::vector<ModuleType> moduleList;

    template <class T>
    bool insert(T* module, ModuleType type)
    {
        // check if module is already set up
        if (std::find(moduleList.begin(), moduleList.end(), type) !=
            moduleList.end())
        {
            return false;
        }

        return modules.insert<T>(module);
    }
};
}  // namespace Parafoil
