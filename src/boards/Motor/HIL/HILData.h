/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano, Giulia Facchi
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

#include <Motor/Configs/HILSimulationConfig.h>
#include <sensors/HILSimulatorData.h>

namespace Motor
{

// Sensors Data
using MotorHILChamberBarometerData =
    Boardcore::BarometerSimulatorData<Config::HIL::N_DATA_BARO_CHAMBER>;

enum class MotorFlightPhases
{
    SIMULATION_STARTED
};

struct ActuatorsStateHIL
{
    float mainValvePercentage    = 0;
    float ventingValvePercentage = 0;

    ActuatorsStateHIL()
        : mainValvePercentage(0.0f), ventingValvePercentage(0.0f)
    {
    }

    ActuatorsStateHIL(float mainValvePercentage, float ventingValvePercentage)
        : mainValvePercentage(mainValvePercentage),
          ventingValvePercentage(ventingValvePercentage)
    {
    }

    void print()
    {
        printf(
            "mainValve: %f perc\n"
            "venting: %f perc\n",
            mainValvePercentage * 100, ventingValvePercentage * 100);
    }
};

/**
 * @brief Data structure used by the simulator in order to directly deserialize
 * the data received
 *
 * This structure then is accessed by sensors and other components in order to
 * get the data they need
 */
struct SimulatorData
{
    MotorHILChamberBarometerData pressureChamber;
    float signal;
};

/**
 * @brief Data structure expected by the simulator
 */
struct ActuatorData
{
    ActuatorsStateHIL actuatorsState;

    ActuatorData() : actuatorsState() {}

    ActuatorData(ActuatorsStateHIL actuatorsState)
        : actuatorsState(actuatorsState)
    {
    }

    void print() { actuatorsState.print(); }
};

}  // namespace Motor