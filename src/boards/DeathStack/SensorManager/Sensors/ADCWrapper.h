/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 */

#pragma once

#include <drivers/adc/ADC.h>
#include "ADCWrapperData.h"
#include "DeathStack/configs/SensorManagerConfig.h"
#include "sensors/Sensor.h"

namespace DeathStackBoard
{
typedef ::SensorADC<INTERNAL_ADC_NUM> adc_t;
/**
 * ADC wrapper to abstract reading of battery voltage & current sense on
 * the R2A Hermes rocket.
 *
 * @tparam ADC_n
 */
class ADCWrapper
{
public:
    ADCWrapper() : battery_sensor(*this), current_sensor(*this) {}
    /**
     * Abstracts the current sense reading on the adc as if it was an
     * independent sensor.
     */
    class CurrentSensor : public ::Sensor
    {
    public:
        CurrentSensor(ADCWrapper& parent) : parent(parent) {}

        bool init() override { return true; }

        bool selfTest() override { return true; }

        /**
         * @brief Converts the channels associated with the current sense.
         */
        bool onSimpleUpdate() override
        {
            current_data.timestamp = miosix::getTick();
            current_data.raw_value_1 = parent.adc.convertChannel(CS_1_CHANNEL);

            current_data.raw_value_2 = parent.adc.convertChannel(CS_2_CHANNEL);

            current_data.current_1 = adcToCurrent(current_data.raw_value_1);
            current_data.current_2 = adcToCurrent(current_data.raw_value_2);

            return true;
        }

        /**
         * @brief Returns a pointer to the last conveted current sense value
         */
        CurrentSenseData* getCurrentDataPtr() { return &current_data; }

    private:
        float adcToCurrent(uint16_t adc_in) { return (adc_in - 107) / 32.4f; }

        const adc_t::Channel CS_1_CHANNEL =
            static_cast<adc_t::Channel>(ADC_CURRENT_SENSE_1_CHANNEL);

        const adc_t::Channel CS_2_CHANNEL =
            static_cast<adc_t::Channel>(ADC_CURRENT_SENSE_2_CHANNEL);

        ADCWrapper& parent;
        CurrentSenseData current_data;
    };

    /**
     * Abstracts the battery voltage reading on the adc as if it was an
     * independent sensor.
     */
    class BatterySensor : public ::Sensor
    {
    public:
        BatterySensor(ADCWrapper& parent) : parent(parent) {}

        bool init() override { return true; }

        bool selfTest() override { return true; }

        /**
         * @brief Converts the channel associated with the battery voltage
         */
        bool onSimpleUpdate() override
        {
            uint16_t battery_volt =
                parent.adc.convertChannel(BATTERY_VOLT_CHANNEL);

            battery_data.timestamp = miosix::getTick();
            battery_data.raw_value = battery_volt;
            battery_data.volt      = battery_volt * 12 / 2686.0f;

            return true;
        }
        /**
         * @brief Returns the pointer to the result of the last conversion.
         */
        BatteryVoltageData* getBatteryDataPtr() { return &battery_data; }

    private:
        const adc_t::Channel BATTERY_VOLT_CHANNEL =
            static_cast<adc_t::Channel>(ADC_BATTERY_VOLTAGE_CHANNEL);
        ADCWrapper& parent;

        BatteryVoltageData battery_data;
    };

    BatterySensor* getBatterySensorPtr() { return &battery_sensor; }

    CurrentSensor* getCurrentSensorPtr() { return &current_sensor; }

private:
    BatterySensor battery_sensor;
    CurrentSensor current_sensor;
    adc_t adc;
};

}  // namespace DeathStackBoard