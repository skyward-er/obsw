/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

#include <Motor/Sensors/Sensors.h>

namespace Motor
{

namespace SensorsConfig
{

constexpr Boardcore::InternalADC::Channel ADC_BATTERY_VOLTAGE_CH =
    Boardcore::InternalADC::Channel::CH15;
// The battery is connected to the stm32's adc with a voltage divider
constexpr float ADC_BATTERY_VOLTAGE_COEFF = (10 + 20) / 10;

// Boardcore::LSM6DSRXConfig LSM6_SENSOR_CONFIG{
//     .bdu       = Boardcore::LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE,
//     .odrAcc    = Boardcore::LSM6DSRXConfig::ACC_ODR::HZ_1660,
//     .opModeAcc = Boardcore::LSM6DSRXConfig::OPERATING_MODE::NORMAL,
//     .fsAcc     = Boardcore::LSM6DSRXConfig::ACC_FULLSCALE::G16,
//     .odrGyr    = Boardcore::LSM6DSRXConfig::GYR_ODR::HZ_1660,
//     .opModeGyr = Boardcore::LSM6DSRXConfig::OPERATING_MODE::NORMAL,
//     .fsGyr     = Boardcore::LSM6DSRXConfig::GYR_FULLSCALE::DPS_1000,
//     .fifoMode  = Boardcore::LSM6DSRXConfig::FIFO_MODE::CONTINUOUS,
//     .fifoTimestampDecimation =
//         Boardcore::LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DEC_1,
//     .fifoTemperatureBdr =
//         Boardcore::LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::DISABLED,
//     .int1InterruptSelection = Boardcore::LSM6DSRXConfig::INTERRUPT::NOTHING,
//     .int2InterruptSelection =
//         Boardcore::LSM6DSRXConfig::INTERRUPT::FIFO_THRESHOLD,
//     .fifoWatermark = 170,
// };
// Boardcore::SPIBusConfig LSM6_SPI_CONFIG{
//     Boardcore::SPI::ClockDivider::DIV_16,
//     Boardcore::SPI::Mode::MODE_0,
// };

Boardcore::H3LIS331DLDefs::OutputDataRate H3LIS_ODR =
    Boardcore::H3LIS331DLDefs::OutputDataRate::ODR_1000;
Boardcore::H3LIS331DLDefs::BlockDataUpdate H3LIS_BDU =
    Boardcore::H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE;
Boardcore::H3LIS331DLDefs::FullScaleRange H3LIS_FSR =
    Boardcore::H3LIS331DLDefs::FullScaleRange::FS_100;
Boardcore::SPIBusConfig H3LIS_SPI_CONFIG{
    Boardcore::SPI::ClockDivider::DIV_16,
};

Boardcore::LIS2MDL::Config LIS2_SENSOR_CONFIG{
    .odr                = Boardcore::LIS2MDL::ODR_100_HZ,
    .deviceMode         = Boardcore::LIS2MDL::MD_CONTINUOUS,
    .temperatureDivider = 100,
};
Boardcore::SPIBusConfig LIS2_SPI_CONFIG{
    Boardcore::SPI::ClockDivider::DIV_16,
    Boardcore::SPI::Mode::MODE_3,
    Boardcore::SPI::Order::MSB_FIRST,
    Boardcore::SPI::Order::LSB_FIRST,
};

Boardcore::LPS22DF::Config LPS22_SENSOR_CONFIG{
    .odr = Boardcore::LPS22DF::ODR_100,
    .avg = Boardcore::LPS22DF::AVG_512,
};
Boardcore::SPIBusConfig LPS22_SPI_CONFIG{
    Boardcore::SPI::ClockDivider::DIV_16,
    Boardcore::SPI::Mode::MODE_3,
    Boardcore::SPI::Order::MSB_FIRST,
    Boardcore::SPI::Order::LSB_FIRST,
};

Boardcore::ADS131M08::Config ADS131_SENSOR_CONFIG{
    .channelsConfig =
        {
            {.enabled = false},
            // For the servo motor's current sensor the current range is from 0A
            // to 20A and the output voltage from 0.65V to 2.65V. The sensor is
            // connected to the adc through a voltage divider with a gain
            // of 4.166666 So at the ADC side we have maximum 0.636 and we have
            // to set a gain of 1 on the ADC
            {
                .enabled = true,
                .pga     = Boardcore::ADS131M08Defs::PGA::PGA_1,
            },
            {.enabled = false},
            {.enabled = false},
            {.enabled = false},
            // The analog pressure sensors are measured with a 30ohm shunt
            // resistor. The current ranges from 4mAh to 20mAh. So we can have a
            // voltage up to 0.6V We set a gain of 2 to maximize resolution
            {
                .enabled = true,
                .pga     = Boardcore::ADS131M08Defs::PGA::PGA_2,
            },
            {
                .enabled = true,
                .pga     = Boardcore::ADS131M08Defs::PGA::PGA_2,
            },
            {
                .enabled = true,
                .pga     = Boardcore::ADS131M08Defs::PGA::PGA_2,
            },
        },
    .oversamplingRatio = Boardcore::ADS131M08Defs::OversamplingRatio::OSR_4096,
    .globalChopModeEnabled = false,
};
Boardcore::SPIBusConfig ADS131_SPI_CONFIG{
    Boardcore::SPI::ClockDivider::DIV_8,
    Boardcore::SPI::Mode::MODE_1,
};
Boardcore::ADS131M08Defs::Channel ADS131_CHAMBER_PRESSURE_CH =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_5;
Boardcore::ADS131M08Defs::Channel ADS131_TANK_PRESSURE_1_CH =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_6;
Boardcore::ADS131M08Defs::Channel ADS131_TANK_PRESSURE_2_CH =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_7;
Boardcore::ADS131M08Defs::Channel ADS131_SERVO_CURRENT_CH =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_1;

// The chamber pressure sensor is a 0-40bar sensor with a 4-20mA output
// On the motor motherboard there a 30ohm shunt. The shunt then goes through a
// filter with 491Hz of bandwidth and unitary gain.
// We use only a gain coefficient because the offset is removed by the ADC
// offset calibration feature.
constexpr float CHAMBER_PRESSURE_MAX      = 40;
constexpr float CHAMBER_PRESSURE_MIN      = 0;
constexpr float CHAMBER_PRESSURE_SHUNT    = 30;     // [ohm]
constexpr float CHAMBER_PRESSURE_CURR_MIN = 0.004;  // [A]
constexpr float CHAMBER_PRESSURE_CURR_MAX = 0.020;  // [A]
constexpr float CHAMBER_PRESSURE_COEFF =
    CHAMBER_PRESSURE_MAX /
    (CHAMBER_PRESSURE_CURR_MAX - CHAMBER_PRESSURE_CURR_MIN);  // [bar/A]

// The tank pressure sensors are 0-100bar sensors with a 4-20mA output
// On the motor motherboard there a 30ohm shunt. The shunt then goes through a
// filter with 491Hz of bandwidth and unitary gain.
// We use only a gain coefficient because the offset is removed by the ADC
// offset calibration feature.
constexpr float TANK_PRESSURE_1_MAX      = 100;
constexpr float TANK_PRESSURE_1_MIN      = 0;
constexpr float TANK_PRESSURE_1_SHUNT    = 30;     // [ohm]
constexpr float TANK_PRESSURE_1_CURR_MIN = 0.004;  // [A]
constexpr float TANK_PRESSURE_1_CURR_MAX = 0.020;  // [A]
constexpr float TANK_PRESSURE_1_COEFF =
    TANK_PRESSURE_1_MAX /
    (TANK_PRESSURE_1_CURR_MAX - TANK_PRESSURE_1_CURR_MIN);  // [bar/A]
constexpr float TANK_PRESSURE_2_MAX      = 100;
constexpr float TANK_PRESSURE_2_MIN      = 0;
constexpr float TANK_PRESSURE_2_SHUNT    = 30;     // [ohm]
constexpr float TANK_PRESSURE_2_CURR_MIN = 0.004;  // [A]
constexpr float TANK_PRESSURE_2_CURR_MAX = 0.020;  // [A]
constexpr float TANK_PRESSURE_2_COEFF =
    TANK_PRESSURE_2_MAX /
    (TANK_PRESSURE_2_CURR_MAX - TANK_PRESSURE_2_CURR_MIN);  // [bar/A]

// The current sensor used to measure the servo motors current consumption:
// - A range of 0A to 20A
// - Has an output from 0.65V to 2.65V
// - Is connected to a voltage divider with a coefficient of 12 / (38.3 + 12)
//
// We use only a gain coefficient because the offset is removed by the ADC
// offset calibration feature.
constexpr float SERVO_CURRENT_COEFF =
    20 / (2.65 - 0.65) / (12 / (38.3 + 12));  // [A/V]

// Sampling periods
constexpr uint32_t SAMPLE_PERIOD_ADC    = 1000;
constexpr uint32_t SAMPLE_PERIOD_LSM6   = 10;
constexpr uint32_t SAMPLE_PERIOD_H3LIS  = 100;
constexpr uint32_t SAMPLE_PERIOD_LIS2   = 10;
constexpr uint32_t SAMPLE_PERIOD_LPS22  = 10;
constexpr uint32_t SAMPLE_PERIOD_ADS131 = 1;
constexpr uint32_t SAMPLE_PERIOD_MAX    = 10;

}  // namespace SensorsConfig

}  // namespace Motor