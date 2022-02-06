/* Copyright (c) 2015-2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <drivers/adc/InternalADC.h>
#include <interfaces-impl/hwmapping.h>
#include <sensors/ADS1118/ADS1118.h>
#include <sensors/BMX160/BMX160Config.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/calibration/Calibration.h>

using miosix::Gpio;

namespace DeathStackBoard
{

namespace SensorConfigs
{
static constexpr float INTERNAL_ADC_VREF = 3.3;
static constexpr Boardcore::InternalADC::Channel ADC_BATTERY_VOLTAGE =
    Boardcore::InternalADC::Channel::CH5;

static constexpr float BATTERY_VOLTAGE_COEFF    = 5.98;
static constexpr float BATTERY_MIN_SAFE_VOLTAGE = 10.5;  // [V]

static constexpr Boardcore::ADS1118::ADS1118Mux ADC_CH_STATIC_PORT =
    Boardcore::ADS1118::MUX_AIN0_GND;
static constexpr Boardcore::ADS1118::ADS1118Mux ADC_CH_PITOT_PORT =
    Boardcore::ADS1118::MUX_AIN1_GND;
static constexpr Boardcore::ADS1118::ADS1118Mux ADC_CH_DPL_PORT =
    Boardcore::ADS1118::MUX_AIN2_GND;
static constexpr Boardcore::ADS1118::ADS1118Mux ADC_CH_VREF =
    Boardcore::ADS1118::MUX_AIN3_GND;

static constexpr Boardcore::ADS1118::ADS1118DataRate ADC_DR_STATIC_PORT =
    Boardcore::ADS1118::DR_860;
static constexpr Boardcore::ADS1118::ADS1118DataRate ADC_DR_PITOT_PORT =
    Boardcore::ADS1118::DR_860;
static constexpr Boardcore::ADS1118::ADS1118DataRate ADC_DR_DPL_PORT =
    Boardcore::ADS1118::DR_860;
static constexpr Boardcore::ADS1118::ADS1118DataRate ADC_DR_VREF =
    Boardcore::ADS1118::DR_860;

static constexpr Boardcore::ADS1118::ADS1118Pga ADC_PGA_STATIC_PORT =
    Boardcore::ADS1118::FSR_6_144;
static constexpr Boardcore::ADS1118::ADS1118Pga ADC_PGA_PITOT_PORT =
    Boardcore::ADS1118::FSR_6_144;
static constexpr Boardcore::ADS1118::ADS1118Pga ADC_PGA_DPL_PORT =
    Boardcore::ADS1118::FSR_6_144;
static constexpr Boardcore::ADS1118::ADS1118Pga ADC_PGA_VREF =
    Boardcore::ADS1118::FSR_6_144;

// Sampling periods in milliseconds
static constexpr unsigned int SAMPLE_PERIOD_INTERNAL_ADC =
    1000;  // only for battery voltage
static constexpr unsigned int SAMPLE_PERIOD_ADC_ADS1118 = 6;

static constexpr unsigned int SAMPLE_PERIOD_PRESS_DIGITAL = 15;
static constexpr unsigned int TEMP_DIVIDER_PRESS_DIGITAL  = 5;

static constexpr unsigned int SAMPLE_PERIOD_PRESS_PITOT =
    SAMPLE_PERIOD_ADC_ADS1118 * 4;
static constexpr unsigned int SAMPLE_PERIOD_PRESS_DPL =
    SAMPLE_PERIOD_ADC_ADS1118 * 4;
static constexpr unsigned int SAMPLE_PERIOD_PRESS_STATIC =
    SAMPLE_PERIOD_ADC_ADS1118 * 4;

static constexpr unsigned int PRESS_PITOT_CALIB_SAMPLES_NUM  = 500;
static constexpr unsigned int PRESS_STATIC_CALIB_SAMPLES_NUM = 500;
static constexpr float PRESS_STATIC_MOVING_AVG_COEFF         = 0.95;

static constexpr Boardcore::BMX160Config::AccelerometerRange
    IMU_BMX_ACC_FULLSCALE_ENUM =
        Boardcore::BMX160Config::AccelerometerRange::G_16;
static constexpr Boardcore::BMX160Config::GyroscopeRange
    IMU_BMX_GYRO_FULLSCALE_ENUM =
        Boardcore::BMX160Config::GyroscopeRange::DEG_1000;

static constexpr unsigned int IMU_BMX_ACC_GYRO_ODR = 1600;
static constexpr Boardcore::BMX160Config::OutputDataRate
    IMU_BMX_ACC_GYRO_ODR_ENUM =
        Boardcore::BMX160Config::OutputDataRate::HZ_1600;
static constexpr unsigned int IMU_BMX_MAG_ODR = 100;
static constexpr Boardcore::BMX160Config::OutputDataRate IMU_BMX_MAG_ODR_ENUM =
    Boardcore::BMX160Config::OutputDataRate::HZ_100;

static constexpr unsigned int IMU_BMX_FIFO_HEADER_SIZE = 1;
static constexpr unsigned int IMU_BMX_ACC_DATA_SIZE    = 6;
static constexpr unsigned int IMU_BMX_GYRO_DATA_SIZE   = 6;
static constexpr unsigned int IMU_BMX_MAG_DATA_SIZE    = 8;

static constexpr unsigned int IMU_BMX_FIFO_WATERMARK = 80;

// How many bytes go into the fifo each second
static constexpr unsigned int IMU_BMX_FIFO_FILL_RATE =
    IMU_BMX_ACC_GYRO_ODR * (IMU_BMX_FIFO_HEADER_SIZE + IMU_BMX_ACC_DATA_SIZE +
                            IMU_BMX_GYRO_DATA_SIZE) +
    IMU_BMX_MAG_ODR * (IMU_BMX_MAG_DATA_SIZE + IMU_BMX_FIFO_HEADER_SIZE);

// How long does it take for the bmx fifo to fill up
static constexpr unsigned int IMU_BMX_FIFO_FILL_TIME =
    1024 * 1000 / IMU_BMX_FIFO_FILL_RATE;

// Sample before the fifo is full, but slightly after the watermark level
// (watermark + 30) ---> high slack due to scheduler imprecision,
//                       avoid clearing the fifo before the interrupt
static constexpr unsigned int SAMPLE_PERIOD_IMU_BMX =
    IMU_BMX_FIFO_FILL_TIME * (IMU_BMX_FIFO_WATERMARK + 30) * 4 / 1024;

static constexpr unsigned int IMU_BMX_TEMP_DIVIDER = 1;

// IMU axis rotation
static const Boardcore::AxisOrthoOrientation BMX160_AXIS_ROTATION = {
    Boardcore::Direction::NEGATIVE_Z, Boardcore::Direction::POSITIVE_Y};

static constexpr char BMX160_CORRECTION_PARAMETERS_FILE[30] =
    "/sd/bmx160_params.csv";

static constexpr unsigned int SAMPLE_PERIOD_MAG_LIS = 15;
static constexpr Boardcore::LIS3MDL::ODR MAG_LIS_ODR_ENUM =
    Boardcore::LIS3MDL::ODR_80_HZ;
static constexpr Boardcore::LIS3MDL::FullScale MAG_LIS_FULLSCALE =
    Boardcore::LIS3MDL::FS_4_GAUSS;

static constexpr unsigned int GPS_SAMPLE_RATE   = 25;
static constexpr unsigned int GPS_SAMPLE_PERIOD = 1000 / GPS_SAMPLE_RATE;
static constexpr unsigned int GPS_BAUD_RATE     = 460800;

static constexpr float REFERENCE_VOLTAGE = 5.0;
}  // namespace SensorConfigs

}  // namespace DeathStackBoard
