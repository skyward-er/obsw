/* Copyright (c) 2015-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio, Alberto Nidasio
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
#include <drivers/usart/USART.h>
#include <sensors/ADS131M04/ADS131M04.h>
#include <sensors/BMX160/BMX160Config.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/calibration/AxisOrientation.h>

namespace Main
{

namespace SensorsConfig
{

// Internal ADC and connected sensors
constexpr float INTERNAL_ADC_VREF = 3.3;
constexpr Boardcore::InternalADC::Channel INTERNAL_ADC_CH_5V_CURRENT =
    Boardcore::InternalADC::Channel::CH11;
constexpr Boardcore::InternalADC::Channel INTERNAL_ADC_CH_CUTTER_CURRENT =
    Boardcore::InternalADC::Channel::CH12;
constexpr Boardcore::InternalADC::Channel INTERNAL_ADC_CH_CUTTER_SENSE =
    Boardcore::InternalADC::Channel::CH15;

constexpr float CUTTER_SENSING_MOV_MEAN_COEFF = 1.0 / 100.0;
constexpr float CUTTER_SENSING_THRESHOLD      = 0.1 + 1.3 / 2;

// ADS131M04 and connected sensors
constexpr Boardcore::ADS131M04::Channel ADC_CH_STATIC_PORT =
    Boardcore::ADS131M04::Channel::CHANNEL_0;
constexpr Boardcore::ADS131M04::Channel ADC_CH_DPL_PORT =
    Boardcore::ADS131M04::Channel::CHANNEL_1;
constexpr Boardcore::ADS131M04::Channel ADC_CH_LOAD_CELL =
    Boardcore::ADS131M04::Channel::CHANNEL_2;
constexpr Boardcore::ADS131M04::Channel ADC_CH_VBAT =
    Boardcore::ADS131M04::Channel::CHANNEL_3;

constexpr float REFERENCE_VOLTAGE        = 5.0;
constexpr float BATTERY_VOLTAGE_COEFF    = 10.88;
constexpr float BATTERY_MIN_SAFE_VOLTAGE = 10.5;  // [V]

// Load cell
constexpr float LOAD_CELL_MV_TO_V           = 2;   // [mV/V]
constexpr unsigned int LOAD_CELL_FULL_SCALE = 10;  // [Kg]
constexpr float LOAD_CELL_SUPPLY_VOLTAGE    = 5;   // [V]

// BMX160
constexpr Boardcore::BMX160Config::AccelerometerRange IMU_BMX_ACC_FSR_ENUM =
    Boardcore::BMX160Config::AccelerometerRange::G_16;
constexpr Boardcore::BMX160Config::GyroscopeRange IMU_BMX_GYRO_FSR_ENUM =
    Boardcore::BMX160Config::GyroscopeRange::DEG_1000;
constexpr unsigned int IMU_BMX_ACC_GYRO_ODR = 1600;
constexpr Boardcore::BMX160Config::OutputDataRate IMU_BMX_ACC_GYRO_ODR_ENUM =
    Boardcore::BMX160Config::OutputDataRate::HZ_1600;
constexpr unsigned int IMU_BMX_MAG_ODR = 100;
constexpr Boardcore::BMX160Config::OutputDataRate IMU_BMX_MAG_ODR_ENUM =
    Boardcore::BMX160Config::OutputDataRate::HZ_100;

constexpr unsigned int IMU_BMX_FIFO_HEADER_SIZE = 1;
constexpr unsigned int IMU_BMX_ACC_DATA_SIZE    = 6;
constexpr unsigned int IMU_BMX_GYRO_DATA_SIZE   = 6;
constexpr unsigned int IMU_BMX_MAG_DATA_SIZE    = 8;

constexpr unsigned int IMU_BMX_FIFO_WATERMARK = 40;

// MPU9250
constexpr Boardcore::MPU9250::AccelFSR IMU_MPU_ACC_FSR =
    Boardcore::MPU9250::AccelFSR::ACCEL_FSR_16G;
constexpr Boardcore::MPU9250::GyroFSR IMU_MPU_GYRO_FSR =
    Boardcore::MPU9250::GyroFSR::GYRO_FSR_1000DPS;

// How many bytes go into the fifo each second
constexpr unsigned int IMU_BMX_FIFO_FILL_RATE =
    IMU_BMX_ACC_GYRO_ODR * (IMU_BMX_FIFO_HEADER_SIZE + IMU_BMX_ACC_DATA_SIZE +
                            IMU_BMX_GYRO_DATA_SIZE) +
    IMU_BMX_MAG_ODR * (IMU_BMX_MAG_DATA_SIZE + IMU_BMX_FIFO_HEADER_SIZE);

// How long does it take for the bmx fifo to fill up
constexpr unsigned int IMU_BMX_FIFO_FILL_TIME =
    1024 * 1000 / IMU_BMX_FIFO_FILL_RATE;

// Axis rotation
static const Boardcore::AxisOrthoOrientation BMX160_AXIS_ROTATION = {
    Boardcore::Direction::POSITIVE_Z, Boardcore::Direction::NEGATIVE_Y};

// GPS
constexpr unsigned int GPS_SAMPLE_RATE = 5;

// Sampling periods and dividers
constexpr unsigned int SAMPLE_PERIOD_ADC_ADS131M04 = 1;
constexpr unsigned int SAMPLE_PERIOD_INTERNAL_ADC  = 1;
constexpr unsigned int SAMPLE_PERIOD_INTERNAL_TEMP = 200;
constexpr unsigned int SAMPLE_PERIOD_PRESS_DIGITAL = 15;
constexpr unsigned int TEMP_DIVIDER_PRESS_DIGITAL  = 5;
constexpr unsigned int SAMPLE_PERIOD_IMU_MPU       = 5;
constexpr unsigned int SAMPLE_PERIOD_GPS           = 1000 / GPS_SAMPLE_RATE;
constexpr unsigned int SAMPLE_PERIOD_VN100         = 5;

// Sample before the fifo is full, but slightly after the watermark level
// (watermark + 30) ---> high slack due to scheduler imprecision,
//                       avoid clearing the fifo before the interrupt
constexpr unsigned int SAMPLE_PERIOD_IMU_BMX =
    IMU_BMX_FIFO_FILL_TIME * (IMU_BMX_FIFO_WATERMARK + 30) * 4 / 1024;
constexpr unsigned int IMU_BMX_TEMP_DIVIDER = 1;

constexpr unsigned int CALIBRATION_DURATION = 2000;

}  // namespace SensorsConfig

}  // namespace Main
