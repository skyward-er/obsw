/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio, Matteo Pignataro
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

#include <drivers/adc/InternalADC.h>
#include <drivers/usart/USART.h>
#include <interfaces-impl/hwmapping.h>
#include <sensors/ADS1118/ADS1118.h>
#include <sensors/BMX160/BMX160Config.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/calibration/AxisOrientation.h>

namespace Parafoil
{

namespace SensorsConfig
{

constexpr float INTERNAL_ADC_VREF                 = 3.3;
constexpr float BATTERY_VOLTAGE_COEFF             = 5.98;
constexpr unsigned int PRESS_DIGITAL_TEMP_DIVIDER = 5;
constexpr float REFERENCE_VOLTAGE                 = 5.0;
constexpr Boardcore::InternalADC::Channel ADC_BATTERY_VOLTAGE =
    Boardcore::InternalADC::Channel::CH5;

// ADS1118 and connected sensors
constexpr Boardcore::ADS1118::ADS1118Mux ADC_CH_STATIC_PORT =
    Boardcore::ADS1118::MUX_AIN0_GND;
constexpr Boardcore::ADS1118::ADS1118DataRate ADC_DR_STATIC_PORT =
    Boardcore::ADS1118::DR_860;
constexpr Boardcore::ADS1118::ADS1118Pga ADC_PGA_STATIC_PORT =
    Boardcore::ADS1118::FSR_6_144;

constexpr Boardcore::ADS1118::ADS1118Mux ADC_CH_PITOT_PORT =
    Boardcore::ADS1118::MUX_AIN1_GND;
constexpr Boardcore::ADS1118::ADS1118DataRate ADC_DR_PITOT_PORT =
    Boardcore::ADS1118::DR_860;
constexpr Boardcore::ADS1118::ADS1118Pga ADC_PGA_PITOT_PORT =
    Boardcore::ADS1118::FSR_6_144;

constexpr Boardcore::ADS1118::ADS1118Mux ADC_CH_DPL_PORT =
    Boardcore::ADS1118::MUX_AIN2_GND;
constexpr Boardcore::ADS1118::ADS1118DataRate ADC_DR_DPL_PORT =
    Boardcore::ADS1118::DR_860;
constexpr Boardcore::ADS1118::ADS1118Pga ADC_PGA_DPL_PORT =
    Boardcore::ADS1118::FSR_6_144;

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
    Boardcore::Direction::NEGATIVE_Y, Boardcore::Direction::NEGATIVE_Z};

// Correction parameter file
constexpr char BMX160_CORRECTION_PARAMETERS_FILE[30] = "/sd/bmx160_params.csv";

// LIS magnetometer
constexpr Boardcore::LIS3MDL::ODR MAG_LIS_ODR_ENUM =
    Boardcore::LIS3MDL::ODR_80_HZ;
constexpr Boardcore::LIS3MDL::FullScale MAG_LIS_FULLSCALE =
    Boardcore::LIS3MDL::FS_4_GAUSS;

// GPS
static constexpr Boardcore::USARTInterface::Baudrate GPS_BAUD_RATE =
    Boardcore::USARTInterface::Baudrate::B460800;
static constexpr unsigned int GPS_SAMPLE_RATE = 10;

// Sampling periods and dividers
constexpr unsigned int SAMPLE_PERIOD_PRESS_DIGITAL = 15;
constexpr unsigned int SAMPLE_PERIOD_INTERNAL_ADC  = 1000;
constexpr unsigned int SAMPLE_PERIOD_INTERNAL_TEMP = 2000;
constexpr unsigned int SAMPLE_PERIOD_ADS1118       = 6;
constexpr unsigned int SAMPLE_PERIOD_MAG_LIS       = 15;
constexpr unsigned int SAMPLE_PERIOD_GPS           = 1000 / GPS_SAMPLE_RATE;

// Sample before the fifo is full, but slightly after the watermark level
// (watermark + 30) ---> high slack due to scheduler imprecision,
//                       avoid clearing the fifo before the interrupt
constexpr unsigned int SAMPLE_PERIOD_IMU_BMX =
    IMU_BMX_FIFO_FILL_TIME * (IMU_BMX_FIFO_WATERMARK + 30) * 4 / 1024;
constexpr unsigned int IMU_BMX_TEMP_DIVIDER = 1;

constexpr unsigned int PITOT_TRANSMISSION_PERIOD = 50;

// Calibration
constexpr unsigned int STATIC_PRESS_CALIB_SAMPLES_NUM = 50;
constexpr unsigned int PITOT_PRESS_CALIB_SAMPLES_NUM  = 500;
constexpr float STATIC_PRESS_MOVING_AVG_COEFF         = 0.95;
constexpr unsigned int CALIBRATION_DURATION           = 2000;

constexpr float LOAD_CELL1_OFFSET       = -353697.5;
constexpr float LOAD_CELL1_SCALE        = 3.0 / 1300000.0;
constexpr float LOAD_CELL_SAMPLE_PERIOD = 1.0 / 80.0;
}  // namespace SensorsConfig

}  // namespace Parafoil
