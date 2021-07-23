/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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

#include <Common.h>
#include <drivers/HardwareTimer.h>
#include <drivers/interrupt/external_interrupts.h>
#include <drivers/spi/SPIDriver.h>
#include <miosix.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/BMX160/BMX160Config.h>
#include <sensors/calibration/SixParameterCalibration.h>
#include <sensors/calibration/SoftIronCalibration.h>

#include <fstream>
#include <iostream>

#include "Sensors/BMX160Calibrator.h"

using namespace miosix;

SPIBus bus(SPI1);

BMX160* sensor = nullptr;
uint32_t tick  = 0;

HardwareTimer<uint32_t> hrclock{
    TIM5, TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    tick = hrclock.toIntMicroSeconds(hrclock.tick());
    if (sensor)
    {
        sensor->IRQupdateTimestamp(tick);
    }
}

void waitForInput()
{

    char input[100];

    printf("To continue enter 'next'...");
    fgets(input, 100, stdin);
}

constexpr unsigned sleepTime = 25, accelSamples = 200, gyroSamples = 200,
                   magnetoSamples = 500;

constexpr unsigned numOrientations = 6;
AxisOrthoOrientation orientations[numOrientations]{
    {Direction::POSITIVE_X, Direction::POSITIVE_Y},
    {Direction::POSITIVE_Y, Direction::POSITIVE_Z},
    {Direction::POSITIVE_Z, Direction::POSITIVE_X},
    {Direction::POSITIVE_Y, Direction::POSITIVE_X},
    {Direction::POSITIVE_Z, Direction::POSITIVE_Y},
    {Direction::POSITIVE_X, Direction::POSITIVE_Z},
};

enum _ExecutionMode : uint8_t
{
    ACCEL   = 0x1,
    MAGNETO = 0x2,
    GYRO    = 0x4,
    ALL     = ACCEL | MAGNETO | GYRO
} executionMode;

int main()
{
    TimestampTimer::enableTimestampTimer();

    {
        miosix::FastInterruptDisableLock _lock;

        // Enable TIM5 and SPI3 bus
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    };

    hrclock.setPrescaler(382);
    hrclock.start();

    enableExternalInterrupt(GPIOE_BASE, 5, InterruptTrigger::FALLING_EDGE);

    BMX160Config config;
    config.fifo_mode      = BMX160Config::FifoMode::HEADER;
    config.fifo_int       = BMX160Config::FifoInt::PIN_INT1;
    config.mag_odr        = BMX160Config::Odr::HZ_50;
    config.acc_odr        = BMX160Config::Odr::HZ_1600;
    config.gyr_odr        = BMX160Config::Odr::HZ_1600;
    config.acc_range      = BMX160Config::AccRange::G_16;
    config.gyr_range      = BMX160Config::GyrRange::DEG_2000;
    config.fifo_watermark = 100;
    config.temp_divider   = 1;

    sensor = new BMX160(bus, miosix::sensors::bmx160::cs::getPin(), config);
    sensor->init();

    BMX160CorrectionParameters generatedParams;

    printf("-------------------------------------");
    printf("Welcome to the calibration procedure!");
    printf("For which sensor I have to generate the parameters?\n");
    printf("a -> accelerometer\n");
    printf("m -> magnetometer\n");
    printf("g -> gyroscope\n");
    printf("* -> all of them\n\n");

    char input[50];

    printf("Choose one...");
    fgets(input, 50, stdin);

    switch (input[0])
    {
        case 'a':
            executionMode = ACCEL;
            break;

        case 'm':
            executionMode = MAGNETO;
            break;

        case 'g':
            executionMode = GYRO;
            break;

        case 't':
            executionMode = ALL;
            break;

        default:
            printf("Invalid option.\n");
            return 1;
    }

    if (executionMode & ACCEL)
    {
        auto* model =
            new SixParameterCalibration<AccelerometerData,
                                        accelSamples * numOrientations>;
        model->setReferenceVector({0, 0, 1});

        printf("Now I will calibrate the accelerometer.\n");
        for (unsigned i = 0; i < numOrientations; i++)
        {
            printf(
                "Orientation n.%d/%d\nPlease rotate the board so that "
                "X axis is towards %s, and Y axis toward %s.\n",
                i, numOrientations,
                humanFriendlyDirection[static_cast<uint8_t>(
                    orientations[i].xAxis)],
                humanFriendlyDirection[static_cast<uint8_t>(
                    orientations[i].yAxis)]);
            waitForInput();

            for (unsigned x = 0; x < accelSamples; x++)
            {
                Thread::sleep(sleepTime);

                sensor->sample();
                uint8_t size = sensor->getLastFifoSize();
                auto data    = sensor->getFifoElement(size - 1);
                model->feed(data, orientations[i]);

                printf("Feeding sample: x = %f, y = %f, z = %f\n", data.accel_x,
                       data.accel_y, data.accel_z);
            }
        }

        printf("Computing the result....");
        auto corrector = model->computeResult();

        Matrix<float, 3, 2> m;
        corrector >> m;
        corrector >> generatedParams.accelParams;

        printf("b: the bias vector\n");
        printf("b = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n", m(0, 1), m(1, 1),
               m(2, 1));

        printf("M: the matrix to be multiplied to the input vector\n");

        printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", m(0, 0), 0.f, 0.f);
        printf("M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, m(1, 0), 0.f);
        printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, 0.f, m(2, 0));

        delete model;
    }

    if (executionMode & GYRO)
    {
        auto* model = new BiasCalibration<GyroscopeData>;
        model->setReferenceVector({0, 0, 1});

        printf("Now I will calibrate the gyroscope.\n");

        for (unsigned i = 0; i < numOrientations; i++)
        {
            printf(
                "Orientation n.%d/%d\nPlease rotate the board so that "
                "X axis is towards %s, and Y axis toward %s.\n",
                i, numOrientations,
                humanFriendlyDirection[static_cast<uint8_t>(
                    orientations[i].xAxis)],
                humanFriendlyDirection[static_cast<uint8_t>(
                    orientations[i].yAxis)]);
            waitForInput();

            for (unsigned x = 0; x < gyroSamples; x++)
            {
                Thread::sleep(sleepTime);

                sensor->sample();
                uint8_t size = sensor->getLastFifoSize();
                auto data    = sensor->getFifoElement(size - 1);
                model->feed(data, orientations[i]);

                printf("Feeding sample: x = %f, y = %f, z = %f\n", data.gyro_x,
                       data.gyro_y, data.gyro_z);
            }
        }

        printf("Computing the result....\n");
        auto corrector = model->computeResult();

        Vector3f b;
        corrector >> b;

        printf("b: the bias vector\n");
        printf("b = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n", b(0), b(1),
               b(2));

        delete model;
    }

    if (executionMode & MAGNETO)
    {
        auto* model = new SoftIronCalibration<magnetoSamples>;

        printf("Now I will calibrate the gyroscope.\n");
        printf(
            "Please rotate the gyroscope in the most possible different "
            "orientations after entering 'next'.\n");
        waitForInput();

        for (unsigned x = 0; x < magnetoSamples; x++)
        {
            Thread::sleep(sleepTime);

            sensor->sample();
            uint8_t size = sensor->getLastFifoSize();
            auto data    = sensor->getFifoElement(size - 1);
            model->feed(data);

            printf("Feeding sample: x = %f, y = %f, z = %f\n", data.mag_x,
                   data.mag_y, data.mag_z);
        }

        printf("Computing the result....");
        auto corrector = model->computeResult();

        Matrix<float, 3, 2> m;
        corrector >> m;
        corrector >> generatedParams.magnetoParams;

        printf("b: the bias vector\n");
        printf("b = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n", m(0, 1), m(1, 1),
               m(2, 1));

        printf("M: the matrix to be multiplied to the input vector\n");

        printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", m(0, 0), 0.f, 0.f);
        printf("M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, m(1, 0), 0.f);
        printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, 0.f, m(2, 0));

        delete model;
    }

    printf("Now I am writing params on the SD Card.\n");

    std::cout << BMX160CorrectionParameters::header() << std::endl;
    generatedParams.print(std::cout);
    std::cout << std::endl << "Done." << std::endl;

    {
        // Writing to file
        std::ofstream paramsFile(
            DeathStackBoard::SensorConfigs::BMX160_CORRECTION_PARAMETERS_FILE);
        paramsFile << BMX160CorrectionParameters::header() << std::endl;
        generatedParams.print(paramsFile);
    }

    for (;;)
        ;
}
