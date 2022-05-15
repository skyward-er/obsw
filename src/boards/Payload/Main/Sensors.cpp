/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
#include "Sensors.h"

#include <Payload/Configs/SensorsConfig.h>
#include <Payload/Payload.h>
#include <drivers/interrupt/external_interrupts.h>
#include <miosix.h>
#include <sensors/SensorInfo.h>

using namespace Boardcore;
using namespace std;

namespace Payload
{
Sensors::Sensors(SPIBusInterface& spiBus, TaskScheduler* scheduler)
    : spiBus(spiBus)
{
    // Register the SDlogger
    SDlogger = &Logger::getInstance();

    // Add the sensors to the map ordering them by increasing period
    adcADS1118Init();
    magnetometerLIS3MDLInit();
    imuBMX160Init();
    correctedImuBMX160Init();
    digitalPressureInit();
    pitotPressureInit();
    dplVanePressureInit();
    staticPortPressureInit();
    gpsUbloxInit();
    internalAdcInit();
    batteryVoltageInit();

    // Now create the sensor manager with all the inserted sensors inside the
    // map
    sensorManager = new SensorManager(sensorsMap, scheduler);
}

Sensors::~Sensors()
{
    delete internalAdc;
    delete batteryVoltage;
    delete digitalPressure;
    delete adcADS1118;
    delete dplVanePressure;
    delete staticPortPressure;
    delete pitotPressure;
    delete imuBMX160;
    delete correctedImuBMX160;
    delete magnetometerLIS3MDL;
    delete gpsUblox;
}

bool Sensors::start()
{
    // BMX160 IMU enable the external interrupt on the correct pin
    miosix::GpioPin interruptPin = miosix::sensors::bmx160::intr::getPin();
    enableExternalInterrupt(interruptPin.getPort(), interruptPin.getNumber(),
                            InterruptTrigger::FALLING_EDGE, 0);

    // Start the GPS
    // TODO uncomment this line with the SERIAL driver
    // gpsUblox->start();

    // Start all the sensors, record the result and log it
    bool startResult = sensorManager->start();

    if (!startResult)
    {
        updateSensorsStatus();
    }

    // Log the sensors status
    SDlogger->log(status);
    return startResult;
}

void Sensors::calibrate()
{
    // Calibrate the IMU and log the biases result
    correctedImuBMX160->calibrate();
    SDlogger->log(correctedImuBMX160->getGyroscopeBiases());

    // Mean some digital pressure samples to calibrate the analog sensors
    float mean = 0;
    for (unsigned int i = 0; i < PRESS_STATIC_CALIB_SAMPLES_NUM; i++)
    {
        Thread::sleep(PRESS_DIGITAL_SAMPLE_PERIOD);
        mean += digitalPressure->getLastSample().pressure;
    }
    staticPortPressure->setReferencePressure(mean /
                                             PRESS_STATIC_CALIB_SAMPLES_NUM);
    staticPortPressure->calibrate();

    // Wait for differential and static barometers calibration
    // TODO check the OR expression, it used to be an AND (?)
    while (pitotPressure->isCalibrating() ||
           staticPortPressure->isCalibrating())
    {
        Thread::sleep(10);
    }
}

void Sensors::internalAdcInit()
{
    // Set the internal adc
    internalAdc = new InternalADC(ADC3, INTERNAL_ADC_VREF);
    internalAdc->enableChannel(ADC_BATTERY_VOLTAGE);

    // Create the sensor info
    SensorInfo info("InternalADC", INTERNAL_ADC_SAMPLE_PERIOD,
                    bind(&Sensors::internalAdcCallback, this));
    sensorsMap.emplace(make_pair(internalAdc, info));

    LOG_INFO(logger, "InternalADC setup done!");
}

void Sensors::batteryVoltageInit()
{
    // Crate a function that calls the internal ADC to read the battery voltage
    function<ADCData()> readVoltage(
        bind(&InternalADC::getVoltage, internalAdc, ADC_BATTERY_VOLTAGE));

    // Now i create a battery voltage sensor that uses the internal ADC
    batteryVoltage =
        new BatteryVoltageSensor(readVoltage, BATTERY_VOLTAGE_COEFF);

    // Create the sensor info
    SensorInfo info("BatterySensor", INTERNAL_ADC_SAMPLE_PERIOD,
                    bind(&Sensors::batteryVoltageCallback, this));
    sensorsMap.emplace(make_pair(batteryVoltage, info));

    LOG_INFO(logger, "Battery voltage sensor setup done!");
}

void Sensors::digitalPressureInit()
{
    // Setup the SPI
    SPIBusConfig config{};
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Set the digital barometer
    miosix::GpioPin cs = miosix::sensors::ms5803::cs::getPin();
    digitalPressure =
        new MS5803(spiBus, cs, config, PRESS_DIGITAL_TEMP_DIVIDER);

    // Create the sensor info
    SensorInfo info("DigitalBarometer", PRESS_DIGITAL_SAMPLE_PERIOD,
                    bind(&Sensors::digitalPressureCallback, this));
    sensorsMap.emplace(make_pair(digitalPressure, info));

    LOG_INFO(logger, "MS5803 pressure sensor setup done!");
}

}  // namespace Payload