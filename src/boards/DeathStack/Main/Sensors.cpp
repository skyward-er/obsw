/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <ApogeeDetectionAlgorithm/ADAController.h>
#include <DeathStack.h>
#include <Debug.h>
#include <LoggerService/LoggerService.h>
#include <TimestampTimer.h>
#include <configs/SensorsConfig.h>
#include <drivers/interrupt/external_interrupts.h>
#include <interfaces-impl/hwmapping.h>
#include <sensors/Sensor.h>
#include <utils/aero/AeroUtils.h>

#include <functional>
#include <utility>

using std::bind;
using std::function;

// BMX160 Watermark interrupt
void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    using namespace DeathStackBoard;

    if (DeathStack::getInstance()->sensors->imu_bmx160 != nullptr)
    {
        DeathStack::getInstance()->sensors->imu_bmx160->IRQupdateTimestamp(
            TimestampTimer::getTimestamp());
    }
}

namespace DeathStackBoard
{
using namespace SensorConfigs;

Sensors::Sensors(SPIBusInterface& spi1_bus, TaskScheduler* scheduler)
    : spi1_bus(spi1_bus)
{
    // sensors are added to the map ordered by increasing period
    ADS1118Init();
    magLISinit();
    imuBMXInit();
    imuBMXWithCorrectionInit();
    pressDigiInit();
    pressPitotInit();
    pressDPLVaneInit();
    pressStaticInit();
    gpsUbloxInit();
    internalAdcInit();
    batteryVoltageInit();
#ifdef HARDWARE_IN_THE_LOOP
    hilImuInit();
    hilBarometerInit();
    hilGpsInit();
#elif defined(USE_MOCK_SENSORS)
    mockSensorsInit();
#endif

    sensor_manager = new SensorManager(scheduler, sensors_map);
}

Sensors::~Sensors()
{
    delete imu_bmx160;
    delete press_digital;
    delete gps_ublox;
    delete internal_adc;
    delete battery_voltage;
    delete adc_ads1118;
    delete press_pitot;
    delete press_dpl_vane;
    delete press_static_port;
    delete mag_lis3mdl;
#ifdef HARDWARE_IN_THE_LOOP
    delete hil_imu;
    delete hil_baro;
    delete hil_gps;
#elif defined(USE_MOCK_SENSORS)
    delete mock_imu;
    delete mock_baro;
    delete mock_gps;
#endif

    sensor_manager->stop();
    delete sensor_manager;
}

bool Sensors::start()
{
    GpioPin int_pin = miosix::sensors::bmx160::intr::getPin();
    enableExternalInterrupt(int_pin.getPort(), int_pin.getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    gps_ublox->start();

    bool sm_start_result = sensor_manager->start();

    // if not init ok, set failing sensors in sensors status
    if (!sm_start_result)
    {
        updateSensorsStatus();
    }

    LoggerService::getInstance()->log(status);

    return sm_start_result;
}

void Sensors::calibrate()
{
    imu_bmx160_with_correction->calibrate();
    LoggerService::getInstance()->log(
        imu_bmx160_with_correction->getGyroscopeBiases());

    press_pitot->calibrate();

    press_static_port->setReferencePressure(
        press_digital->getLastSample().press);
    press_static_port->calibrate();

    // wait differential and static barometers calibration end
    while (press_pitot->isCalibrating() && press_static_port->isCalibrating())
    {
        Thread::sleep(10);
    }
}

#ifdef USE_MOCK_SENSORS
void Sensors::signalLiftoff()
{
    mock_imu->signalLiftoff();
    mock_baro->signalLiftoff();
    mock_gps->signalLiftoff();
}
#endif

void Sensors::internalAdcInit()
{
    internal_adc = new InternalADC(*ADC3, INTERNAL_ADC_VREF);

    internal_adc->enableChannel(ADC_BATTERY_VOLTAGE);

    SensorInfo info("InternalADC", SAMPLE_PERIOD_INTERNAL_ADC,
                    bind(&Sensors::internalAdcCallback, this), false, true);
    sensors_map.emplace(std::make_pair(internal_adc, info));

    LOG_INFO(log, "InternalADC setup done!");
}

void Sensors::batteryVoltageInit()
{
    function<ADCData()> voltage_fun(
        bind(&InternalADC::getVoltage, internal_adc, ADC_BATTERY_VOLTAGE));
    battery_voltage =
        new BatteryVoltageSensor(voltage_fun, BATTERY_VOLTAGE_COEFF);

    SensorInfo info("BatterySensor", SAMPLE_PERIOD_INTERNAL_ADC,
                    bind(&Sensors::batteryVoltageCallback, this), false, true);

    sensors_map.emplace(std::make_pair(battery_voltage, info));

    LOG_INFO(log, "Battery voltage sensor setup done!");
}

void Sensors::pressDigiInit()
{
    SPIBusConfig spi_cfg{};
    spi_cfg.clock_div = SPIClockDivider::DIV16;

    press_digital = new MS5803(spi1_bus, miosix::sensors::ms5803::cs::getPin(),
                               spi_cfg, TEMP_DIVIDER_PRESS_DIGITAL);

    SensorInfo info("DigitalBarometer", SAMPLE_PERIOD_PRESS_DIGITAL,
                    bind(&Sensors::pressDigiCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_digital, info));

    LOG_INFO(log, "MS5803 pressure sensor setup done!");
}

void Sensors::ADS1118Init()
{
    SPIBusConfig spi_cfg = ADS1118::getDefaultSPIConfig();
    spi_cfg.clock_div    = SPIClockDivider::DIV64;

    ADS1118::ADS1118Config ads1118Config = ADS1118::ADS1118_DEFAULT_CONFIG;
    ads1118Config.bits.mode = ADS1118::ADS1118Mode::CONTIN_CONV_MODE;

    adc_ads1118 = new ADS1118(spi1_bus, miosix::sensors::ads1118::cs::getPin(),
                              ads1118Config, spi_cfg);

    adc_ads1118->enableInput(ADC_CH_STATIC_PORT, ADC_DR_STATIC_PORT,
                             ADC_PGA_STATIC_PORT);

    adc_ads1118->enableInput(ADC_CH_PITOT_PORT, ADC_DR_PITOT_PORT,
                             ADC_PGA_PITOT_PORT);
    adc_ads1118->enableInput(ADC_CH_DPL_PORT, ADC_DR_DPL_PORT,
                             ADC_PGA_DPL_PORT);

    adc_ads1118->enableInput(ADC_CH_VREF, ADC_DR_VREF, ADC_PGA_VREF);

    SensorInfo info("ADS1118", SAMPLE_PERIOD_ADC_ADS1118,
                    bind(&Sensors::ADS1118Callback, this), false, true);
    sensors_map.emplace(std::make_pair(adc_ads1118, info));

    LOG_INFO(log, "ADS1118 setup done!");
}

void Sensors::pressPitotInit()
{
    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_PITOT_PORT));
    press_pitot = new SSCDRRN015PDA(voltage_fun, REFERENCE_VOLTAGE,
                                    PRESS_PITOT_CALIB_SAMPLES_NUM);

    SensorInfo info("DiffBarometer", SAMPLE_PERIOD_PRESS_PITOT,
                    bind(&Sensors::pressPitotCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_pitot, info));

    LOG_INFO(log, "Diff pressure sensor setup done!");
}

void Sensors::pressDPLVaneInit()
{
    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_DPL_PORT));
    press_dpl_vane = new SSCDANN030PAA(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info("DeploymentBarometer", SAMPLE_PERIOD_PRESS_DPL,
                    bind(&Sensors::pressDPLVaneCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_dpl_vane, info));

    LOG_INFO(log, "DPL pressure sensor setup done!");
}

void Sensors::pressStaticInit()
{
    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_STATIC_PORT));
    press_static_port = new MPXHZ6130A(voltage_fun, REFERENCE_VOLTAGE,
                                       PRESS_STATIC_CALIB_SAMPLES_NUM,
                                       PRESS_STATIC_MOVING_AVG_COEFF);

    SensorInfo info("StaticPortsBarometer", SAMPLE_PERIOD_PRESS_STATIC,
                    bind(&Sensors::pressStaticCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_static_port, info));

    LOG_INFO(log, "Static pressure sensor setup done!");
}

void Sensors::imuBMXInit()
{
    SPIBusConfig spi_cfg;
    spi_cfg.clock_div = SPIClockDivider::DIV8;

    BMX160Config bmx_config;
    bmx_config.fifo_mode      = BMX160Config::FifoMode::HEADER;
    bmx_config.fifo_watermark = IMU_BMX_FIFO_WATERMARK;
    bmx_config.fifo_int       = BMX160Config::FifoInterruptPin::PIN_INT1;

    bmx_config.temp_divider = IMU_BMX_TEMP_DIVIDER;

    bmx_config.acc_range = IMU_BMX_ACC_FULLSCALE_ENUM;
    bmx_config.gyr_range = IMU_BMX_GYRO_FULLSCALE_ENUM;

    bmx_config.acc_odr = IMU_BMX_ACC_GYRO_ODR_ENUM;
    bmx_config.gyr_odr = IMU_BMX_ACC_GYRO_ODR_ENUM;
    bmx_config.mag_odr = IMU_BMX_MAG_ODR_ENUM;

    bmx_config.gyr_unit = BMX160Config::GyroscopeMeasureUnit::RAD;

    imu_bmx160 = new BMX160(spi1_bus, miosix::sensors::bmx160::cs::getPin(),
                            bmx_config, spi_cfg);

    SensorInfo info("BMX160", SAMPLE_PERIOD_IMU_BMX,
                    bind(&Sensors::imuBMXCallback, this), false, true);

    sensors_map.emplace(std::make_pair(imu_bmx160, info));

    LOG_INFO(log, "BMX160 Setup done!");
}

void Sensors::imuBMXWithCorrectionInit()
{
    // Read the correction parameters
    BMX160CorrectionParameters correctionParameters =
        BMX160WithCorrection::readCorrectionParametersFromFile(
            BMX160_CORRECTION_PARAMETERS_FILE);

    // Print the calibration parameters
    TRACE("[Sensors] Current accelerometer bias vector\n");
    TRACE("[Sensors] b = [    % 2.5f    % 2.5f    % 2.5f    ]\n",
          correctionParameters.accelParams(0, 1),
          correctionParameters.accelParams(1, 1),
          correctionParameters.accelParams(2, 1));
    TRACE("[Sensors] Matrix to be multiplied to the input vector\n");
    TRACE("[Sensors]     |    % 2.5f    % 2.5f    % 2.5f    |\n",
          correctionParameters.accelParams(0, 0), 0.f, 0.f);
    TRACE("[Sensors] M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f,
          correctionParameters.accelParams(1, 0), 0.f);
    TRACE("[Sensors]     |    % 2.5f    % 2.5f    % 2.5f    |\n\n", 0.f, 0.f,
          correctionParameters.accelParams(2, 0));
    TRACE("[Sensors] Current magnetometer bias vector\n");
    TRACE("[Sensors] b = [    % 2.5f    % 2.5f    % 2.5f    ]\n",
          correctionParameters.magnetoParams(0, 1),
          correctionParameters.magnetoParams(1, 1),
          correctionParameters.magnetoParams(2, 1));
    TRACE("[Sensors] Matrix to be multiplied to the input vector\n");
    TRACE("[Sensors]     |    % 2.5f    % 2.5f    % 2.5f    |\n",
          correctionParameters.magnetoParams(0, 0), 0.f, 0.f);
    TRACE("[Sensors] M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f,
          correctionParameters.magnetoParams(1, 0), 0.f);
    TRACE("[Sensors]     |    % 2.5f    % 2.5f    % 2.5f    |\n\n", 0.f, 0.f,
          correctionParameters.magnetoParams(2, 0));
    TRACE(
        "[Sensors] The current minimun number of gyroscope samples for "
        "calibration is %d\n",
        correctionParameters.minGyroSamplesForCalibration);

    imu_bmx160_with_correction = new BMX160WithCorrection(
        imu_bmx160, correctionParameters, BMX160_AXIS_ROTATION);

    SensorInfo info("BMX160WithCorrection", SAMPLE_PERIOD_IMU_BMX,
                    bind(&Sensors::imuBMXWithCorrectionCallback, this), false,
                    true);

    sensors_map.emplace(std::make_pair(imu_bmx160_with_correction, info));

    LOG_INFO(log, "BMX160WithCorrection Setup done!");
}

void Sensors::magLISinit()
{
    SPIBusConfig busConfig;
    busConfig.clock_div = SPIClockDivider::DIV32;

    LIS3MDL::Config config;
    config.odr                = MAG_LIS_ODR_ENUM;
    config.scale              = MAG_LIS_FULLSCALE;
    config.temperatureDivider = 1;

    mag_lis3mdl = new LIS3MDL(spi1_bus, miosix::sensors::lis3mdl::cs::getPin(),
                              busConfig, config);

    SensorInfo info("LIS3MDL", SAMPLE_PERIOD_MAG_LIS,
                    bind(&Sensors::magLISCallback, this), false, true);

    sensors_map.emplace(std::make_pair(mag_lis3mdl, info));

    LOG_INFO(log, "LIS3MDL Setup done!");
}

void Sensors::gpsUbloxInit()
{
    gps_ublox = new UbloxGPS(GPS_BAUD_RATE, GPS_SAMPLE_RATE);

    SensorInfo info("UbloxGPS", GPS_SAMPLE_PERIOD,
                    bind(&Sensors::gpsUbloxCallback, this), false, true);

    sensors_map.emplace(std::make_pair(gps_ublox, info));

    LOG_INFO(log, "Ublox GPS Setup done!");
}

void Sensors::internalAdcCallback()
{
    // LoggerService::getInstance()->log(internal_adc->getLastSample());
}

void Sensors::batteryVoltageCallback()
{
    static float v = battery_voltage->getLastSample().bat_voltage;
    if (v < BATTERY_MIN_SAFE_VOLTAGE)
    {
        battery_critical_counter++;
        // every 30 seconds to avoid filling the log (if debug disabled)
        if (battery_critical_counter % 30 == 0)
        {
            LOG_CRIT(log, "*** LOW BATTERY *** ---> Voltage = {:02f}", v);
            battery_critical_counter = 0;
        }
    }

    LoggerService::getInstance()->log(battery_voltage->getLastSample());
}

#ifdef HARDWARE_IN_THE_LOOP
void Sensors::hilBarometerInit()
{
    HILTransceiver* simulator = HIL::getInstance()->simulator;

    hil_baro = new HILBarometer(simulator, N_DATA_BARO);

    SensorInfo info_baro("HILBaro", HIL_BARO_PERIOD,
                         bind(&Sensors::hilBaroCallback, this), false, true);

    sensors_map.emplace(std::make_pair(hil_baro, info_baro));

    LOG_INFO(log, "HIL barometer setup done!");
}
void Sensors::hilImuInit()
{
    HILTransceiver* simulator = HIL::getInstance()->simulator;

    hil_imu = new HILImu(simulator, N_DATA_IMU);

    SensorInfo info_imu("HILImu", HIL_IMU_PERIOD,
                        bind(&Sensors::hilIMUCallback, this), false, true);

    sensors_map.emplace(std::make_pair(hil_imu, info_imu));

    LOG_INFO(log, "HIL IMU setup done!");
}

void Sensors::hilGpsInit()
{
    HILTransceiver* simulator = HIL::getInstance()->simulator;

    hil_gps = new HILGps(simulator, N_DATA_GPS);

    SensorInfo info_gps("HILGps", HIL_GPS_PERIOD,
                        bind(&Sensors::hilGPSCallback, this), false, true);

    sensors_map.emplace(std::make_pair(hil_gps, info_gps));

    LOG_INFO(log, "HIL GPS setup done!");
}
#elif defined(USE_MOCK_SENSORS)
void Sensors::mockSensorsInit()
{
    mock_imu = new MockIMU();
    mock_baro = new MockPressureSensor();
    mock_gps = new MockGPS();

    SensorInfo info_baro("MockBaro", SAMPLE_PERIOD_PRESS_STATIC,
                         bind(&Sensors::mockBaroCallback, this), false, true);
    SensorInfo info_imu("MockIMU", SAMPLE_PERIOD_IMU_BMX,
                        bind(&Sensors::mockImuCallback, this), false, true);
    SensorInfo info_gps("MockGPS", GPS_SAMPLE_PERIOD,
                        bind(&Sensors::mockGpsCallback, this), false, true);

    sensors_map.emplace(std::make_pair(mock_baro, info_baro));
    sensors_map.emplace(std::make_pair(mock_imu, info_imu));
    sensors_map.emplace(std::make_pair(mock_gps, info_gps));

    LOG_INFO(log, "Mock sensors setup done!");
}
#endif

void Sensors::pressDigiCallback()
{
    LoggerService::getInstance()->log(press_digital->getLastSample());
}

void Sensors::ADS1118Callback()
{
    LoggerService::getInstance()->log(adc_ads1118->getLastSample());
}

void Sensors::pressPitotCallback()
{
    SSCDRRN015PDAData d = press_pitot->getLastSample();
    LoggerService::getInstance()->log(d);

    ADAReferenceValues rv =
        DeathStack::getInstance()
            ->state_machines->ada_controller->getReferenceValues();

    float rel_density = aeroutils::relDensity(
        press_digital->getLastSample().press, rv.ref_pressure, rv.ref_altitude,
        rv.ref_temperature);
    if (rel_density != 0.0f)
    {
        float airspeed = sqrtf(2 * fabs(d.press) / rel_density);

        AirSpeedPitot aspeed_data{TimestampTimer::getTimestamp(), airspeed};
        LoggerService::getInstance()->log(aspeed_data);
    }
}

void Sensors::pressDPLVaneCallback()
{
    LoggerService::getInstance()->log(press_dpl_vane->getLastSample());
}

void Sensors::pressStaticCallback()
{
    LoggerService::getInstance()->log(press_static_port->getLastSample());
}

void Sensors::imuBMXCallback()
{
    uint8_t fifo_size = imu_bmx160->getLastFifoSize();
    auto& fifo        = imu_bmx160->getLastFifo();

    LoggerService::getInstance()->log(imu_bmx160->getTemperature());

    for (uint8_t i = 0; i < fifo_size; ++i)
    {
        LoggerService::getInstance()->log(fifo.at(i));
    }

    LoggerService::getInstance()->log(imu_bmx160->getFifoStats());
}

void Sensors::imuBMXWithCorrectionCallback()
{
    LoggerService::getInstance()->log(
        imu_bmx160_with_correction->getLastSample());
}

void Sensors::magLISCallback()
{
    LoggerService::getInstance()->log(mag_lis3mdl->getLastSample());
}

void Sensors::gpsUbloxCallback()
{
    LoggerService::getInstance()->log(gps_ublox->getLastSample());
}

#ifdef HARDWARE_IN_THE_LOOP
void Sensors::hilIMUCallback()
{
    LoggerService::getInstance()->log(hil_imu->getLastSample());
}

void Sensors::hilBaroCallback()
{
    LoggerService::getInstance()->log(hil_baro->getLastSample());
}

void Sensors::hilGPSCallback()
{
    LoggerService::getInstance()->log(hil_gps->getLastSample());
}
#elif defined(USE_MOCK_SENSORS)
void Sensors::mockBaroCallback()
{
    LoggerService::getInstance()->log(mock_imu->getLastSample());
}

void Sensors::mockImuCallback()
{
    LoggerService::getInstance()->log(mock_baro->getLastSample());
}

void Sensors::mockGpsCallback()
{
    LoggerService::getInstance()->log(mock_gps->getLastSample());
}
#endif

void Sensors::updateSensorsStatus()
{
    SensorInfo info;

    info = sensor_manager->getSensorInfo(imu_bmx160);
    if (!info.is_initialized)
    {
        status.bmx160 = SensorDriverStatus::DRIVER_ERROR;
    }

    info = sensor_manager->getSensorInfo(mag_lis3mdl);
    if (!info.is_initialized)
    {
        status.lis3mdl = SensorDriverStatus::DRIVER_ERROR;
    }

    info = sensor_manager->getSensorInfo(gps_ublox);
    if (!info.is_initialized)
    {
        status.gps = SensorDriverStatus::DRIVER_ERROR;
    }

    info = sensor_manager->getSensorInfo(internal_adc);
    if (!info.is_initialized)
    {
        status.internal_adc = SensorDriverStatus::DRIVER_ERROR;
    }

    info = sensor_manager->getSensorInfo(adc_ads1118);
    if (!info.is_initialized)
    {
        status.ads1118 = SensorDriverStatus::DRIVER_ERROR;
    }
}

}  // namespace DeathStackBoard