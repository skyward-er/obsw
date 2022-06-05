/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "TMRepository.h"

#include <Main/Actuators/Actuators.h>
#include <Main/Configs/SensorsConfig.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/NavigationAttitudeSystem/NASController.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <drivers/timer/TimestampTimer.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

using namespace Boardcore;
using namespace Main::SensorConfigs;

namespace Main
{

mavlink_message_t TMRepository::packSystemTm(SystemTMList reqTm)
{
    mavlink_message_t msg;

    // Prevent preemption, MUST not yeld or use the kernel!
    miosix::PauseKernelLock kLock;

    switch (reqTm)
    {
        // System telemetries
        case SystemTMList::MAV_SYS_ID:
        {
            break;
        }
        case SystemTMList::MAV_FSM_ID:
        {
            break;
        }
        case SystemTMList::MAV_PIN_OBS_ID:
        {
            break;
        }
        case SystemTMList::MAV_LOGGER_ID:
        {
            break;
        }
        case SystemTMList::MAV_MAVLINK_STATS:
        {
            break;
        }
        case SystemTMList::MAV_TASK_STATS_ID:
        {
            break;
        }
        case SystemTMList::MAV_DPL_ID:
        {
            break;
        }
        case SystemTMList::MAV_ADA_ID:
        {
            break;
        }
        case SystemTMList::MAV_NAS_ID:
        {
            break;
        }
        case SystemTMList::MAV_CAN_ID:
        {
            break;
        }
        case SystemTMList::MAV_FLIGHT_ID:
        {
            mavlink_rocket_flight_tm_t tm;
            Sensors &sensors = Sensors::getInstance();

            tm.timestamp     = TimestampTimer::getTimestamp();
            tm.ada_state     = 0;
            tm.fmm_state     = 0;
            tm.dpl_state     = 0;
            tm.ab_state      = 0;
            tm.nas_state     = 0;
            tm.pressure_ada  = 0;
            tm.pressure_digi = sensors.ms5803->getLastSample().pressure;
            tm.pressure_static =
                sensors.staticPressure->getLastSample().pressure;
            tm.pressure_dpl   = sensors.dplPressure->getLastSample().pressure;
            tm.airspeed_pitot = sensors.pitot->getLastSample().airspeed;
            tm.msl_altitude   = 0;
            tm.ada_vert_speed = 0;
            tm.ada_vert_accel = 0;
            tm.acc_x          = sensors.bmx160->getLastSample().accelerationX;
            tm.acc_y          = sensors.bmx160->getLastSample().accelerationY;
            tm.acc_z          = sensors.bmx160->getLastSample().accelerationZ;
            tm.gyro_x      = sensors.bmx160->getLastSample().angularVelocityX;
            tm.gyro_y      = sensors.bmx160->getLastSample().angularVelocityY;
            tm.gyro_z      = sensors.bmx160->getLastSample().angularVelocityZ;
            tm.mag_x       = sensors.bmx160->getLastSample().magneticFieldX;
            tm.mag_y       = sensors.bmx160->getLastSample().magneticFieldY;
            tm.mag_z       = sensors.bmx160->getLastSample().magneticFieldZ;
            tm.gps_fix     = 0;
            tm.gps_lat     = 0;
            tm.gps_lon     = 0;
            tm.gps_alt     = 0;
            tm.vbat        = sensors.batteryVoltage->getLastSample().batVoltage;
            tm.vsupply_5v  = sensors.ads1118->getVoltage(ADC_CH_VREF).voltage;
            tm.temperature = sensors.ms5803->getLastSample().temperature;
            tm.pin_launch  = 0;
            tm.pin_nosecone = 0;
            tm.servo_sensor = 0;
            tm.ab_angle =
                Actuators::getInstance().getServoPosition(AIRBRAKES_SERVO);
            tm.ab_estimated_cd = 0;

            auto nasState    = NASController::getInstance().getNasState();
            auto orientation = SkyQuaternion::quat2eul(
                {nasState.qx, nasState.qy, nasState.qz, nasState.qw});

            tm.nas_x        = nasState.n;
            tm.nas_y        = nasState.e;
            tm.nas_z        = nasState.d;
            tm.nas_vx       = nasState.vn;
            tm.nas_vy       = nasState.ve;
            tm.nas_vz       = nasState.vd;
            tm.nas_yaw      = orientation(0);
            tm.nas_pitch    = orientation(1);
            tm.nas_roll     = orientation(2);
            tm.nas_bias0    = nasState.bx;
            tm.nas_bias1    = nasState.by;
            tm.nas_bias2    = nasState.bz;
            tm.logger_error = Logger::getInstance().getStats().lastWriteError;

            mavlink_msg_rocket_flight_tm_encode(RadioConfigs::MAV_SYSTEM_ID,
                                                RadioConfigs::MAV_COMPONENT_ID,
                                                &msg, &tm);
            break;
        }
        case SystemTMList::MAV_FLIGHT_STATS_ID:
        {
            mavlink_rocket_stats_tm_t tm;

            tm.liftoff_ts            = 0;
            tm.liftoff_max_acc_ts    = 0;
            tm.liftoff_max_acc       = 0;
            tm.max_z_speed_ts        = 0;
            tm.max_z_speed           = 0;
            tm.max_airspeed_pitot    = 0;
            tm.max_speed_altitude    = 0;
            tm.apogee_ts             = 0;
            tm.apogee_lat            = 0;
            tm.apogee_lon            = 0;
            tm.static_min_pressure   = 0;
            tm.digital_min_pressure  = 0;
            tm.ada_min_pressure      = 0;
            tm.baro_max_altitutde    = 0;
            tm.gps_max_altitude      = 0;
            tm.drogue_dpl_ts         = 0;
            tm.drogue_dpl_max_acc    = 0;
            tm.dpl_vane_max_pressure = 0;
            tm.main_dpl_altitude_ts  = 0;
            tm.main_dpl_altitude     = 0;
            tm.main_dpl_zspeed       = 0;
            tm.main_dpl_acc          = 0;
            tm.cpu_load              = CpuMeter::getCpuStats().mean;

            mavlink_msg_rocket_stats_tm_encode(RadioConfigs::MAV_SYSTEM_ID,
                                               RadioConfigs::MAV_COMPONENT_ID,
                                               &msg, &tm);
            break;
        }
        case SystemTMList::MAV_SENSORS_STATE_ID:
        {
            break;
        }

        default:
        {
            LOG_DEBUG(logger, "Unknown telemetry id: {}", reqTm);
            mavlink_nack_tm_t nack;
            nack.recv_msgid = 0;
            nack.seq_ack    = 0;
            mavlink_msg_nack_tm_encode(RadioConfigs::MAV_SYSTEM_ID,
                                       RadioConfigs::MAV_COMPONENT_ID, &msg,
                                       &nack);
            break;
        }
    }

    return msg;
}

mavlink_message_t TMRepository::packSensorsTm(SensorsTMList reqTm)
{
    mavlink_message_t msg;

    // Prevent preemption, MUST not yeld or use the kernel!
    miosix::PauseKernelLock kLock;

    switch (reqTm)
    {
        // Sensors telemetries
        case SensorsTMList::MAV_GPS_ID:
            break;
        case SensorsTMList::MAV_BMX160_ID:
            break;
        case SensorsTMList::MAV_VN100_ID:
            break;
        case SensorsTMList::MAV_MPU9250_ID:
            break;
        case SensorsTMList::MAV_CURRENT_SENSE_ID:
            break;
        case SensorsTMList::MAV_LIS3MDL_ID:
            break;
        case SensorsTMList::MAV_DPL_PRESS_ID:
            break;
        case SensorsTMList::MAV_STATIC_PRESS_ID:
            break;
        case SensorsTMList::MAV_PITOT_PRESS_ID:
            break;
        case SensorsTMList::MAV_BATTERY_VOLTAGE_ID:
            break;
        case SensorsTMList::MAV_STRAIN_GAUGE_ID:
            break;

        default:
            LOG_DEBUG(logger, "Unknown telemetry id: {}", reqTm);
            mavlink_nack_tm_t nack;
            nack.recv_msgid = 0;
            nack.seq_ack    = 0;
            mavlink_msg_nack_tm_encode(RadioConfigs::MAV_SYSTEM_ID,
                                       RadioConfigs::MAV_COMPONENT_ID, &msg,
                                       &nack);
            break;
    }

    return msg;
}

}  // namespace Main
