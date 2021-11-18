# Copyright (c) 2021 Skyward Experimental Rocketry
# Author: Damiano Amatruda
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

set(OBSW_INCLUDE_DIRS src src/boards/DeathStack)

set(DEATHSTACK_NEW_SOURCES
    src/boards/DeathStack/events/EventStrings.cpp
    src/boards/DeathStack/PinHandler/PinHandler.cpp
    src/boards/DeathStack/TelemetriesTelecommands/TMTCController.cpp
    src/boards/DeathStack/TelemetriesTelecommands/TCHandler.cpp
    src/boards/DeathStack/TelemetriesTelecommands/TmRepository.cpp
    src/boards/DeathStack/Main/Radio.cpp
    src/boards/DeathStack/Main/Sensors.cpp
    src/boards/DeathStack/Main/StateMachines.cpp
    src/boards/DeathStack/Deployment/DeploymentController.cpp
    src/boards/DeathStack/FlightModeManager/FMMController.cpp
    src/boards/DeathStack/ApogeeDetectionAlgorithm/ADACalibrator.cpp
    src/boards/DeathStack/ApogeeDetectionAlgorithm/ADAAlgorithm.cpp
    src/boards/DeathStack/NavigationAttitudeSystem/NASCalibrator.cpp
    src/boards/DeathStack/NavigationAttitudeSystem/ExtendedKalmanEigen.cpp
    src/boards/DeathStack/FlightStatsRecorder/FSRController.cpp
    src/boards/DeathStack/AirBrakes/AirBrakesServo.cpp
)
#set(DEATHSTACK_SOURCES
#    src/boards/DeathStack/LoggerService/LoggerService.cpp
#    src/boards/DeathStack/events/EventStrings.cpp
#    src/boards/DeathStack/FlightModeManager/FMMController.cpp
#    src/boards/DeathStack/SensorManager/SensorManager.cpp
#    src/boards/DeathStack/Deployment/DeploymentController.cpp
#    src/boards/DeathStack/PinHandler/PinHandler.cpp
#    src/boards/DeathStack/TMTCManager/TMTCManager.cpp
#    src/boards/DeathStack/LoggerService/TmRepository.cpp
#    src/boards/DeathStack/ApogeeDetectionAlgorithm/ADAAlgorithm.cpp
#    src/boards/DeathStack/ApogeeDetectionAlgorithm/ADACalibrator.cpp
#    src/boards/DeathStack/LoggerService/FSRController.cpp
#    src/boards/DeathStack/TMTCManager/XbeeInterrupt.cpp
#    src/boards/DeathStack/AirBrakes/AirBrakesServo.cpp
#)
set(ADA_SOURCES
    src/boards/DeathStack/ApogeeDetectionAlgorithm/ADAAlgorithm.cpp
    src/boards/DeathStack/ApogeeDetectionAlgorithm/ADACalibrator.cpp
)
set(DEPLOYMENT_SOURCES
    src/boards/DeathStack/Deployment/DeploymentController.cpp
)
set(AIRBRAKES_SOURCES
    src/boards/DeathStack/AirBrakes/AirBrakesServo.cpp
)
set(PINHANDLER_SOURCES
    src/boards/DeathStack/PinHandler/PinHandler.cpp
)
set(FMM_SOURCES
    src/boards/DeathStack/FlightModeManager/FMMController.cpp
)
#set(SENSORS_SOURCES
#    src/boards/DeathStack/Sensors/BMX160Calibrator.cpp
#    skyward-boardcore/src/shared/sensors/calibration/SensorDataExtra.cpp
#)
#set(TMTC_SOURCES
#    src/boards/DeathStack/TelemetriesTelecommands/TCHandler.cpp
#    src/boards/DeathStack/TelemetriesTelecommands/TMTCManager.cpp
#)
set(ALDEERAN_SOURCES
    src/boards/Ignition/IgnitionManager.cpp
)
set(LOGSERVICE_SOURCES
    src/boards/DeathStack/TelemetriesTelecommands/TmRepository.cpp
    src/boards/DeathStack/FlightStatsRecorder/FSRController.cpp
)
set(ADA_TEST_SOURCES
    src/tests/catch/ada/ada_kalman_p/test-ada-data.cpp
)
set(KALMAN_TEST_SOURCES
    src/tests/catch/ada/kalman_acc/test-kalman-acc-data.cpp
)
set(MOCK_SENSORS_DATA_SOURCES
    src/mocksensors/lynx_flight_data/lynx_imu_data.cpp
    src/mocksensors/lynx_flight_data/lynx_press_data.cpp
    src/mocksensors/lynx_flight_data/lynx_pressure_static_data.cpp
    src/mocksensors/lynx_flight_data/lynx_gps_data.cpp
    src/mocksensors/lynx_flight_data/lynx_airspeed_data.cpp
)
set(HERMES_TESTS_SOURCES
    src/tests/catch/fsm/test-ada.cpp
)
set(RAM_TEST_SOURCES
    src/tests/ram_test/sha1.cpp
)
set(EVT_FUNCTIONS_SOURCES
    src/boards/DeathStack/events/EventStrings.cpp
)
set(NAVIGATION_SYSTEM_SOURCES
    src/boards/DeathStack/NavigationAttitudeSystem/NASCalibrator.cpp
    src/boards/DeathStack/NavigationAttitudeSystem/ExtendedKalmanEigen.cpp
)
set(TESTS_OBSW_SOURCES
    src/tests/catch/fsm/test-fmm.cpp
    src/tests/catch/fsm/test-tmtc.cpp
    src/tests/catch/fsm/test-ada.cpp
    src/tests/catch/ada/ada_kalman_p/test-ada-simulation.cpp
    src/tests/catch/fsm/test-deployment.cpp
    src/tests/catch/fsm/test-flightstatsrecorder.cpp
    src/tests/catch/fsm/test-airbrakes.cpp
    #src/tests/catch/fsm/test-nas.cpp
    #src/tests/catch/nas/test-nas-simulation.cpp
)
