/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
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

#include <miosix.h>

#include "CutterData.h"
#include "configs/CutterConfig.h"
#include "drivers/pwm/pwm.h"

using miosix::GpioPin;
using miosix::Thread;

namespace DeathStackBoard
{

/**
 * @brief Interface class to operate the thermal cutters on the Hermev V1
 * rocket.
 *
 * Provides the ability to enable a cutter using the "nominal" duty cycle or
 * using a "test" duty cycle (lower duty cycle that does not cut the parachute
 * but is used to check if current is flowing)
 */
class Cutter
{
public:
    /**
     * @brief Create a new Cutter
     *
     * @param    frequency          Frequency of the PWM driving the cutters
     * @param    duty_cycle         Duty cycle of the PWM driving the cutters
     * @param    test_duty_cycle    Duty cycle to be used when testing the
     *                              cutters for continuity
     */
    Cutter(unsigned int frequency, float duty_cycle, float test_duty_cycle)
        : pwm(CUTTER_TIM, frequency),
          pin_enable_primary(PrimaryCutterEna::getPin()),
          pin_enable_backup(BackupCutterEna::getPin()),
          cut_duty_cycle(duty_cycle), test_duty_cycle(test_duty_cycle)
    {
        pin_enable_primary.low();
        pin_enable_backup.low();

        // Start PWM with 0 duty cycle to keep IN pins low
        pwm.enableChannel(CUTTER_CHANNEL_PRIMARY, 0.0f);
        pwm.enableChannel(CUTTER_CHANNEL_BACKUP, 0.0f);

        pwm.start();
    }

    ~Cutter()
    {
        disablePrimaryCutter();
        disableBackupCutter();

        pwm.stop();
    }

    /**
     * @brief Activates the primary cutter.
     */
    void enablePrimaryCutter()
    {
        enableCutter(CUTTER_CHANNEL_PRIMARY, pin_enable_primary,
                     cut_duty_cycle);
        status.state = CutterState::CUTTING_PRIMARY;
    }

    /**
     * @brief Deactivates the primary cutter
     */
    void disablePrimaryCutter()
    {
        if (status.state == CutterState::CUTTING_PRIMARY)
        {
            disableCutter(CUTTER_CHANNEL_PRIMARY, pin_enable_primary);
            status.state = CutterState::IDLE;
        }
    }

    /**
     * @brief Activates the backup cutter.
     */
    void enableBackupCutter()
    {
        enableCutter(CUTTER_CHANNEL_BACKUP, pin_enable_backup, cut_duty_cycle);
        status.state = CutterState::CUTTING_BACKUP;
    }

    /**
     * @brief Deactivates the pbackup cutter
     */
    void disableBackupCutter()
    {
        if (status.state == CutterState::CUTTING_BACKUP)
        {
            disableCutter(CUTTER_CHANNEL_BACKUP, pin_enable_backup);
            status.state = CutterState::IDLE;
        }
    }

    /**
     * @brief Enables the primary cutter using the "test" duty cycle
     *
     * call disablePrimaryCutter() to disable
     */
    void enableTestPrimaryCutter()
    {
        enableCutter(CUTTER_CHANNEL_PRIMARY, pin_enable_primary, test_duty_cycle);
        status.state = CutterState::TESTING_PRIMARY;
    }

    /**
     * @brief Enables the backup cutter using the "test" duty cycle
     *
     * call disableBackupCutter() to disable
     */
    void enableTestBackupCutter()
    {
        enableCutter(CUTTER_CHANNEL_BACKUP, pin_enable_backup, test_duty_cycle);
        status.state = CutterState::TESTING_BACKUP;
    }

    CutterStatus getStatus() { return status; }

private:
    void enableCutter(PWMChannel channel, miosix::GpioPin& ena_pin,
                      float duty_cycle)
    {
        // Only enable if the cutter is currently idle
        if (status.state == CutterState::IDLE)
        {
            // Enable PWM Generation
            pwm.setDutyCycle(channel, duty_cycle);

            // enable
            ena_pin.high();
        }
    }

    void disableCutter(PWMChannel channel, miosix::GpioPin& ena_pin)
    {
        pwm.setDutyCycle(channel, 0.0f);  // Set duty cycle to 0 to leave the IN
                                          // pin of the Half-H bridge low

        Thread::sleep(CUTTER_DISABLE_DELAY_MS);  // Wait a short delay

        ena_pin.low();  // Disable hbridge
    }

    Cutter(const Cutter& c) = delete;

    PWM pwm;
    GpioPin pin_enable_primary;
    GpioPin pin_enable_backup;

    float cut_duty_cycle;
    float test_duty_cycle;
    CutterStatus status;
};

}  // namespace DeathStackBoard