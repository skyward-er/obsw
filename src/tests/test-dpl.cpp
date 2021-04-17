/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
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

#include <logger/Logger.h>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "DeploymentController/DeploymentController.h"
#include "DeploymentController/ThermalCutter/Cutter.h"
//#include "SensorManager/Sensors/ADCWrapper.h"
#include "configs/CutterConfig.h"
#include "events/EventInjector.h"
#include "events/Events.h"
#include "utils/testutils/TestHelper.h"

using namespace DeathStackBoard;

using namespace std;

void cuttingSequence();
void cutterContinuity();
void noseconeEjection();
void manualEvent();

float adcToCurrent(uint16_t adc_in) { return (adc_in - 107) / 32.4f; }

int main()
{
    sEventBroker->start();

    string temp;
    for (;;)
    {
        int choice;
        cout << "\n\nWhat do you want to do?: \n";
        cout << "1. Test cutting sequence\n";
        cout << "2. Non-destructive cutter test\n";
        cout << "3. Nosecone ejection\n";
        cout << "4. Manual event input\n";

        getline(cin, temp);
        stringstream(temp) >> choice;

        switch (choice)
        {
            case 1:
                cuttingSequence();
                break;
            case 2:
                cutterContinuity();
                break;
            case 3:
                noseconeEjection();
                break;
            case 4:
                manualEvent();
                break;
            default:
                break;
        }
    }
}

bool print = false;

/*
void csense(void*)
{
    ADCWrapper adc;
    adc.getCurrentSensorPtr()->init();

    for (;;)
    {
        adc.getCurrentSensorPtr()->onSimpleUpdate();
        uint16_t raw1 =
            adc.getCurrentSensorPtr()->getCurrentDataPtr()->raw_value_1;
        uint16_t raw2 =
            adc.getCurrentSensorPtr()->getCurrentDataPtr()->raw_value_2;

        float current1 =
            adc.getCurrentSensorPtr()->getCurrentDataPtr()->current_1;
        float current2 =
            adc.getCurrentSensorPtr()->getCurrentDataPtr()->current_2;
        if (print)
        {
            printf("%d,%d,%d,%f,%f\n", (int)miosix::getTick(), (int)raw1,
                   (int)raw2, current1, current2);
        }
        Thread::sleep(100);
    }
}
*/ 

void cuttingSequence()
{
    string temp;
    char yn;

    cout << "\n\n** CUTTER SEQUENCE TEST **\n\n";
    cout << "Use predefined cutter parameters? (y/n)\n";
    getline(cin, temp);
    stringstream(temp) >> yn;
    unsigned int freq = CUTTER_PWM_FREQUENCY;
    float duty        = CUTTER_PWM_DUTY_CYCLE * 100;

    if (yn == 'n')
    {
        do
        {
            cout << "Insert frequency (Hz): \n";
            getline(cin, temp);
            stringstream(temp) >> freq;

            cout << "Insert duty  cycle(%): \n";
            getline(cin, temp);
            stringstream(temp) >> duty;
        } while (freq < 100 || freq > 30000 || duty < 0 || duty > 100);

        printf("Using custom parameters: Freq: %d, Duty: %.2f\n", freq, duty);
    }
    else
    {
        printf("Using default parameters: Freq: %d, Duty: %.2f\n", freq, duty);
    }

    do
    {
        cout << "Write 'start' to begin:\n";
        getline(cin, temp);
    } while (temp != "start");

    {
        Cutter c{freq, duty / 100.0f, CUTTER_TEST_PWM_DUTY_CYCLE};
        Servo s{DeploymentConfigs::SERVO_TIMER};

        DeploymentController dpl{c, s};
        dpl.start();
        print = true;
        Thread::sleep(1000);
        cout << "Activating primary cutter...\n";
        sEventBroker->post({EV_CUT_DROGUE}, TOPIC_DEPLOYMENT);

        waitForEvent(EV_TIMEOUT_CUTTING, TOPIC_DEPLOYMENT);
        cout << "Primary cutter is done.\n";
        cout << "Activating secondary cutter\n";

        waitForEvent(EV_TIMEOUT_CUTTING, TOPIC_DEPLOYMENT);
        cout << "Backup cutter is done.\n";
        cout << "\n Test finished!\n\n";
        Thread::sleep(1000);
        print = false;
    }
}

void cutterContinuity()
{
    string temp;
    int cutter;
    cout << "\n\n** NON-DESTRUCTIVE CUTTER TEST **\n\n";
    do
    {
        cout << "Which cutter to test? (1-primary / 2-backup)\n";

        string temp;
        getline(cin, temp);
        stringstream(temp) >> cutter;
    } while (cutter != 1 && cutter != 2);

    cout << "Use predefined cutter parameters? (y/n)\n";

    char yn;
    getline(cin, temp);
    stringstream(temp) >> yn;
    unsigned int freq = CUTTER_PWM_FREQUENCY;
    float duty        = CUTTER_TEST_PWM_DUTY_CYCLE * 100;

    if (yn == 'n')
    {
        do
        {
            cout << "Insert frequency (Hz): \n";
            getline(cin, temp);
            stringstream(temp) >> freq;

            cout << "Insert test duty cycle(%): \n";
            getline(cin, temp);
            stringstream(temp) >> duty;
        } while (freq < 100 || freq > 30000 || duty < 0 || duty > 100);

        printf("Using custom parameters: Freq: %d, Test Duty: %.2f\n", freq,
               duty);
    }
    else
    {
        printf("Using default parameters: Freq: %d, Test Duty: %.2f\n", freq,
               duty);
    }

    do
    {
        cout << "Write 'start' to begin:\n";
        getline(cin, temp);
    } while (temp != "start");

    {
        Cutter c{freq, CUTTER_PWM_DUTY_CYCLE, duty};
        Servo s{DeploymentConfigs::SERVO_TIMER};

        DeploymentController dpl{c, s};
        dpl.start();
        Thread::sleep(1000);
        print = true;
        if (cutter == 1)
        {
            cout << "Activating primary cutter...\n";
            sEventBroker->post({EV_TEST_CUTTER_PRIMARY}, TOPIC_DEPLOYMENT);
        }
        else
        {
            cout << "Activating backup cutter...\n";
            sEventBroker->post({EV_TEST_CUTTER_BACKUP}, TOPIC_DEPLOYMENT);
        }

        waitForEvent(EV_TIMEOUT_CUTTING, TOPIC_DEPLOYMENT);

        cout << "Cutter test is done.\n";
        cout << "\n Test finished!\n\n";

        Thread::sleep(1000);
        print = false;

        dpl.stop();
    }
}

void noseconeEjection()
{
    Servo s{DeploymentConfigs::SERVO_TIMER};
    s.setMinPulseWidth(800);
    s.setMaxPulseWidth(2200);
    s.enable(DeploymentConfigs::SERVO_CHANNEL);

    s.setPosition(DeploymentConfigs::SERVO_CHANNEL,
                  DeploymentConfigs::SERVO_RESET_POS);

    s.start();

    cout << "\n\n** NOSECONE EJECTION TEST **\n\n";
    string temp;
    do
    {
        cout << "Write 'yeet' to yeet out the nosecone:\n";
        getline(cin, temp);
    } while (temp != "yeet");

    {
        EventCounter counter{*sEventBroker};

        Cutter c{CUTTER_PWM_FREQUENCY, CUTTER_PWM_DUTY_CYCLE,
                 CUTTER_TEST_PWM_DUTY_CYCLE};

        DeploymentController dpl{c, s};
        dpl.start();
        counter.subscribe(TOPIC_DEPLOYMENT);
        counter.start();

        cout << "Activating primary cutter...\n";
        sEventBroker->post({EV_NC_OPEN}, TOPIC_DEPLOYMENT);

        for (;;)
        {
            if (counter.getCount(EV_TIMEOUT_NC_OPEN) ==
                DeploymentConfigs::MAX_EJECTION_ATTEMPTS)
                break;
            if (counter.getCount(EV_NC_DETACHED) >= 1)
                break;
            Thread::sleep(10);
        }

        cout << "Cutter test is done.\n";
        cout << "\n Test finished!\n\n";

        dpl.stop();
        counter.stop();
    }
}

void manualEvent()
{
    cout << "\n\n** MANUAL MODE **\n\n";
    cout << "Hope you know what you are doing\n";

    Cutter c{CUTTER_PWM_FREQUENCY, CUTTER_PWM_DUTY_CYCLE,
             CUTTER_TEST_PWM_DUTY_CYCLE};

    Servo s{DeploymentConfigs::SERVO_TIMER};
    s.setMinPulseWidth(800);
    s.setMaxPulseWidth(2200);

    DeploymentController dpl{c, s};
    EventInjector ei;

    ei.start();
    dpl.start();
}
