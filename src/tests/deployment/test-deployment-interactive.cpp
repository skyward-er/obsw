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

#include <Deployment/DeploymentController.h>
#include <Deployment/DeploymentServo.h>
#include <logger/Logger.h>
#include <miosix.h>

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "configs/CutterConfig.h"
#include "events/Events.h"
#include "utils/testutils/TestHelper.h"

using namespace DeathStackBoard;

using namespace std;

void cuttingSequence();
void cutterContinuity();
void noseconeEjection();
void wiggleServo();
void setServoFullyOpen();
void setServoFullyClose();
void resetServo();
void manualServoControl();
void setServoParameters();
void setPrimaryCutterParameters();
void setBackupCutterParameters();
void resetServoParameters();
void resetPrimaryCutterParameters();
void resetBackupCutterParameters();

void waitUserInput();

float minPosition   = DeploymentConfigs::DPL_SERVO_MIN_POS;
float maxPosition   = DeploymentConfigs::DPL_SERVO_MAX_POS;
float resetPosition = DeploymentConfigs::DPL_SERVO_RESET_POS;

unsigned int primaryCutterFrequency =
    CutterConfig::PRIMARY_CUTTER_PWM_FREQUENCY;
float primaryCutterDutyCycle = CutterConfig::PRIMARY_CUTTER_PWM_DUTY_CYCLE;
unsigned int backupCutterFrequency = CutterConfig::BACKUP_CUTTER_PWM_FREQUENCY;
float backupCutterDutyCycle        = CutterConfig::BACKUP_CUTTER_PWM_DUTY_CYCLE;
float cutterTestDutyCycle          = primaryCutterDutyCycle / 100.0f;

int main()
{
    // avoid servo to move while resetting the board
    DeploymentServo ejection_servo;
    ejection_servo.enable();
    ejection_servo.reset();

    sEventBroker->start();

    string temp;
    for (;;)
    {
        int choice;
        cout << "\n\nWhat do you want to do?: \n";
        cout << "1.  Test cutting sequence\n";
        cout << "2.  Continuity cutter test (non destructive)\n";
        cout << "3.  Nosecone ejection\n";
        cout << "4.  Servo wiggle (self test)\n";
        cout << "5.  Servo fully open\n";
        cout << "6.  Servo fully close\n";
        cout << "7.  Servo reset\n";
        cout << "8.  Servo manual control\n";
        cout << "9.  Set servo parameters\n";
        cout << "10. Set primary cutter parameters\n";
        cout << "11. Set backup cutter parameters\n";
        cout << "12. Reset servo parameters (default)\n";
        cout << "13. Reset primary cutter parameters (default)\n";
        cout << "14. Reset backup cutter parameters (default)\n";

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
                wiggleServo();
                break;
            case 5:
                setServoFullyOpen();
                break;
            case 6:
                setServoFullyClose();
                break;
            case 7:
                resetServo();
                break;
            case 8:
                manualServoControl();
                break;
            case 9:
                setServoParameters();
                break;
            case 10:
                setPrimaryCutterParameters();
                break;
            case 11:
                setBackupCutterParameters();
                break;
            case 12:
                resetServoParameters();
                break;
            case 13:
                resetPrimaryCutterParameters();
                break;
            case 14:
                resetBackupCutterParameters();
                break;
            default:
                cout << "Invalid option\n";
                break;
        }
    }
}

int waitEventTimeout = 10 * 1000;

void cuttingSequence()
{
    cout << "\n\n** CUTTER SEQUENCE TEST **\n\n";

    waitUserInput();

    // DeploymentController initialization
    HBridge primaryCutter{PrimaryCutterEna::getPin(), CUTTER_TIM,
                          CUTTER_CHANNEL_PRIMARY, primaryCutterFrequency,
                          primaryCutterDutyCycle};
    HBridge backupCutter{BackupCutterEna::getPin(), CUTTER_TIM,
                         CUTTER_CHANNEL_BACKUP, backupCutterFrequency,
                         backupCutterDutyCycle};
    DeploymentServo ejection_servo;

    DeploymentController deploymentController{&primaryCutter, &backupCutter,
                                              &ejection_servo};

    deploymentController.start();

    Thread::sleep(1000);
    sEventBroker->post({EV_CUT_DROGUE}, TOPIC_DPL);

    waitForEvent(EV_CUTTING_TIMEOUT, TOPIC_DPL, waitEventTimeout);

    cout << "Primary cutter is done.\n";

    waitForEvent(EV_CUTTING_TIMEOUT, TOPIC_DPL, waitEventTimeout);
    cout << "Backup cutter is done.\n";

    deploymentController.stop();

    cout << "\n\tTest finished!\n\n";
}

void cutterContinuity()
{
    string temp;
    int cutter;

    cout << "\n\n** CONTINUITY CUTTER TEST **\n\n";
    cout << "Non-destructive cutter test : duty cycle inserted is divided by "
            "100\n\n";
    do
    {
        cout << "Which cutter to test? (1-primary / 2-backup)\n";

        getline(cin, temp);
        stringstream(temp) >> cutter;
    } while (cutter != 1 && cutter != 2);

    waitUserInput();

    // DeploymentController initialization
    HBridge primaryCutter{PrimaryCutterEna::getPin(), CUTTER_TIM,
                          CUTTER_CHANNEL_PRIMARY, primaryCutterFrequency,
                          cutterTestDutyCycle};
    HBridge backupCutter{BackupCutterEna::getPin(), CUTTER_TIM,
                         CUTTER_CHANNEL_BACKUP, backupCutterFrequency,
                         cutterTestDutyCycle};
    DeploymentServo ejection_servo;
    DeploymentController deploymentController{&primaryCutter, &backupCutter,
                                              &ejection_servo};

    deploymentController.start();

    if (cutter == 1)
    {
        cout << "Activating primary cutter...\n";
        sEventBroker->post({EV_TEST_CUT_PRIMARY}, TOPIC_DPL);
    }
    else
    {
        cout << "Activating backup cutter...\n";
        sEventBroker->post({EV_TEST_CUT_BACKUP}, TOPIC_DPL);
    }

    waitForEvent(EV_CUTTING_TIMEOUT, TOPIC_DPL);

    cout << "Cutter test is done.\n";

    deploymentController.stop();

    cout << "\n\tTest finished!\n\n";
}

void noseconeEjection()
{
    cout << "\n\n** NOSECONE EJECTION TEST **\n\n";

    waitUserInput();

    EventCounter counter{*sEventBroker};

    HBridge primaryCutter{PrimaryCutterEna::getPin(), CUTTER_TIM,
                          CUTTER_CHANNEL_PRIMARY, primaryCutterFrequency,
                          primaryCutterDutyCycle};
    HBridge backupCutter{BackupCutterEna::getPin(), CUTTER_TIM,
                         CUTTER_CHANNEL_BACKUP, backupCutterFrequency,
                         backupCutterDutyCycle};
    DeploymentServo ejection_servo;
    DeploymentController deploymentController{&primaryCutter, &backupCutter,
                                              &ejection_servo};

    counter.subscribe(TOPIC_DPL);
    counter.start();

    cout << "Nosecone ejection in\n";
    for (int i = 3; i > 0; i--)
    {
        cout << i << "\n";
        Thread::sleep(1000);
    }

    deploymentController.start();
    sEventBroker->post({EV_NC_OPEN}, TOPIC_DPL);

    for (;;)
    {
        if (counter.getCount(EV_NC_OPEN_TIMEOUT) == 1)
            break;
        if (counter.getCount(EV_NC_DETACHED) >= 1)
            break;
        Thread::sleep(10);
    }

    cout << "Nosecone ejection test is done.\n";

    deploymentController.stop();
    counter.stop();

    cout << "\n\tTest finished!\n\n";
}

void wiggleServo()
{
    cout << "\n\n** WIGGLE SERVO **\n\n";

    waitUserInput();

    DeploymentServo servo{minPosition, maxPosition, resetPosition};
    servo.enable();
    servo.reset();

    cout << "Wiggling ...\n";
    servo.selfTest();
    Thread::sleep(1000);
    servo.disable();

    cout << "\n\tTest finished!\n\n";
}

void setServoFullyOpen()
{
    cout << "\n\n** SERVO FULLY OPEN **\n\n";

    waitUserInput();

    DeploymentServo servo{minPosition, maxPosition, resetPosition};
    servo.enable();
    servo.setMaxPosition();
    Thread::sleep(1000);
    servo.disable();

    cout << "\n\tTest finished!\n\n";
}

void setServoFullyClose()
{
    cout << "\n\n** SERVO FULLY CLOSE **\n\n";

    waitUserInput();

    DeploymentServo servo{minPosition, maxPosition, resetPosition};
    servo.enable();
    servo.setMinPosition();
    Thread::sleep(1000);
    servo.disable();

    cout << "\n\tTest finished!\n\n";
}

void resetServo()
{
    cout << "\n\n** RESET SERVO **\n\n";

    waitUserInput();

    DeploymentServo servo{minPosition, maxPosition, resetPosition};
    servo.enable();
    servo.reset();
    Thread::sleep(1000);
    servo.disable();

    cout << "\n\tTest finished!\n\n";
}

void manualServoControl()
{
    cout << "\n\n** MANUAL SERVO CONTROL **\n\n";

    string temp;
    float angle;

    do
    {
        cout << "Write the servo postion in degrees:\n";
        getline(cin, temp);
        stringstream(temp) >> angle;
    } while (angle < minPosition || angle > maxPosition);

    DeploymentServo servo{minPosition, maxPosition, resetPosition};

    servo.enable();
    servo.set(angle);
    Thread::sleep(1000);
    servo.disable();

    cout << "\n\tTest finished!\n\n";
}

void setServoParameters()
{
    cout << "\n\n** SET SERVO PARAMETERS **\n\n";

    string temp;
    do
    {
        cout << "Write the servo minimum postion (degrees):\n";
        getline(cin, temp);
        stringstream(temp) >> minPosition;
    } while (minPosition < 0 || minPosition > 180.0f);

    do
    {
        cout << "Write the servo maximum postion (degrees):\n";
        getline(cin, temp);
        stringstream(temp) >> maxPosition;
    } while (maxPosition < 0 || maxPosition > 180.0f);

    do
    {
        cout << "Write the servo reset postion (degrees):\n";
        getline(cin, temp);
        stringstream(temp) >> resetPosition;
    } while (resetPosition < 0 || resetPosition > 180.0f);

    cout << "Configured servo parameteres:\n";
    cout << "\tminimum position: " << minPosition << "\n";
    cout << "\tmaximum position: " << maxPosition << "\n";
    cout << "\treset position: " << resetPosition << "\n";

    // Reset servo
    DeploymentServo servo{minPosition, maxPosition, resetPosition};
    servo.enable();
    servo.reset();
    Thread::sleep(1000);
    servo.disable();
}

void setPrimaryCutterParameters()
{
    cout << "\n\n** SET PRIMARY CUTTER PARAMETERS **\n\n";

    string temp;

    cout << "Write the primary cutter frequency in hertz:\n";
    getline(cin, temp);
    stringstream(temp) >> primaryCutterFrequency;

    do
    {
        cout << "Write the primary cutter duty cycle in %:\n";
        getline(cin, temp);
        stringstream(temp) >> primaryCutterDutyCycle;
        primaryCutterDutyCycle /= 100;
    } while (primaryCutterDutyCycle < 0 || primaryCutterDutyCycle > 1.0F);

    cout << "Configured primary cutter parameteres:\n";
    cout << "\tfrequency: " << primaryCutterFrequency << "Hz\n";
    cout << "\tduty cycle: " << primaryCutterDutyCycle * 100 << "%\n";
}

void setBackupCutterParameters()
{
    cout << "\n\n** SET BACKUP CUTTER PARAMETERS **\n\n";

    string temp;

    cout << "Write the backup cutter frequency in hertz:\n";
    getline(cin, temp);
    stringstream(temp) >> backupCutterFrequency;

    do
    {
        cout << "Write the backup cutter duty cycle in %:\n";
        getline(cin, temp);
        stringstream(temp) >> backupCutterDutyCycle;
        backupCutterDutyCycle /= 100;
    } while (backupCutterDutyCycle < 0 || backupCutterDutyCycle > 1.0F);

    cout << "Configured backup cutter parameteres:\n";
    cout << "\tfrequency: " << backupCutterFrequency << "Hz\n";
    cout << "\tduty cycle: " << backupCutterDutyCycle * 100 << "%\n";
}

void resetServoParameters()
{
    minPosition   = DeploymentConfigs::DPL_SERVO_MIN_POS;
    maxPosition   = DeploymentConfigs::DPL_SERVO_MAX_POS;
    resetPosition = DeploymentConfigs::DPL_SERVO_MIN_POS;

    cout << "Configured servo parameteres (default):\n";
    cout << "\tminimum position: " << minPosition << "\n";
    cout << "\tmaximum position: " << maxPosition << "\n";
    cout << "\treset position: " << resetPosition << "\n";
}

void resetPrimaryCutterParameters()
{
    primaryCutterFrequency = CutterConfig::PRIMARY_CUTTER_PWM_FREQUENCY;
    primaryCutterDutyCycle = CutterConfig::PRIMARY_CUTTER_PWM_DUTY_CYCLE;

    cout << "Configured primary cutter parameteres:\n";
    cout << "\tfrequency: " << primaryCutterFrequency << "Hz\n";
    cout << "\tduty cycle: " << primaryCutterDutyCycle * 100 << "%\n";
}

void resetBackupCutterParameters()
{
    backupCutterFrequency = CutterConfig::BACKUP_CUTTER_PWM_FREQUENCY;
    backupCutterDutyCycle = CutterConfig::BACKUP_CUTTER_PWM_DUTY_CYCLE;

    cout << "Configured backup cutter parameteres:\n";
    cout << "\tfrequency: " << backupCutterFrequency << "Hz\n";
    cout << "\tduty cycle: " << backupCutterDutyCycle * 100 << "%\n";
}

void waitUserInput()
{
    string temp;

    do
    {
        cout << "Write 'start' to begin the test:\n";
        getline(cin, temp);
    } while (temp != "start");
}