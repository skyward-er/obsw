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

#include <cmath>
#include <fstream>
#include <iostream>

#include "Sensors/BMX160Calibrator.h"
#include "configs/SensorManagerConfig.h"
#include "sensors/BMX160/BMX160.h"

int main()
{
    std::srand(123456);

    BMX160CorrectionParameters params;
    for (int i = 0; i < 6; i++)
    {
        params.accelParams(i % 3, i / 3)   = std::rand() % 100 * 0.1;
        params.magnetoParams(i % 3, i / 3) = std::rand() % 100 * 0.1;
        params.gyroParams(i % 3, i / 3)    = std::rand() % 100 * 0.1;
    }

    std::cout << "Generated params: " << std::endl;
    std::cout << BMX160CorrectionParameters::header() << std::endl;
    params.print(std::cout);

    {
        // Writing to file
        std::ofstream paramsFile(
            DeathStackBoard::SensorConfigs::BMX160_CORRECTION_PARAMETERS_FILE);
        paramsFile << BMX160CorrectionParameters::header() << std::endl;
        params.print(paramsFile);
    }

    BMX160Data data;
    BMX160Calibrator corrector;

    // Reading from same file
    corrector.readParametersFromFile();
    params = corrector.getParameters();

    std::cout << std::endl << "Read params: " << std::endl;
    std::cout << BMX160CorrectionParameters::header() << std::endl;
    params.print(std::cout);

    std::cout << std::flush;
    for (;;)
        ;
}
