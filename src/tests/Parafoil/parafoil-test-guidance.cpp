/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

#include <Parafoil/Wing/AutomaticWingAlgorithm.h>
#include <Parafoil/Wing/Guidance/EarlyManeuversGuidanceAlgorithm.h>
#include <algorithms/NAS/NASState.h>
#include <algorithms/PIController.h>
#include <algorithms/ReferenceValues.h>

#include <iostream>

#include "parafoil-test-guidance-data.h"

using namespace Eigen;
using namespace Parafoil;
using namespace Boardcore;

class MockWingAlgo : protected AutomaticWingAlgorithm
{
public:
    MockWingAlgo(float kp, float ki, ServosList servo1, ServosList servo2,
                 GuidanceAlgorithm& guidance)
        : AutomaticWingAlgorithm(kp, ki, servo1, servo2, guidance)
    {
    }

    float fakeStep(NASState state, ReferenceValues reference,
                   Vector2f targetPosition, Vector2f wind)
    {
        return this->algorithmStep(state, reference, targetPosition, wind);
    }
};
int main()
{
    float threshold = 0.1;
    EarlyManeuversGuidanceAlgorithm guidance;
    MockWingAlgo algo(kp, ki, ServosList::PARAFOIL_LEFT_SERVO,
                      ServosList::PARAFOIL_RIGHT_SERVO, guidance);
    std::cout << "Parafoil control test" << std::endl;

    for (unsigned i = 1; i < NAS.size(); i++)
    {
        NASState state;
        state.n  = NAS[i][0];
        state.e  = NAS[i][1];
        state.d  = NAS[i][2];
        state.vn = NAS[i][3];
        state.ve = NAS[i][4];

        Eigen::Vector2f wind = {WES[i][0], WES[i][1]};

        ReferenceValues ref{};

        ref.refLatitude  = TARGET[0];
        ref.refLongitude = TARGET[1];

        Eigen::Vector2f target(TARGET[0], TARGET[1]);

        float result = algo.fakeStep(state, ref, target, wind);

        if (result < OUT_EM[i] - threshold || result > OUT_EM[i] + threshold)
        {
            std::cout << "The estimated mass differs from the correct one ["
                      << i << "]: " << result << " != " << OUT_EM[i]
                      << std::endl;
        }
    }

    return 0;
}
