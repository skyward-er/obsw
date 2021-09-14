/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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
#include <miosix.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iomanip>
#include <iostream>

#include "drivers/HardwareTimer.h"

using namespace std;
using namespace Eigen;
using namespace miosix;
using miosix::Thread;

void nProducts2Mat(int n, MatrixXd& m1, MatrixXd& m2)
{
    HardwareTimer<uint32_t, 2>& hrclock =
        HardwareTimer<uint32_t, 2>::instance();
    hrclock.setPrescaler(127);
    hrclock.start();

    int i       = 0;
    uint32_t t1 = hrclock.tick();

    for (i = 0; i < n; i++)
        MatrixXd m = m1 * m2;

    uint32_t t2 = hrclock.tick();
    double time = hrclock.toMilliSeconds(t2 - t1);
    hrclock.stop();

    TRACE("\nTime for %d products using 2 matrices: %f [ms] \n\n", n, time);
}

void nProducts3Mat(int n, MatrixXd& m1, MatrixXd& m2, MatrixXd& m3)
{
    HardwareTimer<uint32_t, 2>& hrclock =
        HardwareTimer<uint32_t, 2>::instance();
    hrclock.setPrescaler(127);
    hrclock.start();

    int i       = 0;
    uint32_t t1 = hrclock.tick();

    for (i = 0; i < n; i++)
        MatrixXd m = m1 * m2 * m3;

    uint32_t t2 = hrclock.tick();
    double time = hrclock.toMilliSeconds(t2 - t1);
    hrclock.stop();

    TRACE("\nTime for %d products using 3 matrices: %f [ms] \n\n", n, time);
}

void kalmanOperations(MatrixXd& m1, MatrixXd& m2, MatrixXd& m3, MatrixXd& m4,
                      MatrixXd& m5, MatrixXd& eye, MatrixXd& v1, MatrixXd& v2)
{
    HardwareTimer<uint32_t, 2>& hrclock =
        HardwareTimer<uint32_t, 2>::instance();
    hrclock.setPrescaler(127);
    hrclock.start();

    auto x = v1;
    auto P = m2;
    auto Q = m3;
    auto R = m5;
    auto y = v2;

    uint32_t t1 = hrclock.tick();

    auto F     = 0.5 * m1;
    x          = F * x;
    P          = F * P * (F.transpose()) + Q;
    auto H     = 2 * m4;
    auto K     = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
    auto U     = K * (y.transpose() - H * x);
    auto x_new = x + U;
    auto P_new = (eye - K * H) * P;

    uint32_t t2 = hrclock.tick();
    double time = hrclock.toMilliSeconds(t2 - t1);
    hrclock.stop();

    TRACE("\nTime for a single kalman cycle: %f [ms] \n\n", time);
}

void sparseKalmanOperations(MatrixXd& m1, MatrixXd& m2, MatrixXd& m3,
                            MatrixXd& m4, MatrixXd& m5, MatrixXd& eye,
                            MatrixXd& v1, MatrixXd& v2)
{
    // H, P and R can't be sparse since their product needs to be inverted.
    HardwareTimer<uint32_t, 2>& hrclock =
        HardwareTimer<uint32_t, 2>::instance();
    hrclock.setPrescaler(127);
    hrclock.start();

    auto x = v1;
    auto P = m2;
    auto Q = m3.sparseView();
    auto R = m5;
    auto y = v2;

    uint32_t t1 = hrclock.tick();

    auto F = 0.5 * m1.sparseView();
    x      = F * x;
    P      = F * P * (F.transpose()) + Q;
    auto H = 2 * m4;
    auto K = (P * (H.transpose()) * ((H * P * H.transpose() + R).inverse()))
                 .sparseView();
    auto U     = (K * (y.transpose() - H * x)).sparseView();
    auto x_new = x + U;
    auto P_new = (eye - K * H) * P;

    uint32_t t2 = hrclock.tick();
    double time = hrclock.toMilliSeconds(t2 - t1);
    hrclock.stop();

    TRACE(
        "\nTime for a single kalman cycle with some sparse matrices: %f [ms] "
        "\n\n",
        time);
}

void determinant(MatrixXd& m1)
{
    HardwareTimer<uint32_t, 2>& hrclock =
        HardwareTimer<uint32_t, 2>::instance();
    hrclock.setPrescaler(127);
    hrclock.start();

    uint32_t t1 = hrclock.tick();

    float det = m1.determinant();

    uint32_t t2 = hrclock.tick();
    double time = hrclock.toMilliSeconds(t2 - t1);
    hrclock.stop();

    TRACE("\nTime to find the determinant: %f [ms] \n\n", time);
}

int main()
{
    static const int ROWS = 10;
    static const int COL  = 10;
    static const int N    = 100;

    MatrixXd m1  = MatrixXd::Random(ROWS, COL);
    MatrixXd m2  = MatrixXd::Random(ROWS, COL);
    MatrixXd m3  = MatrixXd::Random(ROWS, COL);
    MatrixXd m4  = MatrixXd::Random(ROWS, COL);
    MatrixXd m5  = MatrixXd::Random(ROWS, COL);
    MatrixXd eye = MatrixXd::Identity(ROWS, ROWS);

    MatrixXd v1 = MatrixXd::Random(ROWS, 1);
    MatrixXd v2 = MatrixXd::Random(1, ROWS);

    determinant(m1);
    nProducts2Mat(N, m1, m2);
    nProducts3Mat(N, m1, m2, m3);
    kalmanOperations(m1, m2, m3, m4, m5, eye, v1, v2);
    sparseKalmanOperations(m1, m2, m3, m4, m5, eye, v1, v2);

    return 0;
}