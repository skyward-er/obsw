/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#pragma once

#include <algorithms/AirBrakes/TrajectorySet.h>

namespace Main
{
namespace ABKTrajectories
{

//['Heights ', 'Vz_closed_m28 ', 'Vz_closed_m28_2 ', 'Vz_closed_m28_4 ',
//'Vz_closed_m28_6 ', 'Vz_closed_m28_8 ', 'Vz_closed_m29 ', 'Vz_closed_m29_2 ',
//'Vz_closed_m29_4 ', 'Vz_closed_m29_6 ', 'Vz_closed_m29_8 ', 'Vz_closed_m30 ',
//'Vz_open_m28 ', 'Vz_open_m28_2 ', 'Vz_open_m28_4 ', 'Vz_open_m28_6 ',
//'Vz_open_m28_8 ', 'Vz_open_m29 ', 'Vz_open_m29_2 ', 'Vz_open_m29_4 ',
//'Vz_open_m29_6 ', 'Vz_open_m29_8 ', 'Vz_open_m30 ']
Boardcore::TrajectoryPoint t0_closed[] = {
    Boardcore::TrajectoryPoint(0, 284.315896277823),
    Boardcore::TrajectoryPoint(10, 283.657136638085),
    Boardcore::TrajectoryPoint(20, 282.998894420936),
};
Boardcore::TrajectoryPoint t1_closed[] = {
    Boardcore::TrajectoryPoint(0, 283.979995680513),
    Boardcore::TrajectoryPoint(10, 283.323354974336),
    Boardcore::TrajectoryPoint(20, 282.667394812745),
};
Boardcore::TrajectoryPoint t2_closed[] = {
    Boardcore::TrajectoryPoint(0, 283.649384988986),
    Boardcore::TrajectoryPoint(10, 282.994911393542),
    Boardcore::TrajectoryPoint(20, 282.341195240687),
};
Boardcore::TrajectoryPoint t3_closed[] = {
    Boardcore::TrajectoryPoint(0, 283.323943384102),
    Boardcore::TrajectoryPoint(10, 282.671680179665),
    Boardcore::TrajectoryPoint(20, 282.02029630144),
};
Boardcore::TrajectoryPoint t4_closed[] = {
    Boardcore::TrajectoryPoint(0, 283.003553469561),
    Boardcore::TrajectoryPoint(10, 282.353555591389),
    Boardcore::TrajectoryPoint(20, 281.70444652695),
};
Boardcore::TrajectoryPoint t5_closed[] = {
    Boardcore::TrajectoryPoint(0, 282.688101171867),
    Boardcore::TrajectoryPoint(10, 282.040538892303),
    Boardcore::TrajectoryPoint(20, 281.393519239995),
};
Boardcore::TrajectoryPoint t6_closed[] = {
    Boardcore::TrajectoryPoint(0, 282.377587985693),
    Boardcore::TrajectoryPoint(10, 281.732309430491),
    Boardcore::TrajectoryPoint(20, 281.087401454484),
};
Boardcore::TrajectoryPoint t7_closed[] = {
    Boardcore::TrajectoryPoint(0, 282.072241236148),
    Boardcore::TrajectoryPoint(10, 281.428843762445),
    Boardcore::TrajectoryPoint(20, 280.785983514998),
};
Boardcore::TrajectoryPoint t8_closed[] = {
    Boardcore::TrajectoryPoint(0, 281.771482344826),
    Boardcore::TrajectoryPoint(10, 281.13000405443),
    Boardcore::TrajectoryPoint(20, 280.489158984282),
};
Boardcore::TrajectoryPoint t9_closed[] = {
    Boardcore::TrajectoryPoint(0, 281.47521101612),
    Boardcore::TrajectoryPoint(10, 280.835686314498),
    Boardcore::TrajectoryPoint(20, 280.196860974699),
};
Boardcore::TrajectoryPoint t10_closed[] = {
    Boardcore::TrajectoryPoint(0, 281.183329704889),
    Boardcore::TrajectoryPoint(10, 280.545789563713),
    Boardcore::TrajectoryPoint(20, 279.90900554376),
};
Boardcore::TrajectoryPoint t0_open[] = {
    Boardcore::TrajectoryPoint(0, 478.303866669533),
    Boardcore::TrajectoryPoint(10, 476.110678663647),
    Boardcore::TrajectoryPoint(20, 473.908331442012),
};
Boardcore::TrajectoryPoint t1_open[] = {
    Boardcore::TrajectoryPoint(0, 475.966219691446),
    Boardcore::TrajectoryPoint(10, 473.768973368115),
    Boardcore::TrajectoryPoint(20, 471.570350514458),
};
Boardcore::TrajectoryPoint t2_open[] = {
    Boardcore::TrajectoryPoint(0, 473.576902505924),
    Boardcore::TrajectoryPoint(10, 471.403156513521),
    Boardcore::TrajectoryPoint(20, 469.218515885813),
};
Boardcore::TrajectoryPoint t3_open[] = {
    Boardcore::TrajectoryPoint(0, 471.298349939231),
    Boardcore::TrajectoryPoint(10, 469.119867407005),
    Boardcore::TrajectoryPoint(20, 466.941384874778),
};
Boardcore::TrajectoryPoint t4_open[] = {
    Boardcore::TrajectoryPoint(0, 469.040451230612),
    Boardcore::TrajectoryPoint(10, 466.864918440385),
    Boardcore::TrajectoryPoint(20, 464.676979756001),
};
Boardcore::TrajectoryPoint t5_open[] = {
    Boardcore::TrajectoryPoint(0, 466.798381972774),
    Boardcore::TrajectoryPoint(10, 464.61789000206),
    Boardcore::TrajectoryPoint(20, 462.437398031347),
};
Boardcore::TrajectoryPoint t6_open[] = {
    Boardcore::TrajectoryPoint(0, 464.57504489382),
    Boardcore::TrajectoryPoint(10, 462.395679122466),
    Boardcore::TrajectoryPoint(20, 460.205599127208),
};
Boardcore::TrajectoryPoint t7_open[] = {
    Boardcore::TrajectoryPoint(0, 462.397633914683),
    Boardcore::TrajectoryPoint(10, 460.235443345584),
    Boardcore::TrajectoryPoint(20, 458.07037540676),
};
Boardcore::TrajectoryPoint t8_open[] = {
    Boardcore::TrajectoryPoint(0, 460.225158627655),
    Boardcore::TrajectoryPoint(10, 458.060827677211),
    Boardcore::TrajectoryPoint(20, 455.890588079475),
};
Boardcore::TrajectoryPoint t9_open[] = {
    Boardcore::TrajectoryPoint(0, 458.068111201296),
    Boardcore::TrajectoryPoint(10, 455.903232083041),
    Boardcore::TrajectoryPoint(20, 453.731294762838),
};
Boardcore::TrajectoryPoint t10_open[] = {
    Boardcore::TrajectoryPoint(0, 455.925284684661),
    Boardcore::TrajectoryPoint(10, 453.756079330941),
    Boardcore::TrajectoryPoint(20, 451.584396616468),
};
Boardcore::Trajectory t_closed[] = {
    Boardcore::Trajectory{0.0, t0_closed, 3},
    Boardcore::Trajectory{0.0, t1_closed, 3},
    Boardcore::Trajectory{0.0, t2_closed, 3},
    Boardcore::Trajectory{0.0, t3_closed, 3},
    Boardcore::Trajectory{0.0, t4_closed, 3},
    Boardcore::Trajectory{0.0, t5_closed, 3},
    Boardcore::Trajectory{0.0, t6_closed, 3},
    Boardcore::Trajectory{0.0, t7_closed, 3},
    Boardcore::Trajectory{0.0, t8_closed, 3},
    Boardcore::Trajectory{0.0, t9_closed, 3},
    Boardcore::Trajectory{0.0, t10_closed, 3},
};
Boardcore::Trajectory t_open[] = {
    Boardcore::Trajectory{0.0, t0_open, 3},
    Boardcore::Trajectory{0.0, t1_open, 3},
    Boardcore::Trajectory{0.0, t2_open, 3},
    Boardcore::Trajectory{0.0, t3_open, 3},
    Boardcore::Trajectory{0.0, t4_open, 3},
    Boardcore::Trajectory{0.0, t5_open, 3},
    Boardcore::Trajectory{0.0, t6_open, 3},
    Boardcore::Trajectory{0.0, t7_open, 3},
    Boardcore::Trajectory{0.0, t8_open, 3},
    Boardcore::Trajectory{0.0, t9_open, 3},
    Boardcore::Trajectory{0.0, t10_open, 3},
};
const Boardcore::TrajectorySet CLOSED_TRAJECTORY_SET(t_closed, 11);
const Boardcore::TrajectorySet OPEN_TRAJECTORY_SET(t_open, 11);

}  // namespace ABKTrajectories
}  // namespace Main