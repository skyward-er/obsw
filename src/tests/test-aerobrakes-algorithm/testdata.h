/*
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Vincenzo Santomarco
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

static const int LEN_TEST = 169;

struct input_t
{
    float z;
    float vz;
    float vMod;
    float sampleTime;
};

struct output_t
{
    float z_setpoint;
    float vz_setpoint;
    float u;
    float deltas;
    int alpha;
};

struct test_t
{
    input_t input;
    output_t output;
};

const test_t DATA[LEN_TEST] = {
    {{1407.523534, 223.918458, 232.831273, 0.1},
     {1410.698478, 224.370516, 0.000000, 0.000000, 0}},
    {{1429.826222, 222.137686, 231.046739, 0.1},
     {1433.007837, 221.822602, 22.001227, 0.000000, 0}},
    {{1451.951540, 220.370997, 229.276914, 0.1},
     {1455.064742, 219.321178, 83.824886, 0.000000, 0}},
    {{1473.900883, 218.618135, 227.521532, 0.1},
     {1476.873741, 216.864257, 146.806876, 0.000000, 0}},
    {{1495.675620, 216.878841, 225.780334, 0.1},
     {1498.439192, 214.449971, 210.925533, 0.002500, 8}},
    {{1517.277018, 215.150907, 224.051216, 0.1},
     {1519.765268, 212.076566, 275.998544, 0.004500, 15}},
    {{1538.692467, 213.164813, 222.050419, 0.1},
     {1540.855976, 209.742391, 319.912919, 0.006000, 21}},
    {{1559.896941, 210.929504, 219.786565, 0.1},
     {1561.715159, 207.445889, 342.042809, 0.007000, 26}},
    {{1580.868317, 208.503942, 217.321830, 0.1},
     {1582.346512, 205.185595, 345.908908, 0.007000, 26}},
    {{1601.589631, 205.929325, 214.699801, 0.1},
     {1602.753584, 202.960124, 333.870736, 0.007000, 26}},
    {{1622.055564, 203.396178, 212.121054, 0.1},
     {1622.939793, 200.768167, 320.739091, 0.007000, 26}},
    {{1642.270215, 200.903468, 209.584307, 0.1},
     {1642.908428, 198.608488, 306.570638, 0.007000, 26}},
    {{1662.237560, 198.449839, 207.088281, 0.1},
     {1662.662657, 196.479916, 291.390833, 0.007000, 26}},
    {{1681.961982, 196.058211, 204.657393, 0.1},
     {1682.205536, 194.381342, 277.209978, 0.006500, 23}},
    {{1701.451202, 193.731747, 202.294942, 0.1},
     {1701.540011, 192.311716, 264.533577, 0.006500, 23}},
    {{1720.712684, 191.503026, 200.035297, 0.1},
     {1720.668927, 190.270041, 256.296072, 0.006500, 23}},
    {{1739.752816, 189.304576, 197.807258, 0.1},
     {1739.595032, 188.255368, 247.391254, 0.006500, 23}},
    {{1758.574577, 187.135466, 195.609965, 0.1},
     {1758.320980, 186.266798, 237.832929, 0.006000, 21}},
    {{1777.180861, 184.994918, 193.442602, 0.1},
     {1776.849338, 184.303477, 227.643718, 0.006000, 21}},
    {{1795.578146, 182.954926, 191.381404, 0.1},
     {1795.182591, 182.364588, 222.810466, 0.006000, 21}},
    {{1813.772666, 180.939526, 189.345943, 0.1},
     {1813.323143, 180.449357, 217.548324, 0.006000, 21}},
    {{1831.767240, 178.960146, 187.348411, 0.1},
     {1831.273322, 178.557045, 212.859568, 0.006000, 21}},
    {{1849.565570, 177.010170, 185.381850, 0.1},
     {1849.035385, 176.686946, 208.325170, 0.006000, 21}},
    {{1867.170012, 175.082317, 183.438539, 0.1},
     {1866.611519, 174.838389, 203.439009, 0.006000, 21}},
    {{1884.582753, 173.176068, 181.517929, 0.1},
     {1884.003846, 173.010730, 198.214336, 0.006000, 21}},
    {{1901.805928, 171.290886, 179.619473, 0.1},
     {1901.214426, 171.203355, 192.660780, 0.006000, 21}},
    {{1918.841611, 169.426147, 177.742548, 0.1},
     {1918.245256, 169.415678, 186.779343, 0.006000, 21}},
    {{1935.691826, 167.581464, 175.886774, 0.1},
     {1935.098279, 167.647137, 180.588101, 0.006000, 21}},
    {{1952.358556, 165.756368, 174.051669, 0.1},
     {1951.775381, 165.897193, 174.097177, 0.006000, 21}},
    {{1968.843737, 163.950404, 172.236762, 0.1},
     {1968.278396, 164.165332, 167.316651, 0.006000, 21}},
    {{1985.149439, 162.169295, 170.448186, 0.1},
     {1984.609107, 162.451059, 160.761439, 0.005500, 19}},
    {{2001.278287, 160.410614, 168.683383, 0.1},
     {2000.769249, 160.753900, 154.307788, 0.005500, 19}},
    {{2017.233572, 158.697809, 166.967443, 0.1},
     {2016.760511, 159.073401, 149.942314, 0.005500, 19}},
    {{2033.018389, 157.001195, 165.268700, 0.1},
     {2032.584538, 157.409124, 145.412723, 0.005500, 19}},
    {{2048.634338, 155.320412, 163.586791, 0.1},
     {2048.242929, 155.760649, 140.723784, 0.005500, 19}},
    {{2064.082987, 153.655113, 161.921368, 0.1},
     {2063.737245, 154.127573, 135.880349, 0.005000, 17}},
    {{2079.365866, 152.004960, 160.272087, 0.1},
     {2079.069006, 152.509506, 130.886994, 0.005000, 17}},
    {{2094.486432, 150.408612, 158.680445, 0.1},
     {2094.239695, 150.906075, 128.945069, 0.005000, 17}},
    {{2109.448027, 148.825441, 157.102924, 0.1},
     {2109.250756, 149.316919, 126.948556, 0.005000, 17}},
    {{2124.251954, 147.255236, 155.539324, 0.1},
     {2124.103601, 147.741691, 124.903075, 0.005000, 17}},
    {{2138.899517, 145.698979, 153.990723, 0.1},
     {2138.799604, 146.180055, 122.911826, 0.005500, 19}},
    {{2153.392272, 144.158165, 152.458755, 0.1},
     {2153.340110, 144.631690, 121.125681, 0.005500, 19}},
    {{2167.729859, 142.595759, 150.903540, 0.1},
     {2167.726428, 143.096281, 116.544249, 0.005000, 17}},
    {{2181.911853, 141.046238, 149.362216, 0.1},
     {2181.959840, 141.573528, 111.846697, 0.005000, 17}},
    {{2195.941149, 139.541589, 147.869341, 0.1},
     {2196.041597, 140.063140, 109.680869, 0.005000, 17}},
    {{2209.820551, 138.048329, 146.388841, 0.1},
     {2209.972921, 138.564834, 107.486848, 0.005000, 17}},
    {{2223.551186, 136.566227, 144.920493, 0.1},
     {2223.755007, 137.078337, 105.264693, 0.005000, 17}},
    {{2237.134161, 135.095065, 143.464081, 0.1},
     {2237.389022, 135.603386, 103.014797, 0.005000, 17}},
    {{2250.570557, 133.634627, 142.019394, 0.1},
     {2250.876108, 134.139726, 100.737494, 0.005000, 17}},
    {{2263.861436, 132.184701, 140.586228, 0.1},
     {2264.217381, 132.687107, 98.432779, 0.005000, 17}},
    {{2277.007840, 130.745074, 139.164381, 0.1},
     {2277.413934, 131.245290, 96.100334, 0.005000, 17}},
    {{2290.010788, 129.315539, 137.753660, 0.1},
     {2290.466835, 129.814042, 93.739752, 0.005000, 17}},
    {{2302.871343, 127.897638, 136.355777, 0.1},
     {2303.377130, 128.393136, 91.493595, 0.005000, 17}},
    {{2315.590644, 126.489974, 134.969234, 0.1},
     {2316.145842, 126.982353, 89.271851, 0.005000, 17}},
    {{2328.169653, 125.091761, 133.593199, 0.1},
     {2328.773972, 125.581479, 87.028189, 0.005000, 17}},
    {{2340.609305, 123.702814, 132.227500, 0.1},
     {2341.262502, 124.190307, 84.762076, 0.005000, 17}},
    {{2352.910518, 122.322948, 130.871970, 0.1},
     {2353.612390, 122.808635, 82.472658, 0.005000, 17}},
    {{2365.074191, 120.951981, 129.526447, 0.1},
     {2365.824578, 121.436267, 80.159115, 0.005000, 17}},
    {{2377.101205, 119.589741, 128.190773, 0.1},
     {2377.899985, 120.073012, 77.820896, 0.005000, 17}},
    {{2388.992425, 118.236061, 126.864795, 0.1},
     {2389.839515, 118.718683, 75.457721, 0.004500, 15}},
    {{2400.748697, 116.890780, 125.548366, 0.1},
     {2401.644050, 117.373101, 73.069412, 0.004500, 15}},
    {{2412.371612, 115.568806, 124.257936, 0.1},
     {2413.314456, 116.036087, 71.891017, 0.004500, 15}},
    {{2423.862714, 114.254488, 122.976329, 0.1},
     {2424.851582, 114.707470, 70.727132, 0.005000, 17}},
    {{2435.222760, 112.947681, 121.703422, 0.1},
     {2436.256259, 113.387082, 69.575894, 0.005000, 17}},
    {{2446.451788, 111.634162, 120.423512, 0.1},
     {2447.529301, 112.074759, 67.280809, 0.004500, 15}},
    {{2457.549877, 110.328894, 119.153172, 0.1},
     {2458.671507, 110.770342, 65.008026, 0.004500, 15}},
    {{2468.518481, 109.044366, 117.906294, 0.1},
     {2469.683660, 109.473673, 63.796332, 0.004500, 15}},
    {{2479.358982, 107.766798, 116.667643, 0.1},
     {2480.566527, 108.184601, 62.593195, 0.004500, 15}},
    {{2490.072068, 106.496060, 115.437113, 0.1},
     {2491.320859, 106.902976, 61.396891, 0.005000, 17}},
    {{2500.658417, 105.231983, 114.214575, 0.1},
     {2501.947396, 105.628652, 60.202504, 0.005000, 17}},
    {{2511.118055, 103.961930, 112.985948, 0.1},
     {2512.446858, 104.361488, 57.982258, 0.004500, 15}},
    {{2521.451029, 102.698677, 111.765518, 0.1},
     {2522.819957, 103.101343, 55.729628, 0.004500, 15}},
    {{2531.658591, 101.453613, 110.566106, 0.1},
     {2533.067385, 101.848081, 54.388563, 0.004500, 15}},
    {{2541.741959, 100.214775, 109.374353, 0.1},
     {2543.189826, 100.601569, 53.045474, 0.004500, 15}},
    {{2551.701750, 98.982052, 108.190179, 0.1},
     {2553.187947, 99.361675, 51.699514, 0.004500, 15}},
    {{2561.538570, 97.755336, 107.013501, 0.1},
     {2563.062404, 98.128272, 50.349771, 0.004500, 15}},
    {{2571.253014, 96.534517, 105.844243, 0.1},
     {2572.813840, 96.901233, 48.995142, 0.004500, 15}},
    {{2580.845666, 95.319487, 104.682332, 0.1},
     {2582.442885, 95.680437, 47.634353, 0.004500, 15}},
    {{2590.317100, 94.110134, 103.527695, 0.1},
     {2591.950157, 94.465761, 46.266054, 0.004500, 15}},
    {{2599.667867, 92.906047, 102.379915, 0.1},
     {2601.336262, 93.257089, 44.863988, 0.004500, 15}},
    {{2608.898492, 91.707370, 101.239199, 0.1},
     {2610.601795, 92.054303, 43.445653, 0.004000, 13}},
    {{2618.009521, 90.514092, 100.105582, 0.1},
     {2619.747339, 90.857290, 42.017225, 0.004000, 13}},
    {{2627.002040, 89.337126, 98.991641, 0.1},
     {2628.773466, 89.665938, 41.480883, 0.004000, 13}},
    {{2635.877108, 88.165040, 97.884362, 0.1},
     {2637.680735, 88.480138, 40.961420, 0.004500, 15}},
    {{2644.635208, 86.997737, 96.783700, 0.1},
     {2646.469697, 87.299782, 40.456275, 0.004500, 15}},
    {{2653.276298, 85.824893, 95.677786, 0.1},
     {2655.140892, 86.124763, 39.124360, 0.004500, 15}},
    {{2661.800350, 84.656946, 94.578723, 0.1},
     {2663.694846, 84.954979, 37.775662, 0.004500, 15}},
    {{2670.207848, 83.493807, 93.486471, 0.1},
     {2672.132080, 83.790326, 36.409646, 0.004000, 13}},
    {{2678.499269, 82.335394, 92.400999, 0.1},
     {2680.453100, 82.630705, 35.026106, 0.004000, 13}},
    {{2686.675545, 81.190866, 91.333090, 0.1},
     {2688.658406, 81.476016, 34.382681, 0.004000, 13}},
    {{2694.737585, 80.050628, 90.271622, 0.1},
     {2696.748485, 80.326164, 33.745383, 0.004000, 13}},
    {{2702.685811, 78.914571, 89.216537, 0.1},
     {2700.750465, 79.753021, 0.000000, 0.000000, 0}},
    {{2710.520635, 77.782587, 88.167786, 0.1},
     {2708.668599, 78.610244, 0.000000, 0.000000, 0}},
    {{2718.244674, 76.698652, 87.177631, 0.1},
     {2716.472686, 77.472068, 0.000000, 0.000000, 0}},
    {{2725.860456, 75.617439, 86.192585, 0.1},
     {2724.163181, 76.338400, 0.000000, 0.000000, 0}},
    {{2733.368175, 74.537122, 85.210554, 0.1},
     {2731.740532, 75.209151, 0.000000, 0.000000, 0}},
    {{2740.767967, 73.459160, 84.233359, 0.1},
     {2739.205174, 74.084232, 2.638818, 0.000000, 0}},
    {{2748.060095, 72.383840, 83.261426, 0.1},
     {2746.557537, 72.963557, 3.232653, 0.000000, 0}},
    {{2755.244822, 71.311123, 82.294793, 0.1},
     {2753.798041, 71.847038, 3.925782, 0.000000, 0}},
    {{2762.322406, 70.240971, 81.333504, 0.1},
     {2760.927097, 70.734592, 4.714279, 0.000000, 0}},
    {{2769.293100, 69.173345, 80.377603, 0.1},
     {2767.945109, 69.626135, 5.594345, 0.000000, 0}},
    {{2776.157158, 68.108208, 79.427143, 0.1},
     {2774.852471, 68.521586, 6.562208, 0.000000, 0}},
    {{2782.914824, 67.045519, 78.482177, 0.1},
     {2781.649570, 67.420863, 7.614075, 0.000000, 0}},
    {{2789.566341, 65.985236, 77.542764, 0.1},
     {2788.336784, 66.323887, 8.746139, 0.000000, 0}},
    {{2796.111950, 64.927316, 76.608969, 0.1},
     {2794.914485, 65.230580, 9.954627, 0.000000, 0}},
    {{2802.551882, 63.871717, 75.680859, 0.1},
     {2801.383035, 64.140865, 11.235865, 0.000000, 0}},
    {{2808.886369, 62.818398, 74.758508, 0.1},
     {2807.742790, 63.054665, 12.586344, 0.000000, 0}},
    {{2815.115636, 61.767318, 73.841996, 0.1},
     {2813.994097, 61.971905, 14.002770, 0.000000, 0}},
    {{2821.239906, 60.718440, 72.931408, 0.1},
     {2820.137297, 60.892512, 15.482087, 0.001000, 3}},
    {{2827.259396, 59.671727, 72.026834, 0.1},
     {2826.172723, 59.816412, 17.021470, 0.002500, 8}},
    {{2833.174044, 58.621604, 71.121232, 0.1},
     {2832.100700, 58.743534, 18.163957, 0.003500, 11}},
    {{2838.983308, 57.564111, 70.209511, 0.1},
     {2837.921548, 57.673807, 18.557488, 0.004000, 13}},
    {{2844.686607, 56.502319, 69.295655, 0.1},
     {2843.635577, 56.607161, 18.407003, 0.004500, 15}},
    {{2850.283663, 55.439136, 68.383536, 0.1},
     {2849.243093, 55.543527, 17.919713, 0.004500, 15}},
    {{2855.774270, 54.373463, 67.471945, 0.1},
     {2854.744393, 54.482838, 16.989111, 0.004000, 13}},
    {{2861.158442, 53.310395, 66.567992, 0.1},
     {2860.139768, 53.425025, 16.011311, 0.004000, 13}},
    {{2866.436640, 52.253928, 65.677398, 0.1},
     {2865.429503, 52.370024, 15.317990, 0.004000, 13}},
    {{2871.609301, 51.199672, 64.794534, 0.1},
     {2870.613876, 51.317768, 14.573491, 0.003500, 11}},
    {{2876.676648, 50.147640, 63.919552, 0.1},
     {2875.693158, 50.268194, 13.781458, 0.003500, 11}},
    {{2881.639061, 49.100977, 63.056873, 0.1},
     {2880.667613, 49.221238, 13.202782, 0.003000, 9}},
    {{2886.496920, 48.056611, 62.202318, 0.1},
     {2885.537501, 48.176836, 12.604339, 0.003000, 9}},
    {{2891.250636, 47.018088, 61.360933, 0.1},
     {2890.303074, 47.134928, 12.280825, 0.003000, 9}},
    {{2895.900617, 45.981928, 60.527937, 0.1},
     {2894.964578, 46.095452, 11.968529, 0.003500, 11}},
    {{2900.447101, 44.948158, 59.703518, 0.1},
     {2899.522253, 45.058347, 11.674383, 0.003500, 11}},
    {{2904.890167, 43.913570, 58.883223, 0.1},
     {2903.976334, 44.023555, 11.140163, 0.003500, 11}},
    {{2909.229897, 42.881424, 58.072049, 0.1},
     {2908.327049, 42.991016, 10.622481, 0.003000, 9}},
    {{2913.466530, 41.851616, 57.270212, 0.1},
     {2912.574619, 41.960672, 10.118410, 0.003000, 9}},
    {{2917.600434, 40.826803, 56.482155, 0.1},
     {2916.719263, 40.932466, 9.851340, 0.003000, 9}},
    {{2921.631953, 39.803918, 55.703792, 0.1},
     {2920.761191, 39.906341, 9.588696, 0.003500, 11}},
    {{2925.561287, 38.783108, 54.935412, 0.1},
     {2924.700607, 38.882242, 9.346359, 0.003500, 11}},
    {{2929.388524, 37.762009, 54.173486, 0.1},
     {2928.537713, 37.860112, 8.935239, 0.003500, 11}},
    {{2933.113764, 36.743153, 53.422213, 0.1},
     {2932.272702, 36.839897, 8.556173, 0.003500, 11}},
    {{2936.737228, 35.726506, 52.681892, 0.1},
     {2935.905762, 35.821543, 8.212388, 0.003500, 11}},
    {{2940.259136, 34.711992, 51.952833, 0.1},
     {2939.437078, 34.804997, 7.903881, 0.003500, 11}},
    {{2943.679695, 33.699509, 51.235358, 0.1},
     {2942.866827, 33.790206, 7.628086, 0.003500, 11}},
    {{2946.999101, 32.688846, 50.529650, 0.1},
     {2946.195183, 32.777117, 7.373489, 0.003500, 11}},
    {{2950.217493, 31.679292, 49.835035, 0.1},
     {2950.997969, 31.260565, 12.867386, 0.010000, 41}},
    {{2953.335016, 30.671441, 49.153096, 0.1},
     {2954.073563, 30.251509, 12.283864, 0.010000, 48}},
    {{2956.350647, 29.641635, 48.441400, 0.1},
     {2957.048328, 29.243979, 11.696148, 0.010000, 48}},
    {{2959.263303, 28.611932, 47.739490, 0.1},
     {2959.922414, 28.237924, 11.123138, 0.010000, 48}},
    {{2962.073123, 27.584913, 47.052590, 0.1},
     {2962.695966, 27.233296, 10.566719, 0.010000, 48}},
    {{2964.780373, 26.560521, 46.381113, 0.1},
     {2965.369125, 26.230046, 10.026509, 0.010000, 48}},
    {{2967.385314, 25.538714, 45.725492, 0.1},
     {2967.942025, 25.228128, 9.502122, 0.010000, 48}},
    {{2969.888201, 24.519460, 45.086174, 0.1},
     {2970.414798, 24.227493, 8.993173, 0.010000, 48}},
    {{2972.289290, 23.502736, 44.463621, 0.1},
     {2972.787570, 23.228095, 8.499271, 0.010000, 48}},
    {{2974.588832, 22.488518, 43.858309, 0.1},
     {2975.060462, 22.229890, 8.020013, 0.010000, 48}},
    {{2976.787076, 21.476777, 43.270725, 0.1},
     {2977.233591, 21.232831, 7.554990, 0.010000, 48}},
    {{2978.884269, 20.467481, 42.701369, 0.1},
     {2979.307070, 20.236874, 7.103775, 0.010000, 48}},
    {{2980.880652, 19.460587, 42.150748, 0.1},
     {2981.281006, 19.241976, 6.665932, 0.010000, 48}},
    {{2982.776464, 18.456044, 41.619378, 0.1},
     {2983.155503, 18.248095, 6.241007, 0.010000, 48}},
    {{2984.571937, 17.453792, 41.107781, 0.1},
     {2984.930661, 17.255187, 5.828534, 0.010000, 48}},
    {{2986.267297, 16.453765, 40.616482, 0.1},
     {2986.606575, 16.263212, 5.428031, 0.010000, 48}},
    {{2987.862762, 15.455888, 40.146011, 0.1},
     {2988.183337, 15.272130, 5.039001, 0.010000, 48}},
    {{2989.358540, 14.460001, 39.696907, 0.1},
     {2989.661033, 14.281903, 4.660909, 0.010000, 48}},
    {{2990.754830, 13.466130, 39.269688, 0.1},
     {2991.039748, 13.292492, 4.293264, 0.010000, 48}},
    {{2992.051832, 12.474233, 38.864869, 0.1},
     {2992.319561, 12.303862, 3.935539, 0.010000, 48}},
    {{2993.249740, 11.484250, 38.482958, 0.1},
     {2993.500549, 11.315977, 3.587188, 0.010000, 48}},
    {{2994.348743, 10.496116, 38.124445, 0.1},
     {2994.582783, 10.328804, 3.247651, 0.010000, 48}},
    {{2995.349023, 9.509767, 37.789806, 0.1},
     {2995.566335, 9.342312, 2.916352, 0.010000, 48}},
    {{2996.250754, 8.525139, 37.479491, 0.1},
     {2996.451270, 8.356472, 2.592704, 0.010000, 48}},
    {{2997.054106, 7.542171, 37.193927, 0.1},
     {2997.237653, 7.371257, 2.276107, 0.010000, 48}},
    {{2997.759242, 6.560805, 36.933508, 0.1},
     {2997.593906, 6.878876, 0.000000, 0.000000, 18}},
    {{2998.366319, 5.580987, 36.698594, 0.1},
     {2998.232574, 5.894551, 0.000000, 0.000000, 0}},
    {{2998.875647, 4.605733, 36.513507, 0.1},
     {2998.772838, 4.910794, 0.000000, 0.000000, 0}},
    {{2999.287586, 3.633139, 36.370710, 0.1},
     {2999.214753, 3.927587, 0.000000, 0.000000, 0}},
    {{2999.602292, 2.661052, 36.253879, 0.1},
     {2999.558375, 2.944913, 0.000000, 0.000000, 0}},
    {{2999.819812, 1.689438, 36.163201, 0.1},
     {2999.803756, 1.962763, 0.000000, 0.000000, 0}},
    {{2999.940194, 0.718271, 36.098808, 0.1},
     {2999.950947, 0.981127, 0.000000, 0.000000, 0}},
    {{2999.963481, -0.252449, 36.060779, 0.1},
     {2999.950947, 0.981127, 0.000000, 0.000000, 0}},
};
