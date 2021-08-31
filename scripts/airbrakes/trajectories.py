import scipy.io as sio
import sys

#filename = "Trajectories.mat"
outfilename = "Trajectories_data.h"
fieldname = "trajectories_saving"

if len(sys.argv) < 2:
    print("\nError, missing path to file")
    print("Usage : python3 coeffs.py <path_to_mat_file>\n")

mat = sio.loadmat(sys.argv[1])
trajectories = mat[fieldname][0]

with open(outfilename, "w") as f:
    num_trajectories = len(trajectories)
    max_trajectory_len = 0
    for t in trajectories:
        # check length of first column for each trajectory
        if len(t[0]) > max_trajectory_len:
            max_trajectory_len = len(t[0])

    f.write("static const unsigned int TOT_TRAJECTORIES = " + str(num_trajectories) + ";\n")
    f.write("static const unsigned int TRAJECTORY_MAX_LENGTH = " + str(max_trajectory_len) + ";\n\n")

    f.write("const trajectory_t TRAJECTORIES_DATA[TOT_TRAJECTORIES] = {\n")
    for trajectory in trajectories:
        zs, vzs, xs, vxs, ys, vys = trajectory
        f.write("\t{\n")
        f.write("\t\t%d, \n\t\t{\n" % (len(zs)))
        for z, vz in zip(zs, vzs):
            f.write("\t\t\t{%f, %f},\n" % (z, vz))
        f.write("\t\t}\n\t},\n")
    f.write("};")

print("\nFile " + outfilename + " written")
