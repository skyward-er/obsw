import scipy.io as sio
import sys

#filename = "Trajectories.mat"
outfilename = "Trajectories.h"
fieldname = "trajectories_saving"

if len(sys.argv) < 2:
    print("\nError, missing path to file")
    print("Usage : python3 coeffs.py <path_to_mat_file>\n")

outfilename = sys.argv[1].replace(".mat", "") + ".h" # use same name as input file

mat = sio.loadmat(sys.argv[1])
trajectories = mat[fieldname][0]

with open(outfilename, "w") as f:

    num_trajectories = len(trajectories)
    max_trajectory_len = 0
    trajectory_index = 0
    output_string = ""

    for trajectory in trajectories:
        zs, vzs, xs, vxs, ys, vys = trajectory

        for i in range(0, len(zs)):
            if zs[i] >= 0: # find first non negative altitude
                break

        # exctract only non negative altitude elements
        zs = zs[i:]
        vzs = vzs[i:]

        if (len(zs) != len(vzs)):
            print("ERROR : z and vz don't have the same number of elements in trajectory " + str(trajectory_index))

        # check length of first column for each trajectory
        if len(zs) > max_trajectory_len:
            max_trajectory_len = len(zs)

        # output trajectory to file
        output_string += "\t{\n"
        output_string += "\t\t%d, \n\t\t{\n" % (len(zs))
        for z, vz in zip(zs, vzs):
            output_string += "\t\t\t{%f, %f},\n" % (z, vz)
        output_string += "\t\t}\n\t},\n"

        trajectory_index += 1

    output_string += "};"

    # after the preprocessing, output max trajectories length
    s = "static const unsigned int TOT_TRAJECTORIES = " + str(num_trajectories) + ";\n"
    s += "static const unsigned int TRAJECTORY_MAX_LENGTH = " + str(max_trajectory_len) + ";\n\n"
    s += "const trajectory_t TRAJECTORIES_DATA[TOT_TRAJECTORIES] = {\n"

    # final string to be output to file
    output_string = s + output_string

    f.write(output_string)

print("\nFile " + outfilename + " written")
