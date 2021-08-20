import scipy.io as sio
import sys

#filename = "Trajectories.mat"
fieldname = "trajectories_saving"

if len(sys.argv) < 2:
    print("\nError, missing path to file \nUsage : python3 coeffs.py <path_to_mat_file>\n")

mat = sio.loadmat(sys.argv[1])
trajectories = mat[fieldname][0]

with open("Trajectories_data.h", "w") as f:
    f.write("{\n")
    for trajectory in trajectories:
        zs, vzs, xs, vxs, ys, vys, sbar = trajectory
        f.write("\t{\n")
        f.write("\t\t%d, %d,\n\t\t{\n" % (len(zs), sbar))
        for z, vz in zip(zs, vzs):
            f.write("\t\t\t{%f, %f},\n" % (z, vz))
        f.write("\t\t}\n\t},\n")
    f.write("}")
