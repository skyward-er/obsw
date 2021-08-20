import scipy.io as sio
import sys

#filename = "coeffs.mat"
fieldname = "coeffs"

if len(sys.argv) < 2:
    print("\nError, missing path to file")
    print("Usage : python3 coeffs.py <path_to_mat_file>\n")
    
mat = sio.loadmat(sys.argv[1])

data = mat[fieldname][0][0]
names = data.dtype.fields.keys()

coeffs = {}

for coeff, name in zip(data, names):
    coeffs[name] = coeff.item()

print("struct Coefficients\n{")
for name in coeffs: # coeff name = dict key
    print("\tfloat " + name + " = " + str(coeffs[name]) + ";")
print("};")
