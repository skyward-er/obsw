#!/usr/bin/env python3
import csv

# Config
numberOfReferences = 2
fileName = "ABK_references_2024_Lyra_Roccaraso_September.csv"

# The CSV is structured such that the first column is the height,
# and the numberOfReferences successive ones are the closed ones,
# then another numberOfReferences columns which are the open ones.

file = open(fileName)
csvReader = csv.reader(file, delimiter=",")

table = []

# Read the file
lineCount = 0
for row in csvReader:
    if (lineCount == 0):
        # Name row
        print("//" + str([str(i) + " " for i in row]))
        for column in row:
            # Create a list for every column
            table.append([str(column)])

    else:
        # Insert the elements inside the list
        for i in range(len(row)):
            table[i].append(row[i])
    lineCount += 1

# Creation of C++ references

# Closed
for i in range(numberOfReferences):
    # We do not consider the first column (Height)
    index = i + 1
    cppArrayDecl = f"TrajectoryPoint t{i}_closed[] = " + "{\n"

    for r in range(1, len(table[i])):
        z = table[0][r]
        vz = table[index][r]
        # For every entry except of the first, create a trajectory point
        cppArrayDecl += f"\tTrajectoryPoint({z}, {vz}),\n"

    cppArrayDecl += "};"
    print(cppArrayDecl)

# Open
for i in range(numberOfReferences):
    # We do not consider the first column (Height) and shift of the closed ones
    index = i + 1 + numberOfReferences
    cppArrayDecl = f"TrajectoryPoint t{i}_open[] = " + "{\n"

    for r in range(1, len(table[i])):
        z = table[0][r]
        vz = table[index][r]
        # For every entry except of the first, create a trajectory point
        cppArrayDecl += f"\tTrajectoryPoint({z}, {vz}),\n"

    cppArrayDecl += "};"
    print(cppArrayDecl)

# Trajectory sets

# Closed
print("Trajectory t_closed[] = {")
for i in range(numberOfReferences):
    l = len(table[0]) - 1
    print(f"\tTrajectory{{0.0, t{i}_closed, {l}}},")
print("};")

# Open
print("Trajectory t_open[] = {")
for i in range(numberOfReferences):
    l = len(table[0]) - 1
    print(f"\tTrajectory{{0.0, t{i}_open, {l}}},")
print("};")

# Create the trajectory sets
print(
    f"TrajectorySet Main::Data::ABK::CLOSED_TRAJECTORY_SET(t_closed, {numberOfReferences});")
print(
    f"TrajectorySet Main::Data::ABK::OPEN_TRAJECTORY_SET(t_open, {numberOfReferences});")
