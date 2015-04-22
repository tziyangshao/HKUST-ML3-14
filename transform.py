import sys
import subprocess

lines=str.split(subprocess.check_output("wc -l "+sys.argv[1],shell=True))
line=int(lines[0])
with open("head.vtk","w") as datafile:
    datafile.write("# vtk DataFile Version 3.0\n");
    datafile.write("File created by matlab\n");
    datafile.write("ASCII\n");
    datafile.write("DATASET POLYDATA\n");
    datafile.write("POINTS %d float\n"%(line));
with open("tail.vtk","w") as datafile:
    datafile.write("VERTICES %d %d\n"%(line,line*2));
    for n in range(line):
        datafile.write("1 %d\n"%(n))
    datafile.write("POINT_DATA %d"%(line));
print(line)
subprocess.check_output("cat tail.vtk >> "+sys.argv[1],shell=True)
subprocess.check_output("cat "+sys.argv[1]+" >> head.vtk",shell=True)
print(line)
