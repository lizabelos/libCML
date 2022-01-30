import subprocess
from subprocess import Popen
import csv
import os
import numpy as np

def intrange(a, b, c = 1):
    return [int(x) for x in np.arange(a,b,c)]

def floatrange(a, b, c = 1.0):
    return [float(x) for x in np.arange(a,b,c)]

def system(command, disable_openmp=True):
    my_env = os.environ.copy()
    if disable_openmp:
        my_env["OMP_NUM_THREADS"]="1"
    # os.system(command)
    # os.system(command + ">/dev/null 2>/dev/null")
    # return "", 0
    # print(command)
    for l in csv.reader([command], delimiter=' ', quotechar='"'):
        executable = l[0]
        wd = os.path.dirname(os.path.realpath(executable))
        # print("#" + str(l) + " ==> (" + mode + ")" + outputPath)
        p = Popen(l, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=wd, env=my_env)
        out, err = p.communicate()
        errcode = p.returncode
        # print("Command finished")
        return out.decode("utf-8"), err

def dprint(s, end="\n"):
    if "\t" in s:
        sconsole = s.split("\t")
        sconsole = ["{:<15}".format(x) for x in sconsole]
        sconsole = "|".join(sconsole)
        print(sconsole, end=end)
    else:
        print(s, end=end)
    with open("output.csv", "ab") as myfile:
        encoded = str(s) + end
        encoded = encoded.encode("utf-8")
        myfile.write(encoded)
