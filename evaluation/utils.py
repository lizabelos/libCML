import subprocess
from subprocess import Popen
import csv
import os


def system(command):
    # os.system(command)
    # os.system(command + ">/dev/null 2>/dev/null")
    # return "", 0
    for l in csv.reader([command], delimiter=' ', quotechar='"'):
        executable = l[0]
        wd = os.path.dirname(os.path.realpath(executable))
        # print("#" + str(l) + " ==> (" + mode + ")" + outputPath)
        p = Popen(l, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=wd)
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
    with open("output.txt", "a") as myfile:
        myfile.write(str(s) + end)
