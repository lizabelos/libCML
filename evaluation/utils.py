import subprocess
from subprocess import Popen
import csv
import os


def system(command):
    for l in csv.reader([command], delimiter=' ', quotechar='"'):
        executable = l[0]
        wd = os.path.dirname(os.path.realpath(executable))
        # print("#" + str(l) + " ==> (" + mode + ")" + outputPath)
        p = Popen(l, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=wd)
        out, err = p.communicate()
        errcode = p.returncode
        # print("Command finished")
        return out.decode("utf-8"), err
