import math
import subprocess
import time
from subprocess import Popen
import csv
import os
import numpy as np
import threading
import time
from threading import Timer

currentStatus = {}

def autoRound(v):
    n = 100
    p = v
    plast = v
    while n > 1:
        plast = p
        p = round(v, n)
        if not math.isclose(p, v, rel_tol=0.1):
            return plast
        n = n - 1
    return p


def intrange(a, b, c = 1):
    return [int(x) for x in np.arange(a,b,c)]

def floatrange(a, b, c = 1.0):
    return [float(x) for x in np.arange(a,b,c)]

def system(command, comment="", disable_openmp=True, time_limit=None):
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

        start_time = time.time()
        isSLAM = False
        # print("#" + str(l) + " ==> (" + mode + ")" + outputPath)
        p = Popen(l, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, cwd=wd, env=my_env)
        timer = Timer(time_limit, p.kill)

        lines = []
        while p.poll() is None:
            if time_limit is not None:
                if time.time() - start_time > time_limit:
                    p.kill()
                    break
            line = p.stdout.readline().decode("utf-8")
            lines.append(line)
            if "frame " in line and not isSLAM:
                isSLAM = True
                start_time = time.time()
            if "(" in line and ")" in line:
                line = comment + "," + line[line.find("(")+1:line.find(")")]
                currentStatus[threading.get_ident()] = line

        timer.cancel()
        end_time = time.time()

        return lines, 0, end_time - start_time

def cls():
    os.system('cls' if os.name=='nt' else 'clear')

def printStatusThread():
    while True:
        toPrint = ""
        currentStatusCopy = currentStatus
        for threadId in currentStatusCopy:
            toPrint = toPrint + "#" + str(threadId) + ": " + str(currentStatusCopy[threadId]) + "\n"
        cls()
        print(toPrint, end='')
        time.sleep(5)

def launchPrintStatusThread():
    thread = threading.Thread(target = printStatusThread)
    thread.daemon = True
    thread.start()

def dprint(s, end="\n"):
    if "\t" in s:
        sconsole = s.split("\t")
        sconsole = ["{:<15}".format(x) for x in sconsole]
        sconsole = "|".join(sconsole)
        pass
        # currentStatus[threading.get_ident()] = sconsole
        # print(sconsole, end=end)
    else:
        # currentStatus[threading.get_ident()] = s
        # print(s, end=end)
        pass
    with open("output.csv", "ab") as myfile:
        encoded = str(s) + end
        encoded = encoded.encode("utf-8")
        myfile.write(encoded)

def cprint(s, end="\n"):
    currentStatus[threading.get_ident()] = s

if __name__ == "__main__":
    print(autoRound(0.00000000001599999)) # 0.000000000016
    print(autoRound(0.000001599995)) # 0.0000016
    print(autoRound(0.1599999)) # 0.16
    print(autoRound(0.9999999)) # 1.0
    print(autoRound(math.pi)) # 3.14