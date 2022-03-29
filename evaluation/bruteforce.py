import os
import random
import re
import sys
import csv
import math

import concurrent
import concurrent.futures
import multiprocessing
from statistics import mean
import statistics
import numpy as np

from evo.tools.file_interface import csv_read_matrix

import dataset
import evaluator
import slam
from table import FileTable, MedianTableProxy, SumTableProxy
from utils import dprint, cprint, launchPrintStatusThread
from evaluator import evaluateOn


def parse_config():
    config_file = "config.txt"
    content = csv_read_matrix(config_file, delim=" ", comment_str="#")

    datasets = []
    slams = []
    slams_names = []

    for line in content:
        if len(line) == 0:
            continue
        if line[0] == "ModSLAM":
            slams.append([slam.ModSLAM, line[1]])
            slams_names.append(line[0])
        elif line[0] == "TUM":
            datasets = datasets + dataset.TUM(line[1])
        elif line[0] == "KITTI":
            datasets = datasets + dataset.KITTI(line[1])
        elif line[0] == "Stereopolis":
            datasets = datasets + dataset.Stereopolis(line[1])
        else:
            print("Unknown dataset type : " + line[0])

    datasets_names = [x.name() for x in datasets]

    return datasets, datasets_names, slams, slams_names

def intrange(a, b, c = 1):
    return [int(x) for x in np.arange(a,b,c)]

def floatrange(a, b, c = 1.0):
    return [float(x) for x in np.arange(a,b,c)]

def movingAverage(t, iter = 5, delta = 0.75):
    delta_inv = (1 - delta) / 2
    for i in range(0, iter):
        for j in range(1, len(t) - 1):
            t[j] = t[j] * delta + t[j - 1] * delta_inv + t[j + 1] * delta_inv
    return t

def numFramesOf(dataset):

    if not " " in dataset:
        return 1

    if dataset.startswith("TUM"):
        return 6

    numFrames=[4541,1101,4661,801,271,2761,1101,1101,4071,1591,1201]

    return numFrames[int(dataset.split(" ")[1])]

def criteria(dataset, ate, fps):
    if ate is None:
        return 9999999

    return ate / numFramesOf(dataset)

def bruteforceFindBest(currentParam):
    datasets, datasets_names, slams, slams_names = parse_config()
    alteration = list(floatrange(1,10.5,0.5))
    alteration = [1/x for x in alteration[::-1] if x != 1] + alteration

    pow10tmp = [math.pow(10,i) for i in range(-5,5)]
    pow10 = []
    for i in range(0,len(pow10tmp)):
       for j in [1.0, 5.0]:
            pow10 = pow10 + [pow10tmp[i] * j]
    #pow10 = pow10tmp

    print(pow10)

    weightAndInv = floatrange(0,1.25,0.25) + [1/x for x in floatrange(0.25,1,0.25)]
    params = [
        ["trackingMinimumOrbPoint", intrange(0,525,25)],
        ["bacondMinimumOrbPoint",  intrange(0,525,25)],
        ["bacondSaturatedRatio", floatrange(0.0,1.2,0.2)],
        ["trackcondUncertaintyWeight", weightAndInv],
        ["trackcondUncertaintyWindow", intrange(1,30,5)],
        ["bacondScoreWeight", weightAndInv],
        ["bacondScoreWindow", intrange(1,30,5)],
        ["orbInlierRatioThreshold", floatrange(0.0,1.1,0.1)],
        ["orbUncertaintyThreshold", [-1,0.1,1,10,100,1000,10000]],
        ["dsoTracker.saturatedThreshold", floatrange(0.0,1.2,0.2)],
        ["orbKeyframeRatio", floatrange(0.70,0.95,0.01)],
        ["orbInlierNumThreshold", intrange(0,100,10)],
    ]

    dprint("Hello :)\n\n")
    launchPrintStatusThread()
    # currentParam = {'numOrbCorner': 500, 'trackcondUncertaintyWeight': 0.4, 'bacondScoreWeight': 0.02, 'trackcondUncertaintyWindow': 8}
    # currentParam = {'dsoInitializer.densityFactor': 0.9, 'dsoTracker.saturatedThreshold': 0.39}
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=10)
    currentMin = 99999999
    while True:
        bestParamModif = None
        for param in params:
            allSums = []
            allSuccess = []
            futures = []
            progress_i = 0
            progress_tot = 0
            headerLine = param[0]
            for i in range(0, len(datasets)):
                headerLine = headerLine + "\t" + datasets[i].name()
            dprint(currentParam)
            dprint(headerLine)

            def process(param, v):
                toprint = str(v) + "\t"
                # dprint(str(param[0]) + "=" + str(v) + "; ", end='')
                currentSum = 0

                for i in range(0, len(datasets)):

                    s = slams[0]
                    name = slams_names[0]
                    context = s[0](s[1], "modslam2.yaml")
                    for p in currentParam:
                        context.setconfig(p, currentParam[p])
                    context.setconfig(param[0], v)
                    try:
                        ate = evaluateOn(context, datasets[i])
                        fps = context.getTime() / numFramesOf(datasets[i].name())
                        toprint = toprint + str(float(int(ate * 10) / 10)) + " at " + str(int(fps)) + "\t"
                        currentSum = currentSum + criteria(datasets[i].name(), ate, fps)
                    except KeyboardInterrupt:
                        return 0,0,""
                    except Exception as e:
                        slamLog = context.getError()
                        lastLine = [x for x in slamLog.split("\n") if "frame " in x][-1]
                        numFrame = re.findall(r'\d+', lastLine)[0]
                        err = 10000 - int(numFrame)
                        toprint = toprint + str(err) + "\t"
                        currentSum = currentSum + err
                return currentSum, toprint

            for v in param[1]:
                futures = futures + [executor.submit(process, param, v)]
                progress_tot = progress_tot + 1
            # concurrent.futures.wait(futures)
            cprint(str(param[0]) + " : " + str(int(progress_i * 100 / progress_tot)) + "%")
            for f in futures:
                currentSum, toprint = f.result()
                allSums.append(currentSum)
                dprint(toprint)
                progress_i = progress_i + 1
                cprint(str(param[0]) + " : " + str(int(progress_i * 100 / progress_tot)) + "%")

            allSums = movingAverage(allSums)

            currentMinI = None
            for i in range(0, len(allSums)):
                if allSums[i] < currentMin:
                    currentMin = allSums[i]
                    currentMinI = i

            if currentMinI is not None:
                bestParamModif = [param[0],param[1][currentMinI]]
                # currentParam[param[0]] = param[1][currentMinI]

            #dprint("=========================")
            #dprint(currentParam)
            #dprint("=========================")
            dprint("\n\n\n")
            #dprint("Best : " + str(currentMin))
        if bestParamModif is not None:
            currentParam[bestParamModif[0]] = bestParamModif[1]
        else:
            break

        dprint("END OF ITERATION")
        dprint("\n\n\n\n\n")

if __name__ == "__main__":
    bruteforceFindBest({})
    #pow10 = [math.pow(10,i) for i in range(1,11)] + [-1]
    #pow10.reverse()
    #print(pow10)
    #for i in pow10:
    #    p = {"orbUncertaintyThreshold": i}
    #    bruteforceFindBest(p)
