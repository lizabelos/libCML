import os
import random
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
from utils import dprint
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
        else:
            print("Unknown dataset type : " + line[0])

    datasets_names = [x.name() for x in datasets]

    return datasets, datasets_names, slams, slams_names

def intrange(a, b, c = 1):
    return [int(x) for x in np.arange(a,b,c)]

def floatrange(a, b, c = 1.0):
    return [float(x) for x in np.arange(a,b,c)]

def bruteforceFindBest(currentParam):
    datasets, datasets_names, slams, slams_names = parse_config()
    alteration = list(floatrange(1,10.5,0.5))
    alteration = [1/x for x in alteration[::-1] if x != 1] + alteration

    pow10tmp = [math.pow(10,i) for i in range(1,11)]
    pow10 = []
    for i in range(0,len(pow10tmp)):
       for j in [1.0, 5.0]:
            pow10 = pow10 + [pow10tmp[i] * j]
    #pow10 = pow10tmp

    print(pow10)

    params = [
        #["dsoTracer.desiredPointDensity", intrange(250,2200,250)],
        #["dsoTracer.immatureDensity", intrange(250,2200,250)],
       # ["dsoInitializer.densityFactor", floatrange(0.1,1.05,0.05)],
        #
        ["bacondMinimumOrbPoint",  intrange(0,300,10)],
        ["bacondSaturatedRatio", floatrange(0.01,0.16,0.01)],
        ["dsoInitializer.regularizationWeight", floatrange(0.0, 1.05, 0.05)],
        ["dsoInitializer.pointDensity", intrange(600,2200,200)],
        ["orbInlierRatioThreshold", floatrange(0.35,0.75,0.01)],
        #["orbUncertaintyThreshold", [-1] + pow10],
        ["dsoTracker.saturatedThreshold", floatrange(0.30,0.45,0.01)],
        ["orbKeyframeRatio", floatrange(0.30,0.95,0.01)],
        ["orbKeyframeReflimit",  intrange(0,300,10)],
        ["dsoKeyframeResidualRatio", [2.0 * x for x in alteration]],
        ["dsoKeyframeResidualRatio", [1.0 * x for x in alteration]],
        ["orbInlierNumThreshold", intrange(0,35,5)],
        ["trackcondUncertaintyWeight", floatrange(0.1,2,0.1)],
        ["trackcondUncertaintyWindow", intrange(1,30)],
        ["bacondScoreWeight", [0.0125*x for x in alteration]],
        ["bacondScoreWindow", intrange(1,30)],
        #["numOrbCorner", intrange(500,2050,50)],
        ["dsoBa.iterations", intrange(1,8)],
        ["dsoBa.maxFrames", intrange(4,8)],
        #["dsoBa.optimizeLightA", ["true", "false"]],
        #["dsoBa.optimizeLightB", ["true", "false"]],
        #["dsoTracker.optimizeLightA", ["true", "false"]],
        #["dsoTracker.optimizeLightB", ["true", "false"]],
        ["dsoBa.fixedLambda", [0.000030, 0.000035, 0.000040, 0.000045, 0.000046, 0.000047, 0.000048, 0.000049, 0.000050, 0.000051, 0.000052, 0.000053, 0.000054, 0.000055, 0.000060, 0.000065, 0.000070, 0.000075, 0.000080]],
        #["bacondUncertaintyWeight", floatrange(0,2,0.2)],
        #["bacondUncertaintyWindow", intrange(1,8)],
        ["orbBa.numIteration", intrange(0,10)],
        ["orbBa.refineIteration", intrange(0,10)],
        ["orbBa.removeEdge", ["true", "false"]],
        ["bacondSaturatedRatio", floatrange(0,0.25,0.01)]
    ]

    dprint("Hello :)\n\n")
    # currentParam = {'numOrbCorner': 500, 'trackcondUncertaintyWeight': 0.4, 'bacondScoreWeight': 0.02, 'trackcondUncertaintyWindow': 8}
    # currentParam = {'dsoInitializer.densityFactor': 0.9, 'dsoTracker.saturatedThreshold': 0.39}
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=3)
    currentMin = 99999999
    currentMaxSuccess = 0
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
                currentSuccess = 0

                for i in range(0, len(datasets)):

                    s = slams[0]
                    name = slams_names[0]
                    context = s[0](s[1], "modslam.yaml")
                    for p in currentParam:
                        context.setconfig(p, currentParam[p])
                    context.setconfig(param[0], v)
                    if datasets[i].gt() is None:
                        continue
                    try:
                        ate = evaluateOn(context, datasets[i])
                        toprint = toprint + str(ate) + "\t"
                        currentSum = currentSum + ate
                        currentSuccess = currentSuccess + 1
                        if ate > datasets[i].lim() * 1.2:
                            break
                    except Exception as e:
                        toprint = toprint + context.getError() + "\t"
                        break
                return currentSum, currentSuccess, toprint

            for v in param[1]:
                futures = futures + [executor.submit(process, param, v)]
                progress_tot = progress_tot + 1
            # concurrent.futures.wait(futures)
            print(str(int(progress_i * 100 / progress_tot)) + "%", flush=True)
            for f in futures:
                currentSum, currentSuccess, toprint = f.result()
                allSums.append(currentSum)
                allSuccess.append(currentSuccess)
                dprint(toprint)
                progress_i = progress_i + 1
                print(str(int(progress_i * 100 / progress_tot)) + "%", flush=True)

            currentMinI = None
            for i in range(0, len(allSums)):
                if allSuccess[i] < currentMaxSuccess:
                    continue
                if allSuccess[i]>currentMaxSuccess or allSums[i] < currentMin:
                    currentMin = allSums[i]
                    currentMaxSuccess = allSuccess[i]
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
    pow10 = [-1] + [math.pow(10,i) for i in range(1,11)]
    for i in pow10:
        p = {"orbUncertaintyThreshold": i}
        bruteforceFindBest(p)
