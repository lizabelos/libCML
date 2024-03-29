import copy
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

def movingAverage(t, iter = 2, delta = 0.75):
    delta_inv = (1 - delta) / 2
    for i in range(0, iter):
        tcopy = t.copy()
        for j in range(1, len(t) - 1):
            t[j] = tcopy[j] * delta + tcopy[j - 1] * delta_inv + tcopy[j + 1] * delta_inv
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

def processParameters(currentParam):
    currentParam = copy.deepcopy(currentParam)
    currentParamCopy = copy.deepcopy(currentParam)
    for p in currentParamCopy:
        if ":" in p:
            ps = p.split(":")
            p_name = ps[0]
            currentParam[p_name] = ""

    for p in currentParamCopy:
        if ":" in p:
            ps = p.split(":")
            p_name = ps[0]
            if currentParam[p_name] != "":
                currentParam[p_name] = currentParam[p_name] + ";"
            currentParam[p_name] = currentParam[p_name] + str(currentParam[p])
    return currentParam

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

    weightAndInv = floatrange(0,1.05,0.05) + [1/x for x in floatrange(0.05,1,0.05)]


    params = [
        ["graphTrackDist", intrange(0,20,1)],
        ["graphDistScore", intrange(0,2000,100)],
        ["graphDescriptorScoreA", intrange(0,2000,100)],
        ["graphDescriptorScoreB", intrange(0,2000,100)],
        ["graphLevelScore", intrange(0,2000,100)],
        ["graphScoreThreshold", intrange(0,2000,100)],
        ["graphScoreRatio", floatrange(0.0,1.1,0.1)],
        ["graphMininumMatching", intrange(0,200,5)],
    ]

    params = params + [
        ["trackingMinimumOrbPoint", intrange(0,205,5)],
        ["trackcondFlowThreshold", floatrange(0.0,2.1,0.1)],
        ["trackcondUncertaintyWeight", weightAndInv + [-1]],
        ["trackcondUncertaintyWindow", [1, -2, 2, 3, 4, 5, 6, 7, 8, 9, 10]],
        ["bacondMinimumOrbPoint", intrange(0,205,5)],
        ["bacondTrackThresholdOrb", floatrange(0.0,1.05,0.05)],
        ["bacondTrackThresholdDso", floatrange(0.0,1.05,0.05)],
        ["bacondSaturatedRatio", floatrange(0.0,1.05,0.05)],
        ["bacondScoreWeight", weightAndInv],
        ["bacondScoreWindow", intrange(1,20,1)],
        ["numOrbMultiplier", floatrange(1.0,2.1,0.1)],
        ["orbInlierRatioThreshold", floatrange(0.0,1.1,0.1)],
        ["orbKeyframeRatio", floatrange(0.70,0.95,0.01)],
        ["orbInlierNumThreshold", intrange(0,100,5)],
    ]

    # new params
    # Parameter mGraphTrackDist = createParameter("graphTrackDist", 14);
    # Parameter mGraphDistScore = createParameter("graphDistScore", 1000);
    # Parameter mGraphDescriptorScoreA = createParameter("graphDescriptorScoreA", 1000);
    # Parameter mGraphDescriptorScoreB = createParameter("graphDescriptorScoreB", 1000);
    # Parameter mGraphLevelScore = createParameter("graphLevelScore", 1000);
    # Parameter mGraphScoreThreshold = createParameter("graphScoreThreshold", 2000);
    # Parameter mGraphScoreRatio = createParameter("graphScoreRatio", 0.8f);
    # Parameter mGraphMininumMatching = createParameter("graphMininumMatching", 50);



    seedParamName = "dsoInitializer.regularizationWeight"
    seedParamValues = [0.5,0.55,0.6,0.65,0.7]
    #seedParamName = "orb.nLevels"
    #seedParamValues = [1,2,3,4,5]

    dprint("Hello :)\n\n")
    launchPrintStatusThread()
    # currentParam = {'numOrbCorner': 500, 'trackcondUncertaintyWeight': 0.4, 'bacondScoreWeight': 0.02, 'trackcondUncertaintyWindow': 8}
    # currentParam = {'dsoInitializer.densityFactor': 0.9, 'dsoTracker.saturatedThreshold': 0.39}
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=3)
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
                for seed in seedParamValues:
                    headerLine = headerLine + "\t" + datasets[i].name()
            dprint(currentParam)
            dprint(headerLine)

            def process(param, v):
                toprint = str(v) + "\t"
                # dprint(str(param[0]) + "=" + str(v) + "; ", end='')
                currentSum = 0

                for i in range(0, len(datasets)):
                    isAllErrorForThisSeed = True
                    for seed in seedParamValues:

                        s = slams[0]
                        name = slams_names[0]
                        context = s[0](s[1], "modslam2.yaml")

                        currentParamCopy = copy.deepcopy(currentParam)
                        currentParamCopy[param[0]] = v
                        currentParamCopy[seedParamName] = seed
                        currentParamCopy = processParameters(currentParamCopy)

                        for p in currentParamCopy:
                            if ":" in p:
                                continue
                            context.setconfig(p, currentParamCopy[p])

                        try:
                            ate = evaluateOn(context, datasets[i])
                            fps = numFramesOf(datasets[i].name()) / context.getTime()
                            toprint = toprint + datasets[i].name() + "\t"
                            toprint = toprint + str(float(int(ate * 10) / 10)) + " at " + str(int(fps)) + "\t"
                            currentSum = currentSum + criteria(datasets[i].name(), ate, fps)
                            isAllErrorForThisSeed = False
                        except KeyboardInterrupt:
                            return 0,0,""
                        except Exception as e:

                            # dump the exception into a file
                            with open("error.txt", "a") as f:
                                f.write(str(e))
                                # dump the stack
                                import traceback
                                traceback.print_exc(file=f)

                            err = 10000
                            try:
                                slamLog = context.getError()
                                # lastLine = [x for x in slamLog.split("\n") if "frame " in x][-1]
                                # since the update, lines are in this format : "[07:51:02 +02:00] [frame 643;40%] [info] [thread 10052] Applying bundle adjustment step."
                                lastLine = [x for x in slamLog.split("\n") if "frame " in x][-1]
                                lastLine = lastLine.split("frame ")[1]
                                lastLine = lastLine.split(";")[0]
                                numFrame = int(lastLine)
                                err = 10000 - int(numFrame)
                            except Exception as e:

                                # dump the exception into a file
                                with open("error.txt", "a") as f:
                                    f.write(str(e))
                                    # dump the stack
                                    import traceback
                                    traceback.print_exc(file=f)
                            toprint = toprint + str(err) + "\t"
                            currentSum = currentSum + err
                    if False and isAllErrorForThisSeed:
                        # don't continue. consider all the other datasets as err = 10000
                        for j in range(i + 1, len(datasets)):
                            for seed in seedParamValues:
                                toprint = toprint + datasets[j].name() + "\t"
                                toprint = toprint + "nope\t"
                                currentSum = currentSum + 10000
                        break
                toprint = "total : " + str(currentSum) + "\t" + toprint
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
                currentParam[param[0]] = param[1][currentMinI]
                currentMin = 99999999 # todo : remove this

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
    bruteforceFindBest({
        'freeAllDirectPoint': True,
        'dsoTracer.desiredPointDensity': 800,
        'dsoTracer.immatureDensity': 600,
        'dsoInitializer.pointDensity': 2000,
        'dsoBa.iterations': 4,
        'dsoBa.maxFrames': 6
    }
    )
