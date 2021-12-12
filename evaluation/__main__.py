import os
import random
import sys
import csv

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


def parse_config():
    if len(sys.argv) <= 1:
        print("python3 evaluation [config_file]")
        exit(0)

    config_file = sys.argv[1]
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


def main():
    datasets, datasets_names, slams, slams_names = parse_config()

    os.makedirs("result", exist_ok=True)
    table = MedianTableProxy(
        FileTable(["ate of " + x for x in slams_names] + ["rpe of " + x for x in slams_names], datasets_names,
                  "result/table.csv"))

    dprint("Found " + str(len(datasets)) + " videos")

    num_execution = 5

    for n in range(0, num_execution):
        for i in range(0, len(datasets)):
            for j in range(0, len(slams)):
                s = slams[j]
                name = slams_names[j]
                context = s[0](s[1])

                dprint("Evaluating on " + datasets[i].name())
                dprint("Result folder : " + context.outputdir())

                context.run(datasets[i])

                if True:
                    evaluation = evaluator.fromslam(context)
                    ate = evaluation.ape_rmse()
                    rpe = evaluation.rpe_rmse()
                    evaluation.plot(datasets[i].name(), "result/" + datasets[i].name() + "_" + name + ".pdf")
                    dprint("ATE of " + datasets[i].name() + ": " + str(ate))
                    dprint("RPE of " + datasets[i].name() + ": " + str(rpe))

                    table.set("ate of " + name, datasets[i].name(), ate)
                    table.set("rpe of " + name, datasets[i].name(), rpe)
                # except:
                #    dprint("Unable to evaluate " + datasets[i].name())


def statsOn(configName, tableName):
    datasets, datasets_names, slams, slams_names = parse_config()
    table = None

    for i in range(0, len(datasets)):
        s = slams[0]
        name = slams_names[0]
        context = s[0](s[1], configName)

        dprint("Evaluating on " + datasets[i].name())
        dprint("Result folder : " + context.outputdir())

        context.run(datasets[i])

        try:
            evaluation = evaluator.fromslam(context)
            ate = evaluation.ape_rmse()
            rpe = evaluation.rpe_rmse()

            stats = context.getStats()

            if table is None:
                table = FileTable(
                    ["ate", "rpe"] + [x + ".avg" for x in stats.keys()] + [x + ".med" for x in stats.keys()] + [
                        x + ".ecl" for x in stats.keys()] + [x + ".ech" for x in stats.keys()], datasets_names,
                    "result/" + tableName)

            for k in stats.keys():
                v = stats[k]
                v.sort()
                s = len(v)
                table.set(k + ".avg", datasets[i].name(), mean(stats[k]))
                table.set(k + ".med", datasets[i].name(), v[s // 2])
                table.set(k + ".ecl", datasets[i].name(), v[s // 4])
                table.set(k + ".ech", datasets[i].name(), v[s // 4 * 3])

            table.set("ate", datasets[i].name(), ate)
            table.set("rpe", datasets[i].name(), rpe)

        except:
            dprint("Unable to evaluate " + datasets[i].name())


def ablationstudy():
    datasets, datasets_names, slams, slams_names = parse_config()
    # for d in datasets:
    #    d.setuseramdisk(True)

    valuesToTry = [1, 10000, 1 / 10000, 1000, 1 / 1000, 100, 1 / 100, 10, 1 / 10]
    # valuesToTry = [round(1.0 / x, 2) for x in valuesToTry[::-1]] + [1.0] + valuesToTry
    # valuesToTry = [x * 0.0125 for x in valuesToTry]

    dprint(valuesToTry)

    # desiredPointDensity = 2000 // 10
    # immatureDensity = 1500 // 10

    num_execution = 10

    table_ate = MedianTableProxy(FileTable(sorted(valuesToTry), datasets_names, "result/ate.csv"))
    table_error = SumTableProxy(FileTable(sorted(valuesToTry), datasets_names, "result/error.csv"))

    def process(i, v, n):
        datasets[i].use()

        try:
            dprint("Value : %s ; Dataset : %s ; Execution : %d" % (str(v), datasets[i].name(), n))

            s = slams[0]
            name = slams_names[0]
            context = s[0](s[1])

            # context.setconfig("dsoTracer.desiredPointDensity", desiredPointDensity)
            # context.setconfig("dsoTracer.immatureDensity", immatureDensity)
            context.setconfig("bacondUncertaintyWeight", v)

            context.run(datasets[i])

            try:
                evaluation = evaluator.fromslam(context)
                ate = evaluation.ape_rmse()
                table_ate.set(v, datasets[i].name(), ate)
            except:
                table_error.set(v, datasets[i].name(), 1)

        finally:
            datasets[i].unuse()

    # Each SLAM instance will use 2 thread
    # executor = concurrent.futures.ThreadPoolExecutor(max_workers=multiprocessing.cpu_count() // 2)
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=9)
    futures = []

    for n in range(0, num_execution):
        for i in range(0, len(datasets)):
            for v in valuesToTry:
                futures = futures + [executor.submit(process, i, v, n)]

    concurrent.futures.wait(futures)


def ablationstudy2d():
    datasets, datasets_names, slams, slams_names = parse_config()
    # for d in datasets:
    #    d.setuseramdisk(True)

    toTry = [
        "trackcondUncertaintyWeight",
        [0.25, 0.50, 0.75, 1, 1.5, 2, 4],
        "bacondUncertaintyWeight",
        [0.25, 0.50, 0.75, 1, 1.5, 2, 4]
    ]

    # desiredPointDensity = 2000 // 10
    # immatureDensity = 1500 // 10

    num_execution = 5

    table_ate = FileTable(sorted(toTry[1]), sorted(toTry[3]), "result/ate.csv")
    table_error = FileTable(sorted(toTry[1]), sorted(toTry[3]), "result/error.csv")

    def process(i, v, n):
        datasets[i].use()

        result = None

        try:
            dprint("Value : %s ; Dataset : %s ; Execution : %d" % (str(v), datasets[i].name(), n))

            s = slams[0]
            name = slams_names[0]
            context = s[0](s[1])

            # context.setconfig("dsoTracer.desiredPointDensity", desiredPointDensity)
            # context.setconfig("dsoTracer.immatureDensity", immatureDensity)
            context.setconfig(v[0], v[1])
            context.setconfig(v[2], v[3])

            context.run(datasets[i])

            evaluation = evaluator.fromslam(context)
            result = evaluation.ape_rmse()
        except:
            pass

        datasets[i].unuse()

        return result

    # Each SLAM instance will use 2 thread
    # executor = concurrent.futures.ThreadPoolExecutor(max_workers=multiprocessing.cpu_count() // 2)
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=3)

    for v1 in toTry[1]:
        for v2 in toTry[3]:
            futures = []
            for i in range(0, len(datasets)):
                for n in range(0, num_execution):
                    futures = futures + [executor.submit(process, i, [toTry[0], v1, toTry[2], v2], n)]
            concurrent.futures.wait(futures)
            fi = 0
            numerr = 0
            sum = 0
            for i in range(0, len(datasets)):
                values = []
                for n in range(0, num_execution):
                    r = futures[fi].result()
                    if r is None:
                        numerr = numerr + 1
                    else:
                        values.append(r)
                    fi = fi + 1
                if len(values) == 0:
                    sum = None
                if sum is not None:
                    sum = sum + statistics.median(values)
            if sum is None:
                table_ate.set(v1, v2, "Error")
            else:
                table_ate.set(v1, v2, sum)
            table_error.set(v1, v2, numerr)


def build():
    os.makedirs("../build", exist_ok=True)
    os.system("cd .. && cd build && cmake -DENABLE_GUI=OFF -DCMAKE_BUILD_TYPE=Release .. && make ")


def intrange(a, b, c = 1):
    return [int(x) for x in np.arange(a,b,c)]

def floatrange(a, b, c = 1.0):
    return [float(x) for x in np.arange(a,b,c)]

def bruteforceFindBest():
    datasets, datasets_names, slams, slams_names = parse_config()
    params = [
        #["numOrbCorner", intrange(1000,2250,250)],
        ["orbUncertaintyThreshold", [100000, 1000000, 10000, 1000, 100, 10, 0, 0.1, 0.01, -1]],
        #["dsoTracer.desiredPointDensity", intrange(0,2200,250)],
        #["dsoTracer.immatureDensity", intrange(0,2200,250)],
        #["dsoInitializer.pointDensity", intrange(1000,4000,1000)],
        ["trackcondUncertaintyWeight", floatrange(0.2,2,0.2)],
        ["trackcondUncertaintyWindow", intrange(1,8)],
        ["bacondScoreWeight", [0.0125*2,0.0125*1.5,0.0125,0.0125/1.5,0.0125/2]],
        ["bacondScoreWindow", intrange(1,8)],
        ["numOrbCorner", intrange(1000,2250,250)]
        #["bacondSaturatedRatio", [0,0.5,0.10,0.15]],
        #["bacondUncertaintyWeight", floatrange(0,2,0.2)],
        #["bacondUncertaintyWindow", intrange(1,8)],
        #["orbBa.numIteration", intrange(0,10)],
        #["orbBa.refineIteration", intrange(0,10)],
        #["orbBa.removeEdge", ["true", "false"]]
    ]

    dprint("Hello :)")
    currentParam = {}
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=3)
    while True:
        for param in params:
            allSums = []
            allSuccess = []
            futures = []
            dprint("Testing on " + param[0])
            def process(param, v):
                toprint = str(param[0]) + "=" + str(v) + "; "
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
                    context.run(datasets[i])
                    try:
                        evaluation = evaluator.fromslam(context)
                        ate = evaluation.ape_rmse()
                        #dprint(str(ate) + "; ", end='')
                        toprint = toprint + str(ate) + "; "
                        if ate > datasets[i].lim():
                            # dprint("Too big")
                            toprint = toprint + "Too big"
                            # currentSum = None
                            break
                        currentSum = currentSum + ate
                        currentSuccess = currentSuccess + 1
                    except Exception as e:
                        # dprint("Invalid : " + str(e))
                        toprint = toprint + "Invalid : " + str(e)
                        #currentSum = None
                        break
                dprint(toprint)
                return currentSum, currentSuccess

            for v in param[1]:
                futures = futures + [executor.submit(process, param, v)]
                #currentSum, currentSuccess = process(param, v)
                # dprint("Final sum : " + str(currentSum))
            concurrent.futures.wait(futures)
            for f in futures:
                currentSum, currentSuccess = f.result()
                allSums.append(currentSum)
                allSuccess.append(currentSuccess)

            currentMin = 999999
            currentMinI = None
            currentMaxSuccess = 0
            for i in range(0, len(allSums)):
                if allSuccess[i] < currentMaxSuccess:
                    continue
                if allSuccess[i]>currentMaxSuccess or allSums[i] < currentMin:
                    currentMin = allSums[i]
                    currentMaxSuccess = allSuccess[i]
                    currentMinI = i

            if currentMinI is not None:
                currentParam[param[0]] = param[1][currentMinI]

            dprint("")
            dprint("=========================")
            dprint(currentParam)
            dprint("=========================")
            dprint("")
            #dprint("Best : " + str(currentMin))



if __name__ == "__main__":
    #build()
    bruteforceFindBest()
    # statsOn("modslam.yaml", "modslam.csv")
    # statsOn("orb1000.yaml", "orb1000.csv")
    # statsOn("orb2000.yaml", "orb2000.csv")
    # statsOn("dso2000.yaml", "dso2000.csv")
    # statsOn("dso800.yaml", "dso800.csv")
    # ablationstudy2d()
    # main()
