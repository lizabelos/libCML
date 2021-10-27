import os
import sys
import csv

import concurrent
import concurrent.futures
import multiprocessing
from statistics import mean

from evo.tools.file_interface import csv_read_matrix

import dataset
import evaluator
import slam
from table import FileTable, MedianTableProxy, SumTableProxy


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

    print("Found " + str(len(datasets)) + " videos")

    num_execution = 10

    for n in range(0, num_execution):
        for i in range(0, len(datasets)):
            for j in range(0, len(slams)):
                s = slams[j]
                name = slams_names[j]
                context = s[0](s[1])

                print("Evaluating on " + datasets[i].name())
                print("Result folder : " + context.outputdir())

                context.run(datasets[i])

                try:
                    evaluation = evaluator.fromslam(context)
                    ate = evaluation.ape_rmse()
                    rpe = evaluation.rpe_rmse()
                    evaluation.plot(datasets[i].name(), "result/" + datasets[i].name() + "_" + name + ".pdf")
                    print("ATE of " + datasets[i].name() + ": " + str(ate))
                    print("RPE of " + datasets[i].name() + ": " + str(rpe))

                    table.set("ate of " + name, datasets[i].name(), ate)
                    table.set("rpe of " + name, datasets[i].name(), rpe)
                except:
                    print("Unable to evaluate " + datasets[i].name())

def statsOn(configName):
    datasets, datasets_names, slams, slams_names = parse_config()
    table = None

    for i in range(0, len(datasets)):
        s = slams[0]
        name = slams_names[0]
        context = s[0](s[1], configName)

        print("Evaluating on " + datasets[i].name())
        print("Result folder : " + context.outputdir())

        context.run(datasets[i])

        try:
            evaluation = evaluator.fromslam(context)
            ate = evaluation.ape_rmse()
            rpe = evaluation.rpe_rmse()

            stats = context.getStats()

            if table is None:
                table = FileTable(["ate", "rpe"] + [x + ".avg" for x in stats.keys()] + [x + ".med" for x in stats.keys()] + [x + ".ecl" for x in stats.keys()] + [x + ".ech" for x in stats.keys()], datasets_names, "result/table.csv")

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
            print("Unable to evaluate " + datasets[i].name())





def ablationstudy():
    datasets, datasets_names, slams, slams_names = parse_config()
    #for d in datasets:
    #    d.setuseramdisk(True)

    valuesToTry = [1.25, 1.50, 1.75, 2.0]
    valuesToTry = [round(1.0 / x, 2) for x in valuesToTry[::-1]] + [1.0] + valuesToTry
    valuesToTry = [x * 0.0125 for x in valuesToTry]

    print(valuesToTry)

    # desiredPointDensity = 2000 // 10
    # immatureDensity = 1500 // 10

    num_execution = 3

    table_ate = MedianTableProxy(FileTable(valuesToTry, datasets_names, "result/ate.csv"))
    table_error = SumTableProxy(FileTable(valuesToTry, datasets_names, "result/error.csv"))

    def process(i, v, n):
        datasets[i].use()

        try:
            print("Value : %s ; Dataset : %s ; Execution : %d" % (str(v), datasets[i].name(), n))

            s = slams[0]
            name = slams_names[0]
            context = s[0](s[1])

            # context.setconfig("dsoTracer.desiredPointDensity", desiredPointDensity)
            # context.setconfig("dsoTracer.immatureDensity", immatureDensity)
            context.setconfig("bacondScoreWeight", v)

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
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    futures = []

    for i in range(0, len(datasets)):
        for v in valuesToTry:
            for n in range(0, num_execution):
                futures = futures + [executor.submit(process, i, v, n)]

    concurrent.futures.wait(futures)


if __name__ == "__main__":
    #main()
    statsOn("orb.yaml")
    #ablationstudy()
