import os
import sys
import csv

from evo.tools.file_interface import csv_read_matrix

import dataset
import evaluator
import slam
from table import FileTable


def main():
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

    os.makedirs("result", exist_ok=True)
    table = FileTable(["ate of " + x for x in slams_names] + ["rpe of " + x for x in slams_names], datasets_names, "result/table.csv")

    print("Found " + str(len(datasets)) + " videos")

    for i in range(0, len(datasets)):
        for j in range(0, len(slams)):
            s = slams[j]
            name = slams_names[j]
            context = s[0](s[1])

            print("Evaluating on " + datasets[i].name())
            print("Result folder : " + context.outputdir())

            context.run(datasets[i])

            # try:
            evaluation = evaluator.fromslam(context)
            ate = evaluation.ape_rmse()
            rpe = evaluation.rpe_rmse()
            evaluation.plot(datasets[i].name(), "result/" + datasets[i].name() + "_" + name + ".pdf")
            print("ATE of " + datasets[i].name() + ": " + str(ate))
            print("RPE of " + datasets[i].name() + ": " + str(rpe))

            table.set("ate of " + name, datasets[i].name(), ate)
            table.set("rpe of " + name, datasets[i].name(), rpe)
            #except:
            #    print("Unable to evaluate " + datasets[i].name())


if __name__ == "__main__":
    main()
