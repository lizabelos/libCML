import sys
import csv

from evo.tools.file_interface import csv_read_matrix

import dataset
import evaluator
import slam


def main():
    if len(sys.argv) <= 1:
        print("python3 evaluation [config_file]")
        exit(0)

    config_file = sys.argv[1]
    content = csv_read_matrix(config_file, delim=" ", comment_str="#")

    datasets = []
    slams = []

    for line in content:
        if len(line) == 0:
            continue
        if line[0] == "ModSLAM":
            slams.append([slam.ModSLAM, line[1]])
        elif line[0] == "TUM":
            datasets = datasets + dataset.TUM(line[1])
        elif line[0] == "KITTI":
            datasets = datasets + dataset.KITTI(line[1])
        else:
            print("Unknown dataset type : " + line[0])

    print("Found " + str(len(datasets)) + " videos")

    for i in range(0, len(datasets)):
        for s in slams:
            context = s[0](s[1])

            print("Evaluating on " + datasets[i].name())
            print("Result folder : " + context.outputdir())

            context.run(datasets[i])

            #try:
            evaluation = evaluator.fromslam(context)
            print("ATE of " + datasets[i].name() + ": " + str(evaluation.ape_rmse()))
            print("RPE of " + datasets[i].name() + ": " + str(evaluation.rpe_rmse()))
            #except:
            #print("Unable to evaluate " + datasets[i].name())


if __name__ == "__main__":
    main()
