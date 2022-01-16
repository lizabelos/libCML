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


def parse_config():
    #if len(sys.argv) <= 1:
    #    print("python3 evaluation [config_file]")
    #    exit(0)

    config_file = "config2.txt"
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

def statsOn(configName, tableName):
    datasets, datasets_names, slams, slams_names = parse_config()

    for i in range(0, len(datasets)):
        s = slams[0]
        name = slams_names[0]
        context = s[0](s[1], configName)

        print("Evaluating on " + datasets[i].name())

        context.run(datasets[i])

        try:
            evaluation = evaluator.fromslam(context)
            ate = evaluation.ape_rmse()
            rpe = evaluation.rpe_rmse()

            print(ate)
        except:
            print("Error : " + str(context.getError()))

if __name__ == "__main__":
    statsOn("modslam2.yaml", "modslam.csv")