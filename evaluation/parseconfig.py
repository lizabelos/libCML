import sys
from evo.tools.file_interface import csv_read_matrix
import slam
import dataset

def parse_config():
    #if len(sys.argv) <= 1:
    #    print("python3 evaluation [config_file]")
    #    exit(0)

    #config_file = sys.argv[1]
    #content = csv_read_matrix(config_file, delim=" ", comment_str="#")
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