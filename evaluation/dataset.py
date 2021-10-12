import os

class Dataset:

    def __init__(self, n, t, f, g=None):
        self.n = n
        self.t = t
        self.f = f
        self.g = g

    def name(self):
        return self.n

    def type(self):
        return self.t

    def folder(self):
        return self.f

    def groundtruth(self):
        return self.g


def tumGroundtruthPath(tumFolder, i):
    i = str(int(i)).zfill(2)
    return tumFolder + "/sequence_%s/groundtruthSync.txt" % i


def kittiGroundtruthPath(kittiFolder, i):
    i = str(int(i)).zfill(2)
    return kittiFolder + "/poses/%s.txt" % i


def KITTI(folder, r=range(0, 11)):
    result = []
    folder = os.path.join(folder, "dataset")
    for i in r:
        name = "kitti_" + str(i).zfill(2)
        dataset_folder = os.path.join(folder, "sequences/" + str(i).zfill(2))
        result = result + [Dataset("KITTI " + str(i).zfill(2), "kitti", dataset_folder, kittiGroundtruthPath(folder, i))]
    return result


def TUM(folder, r=range(1, 51)):
    result = []
    for i in r:
        dataset_folder = os.path.join(folder,"sequence_" + str(i).zfill(2))
        result = result + [Dataset("TUM " + str(i).zfill(2), "tum", dataset_folder, tumGroundtruthPath(folder, i))]
    return result