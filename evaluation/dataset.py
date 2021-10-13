import os
import threading
import shutil


class Dataset:

    def __init__(self, n, t, f, g=None):
        self.n = n
        self.t = t
        self.f = f
        self.g = g
        self.useramdisk = False
        self.lock = threading.Lock()
        self.nuse = 0
        self.ramdiskfolder = "/ramdisk/" + n

    def name(self):
        return self.n

    def type(self):
        return self.t

    def folder(self):
        if self.useramdisk:
            return self.ramdiskfolder
        else:
            return self.f

    def groundtruth(self):
        return self.g

    def use(self):
        if not self.useramdisk:
            return
        self.lock.acquire()
        if self.nuse == 0:
            self.copytoramdisk()
        self.nuse = self.nuse + 1
        self.lock.release()

    def unuse(self):
        if not self.useramdisk:
            return
        self.lock.acquire()
        self.nuse = self.nuse - 1
        if self.nuse == 0:
            self.removefromramdisk()
        self.lock.release()

    def copytoramdisk(self):
        print("Copying " + self.f + " to " + self.ramdiskfolder)
        try:
            shutil.rmtree(self.ramdiskfolder)
        except:
            pass
        shutil.copytree(self.f, self.ramdiskfolder)

    def removefromramdisk(self):
        print("Removing " + self.ramdiskfolder)
        shutil.rmtree(self.ramdiskfolder)

    def setuseramdisk(self, v):
        self.useramdisk = v


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