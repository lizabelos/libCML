from abc import ABC, abstractmethod
import tempfile
import os
import time
from utils import system


class SLAM(ABC):

    def __init__(self):
        self.tmp = None

    def outputdir(self):
        if self.tmp is None:
            self.tmp = tempfile.mkdtemp()
        return self.tmp

    def run(self, d, onfinish=None):
        self.d = d

        start = time.time()
        self.start(d)
        stop = time.time()
        self.e = (stop - start)

        if onfinish is not None:
            onfinish()

    @abstractmethod
    def start(self, d):
        pass

    @abstractmethod
    def outputtum(self):
        pass

    @abstractmethod
    def outputkitti(self):
        pass

    @abstractmethod
    def elapsed(self):
        pass


class ModSLAM(SLAM):

    def __init__(self, modslampath = None):
        super().__init__()
        self.e = 0
        self.d = None
        self.modslampath = "../cmake-build-release/modslam"
        if modslampath is not None:
            self.modslampath = modslampath

    def start(self, d):
        output = self.outputdir()

        executable = self.modslampath
        args = "-c " + self.modslampath + ".yaml -t -d " + d.folder() + " -r " + os.path.join(output, "result")
        command = executable + " " + args

        system(command, self.outputlog(), "w")

    def outputlog(self):
        return os.path.join(self.outputdir(), "log.txt")

    def outputtum(self):
        return os.path.join(self.outputdir(), "result.tum.txt")

    def outputkitti(self):
        return os.path.join(self.outputdir(), "result.kitti.txt")

    def elapsed(self):
        return self.e
