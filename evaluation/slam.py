import shutil
from abc import ABC, abstractmethod
import tempfile
import os
import time
from utils import system
import yaml


class SLAM(ABC):

    def __init__(self):
        self.tmp = None
        self.stats = {}
        self.error = "Unknown error"

    def __del__(self):
        shutil.rmtree(self.outputdir(), ignore_errors=True)

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

    def getError(self):
        return self.error

    def processLogForStats(self, log):
        log = log.split("\n")

        allDeadly = [x for x in log if "[DEADLY]" in x]
        allError = [x for x in log if "[ERROR]" in x]

        if len(allDeadly) > 0:
            self.error = ";".join([x for x in allDeadly])
        elif len(allError) > 0:
            self.error = allError[-1]
        stats_table = [x.split(" ")[1:] for x in log if x.startswith("STAT")]
        stats = {}
        for name, x, y in stats_table:
            if not name in stats:
                stats[name] = []
            stats[name].append(float(y))
        self.stats = stats

    def getStats(self):
        return self.stats


class ModSLAM(SLAM):

    def __init__(self, modslampath=None, configpath="modslam.yaml"):
        super().__init__()
        self.e = 0
        self.d = None
        self.modslampath = "../cmake-build-release/modslam"
        if modslampath is not None:
            self.modslampath = modslampath

        f = open(configpath, "r")
        self.config = yaml.load(f, Loader=yaml.FullLoader)
        f.close()

    def start(self, d):
        output = self.outputdir()

        config_file, config_filename = tempfile.mkstemp()
        config_file = os.fdopen(config_file, 'w')
        yaml.dump(self.config, config_file)
        config_file.close()

        # print("Configuration file : " + config_filename)

        executable = self.modslampath
        args = "-c \"" + config_filename + "\" -t -z -d \"" + d.folder() + "\" -r \"" + os.path.join(output,"result") + "\""
        command = executable + " " + args

        # print(command)

        out, err = system(command)
        self.processLogForStats(out)

    def outputlog(self):
        return "/dev/null"
        # return os.path.join(self.outputdir(), "log.txt")

    def outputtum(self):
        return os.path.join(self.outputdir(), "result.tum.txt")

    def outputkitti(self):
        return os.path.join(self.outputdir(), "result.kitti.txt")

    def elapsed(self):
        return self.e

    def setconfig(self, name, value):
        self.config[name] = value
