import shutil
from abc import ABC, abstractmethod
import tempfile
import os
import time
from utils import system
import yaml
import hashlib
import statistics

def sha256sum(filename):
    h  = hashlib.sha256()
    b  = bytearray(128*1024)
    mv = memoryview(b)
    with open(filename, 'rb', buffering=0) as f:
        for n in iter(lambda : f.readinto(mv), 0):
            h.update(mv[:n])
    return h.hexdigest()

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

    def run(self, d, onfinish=None, time_limit=None):
        self.d = d

        start = time.time()
        self.start(d, time_limit=time_limit)
        stop = time.time()
        self.e = (stop - start)

        if onfinish is not None:
            onfinish()

    def getHash(self):
        return sha256sum(self.getslampath())

    @abstractmethod
    def getslampath(self):
        pass

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

    def processLogForStats(self, log, fullError = False):

        if fullError:
            self.error = "\n".join(log)
        else:
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
        for stat in stats:
            stats[stat] = statistics.mean(stats[stat])
        self.stats = stats

    def getStats(self):
        return self.stats


class ModSLAM(SLAM):

    def __init__(self, modslampath=None, configpath="modslam.yaml"):
        super().__init__()
        self.e = 0
        self.d = None
        self.tim = 99999
        self.modslampath = "../cmake-build-release/modslam"
        if modslampath is not None:
            self.modslampath = modslampath

        f = open(configpath, "r")
        self.config = yaml.load(f, Loader=yaml.FullLoader)
        f.close()

    def start(self, d, time_limit=None):
        output = self.outputdir()

        config_file, config_filename = tempfile.mkstemp()
        config_file = os.fdopen(config_file, 'w')
        yaml.dump(self.config, config_file)
        config_file.close()

        # print("Configuration file : " + config_filename)

        executable = self.modslampath
        args = "-c \"" + config_filename + "\" -t -z -d \"" + d.folder() + "\" -r \"" + os.path.join(output,"result") + "\""
        if d.isReverse():
            args = args + " -b"
        command = executable + " " + args

        # print(command)

        out, err, tim = system(command, comment=d.name(), time_limit=time_limit)
        self.tim = tim
        self.processLogForStats(out)

    def getTime(self):
        return self.tim

    def executionTime(self):
        return self.tim

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

    def getconfig(self):
        return self.config

    def getslampath(self):
        return self.modslampath