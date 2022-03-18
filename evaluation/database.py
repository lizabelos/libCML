import json
import hashlib
import os
import threading
from utils import autoRound

mutex = threading.Lock()
cachedDatabase = None

def loadJsonFile(path, cache = True):
    if cache == False:
        try:
            with open(path, "r") as infile:
                return json.load(infile)
        except:
            return {}

    global cachedDatabase
    if cachedDatabase is None:
        try:
            with open(path, "r") as infile:
                cachedDatabase = json.load(infile)
        except:
            cachedDatabase = {}
    return cachedDatabase

def saveJsonFile(path, d):
    global cachedDatabase
    if os.path.isfile(path + ".bak.bak"):
        os.remove(path + ".bak.bak")
    with open(path + ".new", "w") as outfile:
        json.dump(d, outfile, indent=4)
    if os.path.isfile(path + ".bak"):
        os.rename(path + ".bak", path + ".bak.bak")
    if os.path.isfile(path):
        os.rename(path, path + ".bak")
    os.rename(path + ".new", path)
    if os.path.isfile(path + ".bak.bak"):
        os.remove(path + ".bak.bak")
    cachedDatabase = d

def hashOfDict(d):
    hash_object = hashlib.md5(json.dumps(d, sort_keys=True).encode('utf-8'))
    return hash_object.hexdigest()

def addResultToJson(hash, parameters, ateOrError, datasetname, tim, statistics = {}):
    global mutex
    mutex.acquire()
    d = loadJsonFile(hash + ".json")
    res = parameters.copy()
    res["datasetname"] = datasetname
    key = hashOfDict(res)
    # res = parameters.copy()
    res["ate"] = ateOrError
    res["stats"] = statistics
    res["time"] = tim
    d[key] = res
    saveJsonFile(hash + ".json", d)
    mutex.release()

def getResultFromJson(hash, parameters, datasetname, exceptionOnError=False, contextToUpdate = None):
    global mutex
    mutex.acquire()
    d = loadJsonFile(hash + ".json")
    res = parameters.copy()
    res["datasetname"] = datasetname
    key = hashOfDict(res)
    if key in d:
        try:
            if d[key]["ate"].startswith("Unknown"):
                mutex.release()
                return None
        except:
            pass
        mutex.release()
        if contextToUpdate is not None:
            contextToUpdate.error = str(d[key]["ate"])
            contextToUpdate.tim = d[key]["time"]
        if exceptionOnError:
            float(d[key]["ate"])
        return d[key]["ate"]
    mutex.release()
    return None

def fixRounding(d):
    fixed = 0
    for h in d:
        for k in d[h]:
            if isinstance(d[h][k], float):
                d[h][k] = autoRound(d[h][k])
                fixed = fixed + 1
    print("Fixed " + str(fixed) + " floating points")
    return d