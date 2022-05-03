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

def commonKeys(dicts):
    if len(dicts) == 0:
        return []
    if len(dicts) == 1:
        return list(dicts[0].keys())
    keys = list(dicts[0].keys())
    for i in range(1, len(dicts)):
        keys = list(set(keys) & set(dicts[i].keys()))
    return keys

def averageOfMultipleDict(dicts):
    if len(dicts) == 0:
        return {}
    if len(dicts) == 1:
        return dicts[0]
    d = {}
    keys = commonKeys(dicts)
    for k in keys:
        d[k] = 0
    for i in range(1, len(dicts)):
        for key in d:
            d[key] = d[key] + dicts[i][key]
    for key in d:
        d[key] = d[key] / len(dicts)
    return d

def averageParameter(d, param):
    newD = {}
    maxAteN = 2
    for h in d:
        pCopy = d[h].copy()
        pCopy.pop(param)
        for node in {"stats", "time", "ate"}:
            pCopy.pop(node)
        key = hashOfDict(pCopy)
        try:
            float(d[h]["ate"])
        except:
            continue
        if key not in newD:
            newD[key] = pCopy
            newD[key]["ate"] = []
            newD[key]["stats"] = []
            newD[key]["time"] = []
        newD[key]["ate"].append(float(d[h]["ate"]))
        newD[key]["stats"].append(d[h]["stats"])
        newD[key]["time"].append(d[h]["time"])

        maxAteN = max(maxAteN, len(newD[key]["ate"]))

    toRemove = []
    toRemoveN = 0

    for h in newD:
        #if len(newD[h]["ate"]) != maxAteN:
        #    toRemove.append(h)
        #    toRemoveN = toRemoveN + len(newD[h]["ate"])
        #    continue

        newD[h]["ate"] = sum(newD[h]["ate"]) / len(newD[h]["ate"])
        newD[h]["stats"] = averageOfMultipleDict(newD[h]["stats"])
        newD[h]["time"] = sum(newD[h]["time"]) / len(newD[h]["time"])

    print("Removed " + str(toRemoveN) + " entries")

    for h in toRemove:
        newD.pop(h)

    return newD

def addDefaultParameters(d):

    params = [
        ["orb.nLevels", 1],
        ["trackingMinimumOrbPoint", 50],
        ["trackcondUncertaintyWeight", 0.7],
        ["bacondMinimumOrbPoint",  100],
        ["numOrbMultiplier", 1.0],
        ["bacondSaturatedRatio", 0.15],
    ]

    for h in d:
        for p in params:
            if p[0] not in d[h]:
                d[h][p[0]] = p[1]

    return d