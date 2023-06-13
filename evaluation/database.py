import copy
import json
import hashlib
import os
import threading
import time
from statistics import median
from math import pi, sqrt, exp, floor, ceil

import numpy as np

from utils import autoRound

mutex = threading.Lock()
cachedDatabase = None
threadStarted = False

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

def saveJsonFile(path, d, isFromThread = False):
    global threadStarted
    if isFromThread or not threadStarted:
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

def saveJsonFileThead(path):
    global cachedDatabase
    global threadStarted
    threadStarted = True
    # Look for modification of cachedDatabase
    cachedDatabaseCopy = copy.copy(cachedDatabase)
    while True:
        if cachedDatabaseCopy != cachedDatabase:
            saveJsonFile(path, cachedDatabase, isFromThread=True)
            cachedDatabaseCopy = copy.copy(cachedDatabase)
        time.sleep(10)

def launchJsonFileThread():
    threading.Thread(target=saveJsonFileThead, args=("./results.json",)).start()

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

def gauss(n=11,sigma=1.0):
    r = range(-int(n/2),int(n/2)+1)
    return [1 / (sigma * sqrt(2*pi)) * exp(-float(x)**2/(2*sigma**2)) for x in r]

def linearInterpolation(y, value):
    x = np.linspace(0, len(y)-1, len(y)) + 0.5
    return np.interp(value, x, y)

def gaussian(a):
    if len(a) == 0:
        raise "Empty array"
    if len(a) == 1:
        return a[0]
    if len(a) == 2:
        return (a[0] + a[1]) / 2
    a = sorted(a)
    sum = 0
    total = 0
    weights = gauss(n=len(a),sigma=(len(a)-1)/6)
    for i in range(0, len(a)):
        weight = weights[i]
        sum = sum + a[i] * weight
        total = total + weight
    sum = sum / total

    # Compute confidence interval
    sum_low = 0
    sum_high = 0
    total_low = 0
    total_high = 0
    for i in range(0, len(a)):
        if a[i] < sum:
            weight = weights[i]
            sum_low = sum_low + a[i] * weight
            total_low = total_low + weight
        else:
            weight = weights[i]
            sum_high = sum_high + a[i] * weight
            total_high = total_high + weight
    sum_low = sum_low / total_low
    sum_high = sum_high / total_high

    if sum_low > sum:
        raise "Error in computing confidence interval"
    if sum_high < sum:
        raise "Error in computing confidence interval"

    return [sum, sum_low, sum_high]

def averageParameter(d, param, mode = "gaussian"):
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
        except ValueError:
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
        newD[h]["stats"] = averageOfMultipleDict(newD[h]["stats"])
        newD[h]["time"] = sum(newD[h]["time"]) / len(newD[h]["time"])
        if len(newD[h]["ate"]) == 0:
            newD[h]["ate"] = "ERROR"
            continue
        if mode == "average":
            newD[h]["ate"] = sum(newD[h]["ate"]) / len(newD[h]["ate"])
        elif mode == "median":
            newD[h]["ate"] = median(newD[h]["ate"])
        elif mode == "gaussian":
            newD[h]["ate"] = gaussian(newD[h]["ate"])

        if isinstance(newD[h]["ate"], list):
            newD[h]["ate_upper"] = newD[h]["ate"][2]
            newD[h]["ate_lower"] = newD[h]["ate"][1]
            newD[h]["ate"] = newD[h]["ate"][0]

    print("Removed " + str(toRemoveN) + " entries")

    for h in toRemove:
        newD.pop(h)

    return newD

def changeSpaceForPlot(d):
    for h in d:
        for p in d[h]:
            if p.startswith("trackcondUncertaintyWeight") or p == "bacondScoreWeight":
                if d[h][p] > 1.0:
                    d[h][p] = 1.0 - (1.0 / d[h][p])
                else:
                    d[h][p] = -1 + d[h][p]
    return d

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