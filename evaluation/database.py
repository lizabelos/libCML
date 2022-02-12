import json
import hashlib
import os
import threading

mutex = threading.Lock()
cachedDatabase = None

def loadJsonFile(path, cache = True):
    global cachedDatabase
    if cachedDatabase is None or cache == False:
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

def addResultToJson(hash, parameters, ateOrError, datasetname):
    global mutex
    mutex.acquire()
    d = loadJsonFile(hash + ".json")
    res = parameters.copy()
    res["datasetname"] = datasetname
    key = hashOfDict(res)
    # res = parameters.copy()
    res["ate"] = ateOrError
    d[key] = res
    saveJsonFile(hash + ".json", d)
    mutex.release()

def getResultFromJson(hash, parameters, datasetname):
    global mutex
    mutex.acquire()
    d = loadJsonFile(hash + ".json")
    res = parameters.copy()
    res["datasetname"] = datasetname
    key = hashOfDict(res)
    if key in d:
        if "datasetname" not in d[key]:
            addResultToJson(hash, parameters, d[key]["ate"], datasetname)
        try:
            if d[key]["ate"].startswith("Unknown"):
                mutex.release()
                return None
        except:
            pass
        mutex.release()
        return d[key]["ate"]
    mutex.release()
    return None
