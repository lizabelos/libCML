import json
import hashlib
import threading

mutex = threading.Lock()

def loadJsonFile(path):
    try:
        with open(path, "r") as infile:
            return json.load(infile)
    except:
        return {}

def saveJsonFile(path, d):
    with open(path, "w") as outfile:
        json.dump(d, outfile, indent=4)

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
