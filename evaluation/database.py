import json
import hashlib

def loadJsonFile(path):
    try:
        with open(path, "r") as infile:
            return json.load(infile)
    except:
        return {}

def saveJsonFile(path, d):
    with open(path, "w") as outfile:
        json.dump(d, outfile)

def hashOfDict(d):
    hash_object = hashlib.md5(json.dumps(d, sort_keys=True).encode('utf-8'))
    return hash_object.hexdigest()

def addResultToJson(hash, parameters, ateOrError, datasetname):
    d = loadJsonFile(hash + ".json")
    res = parameters.copy()
    res["datasetname"] = datasetname
    key = hashOfDict(parameters)
    res = parameters.copy()
    res["ate"] = ateOrError
    d[key] = res
    saveJsonFile(hash + ".json", d)

def getResultFromJson(hash, parameters, datasetname):
    d = loadJsonFile(hash + ".json")
    res = parameters.copy()
    res["datasetname"] = datasetname
    key = hashOfDict(parameters)
    if key in d:
        return d[key]["ate"]
    return None