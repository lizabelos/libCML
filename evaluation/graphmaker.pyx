import math
from tqdm import tqdm
import json

nodesToIgnore = {"ate","edges","datasetname","stats","time"}

cpdef distanceBetweenNode(dict p1, dict p2, str ignore = "", int maxlen = 1):
    cpdef nodesToIgnore = {"ate","edges","datasetname","stats","time"}
    if len((p1.keys()|nodesToIgnore) ^ (p2.keys()|nodesToIgnore)) > 0:
        return None
    if any([not isinstance(p1[index], type(p2[index])) for index in p1 if not index == "ate" and not index == "edges" and not index == "datasetname" and not index == ignore]):
        return None
    cdef diff = []
    for index in p1:
        if index in nodesToIgnore or index == ignore:
            continue
        if isinstance(p1[index], float):
            if not math.isclose(p1[index],p2[index]):
                if len(diff) == maxlen:
                    return None
                diff.append(index)
            continue
        if p1[index] != p2[index]:
            if len(diff) == maxlen:
                return None
            diff.append(index)
            continue
    return diff

cpdef makeGraphEdges_job(list j):
    return j[2], j[3], distanceBetweenNode(j[0], j[1])

cpdef makeGraphEdges(dict d, list params):

    cdef list keys = list(d.keys())
    cdef set paramsSet = set(params)

    cdef paramsHist = {}
    for param in params:
        paramsHist[param] = {}

    for h in d:
        d[h]["edges"] = {}
        d[h]["edges"]["same"] = []
        for param in params:
            d[h]["edges"][param] = []

    for i in range(len(keys)):
        h = keys[i]
        for param in params:
            value = "default"
            if param in d[h]:
                value = d[h][param]
            if value not in paramsHist[param]:
                paramsHist[param][value] = []
            paramsHist[param][value].append(h)

    for i in tqdm(range(len(keys))):
        h = keys[i]
        intersection = {}
        for param in params:
            value = "default"
            if param in d[h]:
                value = d[h][param]
            for h2 in paramsHist[param][value]:
                if h2 not in intersection:
                    intersection[h2] = 0
                intersection[h2] += 1
        for h2 in intersection:
            if intersection[h2] == len(params):
                d[h]["edges"]["same"].append(h2)
            if intersection[h2] == len(params) - 1 and d[h]["datasetname"] == d[h2]["datasetname"]:
                for paramDiff in params:
                    if paramDiff in d[h] and paramDiff in d[h2]:
                        if d[h][paramDiff] != d[h2][paramDiff]:
                            d[h]["edges"][paramDiff].append(h2)
                            break

    return d