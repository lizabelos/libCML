import os
import matplotlib.pyplot as plt
from database import loadJsonFile, hashOfDict
import copy
from statistics import mean
from scipy.stats import sem
from math import sqrt

def distanceBetweenNode(p1, p2):
    if p1["datasetname"] != p2["datasetname"]:
        return 9999, []
    dist = 0
    diff = []
    for index in p1:
        if index == "ate":
            continue
        if index == "edges":
            continue
        # if index == "datasetname":
        #     continue
        if index not in p2:
            dist = 9999
            return dist, diff
        if p1[index] != p2[index]:
            dist = dist + 1
            diff.append(index)
            continue
    return dist, diff

class RangeBasedDict:

    def __init__(self, dataset):
        self.values = []
        self.minX = None
        self.maxX = None
        self.x = None
        self.y = None
        self.yavg = None
        self.num = None
        self.dataset = dataset

    def addValue(self, x, y, param):
        # todo : add a weight proportional to the distance of all the other values !!!
        # todo : graph des failures
        if self.minX is None:
            self.minX = x
            self.maxX = x
        self.minX = min(self.minX, x)
        self.maxX = max(self.maxX, x)
        self.values.append([x, y, None, param])

    def addRangeBasedDict(self, other):
        self.values = self.values + other.values
        self.minX = other.minX
        self.maxX = other.maxX

    def computeWeight(self):
        for i in range(0, len(self.values)):
            distTotal = 0
            for j in range(0, len(self.values)):
                if i == j:
                    continue
                distanceBetweenNode(self.values[i][3], self.values[j][3])
                dist, diff = distanceBetweenNode(self.values[i][3], self.values[j][3])
                distTotal += dist
            self.values[i][2] = distTotal

    def compute(self, resolution = 10, rnone = True):
        increment = (self.maxX - self.minX) / resolution
        if increment == 0:
            return False
        self.computeWeight()
        x = [0 for i in range(0, resolution)]
        y = []
        weight = []
        weightSum = [0 for i in range(0, resolution)]
        weightedY = []
        num = [0 for i in range(0, resolution)]
        for i in range(0, resolution):
            y.append([])
            weight.append([])
            weightedY.append([])
        for value in self.values:
            index = int((value[0] - self.minX) * 0.999 / increment)
            if index < 0 or index >= len(x):
                continue
            x[index] = x[index] + value[0]
            y[index].append(value[1])
            weight[index].append(value[2])
            weightedY[index].append(value[1] * value[2])
            weightSum[index] = weightSum[index] + value[2]
            num[index] = num[index] + 1
        for i in range(0, resolution):
            if num[i] == 0:
                if rnone:
                    return False
            else:
                x[i] = x[i] / num[i]
        self.x = x
        self.y = y
        self.yavg = [sum(weightedY[i]) / weightSum[i] for i in range(0, len(self.y))]
        self.num = num
        return True

def syncMinAndMaxOf(d):
    voteForMin = {}
    voteForMax = {}
    for k in d:
        if k.minX not in voteForMin:
            voteForMin[k.minX] = 0
        if k.maxX not in voteForMax:
            voteForMax[k.maxX] = 0
        voteForMin[k.minX] = voteForMin[k.minX] + 1
        voteForMax[k.maxX] = voteForMax[k.maxX] + 1
    minX = None
    minVote = 0
    for min in voteForMin:
        if voteForMin[min] > minVote:
            minVote = voteForMin[min]
            minX = min
    maxX = None
    maxVote = 0
    for max in voteForMax:
        if voteForMax[max] > maxVote:
            maxVote = voteForMax[max]
            maxX = max
    for k in d:
        k.minX = minX
        k.maxX = maxX

def removeDuplicate(x):
    return list(dict.fromkeys(x))

def elementInCommon(list1, list2):
    list3 = []
    for x in list1:
        for y in list2:
            if x == y:
                list3.append(x)
    return list3

def sortWithKey(x, y):
    l = []
    for i in range(0, len(x)):
        l.append([x[i], y[i]])
    l = sorted(l, key=lambda k : k[0])
    xres = []
    yres = []
    for i in range(0, len(x)):
        xres.append(l[i][0])
        yres.append(l[i][1])
    return xres, yres

def numFramesOf(dataset):

    if dataset.startswith("TUM"):
        return 6

    numFrames=[4541,1101,4661,801,271,2761,1101,1101,4071,1591,1201]

    return numFrames[int(dataset.split(" ")[1])]

def thresholdOf(dataset):

    if dataset.startswith("TUM"):
        return 6

    averageErrorPerFrames = 0.02
    ratio = 2
    numFrames=[4541,1101,4661,801,271,2761,1101,1101,4071,1591,1201]

    return numFrames[int(dataset.split(" ")[1])] * averageErrorPerFrames * ratio

def uncertaintyBaseOf(dataset):

    if dataset.startswith("TUM"):
        return 3

    lim=[110,20,44,2.0,1.4,43,49,17,58,60,15]
    if dataset == "ALL_KITTI":
        return sum(lim)

    return lim[int(dataset.split(" ")[1])]

def lowerErrorMedian(l):
    s = len(l)
    if s == 1:
        return 0
    lsorted = sorted(l)
    sleft = int(s / 4)
    return abs(lsorted[sleft] - mean(l))

def upperErrorMedian(l):
    s = len(l)
    if s == 1:
        return 0
    lsorted = sorted(l)
    sright = int(s / 4 * 3)
    return abs(lsorted[sright] - mean(l))

def lowerErrorBase(l, dataset):
    return uncertaintyBaseOf(dataset) / sqrt(len(l))

def upperErrorBase(l, dataset):
    return uncertaintyBaseOf(dataset) / sqrt(len(l))

def lowerError(l, dataset):
    if len(l) == 0:
        return None
    return lowerErrorMedian(l)

def upperError(l, dataset):
    if len(l) == 0:
        return None
    return upperErrorMedian(l)

def safeMean(l):
    if len(l) == 0:
        return None
    else:
        return mean(l)

def plot(d, param, datasets, folder):
    values = []
    alreadyTaken = set()
    for ref_id in d:
        if d[ref_id]["datasetname"] not in datasets:
            continue

        if param not in d[ref_id]["edges"]:
            continue

        if ref_id in alreadyTaken:
            continue

        rangedBasedDict = RangeBasedDict(d[ref_id]["datasetname"])

        alreadyTaken.add(ref_id)

        ate = None
        try:
            #print(d[id])
            ate = float(d[ref_id]["ate"])
        except:
            continue

        cur_x = d[ref_id][param]
        cur_y = ate
        rangedBasedDict.addValue(cur_x, cur_y, d[ref_id])

        for cur_id in d[ref_id]["edges"][param]:
            alreadyTaken.add(cur_id)
            ate = None
            try:
                #print(d[id])
                ate = float(d[cur_id]["ate"])
            except:
                continue

            cur_x = d[cur_id][param]
            cur_y = ate
            rangedBasedDict.addValue(cur_x, cur_y, d[cur_id])

        values.append(rangedBasedDict)

    syncMinAndMaxOf(values)

    if len(values) == 0:
        return
    print("Found " + str(len(values)) + "potential plot")

    newValues = []
    valuesInDataset = {}
    for value in values:
        if value.compute():
            newValues.append(value)
            if value.dataset not in valuesInDataset:
                valuesInDataset[value.dataset] = []
            valuesInDataset[value.dataset].append(value)
    values = newValues

    if len(values) == 0:
        print("No enough values inside each plot")
        return


    f = plt.figure()

    yInDataset = None
    xGlobal = None

    for dataset in valuesInDataset:
        # todo : use a new rangedBasedDict instead, to get the benefit of the weight !
        # y = [0 for i in range(0, len(valuesInDataset[dataset][0].yavg))]
        # for value in valuesInDataset[dataset]:
        #     for i in range(0, len(valuesInDataset[dataset][0].yavg)):
        #         y[i] = y[i] + value.yavg[i]
        # y = [v / len(valuesInDataset[dataset]) for v in y]
        rbd = RangeBasedDict(dataset)
        for value in valuesInDataset[dataset]:
            rbd.addRangeBasedDict(value)
        rbd.compute()
        y = rbd.yavg
        plt.plot(valuesInDataset[dataset][0].x, y, label=dataset + " (" + str(len(valuesInDataset[dataset])) + " set)", linewidth=1.0)
        if yInDataset is None:
            xGlobal = valuesInDataset[dataset][0].x
            yInDataset = [0 for i in range(0, len(valuesInDataset[dataset][0].yavg))]
        yInDataset = [yInDataset[i] + y[i] for i in range(0,len(y))]

    yInDataset = [v / len(valuesInDataset) for v in yInDataset]
    if len(valuesInDataset) > 1:
        plt.plot(xGlobal, yInDataset, label="Average", linewidth=3.0)

    # plt.errorbar(x_intersection, y_sum, yerr=[lowerErrY_sum, upperErrY_sum], fmt='o')

    plt.legend()

    # naming the x axis
    plt.xlabel(param)
    # naming the y axis
    plt.ylabel('Absolute Trajectory Error')

    # giving a title to my graph
    plt.title("ATE according to " + param)

    # function to show the plot
    #plt.show()
    os.makedirs("plot/" + folder, exist_ok=True)
    f.savefig("plot/" + folder + "/" + param + ".pdf")


def makeGraphEdges(d):
    print("Making the graph...", end="")
    for ref_id in d:
        print(".", end="")
        d[ref_id]["edges"] = {}
        for target_id in d:
            if ref_id == target_id:
                continue
            dist, diff = distanceBetweenNode(d[ref_id], d[target_id])
            if dist == 0:
                continue
            if dist <= 1:
                if diff[0] not in d[ref_id]["edges"]:
                    d[ref_id]["edges"][diff[0]] = set()
                d[ref_id]["edges"][diff[0]].add(target_id)
    print("done!")
    return d

if __name__ == "__main__":
    filenames = [x for x in os.listdir(".") if x.endswith(".json")]
    d = None
    for filename in filenames:
        print("Loading " + filename)
        if d is None:
            d = loadJsonFile(filename)
        else:
            d = {**d, **loadJsonFile(filename)}
    params = [x for x in d[next(iter(d))] if x != "ate" and x != "datasetname"]
    datasets = removeDuplicate([d[x]["datasetname"] for x in d])
    d = makeGraphEdges(d)
    for param in params:
        plot(d, param, datasets, folder="ALL_KITTI")
