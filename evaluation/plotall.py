import json
import os
import matplotlib.pyplot as plt
import yaml

from database import loadJsonFile, hashOfDict
import copy
from statistics import mean
from scipy.stats import sem
from math import sqrt

from utils import floatrange

colorsByDataset = {"KITTI 00" : '#1f77b4',
          "KITTI 01" : '#ff7f0e',
          "KITTI 02" : '#2ca02c',
          "KITTI 03" : '#d62728',
          "KITTI 04" : '#9467bd',
          "KITTI 05" : '#8c564b',
          "KITTI 06" : '#e377c2',
          "KITTI 07" : '#7f7f7f',
          "KITTI 08" : '#bcbd22',
          "KITTI 09" : '#17becf',
          "KITTI 10" : '#1777c2',
          "Group": "#000000"
          }

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

def safeMean(l):
    if len(l) == 0:
        return None
    else:
        return mean(l)

class PlotSet:

    def __init__(self, dataset, all_x = None, all_y = None, lower_y = None, upper_y = None):
        self.dataset = dataset
        if all_x is not None and all_y is not None:
            self.all_x = all_x
            self.all_y = all_y
            self.x_set = set(self.all_x)
            self.lower_y = lower_y
            self.upper_y = upper_y
        else:
            self.all_x = []
            self.x_set = set()
            self.all_y = []
            self.lower_y = []
            self.upper_y = []
        self.errors = {}

    def addValue(self, x, y, param):
        self.all_x.append(x)
        self.all_y.append(y)
        self.x_set.add(x)
        self.lower_y.append(0)
        self.upper_y.append(0)

    def addError(self, x, y, param, n = 1):
        if x not in self.errors:
            self.errors[x] = n
        else:
            self.errors[x] = self.errors[x] + n

    def sort(self):
        all_values = []
        for i in range(0, len(self.all_x)):
            all_values.append([self.all_x[i], self.all_y[i]])
        all_values = sorted(all_values, key=lambda k: k[0])
        for i in range(0, len(self.all_x)):
            self.all_x[i] = all_values[i][0]
            self.all_y[i] = all_values[i][1]

    def tryMergeSameDataset(l, mergingMode = "all"):
        if len(l) == 0:
            return PlotSet("None")
        dataset = l[0].dataset
        all_x_in_common = None
        if mergingMode == "all":
            all_x_in_common = set()
            for i in range(0, len(l)):
                for cur_x in l[i].all_x:
                    all_x_in_common.add(cur_x)
        if mergingMode == "common":
            all_x_in_common = l[0].all_x
            for i in range(1, len(l)):
                all_x_in_common = elementInCommon(all_x_in_common, l[i].all_x)
        all_x_in_common = sorted(list(all_x_in_common))
        new_y = []
        for i in range(0, len(all_x_in_common)):
            current_y = []
            for j in range(0, len(l)):
                for k in range(0, len(l[j].all_x)):
                    if l[j].all_x[k] == all_x_in_common[i]:
                        current_y.append(l[j].all_y[k])
            new_y.append(current_y)

        lower_y = [lowerErrorMedian(elem) for elem in new_y]
        upper_y = [upperErrorMedian(elem) for elem in new_y]
        new_y = [mean(elem) for elem in new_y]
        plotSet = PlotSet(dataset, all_x_in_common, new_y, lower_y, upper_y)
        for elem in l:
            for error in elem.errors:
                plotSet.addError(error, None, None, elem.errors[error])
        return plotSet

    def tryMergeDifferentDataset(l, weighted = False):
        if len(l) == 0:
            return PlotSet("None")
        dataset = "Group"
        all_x_in_common = l[0].all_x
        for i in range(1, len(l)):
            all_x_in_common = elementInCommon(all_x_in_common, l[i].all_x)
        all_x_in_common = sorted(all_x_in_common)
        new_y = []
        new_lower_y = []
        new_upper_y = []
        weight_sum = len(l)
        if weighted:
            weight_sum = sum([numFramesOf(elem.dataset) for elem in l])
        for i in range(0, len(all_x_in_common)):
            current_y = []
            current_lower = []
            current_upper = []
            for j in range(0, len(l)):
                for k in range(0, len(l[j].all_x)):
                    if l[j].all_x[k] == all_x_in_common[i]:
                        if weighted:
                            current_y.append(l[j].all_y[k] * numFramesOf(l[j].dataset))
                        else:
                            current_y.append(l[j].all_y[k])
                        current_lower.append(l[j].lower_y[k])
                        current_upper.append(l[j].upper_y[k])
            new_y.append(current_y)
            new_lower_y.append(current_lower)
            new_upper_y.append(current_upper)

        lower_y = [mean(elem) for elem in new_lower_y]
        upper_y = [mean(elem) for elem in new_upper_y]
        new_y = [sum(elem) / weight_sum for elem in new_y]
        plotSet = PlotSet(dataset, all_x_in_common, new_y, lower_y, upper_y)
        for elem in l:
            for error in elem.errors:
                plotSet.addError(error, None, None, elem.errors[error])
        return plotSet

    def movingAverage(self, iter = 5, delta = 0.75):
        self.sort()
        delta_inv = (1 - delta) / 2
        for i in range(0, iter):
            for j in range(1, len(self.all_y) - 1):
                self.all_y[j] = self.all_y[j] * delta + self.all_y[j - 1] * delta_inv + self.all_y[j + 1] * delta_inv

    def plot(self, axis):
        self.sort()
        axis.errorbar(self.all_x, self.all_y, yerr=[self.lower_y, self.upper_y], marker="o", alpha=0.75, zorder=1000, color=colorsByDataset[self.dataset], linestyle = 'None', label=self.dataset,  markersize=2)

    def plotImportant(self, axis):
        self.sort()
        axis.errorbar(self.all_x, self.all_y, yerr=[self.lower_y, self.upper_y], marker="o", alpha=1.0, zorder=0, color=colorsByDataset[self.dataset], linestyle = 'None', label=self.dataset,  markersize=2)

    def plotPoint(self):
        plt.plot(self.all_x, self.all_y, marker="o", zorder=500, alpha=0.25, color=colorsByDataset[self.dataset], linestyle = 'None', markersize=2)

    def plotError(self, axis):
        x = []
        y = []
        for error in self.errors:
            x.append(error)
            y.append(self.errors[error])
        if len(y) == 0:
            return
        m = min(y)
        y = [elem - m for elem in y]
        m = max(y)
        if m == 0:
            return
        diffs = []
        for a in x:
            for b in x:
                if a == b:
                    continue
                diffs.append(abs(a - b))
        diffs = sorted(diffs)
        minXdiff = diffs[int(len(diffs) / 20)]
        y = [(elem / 50) / m for elem in y]
        # todo : show each bar with color ?
        axis.bar(x, y, width=minXdiff * 0.9, color="#cccccc", label="Error", zorder=0)

def plot(d, param, datasets, folder, removeConstant = False, onlyAverage = False, hashFilter = None):
    values = []
    alreadyTaken = set()
    for ref_id in d:
        if d[ref_id]["datasetname"] not in datasets:
            continue

        if param not in d[ref_id]["edges"]:
            continue

        if ref_id in alreadyTaken:
            continue

        if hashFilter is not None:
            elem = copy.deepcopy(d[ref_id])
            del elem["datasetname"]
            del elem["ate"]
            del elem["edges"]
            h = hashOfDict(elem)
            if h != hashFilter:
                continue

        rangedBasedDict = PlotSet(d[ref_id]["datasetname"])

        alreadyTaken.add(ref_id)

        ate = None
        minAte = 9999999999
        maxAte = 0
        try:
            #print(d[id])
            ate = float(d[ref_id]["ate"]) / numFramesOf(d[ref_id]["datasetname"])
            cur_x = d[ref_id][param]
            cur_y = ate
            minAte = min(minAte,ate)
            maxAte = max(maxAte,ate)
            rangedBasedDict.addValue(cur_x, cur_y, d[ref_id])
        except:
            cur_x = d[ref_id][param]
            cur_y = 1
            rangedBasedDict.addError(cur_x, cur_y, d[ref_id])



        for cur_id in d[ref_id]["edges"][param]:
            alreadyTaken.add(cur_id)
            ate = None
            try:
                #print(d[id])
                ate = float(d[cur_id]["ate"]) / numFramesOf(d[ref_id]["datasetname"])
                cur_x = d[cur_id][param]
                cur_y = ate
                minAte = min(minAte,ate)
                maxAte = max(maxAte,ate)
                rangedBasedDict.addValue(cur_x, cur_y, d[cur_id])
            except:
                cur_x = d[cur_id][param]
                cur_y = 1
                rangedBasedDict.addError(cur_x, cur_y, d[ref_id])

        if (removeConstant == False or maxAte != minAte) and len(rangedBasedDict.x_set) > 3:
            #rangedBasedDict.movingAverage()
            values.append(rangedBasedDict)
        else:
            print("Removing constant values. Diff : " + str(abs(maxAte - minAte)))

    cols = 4
    rows = int((len(values) + 2) / cols) + 2



    fig, axs = plt.subplots(rows, cols, figsize=(8.27,11.69))

    valuesInDataset = {}
    averageByDataset = []
    i = 0
    for value in values:

        axis = axs[int(i/cols),int(i%cols)]

        value.plot(axis)
        value.plotError(axis)
        axis.set_title(value.dataset)
        axis.set(xlabel=param, ylabel='ATE')
        axis.label_outer()
        # axis.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, ncol=3)
        # naming the x axis
        # plt.xlabel(param)
        # naming the y axis
        # plt.ylabel('Absolute Trajectory Error')

        i = i + 1

    if i == 0:
        return

    axis = axs[int(i/cols),int(i%cols)]

    for value in values:
        if value.dataset not in valuesInDataset:
            valuesInDataset[value.dataset] = []
        valuesInDataset[value.dataset].append(value)
        if not onlyAverage:
            value.plotPoint()

    for dataset in valuesInDataset:
        avg = PlotSet.tryMergeSameDataset(valuesInDataset[dataset])
        if avg.dataset != "None":
            if not onlyAverage:
                avg.plot(axis)
            averageByDataset.append(avg)

    avg = PlotSet.tryMergeDifferentDataset(averageByDataset)
    if avg.dataset != "None":
        avg.plotImportant(axis)
        avg.plotError(axis)

    #axis.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, ncol=3)

    axis.set(xlabel=param, ylabel='ATE')

    # giving a title to my graph
    # plt.title("ATE according to " + param)

    # function to show the plot
    #plt.show()
    os.makedirs("plot/" + folder, exist_ok=True)
    fig.savefig("plot/" + folder + "/" + param + ".pdf", bbox_inches="tight")

    plt.clf()

def makeGraphEdges(d):
    print("Making the graph...", end="")
    for ref_id in d:
        #print(".", end="")
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

def printBestResult(d, divideByNumFrames = False, multiplyByNumFrames = False, maximiseBest = False):
    paramToDatasetToAte = {}
    paramToDatasetToAteNonWeigted = {}
    paramToParam = {}
    lim=[77,20,41,1,1,40,49,16,51,58,15]
    for id in d:
        elem = copy.deepcopy(d[id])
        datasetname = elem["datasetname"]
        ate = None
        try:
            ate = float(elem["ate"])
            ate2 = ate
            if divideByNumFrames:
                ate = ate / numFramesOf(datasetname)
            if multiplyByNumFrames:
                ate = ate * numFramesOf(datasetname)
            if maximiseBest:
                ate = ate / lim[int(datasetname.split(" ")[1])]
                if ate < 1:
                    ate = 0
                ate = ate / numFramesOf(datasetname)
        except:
            continue
        del elem["datasetname"]
        #del elem["edges"]
        del elem["ate"]
        h = hashOfDict(elem)
        paramToParam[h] = elem
        if h not in paramToDatasetToAte:
            paramToDatasetToAte[h] = {}
            paramToDatasetToAteNonWeigted[h] = {}
        if datasetname not in paramToDatasetToAte[h]:
            paramToDatasetToAte[h][datasetname] = []
            paramToDatasetToAteNonWeigted[h][datasetname] = []
        paramToDatasetToAte[h][datasetname].append(ate)
        paramToDatasetToAteNonWeigted[h][datasetname].append(ate2)

    bestCount = 0
    bestAvg = 99999
    bestParam = None
    for param in paramToDatasetToAte:
        all_avg = []
        for dataset in paramToDatasetToAte[param]:
            avg = mean(paramToDatasetToAte[param][dataset])
            all_avg.append(avg)
        count = len(all_avg)
        if count < bestCount:
            continue
        avg = sum(all_avg)
        if avg < bestAvg or count > bestCount:
            bestAvg = avg
            bestParam = param
            bestCount = count

    if bestParam is not None:
        bestAvg = sum([mean(paramToDatasetToAteNonWeigted[bestParam][v]) for v in paramToDatasetToAteNonWeigted[bestParam]])
        numBest = 0
        for v in paramToDatasetToAteNonWeigted[bestParam]:
            m = mean(paramToDatasetToAteNonWeigted[bestParam][v])
            l=lim[int(v.split(" ")[1])]
            if m < l:
                numBest = numBest + 1
        print("Best param : " + str(bestParam))
        print("Best average : " + str(bestAvg))
        print("Num best : " + str(numBest))
        print("Comparaison : " + str(bestAvg * 100 / 350) + "%")
        print(paramToDatasetToAteNonWeigted[bestParam])
        with open("best.yaml", "w") as outfile:
            yaml.dump(paramToParam[bestParam], outfile)

    return bestParam


def missingExperiencesForValue(value, range):
    missing = set(range)
    for x in value.all_x:
        if x in missing:
            missing.remove(x)
    for x in value.errors:
        if x in missing:
            missing.remove(x)
    print("Missing : " + str(missing))

def missingExperiencesForParam(d, param, dataset, range):
    print("Computing missing experiences")
    values = []
    alreadyTaken = set()
    for ref_id in d:
        if d[ref_id]["datasetname"] != dataset:
            continue

        if param not in d[ref_id]["edges"]:
            continue

        if ref_id in alreadyTaken:
            continue

        rangedBasedDict = PlotSet(d[ref_id]["datasetname"])

        alreadyTaken.add(ref_id)

        ate = None
        minAte = 9999999999
        maxAte = 0
        try:
            #print(d[id])
            ate = float(d[ref_id]["ate"]) / numFramesOf(d[ref_id]["datasetname"])
            cur_x = d[ref_id][param]
            cur_y = ate
            minAte = min(minAte,ate)
            maxAte = max(maxAte,ate)
            rangedBasedDict.addValue(cur_x, cur_y, d[ref_id])
        except:
            cur_x = d[ref_id][param]
            cur_y = 1
            rangedBasedDict.addError(cur_x, cur_y, d[ref_id])



        for cur_id in d[ref_id]["edges"][param]:
            alreadyTaken.add(cur_id)
            ate = None
            try:
                #print(d[id])
                ate = float(d[cur_id]["ate"]) / numFramesOf(d[ref_id]["datasetname"])
                cur_x = d[cur_id][param]
                cur_y = ate
                minAte = min(minAte,ate)
                maxAte = max(maxAte,ate)
                rangedBasedDict.addValue(cur_x, cur_y, d[cur_id])
            except:
                cur_x = d[cur_id][param]
                cur_y = 1
                rangedBasedDict.addError(cur_x, cur_y, d[ref_id])

        if len(rangedBasedDict.x_set) > 3:
            rangedBasedDict.movingAverage()
            values.append(rangedBasedDict)
        else:
            print("Removing constant values. Diff : " + str(abs(maxAte - minAte)))

    for value in values:
        missingExperiencesForValue(value, range)

def missingExperiences(d, datasets):
    params = [
        ["bacondScoreWeight", floatrange(0.1,2,0.1)],
        ["orbInlierRatioThreshold", floatrange(0.0,1.01,0.01)]
    ]

    for param in params:
        for dataset in datasets:
            missingExperiencesForParam(d, param[0], dataset, param[1])

def merge_two_dicts(x, y):
    z = x.copy()   # start with keys and values of x
    z.update(y)    # modifies z with keys and values of y
    return z

if __name__ == "__main__":
    filenames = [x for x in os.listdir(".") if x.endswith(".json")]
    d = None
    for filename in filenames:
        print("Loading " + filename)
        if d is None:
            d = loadJsonFile(filename, cache=False)
        else:
            d = merge_two_dicts(d, loadJsonFile(filename, cache=False))


    params = [x for x in d[next(iter(d))] if x != "ate" and x != "datasetname"]
    datasets = removeDuplicate([d[x]["datasetname"] for x in d])
    print(datasets)
    print("Best result by sum of ate : ")
    bestHash = printBestResult(d)


    #print("Best result by sum of ate divided by num frames : ")
    #printBestResult(d, divideByNumFrames=True)
    #print("Best result by sum of ate multiplied by num frames : ")
    #printBestResult(d, multiplyByNumFrames=True)
    #print("Best result : ")
    #printBestResult(d, maximiseBest=True)
    d = makeGraphEdges(d)
    # missingExperiences(d, datasets)
    #for dataset in datasets:
    #    for param in params:
    #        plot(d, param, [dataset], folder=dataset)
    for param in params:
        plot(d, param, datasets, folder="All", onlyAverage=True, hashFilter = bestHash)
