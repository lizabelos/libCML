import pyximport
import yaml

pyximport.install()

import json
import math
import os
import shutil
import sys
from statistics import mean
import copy
from graphmaker import *

OUTPUT_DIR = "plots/"

def paramToLegend(n):
    if n == "bacondMinimumOrbPoint":
        return "Minimum number of indirect points"
    if n == "bacondSaturatedRatio":
        return "Minimum direct inliers ratio"
    if n == "bacondScoreWeight":
        return "Weight between the number of direct and indirect points"
    if n == "trackcondUncertaintyWeight":
        return "Weight between the direct and indirect uncertainties"
    if n == "orbInlierRatioThreshold":
        return "Inliers ratio to accept an indirect pose estimation"
    if n == "numOrbCorner":
        return "Number of ORB corners"
    if n == "trackingMinimumOrbPoint":
        return "Minimum number of indirect points"
    return n

def meanOrNone(l, weights = None):
    if weights is None:
        weights = [1] * len(l)
    sum = 0
    total = 0
    for i in range(len(l)):
        if l[i] is not None:
            sum += l[i] * weights[i]
            total += weights[i]
    return sum / total if total > 0 else None

def movingAverage(t, iter = 2, delta = 0.75):
    delta_inv = (1 - delta) / 2
    for i in range(0, iter):
        tcopy = t.copy()
        for j in range(1, len(t) - 1):
            t[j] = tcopy[j] * delta + tcopy[j - 1] * delta_inv + tcopy[j + 1] * delta_inv
    return t

def densify(x):
    newX = []
    for i in range(0, len(x)):
        newX.append(x[i])
        if i < len(x) - 1:
            newX.append((x[i] + x[i + 1]) / 2)
    return newX

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
        return 4000

    numFrames=[4541,1101,4661,801,271,2761,1101,1101,4071,1591,1201]

    return numFrames[int(dataset.split(" ")[1])]

def baselineOf(dataset):
    orbResult = [67,20,43,1.0,0.9,43,49,17,58,60,9]
    dsoResult = [114,20,120,2.1,1.5,52,59,17,111,63,16]

    return numFramesOf(dataset)
    # return orbResult[int(dataset.split(" ")[1])] * 0.5 + dsoResult[int(dataset.split(" ")[1])] * 1.5

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

def plotAutoLabel(labels, importantLabel):
    maxDistance = (max(labels) - min(labels)) / 10

    newLabels = set(labels)
    for label in labels:
        if importantLabel is not None:
            if label == importantLabel:
                continue
            if abs(label - importantLabel) < maxDistance:
                newLabels.remove(label)
    labels = list(newLabels)
    labels.sort()

    i = 0
    while True:
        newLabels = set(labels)

        for label in labels:
            if label == labels[i]:
                continue
            if abs(label - labels[i]) < maxDistance:
                newLabels.remove(label)

        labels = list(newLabels)
        labels.sort()

        i = i + 1
        if i >= len(labels):
            break

    labelsstr = [str(x) for x in labels]
    #for i in range(0, len(labelsstr)):
    #    if labelsstr[i] == str(importantLabel):
    #        labelsstr[i] = "\\textbf{" + labelsstr[i] + "}"

    return labels, labelsstr

class PlotSet:

    def __init__(self, paramname, dataset, all_x = None, all_y = None, lower_y = None, upper_y = None):
        if paramname in nodesToIgnore:
            print("ERROR : " + paramname)
            sys.exit(0)
        self.paramName = paramname
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
        self.params = {}

    def addValue(self, x, y, param, lower_y = None, upper_y = None):
        if x == -1:
            return
        self.all_x.append(x)
        self.all_y.append(y)
        self.x_set.add(x)
        self.lower_y.append(lower_y)
        self.upper_y.append(upper_y)
        self.params[x] = param

    def addValueOnlyOne(self, x, y, param, lower_y = None, upper_y = None):
        if lower_y is not None:
            if lower_y > y:
                raise Exception("Lower bound is higher than upper bound")
        if upper_y is not None:
            if upper_y < y:
                raise Exception("Lower bound is higher than upper bound")
        for i in range(0, len(self.all_x)):
            if math.isclose(x, self.all_x[i]):
                if not math.isclose(y, self.all_y[i]):
                    print("CRITICAL ERROR : Value already exist : " + str(x) + " vs " + str(self.all_x[i]))
                return
        self.addValue(x,y,param,lower_y,upper_y)

    def addError(self, x, y, param, n = 1):
        if x not in self.errors:
            self.errors[x] = n
        else:
            self.errors[x] = self.errors[x] + n

    def sort(self):
        all_values = []
        for i in range(0, len(self.all_x)):
            all_values.append([self.all_x[i], self.all_y[i], self.lower_y[i], self.upper_y[i]])
        all_values = sorted(all_values, key=lambda k: k[0])
        for i in range(0, len(self.all_x)):
            self.all_x[i] = all_values[i][0]
            self.all_y[i] = all_values[i][1]
            self.lower_y[i] = all_values[i][2]
            self.upper_y[i] = all_values[i][3]

    def tryMergeSameDataset(l, mergingMode = "all"):
        if len(l) == 0:
            return PlotSet("None", "None")
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
        new_lower_y = []
        new_upper_y = []
        for i in range(0, len(all_x_in_common)):
            current_y = []
            current_lower_y = []
            current_upper_y = []
            for j in range(0, len(l)):
                for k in range(0, len(l[j].all_x)):
                    if l[j].all_x[k] == all_x_in_common[i]:
                        current_y.append(l[j].all_y[k])
                        if l[j].lower_y[k] is not None:
                            current_lower_y.append(l[j].lower_y[k])
                            current_upper_y.append(l[j].upper_y[k])
            new_y.append(current_y)
            new_lower_y.append(current_lower_y)
            new_upper_y.append(current_upper_y)

        new_y = [mean(elem) for elem in new_y]
        new_lower_y = [meanOrNone(elem) for elem in new_lower_y]
        new_upper_y = [meanOrNone(elem) for elem in new_upper_y]
        plotSet = PlotSet(l[0].paramName, dataset, all_x_in_common, new_y, new_lower_y, new_upper_y)
        for elem in l:
            for error in elem.errors:
                plotSet.addError(error, None, None, elem.errors[error])
        return plotSet

    def tryMergeDifferentDataset(l, weighted = True):
        if len(l) == 0:
            return PlotSet("None", "None")
        dataset = "Group"
        all_x_in_common = l[0].all_x
        for i in range(1, len(l)):
            all_x_in_common = elementInCommon(all_x_in_common, l[i].all_x)
        all_x_in_common = sorted(all_x_in_common)
        new_y = []
        new_lower_y = []
        new_upper_y = []
        new_weights = []
        weight_sum = len(l)
        if weighted:
            weight_sum = sum([numFramesOf(elem.dataset) for elem in l])
        for i in range(0, len(all_x_in_common)):
            current_y = []
            current_lower = []
            current_upper = []
            current_weights = []
            for j in range(0, len(l)):
                for k in range(0, len(l[j].all_x)):
                    if l[j].all_x[k] == all_x_in_common[i]:
                        if weighted:
                            current_y.append(l[j].all_y[k] * numFramesOf(l[j].dataset))
                            current_weights.append(numFramesOf(l[j].dataset))
                        else:
                            current_y.append(l[j].all_y[k])
                            current_weights.append(1)
                        current_lower.append(l[j].lower_y[k])
                        current_upper.append(l[j].upper_y[k])
            new_y.append(current_y)
            new_lower_y.append(current_lower)
            new_upper_y.append(current_upper)
            new_weights.append(current_weights)

        lower_y = []
        upper_y = []
        for i in range(0, len(new_y)):
            lower_y.append(meanOrNone(new_lower_y[i], new_weights[i]))
            upper_y.append(meanOrNone(new_upper_y[i], new_weights[i]))
        new_y = [sum(elem) / weight_sum for elem in new_y]
        plotSet = PlotSet(l[0].paramName, dataset, all_x_in_common, new_y, lower_y, upper_y)
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

    def plotBaseline(self, axis, ourResults):
        if not "KITTI" in self.dataset:
            return
        orbResult = [67,None,43,1.0,0.9,43,49,17,58,60,9]
        dsoResult = [114,None,120,2.1,1.5,52,59,17,111,63,16]
        numDataset = int(self.dataset.split(" ")[1])
        baseline = baselineOf(self.dataset)
        minX = min(self.all_x)
        maxX = max(self.all_x)
        if orbResult[numDataset] is not None:
            axis.axhline(orbResult[numDataset] / baseline, color='red', zorder=10, alpha=0.5)
            axis.text(minX, orbResult[numDataset] / baseline, "ORB-SLAM2", zorder=10, color="red", alpha=0.5)
        if dsoResult[numDataset] is not None:
            axis.axhline(dsoResult[numDataset] / baseline, color='red', zorder=10, alpha=0.5)
            axis.text(maxX, dsoResult[numDataset] / baseline, "DSO", zorder=10, ha='right', color="red", alpha=0.5)
        #if ourResults[numDataset] is not None:
        #    axis.axhline(ourResults[numDataset] / baseline, label='ModSLAM', color='red')
        #    axis.text(minX, ourResults[numDataset] / baseline, "ModSLAM")

    def plot(self, axis, paramFilter = None, xticks = None, plotConfidence = False):
        self.sort()
        if paramFilter is not None:
            axis.set_xticks(xticks[0], xticks[1])
            if self.paramName in paramFilter:
                [t.set_fontweight('bold') for t in axis.xaxis.get_ticklabels() if t.get_text() == str(paramFilter[self.paramName])]

        filtered_x = []
        filtered_y = []
        filtered_lower_y = []
        filtered_upper_y = []
        for i in range(0, len(self.all_x)):
            if self.lower_y[i] is not None and self.upper_y[i] is not None:
                #if self.lower_y[i] > self.all_y[i]:
                #    raise Exception("Lower bound is greater than upper bound")
                #if self.upper_y[i] < self.all_y[i]:
                #    raise Exception("Upper bound is smaller than lower bound")
                filtered_x.append(self.all_x[i])
                filtered_y.append(self.all_y[i])
                filtered_lower_y.append(self.lower_y[i])
                filtered_upper_y.append(self.upper_y[i])

        filtered_lower_y = movingAverage(filtered_lower_y, 5, 0.5)
        filtered_upper_y = movingAverage(filtered_upper_y, 5, 0.5)

        filtered_x = densify(filtered_x)
        filtered_y = densify(filtered_y)
        filtered_lower_y = densify(filtered_lower_y)
        filtered_upper_y = densify(filtered_upper_y)

        filtered_lower_y = movingAverage(filtered_lower_y, 5, 0.5)
        filtered_upper_y = movingAverage(filtered_upper_y, 5, 0.5)

        if plotConfidence:
            if len(filtered_x) > 0:
                axis.fill_between(filtered_x, filtered_lower_y, filtered_upper_y, facecolor='red', alpha=0.2, zorder=1000, label="95% confidence interval smoothed")
            else:
                print("No confidence interval for " + self.paramName)
        axis.plot(self.all_x, self.all_y, marker="o", zorder=1000, color='red', linestyle = 'None', label="Absolute Trajectory Error",  markersize=2)
        axis.tick_params(axis='y', labelcolor='red')


    def plotParam(self, axis, param, label = None):
        self.sort()
        all_x = sorted(self.all_x)
        y = []
        if label is None:
            label = param
        for x in all_x:
            y.append(self.params[x][param])
        y = movingAverage(y, 5, 0.75)
        axis.plot(all_x, y, label=label)

    def plotStat(self, axis, stat, label = None):
        self.sort()
        y = []
        if label is None:
            label = stat
        for x in self.all_x:
            y.append(self.params[x]["stats"][stat])
        axis.plot(self.all_x, y, label=label, color='blue', marker="o", linestyle = 'None', markersize=2)
        axis.tick_params(axis='y', labelcolor='blue')

    def plotImportant(self, axis):
        self.sort()
        axis.errorbar(self.all_x, self.all_y, yerr=[self.lower_y, self.upper_y], marker="o", alpha=1.0, zorder=10, color='black', linestyle = 'None', label=self.dataset,  markersize=2)

    def plotPoint(self, axis):
        axis.plot(self.all_x, self.all_y, marker="o", zorder=500, alpha=0.25, color="red", linestyle = 'None', markersize=2)

    def plotError(self, axis):
        errors = copy.deepcopy(self.errors)
        for x in self.x_set:
            if x not in errors:
                errors[x] = 0

        errors = list(errors.items())
        errors.sort(key=lambda k:k[0])

        errors_x = [elem[0] for elem in errors]
        errors_y = [elem[1] for elem in errors]
        errors_width = [min(abs(errors_x[i] - errors_x[(i - 1) % len(errors_x)]), abs(errors_x[i] - errors_x[(i + 1) % len(errors_x)])) for i in range(0, len(errors_x))]
        errors_width = [x * 0.95 for x in errors_width]

        axis2 = axis.twinx()
        axis.patch.set_visible(False)
        axis2.patch.set_visible(False)
        axis.set_zorder(axis2.get_zorder()+1)
        if self.dataset == "Group":
            axis2.set_ylim([0, 11])
        else:
            axis2.set_ylim([0, 1])
        axis2.get_yaxis().set_ticks([])
        axis2.get_yaxis().set_ticklabels([])
        axis2.tick_params(top=False, labeltop=False, right=False, labelright=False)
        axis2.bar(errors_x, errors_y, errors_width, color="gray", label="Errors", zorder=0)

def plot(d, param, datasets, folder, removeConstant = False, onlyAverage = False, paramFilter = None, ours = None, doNothing = False, separate = False, suffix=""):
    values = []
    alreadyTaken = set()
    for ref_id in d:
        if d[ref_id]["datasetname"] not in datasets:
            continue

        if param not in d[ref_id]:
            continue

        if ref_id in alreadyTaken:
            continue

        if paramFilter is not None:
            r = 0
            for filter in paramFilter:
                if filter in nodesToIgnore:
                    continue
                # if filter == param:
                #     continue
                if filter not in d[ref_id]:
                    r += 1
                    continue
                if type(paramFilter[filter]) != type(d[ref_id][filter]):
                    r += 1
                    continue
                if d[ref_id][filter] != paramFilter[filter]:
                    r += 1
                    continue
            if r > 0:
                continue


        if param not in d[ref_id]["edges"]:
            continue

        rangedBasedDict = PlotSet(param, d[ref_id]["datasetname"])

        minAte = 9999999999
        maxAte = 0

        for cur_id in [ref_id] + list(d[ref_id]["edges"][param]):

            if param not in d[cur_id]:
                continue

            if d[cur_id]["datasetname"] != rangedBasedDict.dataset:
                print("CRITICAL ERROR : This is not the same dataset !")
                continue

            alreadyTaken.add(cur_id)

            cur_x = d[cur_id][param]
            if param.startswith("trackcondUncertaintyWeight") and cur_x < -1.5:
                continue

            try:
                ate = float(d[cur_id]["ate"]) / baselineOf(d[ref_id]["datasetname"])
                cur_y = ate
                minAte = min(minAte,ate)
                maxAte = max(maxAte,ate)
                if "ate_upper" in d[cur_id]:
                    rangedBasedDict.addValueOnlyOne(cur_x, cur_y, d[cur_id], d[cur_id]["ate_lower"] / baselineOf(d[ref_id]["datasetname"]), d[cur_id]["ate_upper"] / baselineOf(d[ref_id]["datasetname"]))
                else:
                    rangedBasedDict.addValueOnlyOne(cur_x, cur_y, d[cur_id])
            except ValueError:
                cur_x = d[cur_id][param]
                cur_y = 1
                rangedBasedDict.addError(cur_x, cur_y, d[ref_id])

        if (removeConstant == False or maxAte != minAte) and len(rangedBasedDict.x_set) > 10:
            #rangedBasedDict.movingAverage()
            l = len(rangedBasedDict.x_set)
            needToAppend = True
            for m in range(0, len(values)):
                if values[m].dataset == d[ref_id]["datasetname"]:
                    if len(values[m].x_set) < l:
                        values[m] = rangedBasedDict
                    needToAppend = False
                    break
            if needToAppend:
                values.append(rangedBasedDict)

    cols = 3
    rows = int(math.ceil(len(values) / cols)) + 1

    if len(values) < 5:
        return False
    else:
        print("Plotting " + str(len(values)) + " datasets")

    if doNothing:
        return True

    minX = min([min(v.all_x) for v in values])
    maxX = max([max(v.all_x) for v in values])
    minY = 0
    maxY = 0.06
    #minY = 0
    #maxY = 1
    allX = set()
    for value in values:
        for x in value.all_x:
            allX.add(x)
    allX = list(allX)
    allX.sort()

    xticks = None
    if param in paramFilter:
        xticks = plotAutoLabel(allX, paramFilter[param])
    else:
        xticks = plotAutoLabel(allX, None)

    import matplotlib
    matplotlib.use('pdf')
    import matplotlib.pyplot as plt
    import tikzplotlib as totex
    # import matplotlib2tikz as totex

    #Direct input
    # plt.rcParams['text.latex.preamble']=[r"\usepackage{lmodern}"]
    #Options
    # params = {'text.usetex' : True,
    #           'font.size' : 11,
    #           'font.family' : 'lmodern',
    #           'text.latex.unicode': True,
    #           }
    # plt.rcParams.update(params)

    fig, axs = None, None

    if not separate:
        fig, axs = plt.subplots(rows, cols, figsize=(8.27 * 2,11.69 * 2), clear=True)

    values.sort(key=lambda v:v.dataset)

    valuesInDataset = {}
    averageByDataset = []
    i = 0
    for value in values:

        axis = None

        if separate:
            fig, axis = plt.subplots(figsize=(8.27 / 2,11.69 / 3), clear=True)
        else:
            axis = axs[int(i/cols),int(i%cols)]

        #if param == "trackcondUncertaintyWeight":
        #    axis.set_xscale("log")

        axis.set_xlim([minX, maxX])
        #axis.set_ylim([minY, maxY])

        value.plot(axis, paramFilter=paramFilter, xticks=xticks)
        value.plotError(axis)
        value.plotBaseline(axis,ours)

        # twinAxis = axis.twinx()
        # fps = 1.0 / 10.0
        # twinAxis.set_ylim([numFramesOf(value.dataset) * fps * 0.5, numFramesOf(value.dataset) * fps * 2.0])
        # value.plotParam(twinAxis, "time")
        # value.plotStat(axis.twinx(), "main.Track_DSO_Var")
        # value.plotStat(axis.twinx(), "main.Track_ORB_Var")
        # value.plotStat(axis.twinx(), "main.Tracking_Decision")
        # value.plotStat(axis.twinx(), "main.Bundle_Adjustment_Decision")
        axis.set_title("Influence of " + paramToLegend(param) + " on " + value.dataset)
        twinAxis = axis.twinx()
        if param.startswith("bacond"):
            twinAxis.set_ylim([0, 1])
            value.plotStat(twinAxis, "main.Bundle_Adjustment_Decision", label="Bundle Adjustment Decision")
        elif param.startswith("track"):
            twinAxis.set_ylim([0, 1])
            value.plotStat(twinAxis, "main.Tracking_Decision", label="Tracking Decision")
        axis.set(xlabel=paramToLegend(param), ylabel='Trajectory Error Per Frame')
        # axis.label_outer()

        lines, labels = axis.get_legend_handles_labels()
        lines2, labels2 = twinAxis.get_legend_handles_labels()
        axis.legend(lines + lines2, labels + labels2)
        # naming the x axis
        # plt.xlabel(param)
        # naming the y axis
        # plt.ylabel('Absolute Trajectory Error')

        if separate:
            # print("Saving plot to plot/" + folder + "/" + param + "/" + value.dataset + ".pdf")
            os.makedirs(OUTPUT_DIR + folder + "/" + param + suffix, exist_ok=True)
            fig.savefig(OUTPUT_DIR + folder + "/" + param + suffix + "/" + value.dataset + ".pdf", bbox_inches="tight", dpi=1000)
            # save params
            f = open(OUTPUT_DIR + folder + "/" + param + suffix + "/" + value.dataset + ".yaml", "w")
            yaml.dump(paramFilter, f)
            f.close()
            # totex.save(OUTPUT_DIR + folder + "/" + param + suffix + "/" + value.dataset + ".tex")

        i = i + 1

    axis = None

    if separate:
        fig, axis = plt.subplots(figsize=(8.27 / 2,11.69 / 3), clear=True)
    else:
        axis = axs[int(i/cols),int(i%cols)]

    axis.set_xlim([minX, maxX])
    axis.set_ylim([minY, maxY])
    # axis.label_outer()

    for value in values:
        if value.dataset not in valuesInDataset:
            valuesInDataset[value.dataset] = []
        valuesInDataset[value.dataset].append(value)
        if not onlyAverage:
            value.plotPoint(axis)

    for dataset in valuesInDataset:
        avg = PlotSet.tryMergeSameDataset(valuesInDataset[dataset])
        if avg.dataset != "None":
            if not onlyAverage:
                avg.plot(axis)
            averageByDataset.append(avg)

    avg = PlotSet.tryMergeDifferentDataset(averageByDataset)
    if avg.dataset != "None":
        avg.plot(axis)

    axis.legend()

    axis.set(xlabel=paramToLegend(param), ylabel='ATE')

    # giving a title to my graph
    # plt.title("ATE according to " + param)

    # function to show the plot
    #plt.show()
    if not separate:
        print("Saving plot to plot/" + folder + "/" + param + suffix + ".pdf")
        os.makedirs(OUTPUT_DIR + folder, exist_ok=True)
        fig.tight_layout()
        fig.savefig(OUTPUT_DIR + folder + "/" + param + suffix + ".pdf", bbox_inches="tight", dpi=1000)
    return True

def copyDictAndSet(d, v):
    d2 = {}
    for k in d:
        d2[k] = v
    return d2

def dictMean(d):
    return sum(d.values()) / len(d)

def dictSum(d):
    return sum(d.values())

def computeBestResults(d, datasetLimits):

    # lim = [67,50,43,1.0,0.9,43,49,17,58,60,9]

    alreadyTaken = set()

    allErrors = []

    for ref_id in d:
        if ref_id in alreadyTaken:
            continue
        if d[ref_id]["datasetname"] not in datasetLimits:
            continue
        res = copyDictAndSet(datasetLimits, 9999)
        isbest = copyDictAndSet(datasetLimits, 0)
        originalates = copyDictAndSet(datasetLimits, 0)
        taken = copyDictAndSet(datasetLimits, None)
        if not "same" in d[ref_id]["edges"]:
            continue
        for id in [ref_id] + list(d[ref_id]["edges"]["same"]):
            alreadyTaken.add(id)
            if d[id]["datasetname"] not in datasetLimits:
                continue
            datasetname = d[id]["datasetname"]
            if taken[datasetname] is not None:
                try:
                    ate = float(d[id]["ate"]) / numFramesOf(datasetname)
                    if not math.isclose(ate,res[datasetname]):
                        print("CRITICAL : Something is wrong : " + str(ate) + "/" + str(res[datasetname]))
                        print(taken[datasetname])
                        print(id)
                except ValueError:
                    print("CRITICAL : Something is wrong")
                continue
            try:
                ate = float(d[id]["ate"]) / numFramesOf(datasetname)
                res[datasetname] = ate
                originalates[datasetname] = float(d[id]["ate"])
                if ate < datasetLimits[datasetname] / numFramesOf(datasetname):
                    isbest[datasetname] = 1
                taken[datasetname] = id
            except ValueError:
                continue
        #if any([x == 9999 for x in res]):
        #    continue
        ate = dictMean(res)
        #if ate > 1000:
        #    continue
        paramCopy = copy.deepcopy(d[ref_id])
        del paramCopy["datasetname"]
        del paramCopy["edges"]
        del paramCopy["ate"]
        allErrors.append([ate, dictSum(isbest), paramCopy, res, isbest, originalates])

    allErrors.sort(key=lambda k:k[0])

    if len(allErrors) > 0:
        # print("Best average : " + str(allErrors[0][0]))
        # print("Num best : " + str(allErrors[0][1]))
        # print("Comparaison : " + str(allErrors[0][0] * len(lim) * 100 / 350) + "%")
        # print(allErrors[0][3])
        # print(allErrors[0][4])
        print(allErrors[0][5])
        # print(allErrors[0][2])

    return allErrors

def merge_two_dicts(x, y):
    z = x.copy()   # start with keys and values of x
    z.update(y)    # modifies z with keys and values of y
    return z

def processFile(filenames,foldername):
    from database import loadJsonFile, changeSpaceForPlot, fixRounding, averageParameter

    d = None
    for filename in filenames:
        if d is None:
            d = loadJsonFile(filename, cache=False)
        else:
            d = merge_two_dicts(d, loadJsonFile(filename, cache=False))
    d = changeSpaceForPlot(d)
    d = fixRounding(d)
    d = averageParameter(d, "dsoInitializer.regularizationWeight")

    if len(d) < 100:
        print("Not enough entries")
        return

    params = set()
    for id in d:
        for x in d[id]:
            if x not in nodesToIgnore:
                params.add(x)
    params = list(params)


    stats = set()
    for id in d:
        if not "stats" in d[id]:
            continue
        for s in d[id]["stats"]:
            stats.add(s)

    datasets = removeDuplicate([d[x]["datasetname"] for x in d])

    d = makeGraphEdges(d, params)

    datasetLimits = {}
    kittilim = [67,50,43,1.0,0.9,43,49,17,58,60,9]
    for i in range(0, 11):
        datasetLimits["KITTI " + str(i).zfill(2)] = kittilim[i]
    for i in range(1, 51):
        datasetLimits["TUM " + str(i).zfill(2)] = 6

    bestResults = computeBestResults(d, datasetLimits)

    # Compute result with most best
    bestNumBest = 0
    resultWithHigherNumBest = None
    for bestResult in bestResults:
        if bestResult[1] > bestNumBest:
            bestNumBest = bestResult[1]
            resultWithHigherNumBest = bestResult

    if True:
        print("Best average : " + str(resultWithHigherNumBest[0]))
        print("Num best : " + str(resultWithHigherNumBest[1]))
        print(resultWithHigherNumBest[3])
        print(resultWithHigherNumBest[4])
        print(resultWithHigherNumBest[5])
        print(resultWithHigherNumBest[2])

    bestResultsToTest = {}

    print("{")

    for i in range(0, len(bestResults)):
        printedThisIteration = False
        for param in bestResults[i][2]:
            if param in nodesToIgnore:
                continue
            if param not in bestResultsToTest:
                if not printedThisIteration:
                    print("    #" + str(i))
                    printedThisIteration = True
                bestResultsToTest[param] = bestResults[i][2][param]
                print("    '" + param + "': " + str(bestResults[i][2][param]) + ",")

    print("}")

    shutil.rmtree(OUTPUT_DIR + "/" + foldername, ignore_errors=True)

    for param in params:

        print("Plotting " + param + "...")

        for i in range(0, len(bestResults)):

            suffix = ""

            if param == "bacondSaturatedRatio":
                if not "bacondMinimumOrbPoints" in bestResults[i][2]:
                    continue
                if bestResults[i][2]["bacondMinimumOrbPoint"] != -1:
                    continue

            if param == "bacondScoreWeight":
                if not "bacondMinimumOrbPoint" in bestResults[i][2] or not "bacondSaturatedRatio" in bestResults[i][2]:
                    continue
                if bestResults[i][2]["bacondMinimumOrbPoint"] != -1 or bestResults[i][2]["bacondSaturatedRatio"] != -1:
                    continue

            if param.startswith("trackcondUncertaintyWeight"):
                if param == "trackcondUncertaintyWeight":
                    if not "trackingMinimumOrbPoint" in bestResults[i][2]:
                        continue
                    if bestResults[i][2]["trackingMinimumOrbPoint"] != -1:
                        continue
                if "trackcondUncertaintyWindow" in bestResults[i][2]:
                    suffix = "_w" + str(bestResults[i][2]["trackcondUncertaintyWindow"])

            if param == "trackcondUncertaintyWindow":
                if not "trackingMinimumOrbPoint" in bestResults[i][2]:
                    continue
                if bestResults[i][2]["trackingMinimumOrbPoint"] != -1:
                    continue

            if os.path.isfile(OUTPUT_DIR + "/" + foldername + "/" + param + suffix + ".pdf"):
                continue
            plot(d, param, datasets, ours=bestResults[i][3], folder=foldername, onlyAverage=True, paramFilter=bestResults[i][2], suffix=suffix, separate=False)
            plot(d, param, datasets, ours=bestResults[i][3], folder=foldername, onlyAverage=True, paramFilter=bestResults[i][2], suffix=suffix, separate=True)

if __name__ == "__main__":
    #dirname = "jsons"
    #filenames = [dirname+"/"+x for x in os.listdir(dirname) if x.endswith(".json")]
    #processFile(filenames, "merged")

    dirname = "jsons"
    # filenames = [x for x in os.listdir(dirname) if x.endswith(".json")]
    filenames = ["787f82896c83445d2e3b55acbccc3735274c6bdb00de6fe948f568f009d96808.json"]
    for filename in filenames:
        processFile([dirname+"/"+filename],filename.split(".")[0])

    #os.system("pdflatex -interaction nonstopmode -file-line-error .\output.tex")
