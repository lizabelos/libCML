import json
import math
import os
import shutil
import sys
from statistics import mean
import copy
from tqdm.contrib.concurrent import process_map

nodesToIgnore = {"ate","edges","datasetname","stats","time"}

OUTPUT_DIR = "3_mixed_slam/2_ablation_study/plot/"

def appendBigTitleToLatex(v):
    with open('output.tex', 'a') as f:
        f.write("\\section{%s}\n" % v)

def appendTitleToLatex(v):
    with open('output.tex', 'a') as f:
        f.write("\\subsection{%s}\n" % v)

def appendSubtitleToLatex(v):
    with open('output.tex', 'a') as f:
        f.write("\\subsubsection{%s}\n" % v)

def appendInputToLatex(v):
    with open('output.tex', 'a') as f:
        f.write("\\input{%s}\n" % v)

def addNewLineToLatex():
    with open('output.tex', 'a') as f:
        f.write("\n\n")

def beginLatexDocument():
    with open('output.tex', 'a') as f:
        f.write("\\begin{document}\n")

def endLatexDocument():
    with open('output.tex', 'a') as f:
        f.write("\\end{document}\n")

def distanceBetweenNode(p1, p2, ignore = "", maxlen = 1):
    if len((p1.keys()|nodesToIgnore) ^ (p2.keys()|nodesToIgnore)) > 0:
        return None
    if any([not isinstance(p1[index], type(p2[index])) for index in p1 if not index == "ate" and not index == "edges" and not index == "datasetname" and not index == ignore]):
        return None
    diff = []
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

    def addValue(self, x, y, param):
        self.all_x.append(x)
        self.all_y.append(y)
        self.x_set.add(x)
        self.lower_y.append(0)
        self.upper_y.append(0)
        self.params[x] = param

    def addValueOnlyOne(self, x, y, param):
        for i in range(0, len(self.all_x)):
            if math.isclose(x, self.all_x[i]):
                if not math.isclose(y, self.all_y[i]):
                    print("CRITICAL ERROR : Value already exist : " + str(x) + " vs " + str(self.all_x[i]))
                return
        self.addValue(x,y,param)

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
        plotSet = PlotSet(l[0].paramName, dataset, all_x_in_common, new_y, lower_y, upper_y)
        for elem in l:
            for error in elem.errors:
                plotSet.addError(error, None, None, elem.errors[error])
        return plotSet

    def tryMergeDifferentDataset(l, weighted = False):
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
        orbResult = [67,None,43,1.0,0.9,43,49,17,58,60,9]
        dsoResult = [114,None,120,2.1,1.5,52,59,17,111,63,16]
        numDataset = int(self.dataset.split(" ")[1])
        baseline = baselineOf(self.dataset)
        minX = min(self.all_x)
        maxX = max(self.all_x)
        if orbResult[numDataset] is not None:
            axis.axhline(orbResult[numDataset] / baseline, color='blue', zorder=10)
            axis.text(minX, orbResult[numDataset] / baseline, "ORB-SLAM2", zorder=10, color="blue")
        if dsoResult[numDataset] is not None:
            axis.axhline(dsoResult[numDataset] / baseline, color='red', zorder=10)
            axis.text(maxX, dsoResult[numDataset] / baseline, "DSO", zorder=10, ha='right', color="red")
        #if ourResults[numDataset] is not None:
        #    axis.axhline(ourResults[numDataset] / baseline, label='ModSLAM', color='red')
        #    axis.text(minX, ourResults[numDataset] / baseline, "ModSLAM")

    def plot(self, axis, paramFilter = None, xticks = None):
        self.sort()
        if paramFilter is not None:
            axis.set_xticks(xticks[0], xticks[1])
            [t.set_fontweight('bold') for t in axis.xaxis.get_ticklabels() if t.get_text() == str(paramFilter[self.paramName])]

        axis.errorbar(self.all_x, self.all_y, yerr=[self.lower_y, self.upper_y], marker="o", alpha=0.75, zorder=1000, color='green', linestyle = 'None', label=self.dataset,  markersize=2)

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
        axis2.bar(errors_x, errors_y, errors_width, color="gray", label="Errors", zorder=0)

def plot(d, param, datasets, folder, removeConstant = False, onlyAverage = False, paramFilter = None, ours = None, doNothing = False, separate = False):
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
            r = distanceBetweenNode(d[ref_id], paramFilter, ignore=param)
            if r is None:
                continue
            if len(r) > 0:
                continue


        if param not in d[ref_id]["edges"]:
            continue

        rangedBasedDict = PlotSet(param, d[ref_id]["datasetname"])

        minAte = 9999999999
        maxAte = 0

        for cur_id in [ref_id] + list(d[ref_id]["edges"][param]):

            if cur_id != ref_id:
                test = distanceBetweenNode(d[ref_id], d[cur_id])
                if len(test) != 1 or test[0] != param:
                    print("CRITICAL ERROR : The graph is not well constructed")
                    print(test)
                    continue

            if d[cur_id]["datasetname"] != rangedBasedDict.dataset:
                print("CRITICAL ERROR : This is not the same dataset !")
                continue

            alreadyTaken.add(cur_id)
            try:
                ate = float(d[cur_id]["ate"]) / baselineOf(d[ref_id]["datasetname"])
                cur_x = d[cur_id][param]
                cur_y = ate
                minAte = min(minAte,ate)
                maxAte = max(maxAte,ate)
                rangedBasedDict.addValueOnlyOne(cur_x, cur_y, d[cur_id])
            except:
                cur_x = d[cur_id][param]
                cur_y = 1
                rangedBasedDict.addError(cur_x, cur_y, d[ref_id])

        if (removeConstant == False or maxAte != minAte) and len(rangedBasedDict.x_set) > 1:
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
    rows = 4

    if len(values) < 5:
        return False

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
    xticks = plotAutoLabel(allX, paramFilter[param])

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

        axis.set_xlim([minX, maxX])
        axis.set_ylim([minY, maxY])

        value.plot(axis, paramFilter=paramFilter, xticks=xticks)
        value.plotError(axis)
        value.plotBaseline(axis,ours)
        # axis.set_title(value.dataset)
        axis.set(xlabel=paramToLegend(param), ylabel='Trajectory Error Per Frame')
        # axis.label_outer()
        axis.legend()
        # naming the x axis
        # plt.xlabel(param)
        # naming the y axis
        # plt.ylabel('Absolute Trajectory Error')

        if separate:
            print("Saving plot to plot/" + folder + "/" + param + "/" + value.dataset + ".pdf")
            os.makedirs(OUTPUT_DIR + folder + "/" + param, exist_ok=True)
            fig.savefig(OUTPUT_DIR + folder + "/" + param + "/" + value.dataset + ".pdf", bbox_inches="tight", dpi=1000)
            #appendSubtitleToLatex(param + " " + value.dataset)
            if i % 2 == 0:
                addNewLineToLatex()
            appendInputToLatex(OUTPUT_DIR + folder + "/" + param + "/" + value.dataset + ".tex")
            totex.save(OUTPUT_DIR + folder + "/" + param + "/" + value.dataset + ".tex")

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
        avg.plotImportant(axis)
        avg.plotError(axis)

    axis.legend()

    axis.set(xlabel=paramToLegend(param), ylabel='ATE')

    # giving a title to my graph
    # plt.title("ATE according to " + param)

    # function to show the plot
    #plt.show()
    if separate:
        print("Saving plot to plot/" + folder + "/" + param + "/" + "avg" + ".pdf")
        os.makedirs(OUTPUT_DIR + folder + "/" + param, exist_ok=True)
        fig.savefig(OUTPUT_DIR + folder + "/" + param + "/" + "avg" + ".pdf", bbox_inches="tight", dpi=1000)
    else:
        print("Saving plot to plot/" + folder + "/" + param + ".pdf")
        os.makedirs(OUTPUT_DIR + folder, exist_ok=True)
        fig.savefig(OUTPUT_DIR + folder + "/" + param + ".pdf", bbox_inches="tight", dpi=1000)
    return True

def makeGraphEdges_job(j):
    return j[2], j[3], distanceBetweenNode(j[0], j[1])

def makeGraphEdges(d, params):
    k = list(d.keys())

    results = None

    print("Creating jobs...")
    jobs = []
    for ref_id_i in range(0, len(k)):
        ref_id = k[ref_id_i]
        for target_id_i in range(ref_id_i + 1, len(k)):
            target_id = k[target_id_i]
            jobs.append([d[ref_id], d[target_id], ref_id, target_id])
    print("Launching jobs...")
    if len(d) < 2000:
        results = list(map(makeGraphEdges_job, jobs))
    else:
        results = process_map(makeGraphEdges_job, jobs, max_workers=20, chunksize=int(len(jobs)/20))

    print("Parsing jobs...")
    for ref_id in d:
        d[ref_id]["edges"] = {}
        for param in params + ["same"]:
            d[ref_id]["edges"][param] = []
    for ref_id,target_id,diff in results:
        if diff is None:
            continue
        dist = len(diff)
        if dist == 0:
            d[ref_id]["edges"]["same"].append(target_id)
            d[target_id]["edges"]["same"].append(ref_id)
        elif dist <= 1:
            if d[ref_id]["datasetname"] != d[target_id]["datasetname"]:
                continue
            d[ref_id]["edges"][diff[0]].append(target_id)
            d[target_id]["edges"][diff[0]].append(ref_id)
    return d

def computeBestResults(d):

    lim = [67,50,43,1.0,0.9,43,49,17,58,60,9]

    alreadyTaken = set()

    allErrors = []

    for ref_id in d:
        if ref_id in alreadyTaken:
            continue
        res = [9999 for x in lim]
        isbest = [0 for x in lim]
        taken = [None for x in lim]
        if not "same" in d[ref_id]["edges"]:
            continue
        for id in [ref_id] + list(d[ref_id]["edges"]["same"]):
            alreadyTaken.add(id)
            datasetname = d[id]["datasetname"]
            if taken[int(datasetname.split(" ")[1])] is not None:
                try:
                    ate = float(d[id]["ate"]) / numFramesOf(datasetname)
                    if not math.isclose(ate,res[int(datasetname.split(" ")[1])]):
                        print("CRITICAL : Something is wrong : " + str(ate) + "/" + str(res[int(datasetname.split(" ")[1])]))
                        print(taken[int(datasetname.split(" ")[1])])
                        print(id)
                except:
                    print("CRITICAL : Something is wrong")
                continue
            try:
                ate = float(d[id]["ate"]) / numFramesOf(datasetname)
                res[int(datasetname.split(" ")[1])] = ate
                if ate < lim[int(datasetname.split(" ")[1])] / numFramesOf(datasetname):
                    isbest[int(datasetname.split(" ")[1])] = 1
                taken[int(datasetname.split(" ")[1])] = id
            except:
                continue
        if any([x == 9999 for x in res]):
            continue
        ate = mean(res)
        if ate > 1000:
            continue
        paramCopy = copy.deepcopy(d[ref_id])
        del paramCopy["datasetname"]
        del paramCopy["edges"]
        del paramCopy["ate"]
        allErrors.append([ate, sum(isbest), paramCopy, res, isbest])

    allErrors.sort(key=lambda k:k[0])

    if len(allErrors) > 0:
        print("Best average : " + str(allErrors[0][0]))
        print("Num best : " + str(allErrors[0][1]))
        print("Comparaison : " + str(allErrors[0][0] * len(lim) * 100 / 350) + "%")
        print(allErrors[0][3])
        print(allErrors[0][4])
        #with open("best.yaml", "w") as outfile:
        #    yaml.dump(paramToParam[bestParam], outfile)

    return allErrors

def merge_two_dicts(x, y):
    z = x.copy()   # start with keys and values of x
    z.update(y)    # modifies z with keys and values of y
    return z

def processFile(filename,foldername):
    from database import loadJsonFile, fixRounding

    d = loadJsonFile(filename, cache=False)
    d = fixRounding(d)

    if len(d) < 100:
        return
    print("Processing " + filename + " with " + str(len(d)) + " entries")
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

    print(stats)

    datasets = removeDuplicate([d[x]["datasetname"] for x in d])

    d = makeGraphEdges(d, params)

    bestResults = computeBestResults(d)

    for i in range(0, len(bestResults)):
        numOrbConer = bestResults[i][2]["numOrbCorner"]

        #os.makedirs(OUTPUT_DIR + foldername, exist_ok=True)
        #with open(OUTPUT_DIR + foldername + "/" + str(i) + "_" + str(numOrbConer) + ".json", "w") as outfile:
        #    toDump = copy.deepcopy(bestResults[i][2])
        #    for toIgnore in nodesToIgnore:
        #        if toIgnore in toDump:
        #            del toDump[toIgnore]
        #    toDump["results"] = [bestResults[i][0], bestResults[i][1], bestResults[i][3], bestResults[i][4]]
        #    json.dump(toDump, outfile, indent=4)

        toPlot = []
        for param in params:
            res = plot(d, param, datasets, ours=bestResults[i][3], folder=foldername + "/" + str(i) + "_" + str(numOrbConer), onlyAverage=True, paramFilter=bestResults[i][2], doNothing=True)
            if res:
                toPlot.append(param)
        if len(toPlot) > 0:
            appendBigTitleToLatex(foldername[0:4] + " " + str(i) + " " + str(numOrbConer))
            for param in toPlot:
                appendTitleToLatex(param)
                plot(d, param, datasets, ours=bestResults[i][3], folder=foldername + "/" + str(i) + "_" + str(numOrbConer), onlyAverage=True, paramFilter=bestResults[i][2])
                plot(d, param, datasets, ours=bestResults[i][3], folder=foldername + "/" + str(i) + "_" + str(numOrbConer), onlyAverage=True, paramFilter=bestResults[i][2], separate=True)
            break

if __name__ == "__main__":
    appendInputToLatex("header.tex")
    beginLatexDocument()
    dirname = "."
    filenames = [x for x in os.listdir(dirname) if x.endswith(".json")]
    for filename in filenames:
        processFile(dirname+"/"+filename,filename.split(".")[0])

    #dirname = "oldjson"
    #filenames = [x for x in os.listdir(dirname) if x.endswith(".json")]
    #for filename in filenames:
    #    processFile(dirname+"/"+filename,filename.split(".")[0])
    #endLatexDocument()

    #os.system("pdflatex -interaction nonstopmode -file-line-error .\output.tex")