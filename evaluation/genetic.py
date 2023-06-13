import copy
import math
import random
import re
from math import sqrt
from os import path
from threading import Thread, Lock
from concurrent.futures import ThreadPoolExecutor, wait, ALL_COMPLETED

from evaluator import evaluateOn
from parseconfig import parse_config

THREAD_POOL = ThreadPoolExecutor(max_workers=10)
NUM_GENERATIONS = 200
ADDITIONAL_PARAMETERS = [

]
BATCH_SIZE = 3
DURATION_LIMIT = 1.25
START_LEARNING_RATE = 1
USE_DT = True

def toFloatOrNone(s):
    if s == "None":
        return None
    return float(s)

def randomFloatBetween(a, b):
    return random.uniform(a, b)


def joinWeightToStr(weights, separatorA=";", separatorB="|"):
    weights = [[weights[i], weights[i + 1]] for i in range(0, len(weights), 2)]
    return separatorA.join([separatorB.join([str(x) for x in w]) for w in weights])

def joinWeightToStr1(weights, separatorA=";"):
    return separatorA.join([str(w) for w in weights])

class GenetricSlam:
    def __init__(self, weights, datasets, slam, num_mutation=0, num_crossover=0, num_average=0):
        self.weights = weights
        self.datasets = datasets
        self.slam = slam
        slam_test = self.slam()
        self.ate = [None for _ in range(len(self.datasets))]
        self.original_ate = [None for _ in range(len(self.datasets))]
        self.num_mutation = num_mutation
        self.num_crossover = num_crossover
        self.num_average = num_average
        self.futures = []

    def internal_run(self, dataset_ids):
        for dataset_id in dataset_ids:
            if self.ate[dataset_id] is not None:
                continue
            dataset = self.datasets[dataset_id]
            context = self.slam("../build/modslam")
            start = 0
            if not USE_DT:
                paWeights = joinWeightToStr(self.weights[0:(7 * 7 + 7)*2])
                baWeights = joinWeightToStr(self.weights[(7 * 7 + 7)*2:(7 * 7 + 7 + 5 * 5 + 5)*2])
                context.setconfig("peDecisionWeightsFc", paWeights)
                context.setconfig("baDecisionWeightsFc", baWeights)
                start = (7 * 7 + 7 + 5 * 5 + 5) * 2
            else:
                paWeights = joinWeightToStr1(self.weights[0:12])
                baWeights = joinWeightToStr1(self.weights[12:12+14])
                context.setconfig("peDecisionWeightsDt", paWeights)
                context.setconfig("baDecisionWeightsDt", baWeights)
                start = 12 + 14
            context.setconfig("orb.useCache", "true")
            for i in range(len(ADDITIONAL_PARAMETERS)):
                p = (self.weights[start + i] + START_LEARNING_RATE) / (START_LEARNING_RATE * 2) * (ADDITIONAL_PARAMETERS[i][1][1] - ADDITIONAL_PARAMETERS[i][1][0]) + ADDITIONAL_PARAMETERS[i][1][0]
                p = ADDITIONAL_PARAMETERS[i][2](p)
                context.setconfig(ADDITIONAL_PARAMETERS[i][0], p)
            try:
                self.original_ate[dataset_id] = evaluateOn(context, dataset, time_limit=dataset.duration * DURATION_LIMIT)
                self.ate[dataset_id] = self.original_ate[dataset_id] / sqrt(dataset.duration)
            except KeyboardInterrupt:
                pass
            except Exception as e:
                self.ate[dataset_id] = 10000
                self.original_ate[dataset_id] = 10000

    def run(self, dataset_ids):
        global THREAD_POOL
        f = THREAD_POOL.submit(self.internal_run, dataset_ids)
        self.futures.append(f)

    def wait(self):
        wait(self.futures, return_when=ALL_COMPLETED)
        self.futures = []

    def ateOf(self, dataset_id):
        return self.ate[dataset_id]

    def originalAteOf(self, dataset_id):
        return self.original_ate[dataset_id]

    def makeMutant(self, learning_rate):
        mutant = self.weights.copy()
        for i in range(len(self.weights)):
            if random.random() < 0.5:
                mutant[i] += randomFloatBetween(-learning_rate, learning_rate)
        self.mutant_ate = [None for _ in range(len(self.datasets))]
        return GenetricSlam(mutant, self.datasets, self.slam, num_mutation=self.num_mutation+1, num_crossover=self.num_crossover, num_average=self.num_average)

    def makeCrossover(self, other, other_weight=0.5):
        crossover = self.weights.copy()
        for i in range(len(self.weights)):
            if random.random() < other_weight:
                crossover[i] = other.weights[i]
        return GenetricSlam(crossover, self.datasets, self.slam, num_mutation=self.num_mutation, num_crossover=self.num_crossover+1, num_average=self.num_average)

    def makeAverage(self, other, other_weight=0.5):
        average = self.weights.copy()
        for i in range(len(self.weights)):
            average[i] = (self.weights[i] * (1 - other_weight) + other.weights[i] * other_weight)
        return GenetricSlam(average, self.datasets, self.slam, num_mutation=self.num_mutation, num_crossover=self.num_crossover, num_average=self.num_average+1)

    def serialize(self):
        # serialize self.weights and ates
        serializedWeights = ";".join([str(i) for i in self.weights])
        serializedAte = ";".join([str(i) for i in self.ate])
        serializedOriginalAte = ";".join([str(i) for i in self.original_ate])
        return serializedWeights + "|" + serializedAte + "|" + serializedOriginalAte

    def deserialize(self, serialized):
        # deserialize self.weights and ates
        serializedWeights, serializedAte, serializedOriginalAte = serialized.split("|")
        self.weights = [toFloatOrNone(i) for i in serializedWeights.split(";")]
        self.ate = [toFloatOrNone(i) for i in serializedAte.split(";")]
        self.original_ate = [toFloatOrNone(i) for i in serializedOriginalAte.split(";")]

    def isEqual(self, other):
        return self.weights == other.weights
        if self.weights == other.weights:
            return True
        isEqual = True
        for i in range(len(self.ate)):
            if self.ate[i] is None:
                if other.ate[i] is None:
                    continue
                else:
                    isEqual = False
                    break
            if other.ate[i] is None:
                isEqual = False
                break
            if math.isclose(self.ate[i], other.ate[i]):
                continue
        if not isEqual:
            return False
        # Compute weight l2 distance
        weight_distance = 0
        for i in range(len(self.weights)):
            weight_distance += (self.weights[i] - other.weights[i]) ** 2
        weight_distance = math.sqrt(weight_distance)
        return weight_distance < START_LEARNING_RATE * 0.1

    def weightDistance(self, other):
        weight_distance = 0
        for i in range(len(self.weights)):
            weight_distance += (self.weights[i] - other.weights[i]) ** 2
        weight_distance = math.sqrt(weight_distance)
        return weight_distance

    def isBetterAte(self, other):
        return sum(self.ate) < sum(other.ate)

def minWithNone(list):
    return min([x for x in list if x is not None])

def randN(n, min, max):
    l = set()
    while len(l) < n:
        l.add(random.randint(min, max - 1))
    return list(l)

class GeneticTrainer:
    def __init__(self, num_weights, datasets, slam):
        self.datasets = datasets
        self.weights_size = num_weights
        self.weightsList = []
        self.slam = slam
        self.learning_rate = START_LEARNING_RATE
        for i in range(NUM_GENERATIONS):
            self.weightsList.append(
                GenetricSlam([randomFloatBetween(-self.learning_rate, self.learning_rate) for _ in range(num_weights)], datasets, self.slam))
        self.bestCriteria = []

    def computeBestAte(self, dataset_id):
        best = None
        for i in range(0, len(self.weightsList)):
            if self.weightsList[i].ateOf(dataset_id) is None:
                continue
            if best is None and self.weightsList[i].ateOf(dataset_id) is not None:
                best = self.weightsList[i]
                continue
            if best.ateOf(dataset_id) > self.weightsList[i].ateOf(dataset_id):
                best = self.weightsList[i]
                continue
        return best

    def computeBestAtes(self):
        bestAtes = [None for _ in range(len(self.datasets))]
        for i in range(len(self.datasets)):
            best = self.computeBestAte(i)
            if best is not None:
                bestAtes[i] = best.ateOf(i)

        return bestAtes

    def criteria(self, geneticSlam, bestAtes, dataset_ids, add_best = True):
        num = 0
        for i in dataset_ids:
            if geneticSlam.ateOf(i) is None:
                continue
            if bestAtes[i] is None:
                continue
            if geneticSlam.ateOf(i) >= self.datasets[dataset_ids[i]].lim() * 1.5:
                continue
            num += 10
            if geneticSlam.ateOf(i) <= bestAtes[i] * 1.1 and add_best:
                num += 1
        for i in range(len(self.datasets)):
            if geneticSlam.ateOf(i) is None:
                continue
            if geneticSlam.ateOf(i) < 10000:
                num += 0.001 / (geneticSlam.ateOf(i) + 0.001)
        return num

    def sortWeights(self, dataset_ids):
        bestAtes = self.computeBestAtes()
        self.weightsList.sort(key=lambda x: self.criteria(x, bestAtes, dataset_ids), reverse=True)
        self.bestCriteria.append(self.criteria(self.weightsList[0], bestAtes, dataset_ids, add_best=False))

    def hasImproved(self):
        if len(self.bestCriteria) < 2:
            return True
        return self.bestCriteria[-1] > self.bestCriteria[-2]

    def run(self, dataset_ids):
        if len(self.weightsList) > NUM_GENERATIONS:
            raise Exception("Too many generations")
        dataset_names = " ".join([self.datasets[i].name() for i in dataset_ids])
        print("Running on " + dataset_names + "...", end="", flush=True)
        for i in range(len(self.weightsList)):
            for dataset_id in dataset_ids:
                self.weightsList[i].run([dataset_id])
            #self.weightsList[i].run(dataset_ids)
        for i in range(len(self.weightsList)):
            self.weightsList[i].wait()
            print("\rRunning on " + dataset_names + "..." + str(i + 1) + "/" + str(len(self.weightsList)), end="", flush=True)
        print("", flush=True)

    def intelligentCross(self, dataset_ids, nit=10, mit=3, oit=2):
        associations = []
        for i in range(0, len(self.weightsList)):
            for j in range(i + 1, len(self.weightsList)):
                sum = 0
                i_weight = 0
                j_weight = 0
                for k in dataset_ids:
                    m = minWithNone([self.weightsList[i].ate[k], self.weightsList[j].ate[k], 10000])
                    sum += m
                    if self.weightsList[i].ate[k] == m:
                        i_weight += 1
                    if self.weightsList[j].ate[k] == m:
                        j_weight += 1
                associations.append([sum, i, j, i_weight, j_weight])
        associations.sort(key=lambda x: x[0])

        newGeneration = []
        for i in range(mit):
            for j in range(nit):
                a = associations[j][1]
                b = associations[j][2]
                a_weight = associations[j][3]
                b_weight = associations[j][4]
                b_weight = b_weight / (a_weight + b_weight)
                b_weight = (b_weight + 0.5) / 2
                newGeneration.append(self.weightsList[a].makeCrossover(self.weightsList[b], b_weight))

        for i in range(oit):
            for j in range(nit):
                a = associations[j][1]
                b = associations[j][2]
                a_weight = associations[j][3]
                b_weight = associations[j][4]
                b_weight = b_weight / (a_weight + b_weight)
                b_weight = (b_weight + 0.5) / 2
                newGeneration.append(self.weightsList[a].makeAverage(self.weightsList[b], b_weight))

        return newGeneration

    def makeNewGeneration(self, dataset_ids, do_intelligent_cross = False):
        if do_intelligent_cross:
            newGeneration = self.intelligentCross(dataset_ids) # 50
            if len(newGeneration) > 50:
                raise Exception("Too many generations during intelligentCross")
            for i in range(int(NUM_GENERATIONS * 0.1)): # 60
                newGeneration.append(self.weightsList[i])
            for n in range(3): # 90
                for i in range(int(NUM_GENERATIONS * 0.1)):
                    newGeneration.append(self.weightsList[i].makeMutant(self.learning_rate))
            if len(newGeneration) > NUM_GENERATIONS:
                raise Exception("Too many generations during makeNewGeneration")
            while len(newGeneration) < NUM_GENERATIONS:
                newGeneration.append(GenetricSlam([randomFloatBetween(-self.learning_rate, self.learning_rate) for _ in range(self.weights_size)], datasets, self.slam))
            self.oldWeightsList = self.weightsList
            self.weightsList = newGeneration
        else:
            newGeneration = []
            for i in range(int(NUM_GENERATIONS * 0.1)):
                newGeneration.append(self.weightsList[i])
            for n in range(3):
                for i in range(int(NUM_GENERATIONS * 0.13)):
                    newGeneration.append(self.weightsList[i].makeMutant(self.learning_rate))
            for n in range(3):
                for i in range(int(NUM_GENERATIONS * 0.13)):
                    j = random.randint(0, int(NUM_GENERATIONS * 0.20))
                    newGeneration.append(self.weightsList[i].makeCrossover(self.weightsList[j]))
            if len(newGeneration) > NUM_GENERATIONS:
                raise Exception("Too many generations during makeNewGeneration")
            while len(newGeneration) < NUM_GENERATIONS:
                newGeneration.append(GenetricSlam([randomFloatBetween(-self.learning_rate, self.learning_rate) for _ in range(self.weights_size)], datasets, self.slam))
            self.oldWeightsList = self.weightsList
            self.weightsList = newGeneration

    def removeSameWeights(self):
        cleaned = []
        for i in range(len(self.weightsList)):
            for j in range(i + 1, len(self.weightsList)):
                if self.weightsList[i].isEqual(self.weightsList[j]):
                    break
            cleaned.append(self.weightsList[i])
        while len(cleaned) < NUM_GENERATIONS:
            cleaned.append(GenetricSlam([randomFloatBetween(-self.learning_rate, self.learning_rate) for _ in range(self.weights_size)], datasets, self.slam))
        self.weightsList = cleaned

    def printBests(self, dataset_ids):
        for j in range(5):
            toPrint = ""
            for k in dataset_ids:
                ate = self.weightsList[j].originalAteOf(k)
                if ate is None:
                    toPrint += "N/A "
                else:
                    toPrint += "%.1f " % (self.weightsList[j].originalAteOf(k))
            print(toPrint)
            print("mutation: " + str(self.weightsList[j].num_mutation) + "; crossover: " + str(self.weightsList[j].num_crossover) + "; average: " + str(self.weightsList[j].num_average))


    def train(self):
        dataset_ids = [i for i in range(len(self.datasets))]
        self.printBests(dataset_ids)
        while True:
            print("Learning rate: " + str(self.learning_rate))
            # dataset_ids = randN(BATCH_SIZE, 0, len(self.datasets))
            self.removeSameWeights()
            self.run(dataset_ids)
            self.sortWeights(dataset_ids)
            print("Best: " + str(self.bestCriteria[-1]))
            if not self.hasImproved():
                self.learning_rate *= 0.9
            self.save("genetic_slam.txt")
            self.printBests(dataset_ids)
            self.makeNewGeneration(dataset_ids)

    def serialize(self):
        serialized = ""
        for i in range(len(self.weightsList)):
            serialized += self.weightsList[i].serialize() + "\n"
        serialized += str(self.learning_rate)
        return serialized

    def deserialize(self, serialized):
        lines = serialized.split("\n")
        for i in range(len(self.weightsList)):
            self.weightsList[i].deserialize(lines[i])
        self.learning_rate = float(lines[-1])

    def save(self, filename):
        with open(filename, "w") as f:
            f.write(self.serialize())

    def load(self, filename):
        with open(filename, "r") as f:
            self.deserialize(f.read())


class Interpolator:

    def __init__(self):
        self.weights = []

    def add(self, weight):
        self.weights.append(weight)

    def computeAllNearest(self):
        allDistances = []
        localMinimumRange = []
        for i in range(len(self.weights)):
            distances = []
            for j in range(len(self.weights)):
                if i != j:
                    distances.append([j, self.weights[i].weightDistance(self.weights[j])])
            distances.sort(key=lambda x: x[1])
            minimumRange = None
            oldI = i
            for distance in distances:
                if not self.weights[oldI].isBetterThan(self.weights[distance[0]]):
                    minimumRange = distance[1]
                    break
                oldI = distance[0]
            if minimumRange is None:
                print("Their is only a global minimum")
                minimumRange = 99999
            localMinimumRange.append([i, minimumRange])
            allDistances.append(distances)
        self.allDistances = allDistances
        self.localMinimumRange = localMinimumRange
        self.localMinimumRange.sort(key=lambda x: x[1], reverse=True)
        # Keep only the best ones
        self.weights = [self.weights[i[0]] for i in self.localMinimumRange[:NUM_GENERATIONS]]



if __name__ == "__main__":
    datasets, datasets_names, slams, slams_names = parse_config()
    datasets = datasets[0:20]
    datasets.sort(key=lambda x: x.duration)
    trainerSize = (7 * 7 + 7 + 5 * 5 + 5) * 2 + len(ADDITIONAL_PARAMETERS)
    if USE_DT:
        trainerSize = 12 + 14 + len(ADDITIONAL_PARAMETERS)
    trainer = GeneticTrainer(trainerSize, datasets, slams[0][0])
    try:
        if path.exists("genetic_slam.txt"):
            trainer.load("genetic_slam.txt")
        trainer.train()
    except KeyboardInterrupt:
        trainer.save("genetic_slam.txt")
