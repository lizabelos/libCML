import os
import math

import concurrent
import concurrent.futures

import evaluator
from table import FileTable
from utils import intrange, floatrange
from parseconfig import parse_config
from evaluator import evaluateOn


def ablationstudy2d():
    datasets, datasets_names, slams, slams_names = parse_config()

    pow10tmp = [math.pow(10,i) for i in range(1,12)]
    #pow10 = []
    #for i in range(0,len(pow10tmp)):
    #   for j in floatrange(1,10,0.5):
    #        pow10 = pow10 + [pow10tmp[i] * j]
    pow10 = pow10tmp

    if True:

        toTry = [
            "orbUncertaintyThreshold", pow10,
            "orbInlierRatioThreshold", floatrange(0.25,0.80,0.05)
        ]

       # toTry = [
        #    "dsoInitializer.densityFactor", floatrange(0.1,1.1,0.1),
        #    "dsoInitializer.regularizationWeight", floatrange(0.1,1.1,0.1)
     #   ]

        lendatasets = len(datasets)
        # lendatasets = 1

        table_ate = FileTable(sorted(toTry[1]), sorted(toTry[3]), "result/table.csv")
        print("result/table.csv", flush=True)

        def process(v):
            result = 0
            errorLimit = 0
            numError = 0
            for i in range(0, lendatasets):
                datasets[i].use()

                toTry0, v1, toTry2, v2 = v
                table_ate.set(v1, v2, "..." + str(result) + "...")

                context = None
                try:
                    s = slams[0]
                    name = slams_names[0]
                    context = s[0](s[1])

                    context.setconfig(v[0], v[1])
                    context.setconfig(v[2], v[3])

                    ate = evaluateOn(context, datasets[i])

                    if ate > datasets[i].lim():
                        result = "o" + str(result) + "o " + str(ate)
                        break

                    # result = result + ate
                    result = result + 1
                except:
                    result = "x" + str(result) + "x " + context.getError()
                    break

            table_ate.set(v1, v2, str(result))
            return v, result

        # Each SLAM instance will use 2 thread
        # executor = concurrent.futures.ThreadPoolExecutor(max_workers=multiprocessing.cpu_count() // 2)
        executor = concurrent.futures.ThreadPoolExecutor(max_workers=6)

        futures = []
        for v1 in toTry[1]:
            for v2 in toTry[3]:
                futures = futures + [executor.submit(process, [toTry[0], v1, toTry[2], v2])]

        concurrent.futures.wait(futures)



def build():
    os.makedirs("../build", exist_ok=True)
    os.system("cd .. && cd build && cmake -DENABLE_GUI=OFF -DCMAKE_BUILD_TYPE=Release .. && make ")

if __name__ == "__main__":
    ablationstudy2d()
