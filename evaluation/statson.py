import evaluator
from parseconfig import parse_config

def statsOn(configName, tableName):
    datasets, datasets_names, slams, slams_names = parse_config()

    for i in range(0, len(datasets)):
        s = slams[0]
        name = slams_names[0]
        context = s[0](s[1], configName)

        print("Evaluating on " + datasets[i].name())

        context.run(datasets[i])

        if True:
            evaluation = evaluator.fromslam(context)
            ate = evaluation.ape_rmse()
            rpe = evaluation.rpe_rmse()

            print(ate)
        #except:
        #    print("Error : " + str(context.getError()))

if __name__ == "__main__":
    statsOn("modslam2.yaml", "modslam.csv")
