import time
import shutil

if __name__ == "__main__":
    while True:
        shutil.copy("result/table.csv", "../documents/modslam/Ablation Study/winhome_supertable1.csv")
        time.sleep(60 * 5)