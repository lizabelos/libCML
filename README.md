# libCML : SLAM library

# 1. About libCML

libCML is a SLAM library and scientific tool, which include a novel fast thread-safe graph map implementation. If you are using this work, please cite :
```bibtex
@article{libcml,
  title={libCML},
  author={Thomas Belos, Pascal Monasse, Eva Dokladalova, Thomas Daumain},
}
```

# 2. About MODSLAM

MODSLAM is a mixed ORB-SLAM2 and DSO SLAM system. If you are using this work, please cite : 
```bibtex
@article{modslam2021,
  title={MOD SLAM: Mixed Method for a More Robust SLAM Without LoopClosing},
  author={Thomas Belos, Pascal Monasse, Eva Dokladalova},
}
```

# 3. Compilation

## a. Windows, Linux and Mac

We recommend you to use CLion from Jetbrains to build and test.
Otherwise, you can :

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j 4
```

☕ Then, take a break, drink a cup of coffee. It can take hours time to build.

## b. Android and iPhone

We recommend you to use Qt Creator, as it is the most simple way to build ModSLAM on Android and iPhone.

# 4. Execution

```bash
cd build
./modslam -d PATH_TO_THE_DATASET
```
where `PATH_TO_THE_DATASET` can be any dataset (TUM, KITTI, RobotCar, a mkv/mp4/avi/other video file...).
MODSLAM will automatically detect the right dataset format.

# 5. Evaluation

```bash
python3 evaluation evaluation/config.txt
```

The configuration file contains the SLAMs and the datasets to evaluate on : 
```bash
ModSLAM "../cmake-build-release/modslam"
TUM "/home/thomas/Datasets/TUM"
KITTI "/home/thomas/Datasets/KITTI"
```

# 6. The author

BELOS Thomas, IMAGINE Team at ENPC, thomas.belos@enpc.fr
