libCML and MODSLAM
===========================

# 1. MOD SLAM: Mixed Method for a More Robust SLAM Without Loop Closing

**Paper link** : [https://hal.archives-ouvertes.fr/hal-03538133](https://hal.archives-ouvertes.fr/hal-03538133)

In recent years, the state-of-the-art  of monocular SLAM has seen remarkable advances in reducing errors and improving robustness. At the same time, this quality of results can be obtained in real-time on small CPUs.
However, most algorithms have a high failure rate out-of-the-box. Systematic error such as drift remains still significant even for the best algorithms. This can be handled by a global measure as a loop closure, but it penalizes online data processing. We propose a mixed SLAM, based on ORB-SLAM2 and DSO: MOD SLAM. It is a fusion of photometric and feature-based methods, without being a simple copy of both. We propose a decision system to predict at each frame which optimization will produce the minimum drift so that only one will be selected to save computational time and resources. We propose a new implementation of the map that is equipped with the ability to actively work with DSO and ORB points at the same time. Our experimental results show that this method increases the overall robustness and reduces the drift without compromising the computational resources. Contrary to the best state-of-the-art algorithms, MOD SLAM can handle 100\% of KITTI, TUM, and random phone videos, without any configuration change.

If you are using this work, please cite : 
```bibtex
@inproceedings{modslam2021,
  title={MOD SLAM: Mixed Method for a More Robust SLAM without Loop Closing},
  author={Thomas Belos, Pascal Monasse, and Eva Dokladalova},
  booktitle={In Proceedings of the 17th International Joint Conference on Computer Vision, Imaging and Computer Graphics Theory and Applications (VISIGRAPP 2022)}
}
```

## Downloads

**Information** : Hyper-threading seems to have a very high impact on the performance. Disable it on the BIOS via : 
```bash
echo off > /sys/devices/system/cpu/smt/control
```

### Windows

https://github.com/belosthomas/libCML/releases

### Linux

Build Coming soon...

### Android

https://github.com/belosthomas/libCML/releases

<img src="https://github.com/belosthomas/libCML/raw/main/documentation/images/screen_recording.gif" width="600" alt=""/>

**Known issues on Android:**
- A lot of crashes
- Not internal calibration is used
- Based on old settings

# 2. About libCML

libCML is a SLAM library and scientific tool, which include a novel fast thread-safe graph map implementation. If you are using this work, please cite :
```bibtex
@misc{libcml,
  title={libCML : SLAM library},
  author={Thomas Belos, Pascal Monasse, Eva Dokladalova},
  url={https://github.com/belosthomas/libCML}
}
```

# 3. Compilation

**Minimum Ubuntu Version :** 20

**Minimum Fedora Version :** 34

## a. Windows, Linux and Mac

If you want to build the GUI, you will need Qt6 with Qt Multimedia and Qt Charts. Otherwise, please disable it with the according option (see below).

We recommend you to use CLion from Jetbrains, or Qt Creator from Nokia, to build and test.
Otherwise, you can :

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j 4
```

☕ Then, take a break, drink a cup of coffee. It can take hours time to build.

This is a list of different option you can pass to cmake : 
```bash
-DENABLE_GUI=OFF # Disable/Enable the gui
-DUSE_GOOGLE_HASH=ON # Disable/Enable Google Hash Map. Enabling this option cause the map to take more memory, but to be faster.
-DENABLE_OPENMP=OFF # Disable/Enable OpenMP
```

## b. Android and iPhone

We recommend you to use Qt Creator, as it is the most simple way to build ModSLAM on Android and iPhone.

1. Download and Install Android Studio
2. Download and Install JDK 17 (Higher version not supporter for the moment)
3. Download and Install Qt (with QtCreator, Qt 6 for Android, Qt Multimedia and Qt Charts)
4. Launch Qt Creator. Go to Android settings. Make sure that all is ok. Download the Android SDK you want.
5. Open the project with Qt Creator, configure it for Android/Release and simply build it

# 4. Execution

```bash
cd build
./modslam -d PATH_TO_THE_DATASET
```
where `PATH_TO_THE_DATASET` can be any dataset (TUM, KITTI, RobotCar, a mkv/mp4/avi/other video file...).
MODSLAM will automatically detect the right dataset format.

Here is a list of arguments you can pass to the executable : 
```bash
-c configfile.yaml # Load a specific config file
-g # Use Gui Mode
-t # Use Terminal Mode
-r path_to_result -f [tum|kitti|all] # Save the results with the specified format
-l log.txt # Write the log to a file
-s path_to_images # Export the 3D rendering to a ton of .jpeg files at specified folder
-z # Print the statistics at each iteration. Used for python script and result analasys.
```

# 5. Datasets

## KITTI Odometry

Download here : https://www.cvlibs.net/datasets/kitti/eval_odometry.php
 - Gray Scale Required
 - Color Optional
 - Groundtruth Optional

```
/KITTI
├── dataset
│   ├── poses
│   │   ├── 00.txt
│   │   ├── 01.txt
│   │   ├── 02.txt
│   │   ├── 03.txt
│   │   ├── 04.txt
│   │   ├── 05.txt
│   │   ├── 06.txt
│   │   ├── 07.txt
│   │   ├── 08.txt
│   │   ├── 09.txt
│   │   └── 10.txt
│   └── sequences
│       ├── 00
│       ├── 01
│       ├── 02
│       ├── 03
│       ├── 04
│       ├── 05
│       ├── 06
│       ├── 07
│       ├── 08
│       ├── 09
│       ├── 10
```

```bash
cd build
./modslam -d /KITTI/dataset/sequences/02
```


## Stereopolis

```
/Stereopolis
├── Camera_0.zip
├── Camera_0.gt.txt
├── Camera_0.mask.png
├── Camera_0.times.txt
├── Camera_0.xml
```

```bash
cd build
./modslam -d /Stereopolis/Camera_0.zip
```

# 5. Evaluation

```bash
cd result
python3 statson.py # Compute the absolute trajectory error of each videos
python3 bruteforce.py # Make statistics for the parameters / Search for a better parameters set
python3 make2dtable.py # Make a 2D table of the variation of ATE between by varying two parameters
```

The configuration file `evaluation/config.txt` contains the SLAMs and the datasets to evaluate on : 
```bash
ModSLAM "../cmake-build-release/modslam"
TUM "/home/thomas/Datasets/TUM"
KITTI "/home/thomas/Datasets/KITTI"
```

The configuration file `evaluation/modslam.yaml` is the SLAM configuration you want to initialize the python script with.

Each result is stored in a `.json` file. Experience are never run twice. If a the result of an experience is already present in the `.json` file, the experience will not run a second time.
You can plot all the result stored in the json database : 
```bash
python3 plotall.py
```

All the result are stored in a `result` folder. This is the kind of result you can except with a few experiences :

<img src="https://github.com/belosthomas/libCML/raw/main/documentation/images/plot.png" width="600" alt=""/>

# Known bugs

## The camera won't open on Android

This is a bug of Qt. Please use at least Qt 6.3.0.

## MSVC / CLang, LLVM Out of memory

This can be cause when you use the 32 bits of the compiler instead of the 64.
Please read https://cmake.org/cmake/help/git-stage/guide/user-interaction/index.html for more details.
