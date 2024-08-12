# HSC
code for paper "HSC: a multi-hierarchy descriptor for loop closure detection in overhead occlusion scenes"

version 2024-08-12

## What is HSC?
HSC is a new method for loop closure detection in overhead occlusion scenes.

It's based on Scan Context. 

## Code structure
```
HSC
├─config                                    // Parameter settings
│      parameter.yaml
│
├─include                                   // Header files
│      HSC.h
│      KDTreeVectorOfVectorsAdaptor.h
│      nanoflann.hpp
│      tictoc.h
│
├─result                                    // folder for result
│      KITTI05
│
├─src                                       // source code
│      HSC.cpp
│      main.cpp
│
│  CMakeLists.txt                           // cmake file
│  README.md
```

## Dependency
- PCL
- Eigen
- yaml-cpp

## How to use?
**1.install the code**

```bash
git clone https://github.com/LMay001/HSC.git
cd HSC
```

**2.Modify main.cpp**

```c++
// Modify it to your path
std::string config_path = "/home/XXX/HSC/config/parameter.yaml";
```

**3.Build the code**

```bash
mkdir build && cd build
cmake ..
make
```

**4.Modify the configuration file**

```yaml
Seq:
  Sequence: your_sequence  # Modify it to your sequence (KITTI00,KITTI02,KITTI05,KITTTI08)

Path:
  Raw_Path: "your_path" # Folder path for point cloud bin files ("/home/XXX/data/raw/")
  Result_Path: "your_path" # Path for result file ("/home/XXX/HSC/result/")
```

**5.Run the code**

```bash
cd build
./hsc
```

## Data

The KITTI dataset can be downloaded at [KITTI](http://www.cvlibs.net/datasets/kitti/raw_data.php). The NCLT dataset can be downloaded at [NCLT](http://robots.engin.umich.edu/nclt/). The JLU campus dataset can be downloaded at [JLU](https://www.kaggle.com/datasets/anacondaspyder/self-collected-dataset).

## Paper

Thank you for citing [HSC](https://link.springer.com/article/10.1007/s40747-024-01581-2) if you use any of this code.

```
@article{lv2024hsc,
  title={HSC: a multi-hierarchy descriptor for loop closure detection in overhead occlusion scenes},
  author={Lv, Weilong and Zhou, Wei and Wang, Gang},
  journal={Complex \& Intelligent Systems},
  pages={1--25},
  year={2024},
  publisher={Springer}
}
```