# HSC
version 2024-08-11

## What is HSC?
HSC is a new method for loop closure detection in overhead occlusion scenes.

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
│
├─src                                       // source code
│      HSC.cpp
│      main.cpp
│
│  CMakeLists.txt                            // cmake file
│  README.md
```

## Dependency
- PCL
- Eigen
- yaml-cpp

## How to use?
**1.Build the code**

```bash
mkdir build && cd build
cmake ..
make
```

**2.Modify the configuration file**

```yaml
Seq:
  Sequence: your_sequence  # Modify it to your sequence (KITTI00,KITTI02,KITTI05,KITTTI08)

Path:
  Raw_Path: "your_path" # Folder path for point cloud bin files ("/home/XXX/data/raw/")
  Result_Path: "your_path" # Path for result file ("/home/XXX/HSC/result/")
```

**3.Modify main.cpp**

```cpp
// Modify it to your path
std::string config_path = "/home/XXX/HSC/config/parameter.yaml";
```

**4.Run the code**

```bash
cd build
./hsc
```

## Data

The KITTI dataset can be downloaded at [KITTI](http:// 1099 www.cvlibs.net/datasets/kitti/raw_data.php). The NCLT dataset can be downloaded at [NCLT](http://robots.engin.umich.edu/nclt/). The JLU campus dataset can be downloaded at [JLU](https://www.kaggle.com/datasets/anacondaspyder/self-collected-dataset).

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