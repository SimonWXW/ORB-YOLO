# ORB-YOLO: An Indoor IMU-aided Visual-Inertial SLAM System for Dynamic Environment
[Paper](https://ieeexplore.ieee.org/document/10457405/authors#authors)

Authors: Xiwen Wu, Yuchen Miao and Zhuo Sun

## Acknowledgment
We greatly appreciate the opensource codebase of [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) and [YOLOv8](https://github.com/ultralytics/ultralytics).

## Introduction
Overall sturcture of ORB-YOLO
![orb-slam3 drawio](https://github.com/SimonWXW/ORB-YOLO/assets/106687022/241d0e88-969f-4c5a-8bfa-4c7bf7591caa)


## Requirements
We have tested the system in **Ubuntu 18.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

### Cmake 
Required at least 3.18. Tested with 3.26

### C++11 Compiler
We use the new thread and chrono functionalities of C++11.

### Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

### OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0**.

### Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

### DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

### Python
Required to calculate the alignment of the trajectory with the ground truth. **Required Numpy module**.

* (win) http://www.python.org/downloads/windows
* (deb) `sudo apt install libpython2.7-dev`
* (mac) preinstalled with osx

### Fastdeploy 
Required to deploy YOLOv8 model in c++ environment to detect dynamic object

You need to download the pre-compiled SDK and uncompress it in ThirdParty folder.
```shell
# Download the FastDeploy precompiled library. Users can choose your appropriate version in the `FastDeploy Precompiled Library` mentioned above
wget https://bj.bcebos.com/fastdeploy/release/cpp/fastdeploy-linux-x64-x.x.x.tgz
tar xvf fastdeploy-linux-x64-x.x.x.tgz

# 1. Download the official converted YOLOv8 ONNX model files
wget https://bj.bcebos.com/paddlehub/fastdeploy/yolov8s.onnx
```

### ROS (optional)

We provide some examples to process input of a monocular, monocular-inertial, stereo, stereo-inertial or RGB-D camera using ROS. Building these examples is optional. These have been tested with ROS Melodic under Ubuntu 18.04.

## Building system

Clone the repository:
```shell
git clone git@github.com:SimonWXW/ORB_SLAM3-with-YOLO.git
```

We provide a script `build.sh` to build the system efficiently:
```shell
cd ORB_SLAM3-with-YOLO
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM3.so**  at *lib* folder and the executables in *Examples* folder.

## Evaluation

### TUM RGB-D dataset

1. Download a sequence from [http://vision.in.tum.de/data/datasets/rgbd-dataset/download](http://vision.in.tum.de/data/datasets/rgbd-dataset/download) and uncompress it.
2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:
```shell
python associate.py rgbd_dataset_freiburg3_walking_xyz/rgb.txt rgbd_dataset_freiburg3_walking_xyz/depth.txt > associations.txt
```
```shell
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml ./dataset/rgbd_dataset_freiburg3_walking_xyz ./dataset/associations.txt cpu
```


In TUM-RGBD ground truth is only available in the room where all sequences start and end. As a result the error measures the drift at the end of the sequence. 

Execute the following script to process sequences and compute the RMS ATE:
```shell
evo_ape tum rgbd-tum-CameraTrajectory.txt ../dataset/rgbd_dataset_freiburg3_walking_xyz/groundtruth.txt -va -p --plot_mode xyz -a --correct_scale
```

### Real World Experiment
Tested with ROS Melodic and ubuntu 18.04.

1. Add the path including *Examples/ROS/ORB_SLAM3* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:
  ```shell
  gedit ~/.bashrc
  ```
and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:

  ```shell
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script:

  ```shell
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  

#### Running RGB-D Node
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM3/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
  rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

#### Running ROS example
Download a rosbag (e.g. V1_02_medium.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab for a Stereo-Inertial configuration:
  ```
  roscore
  rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml true
  rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
  ```
  
Once ORB-SLAM3 has loaded the vocabulary, press space in the rosbag tab.

**Remark:** For rosbags from TUM-VI dataset, some play issue may appear due to chunk size. One possible solution is to rebag them with the default chunk size, for example:
  ```
  rosrun rosbag fastrebag.py dataset-room1_512_16.bag dataset-room1_512_16_small_chunks.bag
  ```

## Results
![result](https://github.com/SimonWXW/ORB-YOLO/assets/106687022/b9413a0c-3f35-474b-ba6a-93750bc98178)

## Cite
```
@INPROCEEDINGS{10457405,
  author={Wu, Xiwen and Miao, Yuchen and Sun, Zhuo},
  booktitle={2023 International Conference on Artificial Intelligence of Things and Systems (AIoTSys)}, 
  title={ORB-YOLO: An Indoor IMU-aided Visual-Inertial SLAM System for Dynamic Environment}, 
  year={2023},
  volume={},
  number={},
  pages={71-78},
  keywords={Visualization;Simultaneous localization and mapping;Measurement units;Dynamics;Pose estimation;Lighting;Robustness;Visual-Inertial SLAM;Dynamic SLAM;Object Detection},
  doi={10.1109/AIoTSys58602.2023.00032}}
```
