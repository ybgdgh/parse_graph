# A Bottom-up Framework for Construction of Structured Semantic 3D Scene Graph

This work is based on our paper (IROS 2020, accepted). We proposed a new framework to construct 3D Structured Semantic Scene Graph. Our work is based on [YOLO (V3)](https://github.com/leggedrobotics/darknet_ros), [Apriltag](https://github.com/AprilRobotics/apriltag_ros) and [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2), implemented in ROS.

**Author:** Bangguo Yu, Chongyu Chen, Fengyu Zhou, Fang Wan, Wenmi Zhuang and Yang Zhao

**Affiliation:** Shandong University and DarkMatter AI Research

## 3D Structured Semantic Scene Graph Framework

The input of the architecture includes the sequence of RGB-D images and scene priors of an improved S-AOG structure. In the first step, we use yolov-v3 and apriltag to recognize object information. Limited by the recognition algorithm for big objects like desk or room, we adopted the apriltag to detect them. After object recognition, the 3D bounding box is estimated by the height and width of 2D bounding box and depth information roughly. Specifically, we use ORB-SLAM2 to estimate the position. The relation extraction module is designed to obtain the relations between objects using 3D position and 3D bounding box. The attribute extraction module is designed to focus on the 3D position, size and color. Then using scene priors, a parse graph *pg* can be calculated for each image by maximizing a posteriori estimation (MAP), which is the best explanation for extracted objects, attributes and relations. The output of MAP is added to keyframe group filter (KGF). KGF module is designed to remove repeated and fake detections. After KGF optimization, parse graph is added to local scene graphs and displayed in real-time. Globe scene graph can be obtained by updating and merging the local scene graphs

![image-20200706200822807](img/system.png)

## Requirements

- Ubuntu 16.04+
- Python 2.7
-  [YOLO (V3)](https://github.com/leggedrobotics/darknet_ros)
- [Apriltag](https://github.com/AprilRobotics/apriltag_ros)
- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
- Graphviz
- ROS kinetic

## Installation

The code has been tested only with Python 2.7, CUDA 9.0 on Ubuntu 16.04.

1. Build a catkin_workspace

   ```sh
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src/
   ```

2. Install  [YOLO (V3)](https://github.com/leggedrobotics/darknet_ros), [Apriltag](https://github.com/AprilRobotics/apriltag_ros) and [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) repository in this workspace.

3. Download and build 3D Structured Semantic Scene Graph repository:

   ```sh
   git clone https://github.com/ybgdgh/parse_graph.git
   catkin_init_workspace
   cd ..
   catkin_make -DCMAKE_BUILD_TYPE=Release
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Download dataset

The scene dataset: 

[rosbag](https://drive.google.com/file/d/1240c6YGeb1wuPbp_4qlhXmR6PRSa4pL4/view?usp=sharing)

The scene prior: 

[scene priors](https://drive.google.com/file/d/1bhT4rW0O4K1JIqagouDjuOtFaCrdwNVQ/view?usp=sharing)

## Result

<img src="img/scene.png" alt="image-20200706211403321" style="zoom:50%;" />!<img src="img/webot.png" alt="image-20200706211403321" style="zoom:67%;" />!

<img src="img/scene_graph.png" alt="scene_graph_update"  />

## Demo Video

[video](https://yubangguo.com/publication/conference-paper/)