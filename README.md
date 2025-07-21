# [RA-L 24] 3D Active Metric-Semantic SLAM
Authors: Yuezhan Tao*, Xu Liu*, Igor Spasojevic, Saurav Agarwal, Vijay Kumar

Project Page: https://tyuezhan.github.io/3D_Active_MS_SLAM/

Full video: [Youtube](https://www.youtube.com/watch?v=Kb3s3IJ-wNg) 



Please consider to cite our paper if you use this project in your research:
- [__3D active metric-semantic slam__](https://ieeexplore.ieee.org/abstract/document/10423804), Yuezhan Tao*, Xu Liu*, Igor Spasojevic, Saurav Agarwal, Vijay Kumar.

```
@ARTICLE{10423804,
  author={Tao, Yuezhan and Liu, Xu and Spasojevic, Igor and Agarwal, Saurav and Kumar, Vijay},
  journal={IEEE Robotics and Automation Letters}, 
  title={3D Active Metric-Semantic SLAM}, 
  year={2024},
  volume={9},
  number={3},
  pages={2989-2996},
  keywords={Semantics;Simultaneous localization and mapping;Three-dimensional displays;Uncertainty;Planning;Autonomous aerial vehicles;Real-time systems;Aerial systems: Perception and autonomy;mapping;perception-action coupling},
  doi={10.1109/LRA.2024.3363542}}
```


## Table of Contents

- [\[RA-L 24\] 3D Active Metric-Semantic SLAM](#ra-l-24-3d-active-metric-semantic-slam)
  - [Table of Contents](#table-of-contents)
  - [Quick Start](#quick-start)
    - [Dependencies:](#dependencies)
    - [Simulation demo:](#simulation-demo)
  - [Exploring Different Environments](#exploring-different-environments)
  - [Known issues](#known-issues)
    - [Compilation issue](#compilation-issue)
  - [Acknowledgement](#acknowledgement)

## Quick Start

This project has been tested on Ubuntu 20.04(ROS Noetic). This work is **primarily developed based on real-world experiments**. But we also provide a minimal simulation setup (with odometry perturbation in positions) for quick tests.

### Dependencies:
1. Catkin tools: [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
2. Install the following through apt:
    ```
    apt install tmux libompl-dev cpufrequtils libgoogle-glog-dev libgflags-dev
    ```
3. Pytorch 1.12 (tested on Quadrotor platform without GPU): 
    ```
    pip install torch==1.12.0+cpu torchvision==0.13.0+cpu torchaudio==0.12.0 --extra-index-url https://download.pytorch.org/whl/cpu
    ```
4. GTSAM 4.0.3: 
    ```
    add-apt-repository ppa:borglab/gtsam-release-4.0
    apt update
    apt install libgtsam-dev libgtsam-unstable-dev
    ```
5. glog (required by [SLOAM](https://github.com/KumarRobotics/sloam)):
    ```
    git clone https://github.com/google/glog.git && \
    cd glog && git checkout tags/v0.6.0 && mkdir build && cd build && \
    cmake .. && make && make install
    ```
6. Ceres (required by [SLOAM](https://github.com/KumarRobotics/sloam)):
    ```
    git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && git checkout 206061a6ba02dc91286b18da48825f7a9ef561f0 && \
    mkdir build && cd build && cmake .. && make && make install
    ```
7. Sophus (required by [SLOAM](https://github.com/KumarRobotics/sloam))
    ```
    git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && git checkout 49a7e1286910019f74fb4f0bb3e213c909f8e1b7 && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && make
    make install
    ```
8. fmt (required by [SLOAM](https://github.com/KumarRobotics/sloam))
    ```
    git clone https://github.com/fmtlib/fmt.git && \
    cd fmt && git checkout 8.0.0 && \
    mkdir build && cd build && \
    cmake .. && make  
    make install
    ```
9.  Python dependencies:
    ```
    pip3 install scikit-learn open3d ultralytics==8.0.38 rosnumpy numpy==1.23.1
    ```

### Simulation demo:
1. Build the workspace
    ```
    cd ${YOUR_WORKSPACE_PATH} && \
    catkin init && \
    catkin config -DCMAKE_BUILD_TYPE=Release && \
    catkin build
    ```

2. Run the simulation demo:
    ```
    source devel/setup.bash && \
    roscd exploration_manager/scripts && \
    ./tmux_sim_slam.sh
    ```

    If there are errors about not being able to find ros packages, you may need to source the workspace in `~/.bashrc`. 

3. Take off and Trigger exploration:

   By default you can see an small room environment. Firstly, click `Motors On` and `Take Off` from the mav manager GUI. Then, Trigger the quadrotor to start exploration by the ```2D Nav Goal``` tool in ```Rviz```.

## Exploring Different Environments
Note: This work is **primarily developed based on real-world experiments**. But we also provide a minimal simulation setup (with odometry perturbation in positions) for quick tests.
The exploration environment in our simulator is Gazebo world and is stored under `exploration_manager/worlds/`.


## Known issues

### Compilation issue

When running this project on Ubuntu 20.04, C++14 is required. Please add the following line in all CMakelists.txt files:

```
set(CMAKE_CXX_STANDARD 14)
```

## Acknowledgement

We gratefully acknowledge Alex Zhou, Fernando Cladera for their help with the hardware platforms, Yuwei Wu and Yifei Shao for their insights in trajectory planning.

We would also like to acknowledge the authors of [SLOAM](https://github.com/KumarRobotics/sloam), [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL), [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) for open-sourcing their code.
