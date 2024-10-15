# contact sequence planning for legged robot using Parallel MCTS 
This open-source C++ code presents a novel contact planning method for multi-legged robots, demonstrated using a hexapod robot. The method leverages parallel Monte Carlo Tree Search (MCTS) to efficiently solve the discrete contact planning problem, incorporating multiple reachability constraints to ensure feasible and realistic motion.

## Pulications
If you use this work in an academic context, please cite the following publication:


    Xu, Peng, et al. "Contact Planning for Multi-Legged Robots under Constraints through Parallel MCTS." 

    Xu, Peng, et al. "Contact sequence planning for hexapod robots in sparse foothold environment based on Monte-Carlo tree." IEEE Robotics and Automation Letters 7.2 (2021): 826-833.

## Dependencies


### 1. ROS

### 2. Grid_map

FIXME: This method installs an old version of grid-map-sdf that seems not compitable

```bash
sudo apt-get install ros-$ROS_DISTRO-grid-map #DEPRECATED
```

Alternatively, choose only grid-map-core to avoid include conflict (grid-map-sdf)

```bash
sudo apt-get install ros-$ROS_DISTRO-grid-map-core ros-$ROS_DISTRO-grid-map-msgs
```

### 3. MPI

- Install from apt

```bash
sudo apt instal mpich
```

- Install from src
  - Download source code from https://www.mpich.org/downloads/
  - ./confiure
  - make & make install 安装

### 4. [GLPK](https://www.gnu.org/software/glpk/)

```bash
# decompress
cd <root-dir>
./configure
make
sudo make install
```

### 5. cddlib

[cddlib Homepage](https://people.inf.ethz.ch/fukudak/cdd_home/),
[Github](https://github.com/cddlib/cddlib)

```bash
tar zxf cddlib-*.tar.gz
cd cddlib-*
./configure
make
```

To install cddlib to /usr/local type

```bash
sudo make install
```

### 6. Eigen


### 7. gurobi 


## Quick Start
    change the path of config.txt at 'planFile/include/user.h'

## Command
    mpirun -n $processor number$ ./bin/main 


