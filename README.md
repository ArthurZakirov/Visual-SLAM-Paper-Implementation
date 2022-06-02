# **Visual SLAM (ICP)**
<img src="img/icp.gif" width="500"/>

 This repository contains my implementations of ICP. Basic parts of the code such as data loading are taken from [3D AI Lab](https://www.3dunderstanding.org/index.html), however I implemented the algorithms myself.

### **Setup**
In order to run this project, you need to set up the following libraries with [cmake](https://cmake.org/).
- [ceres](http://ceres-solver.org/)
- [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [flann](https://github.com/flann-lib/flann)
- [glog](https://github.com/google/glog)


## **Usage**
This repo contains several implementations of ICP. You can set the following parameters in the ```main.cpp```. To apply the algorithm put your dataset inside the ```Data``` dir.

| parameter | meaning |
|-----------|---------|
| ```USE_POINT_TO_PLANE``` | 1-point to plane, 0-point to point |
| ```USE_LINEAR_ICP`` | 1-linearize and solve ATAx=ATb, 0-iterative solver |