# zed2_client
___

![image](img/zed_client2.png)

## Features 

![diagram](img/diagram.png)

* All parameters are described in [yaml file](param/default.yaml)

### 1. Receiving compressed images only and creating points in a client computer 
In an onboard system for ZED camera, 
it is common to use jetson (server of ZED) leveraging nvidia gpu, along with small computer (client) such as NUC having better cpu.
In this case, the Ethernet networking is not feasible for transmitting pointcloud in real time (less than 1Hz in case of 720HD).   
Instead, the compressed RGB and depth can be transported to NUC from jetson at about 10Hz (assuming png compression level 1 for depth image). 
Considering this network limit and computation capability of NUC, this package regenerates pointcloud in the NUC side. 
The decompression process is internalized without using [republish](https://wiki.ros.org/image_transport#republish) node for efficiency. 
> This package assumes that RGB is in jepg compression and depth in png (no openni depth mode). Please observe this when running zed_wrapper in the server side.  

### 2. Filtering and tracking objects of interest (single and dual target only supported)
Although ZED wrapper performs 1) [filtering](https://www.stereolabs.com/docs/ros/object-detection/), 
and efforts to 2) keep track of detected objects, this package aims to enhance these two more robustly. 
We first identify {location, dominant color} from object topic `/zed2/zed_node/obj_det/objects`. 
Then we [match](#matching-between-the-tracked-objects-and-newly-detected-objects) 
them with the tracked objects until now (a.k.a targets) using various [metrics](#matching-score) and [assumptions](#assumptions).
The new information of position and color will be collected with smoothing for reliability. 
The trace of tracked objects is visualized in the topics `observation_filtered/target_*` with color encoding as the [left figure](#zed2_client).

### 3. Removing pointcloud related with target objects 
In many robotic applications, it might be better to wipe out the pointcloud of the targets 
(e.g. case: the target should not be considered as occupied region for planning purpose). 
To accomplish this, three steps were implemented: 1) 2d-masking in image space,2) 3d-masking 
using bounding box and extruding along z-axis, and 3) [speckle removal](https://pointclouds.org/documentation/tutorials/remove_outliers.html).
Currently, the total process runs at about 10Hz for 1280 x 720 camera setting in i7 computer with 32GB RAM. 
> You can turn off this function and remove the necessity of object topic by setting `mask_object` false. 


## Getting started

### 1. Dependencies
#### [compressed_depth_image_transport](http://wiki.ros.org/compressed_depth_image_transport)
```
 sudo apt-get install ros-${ROS_DISTRO}-compressed-depth-image-transport
```
    
#### [zed_interfaces](https://github.com/stereolabs/zed-ros-wrapper/tree/master/zed_interfaces)
```
cd catkin_ws/src
git clone https://github.com/stereolabs/zed-ros-wrapper.git
catkin build zed_interfaces 
catkin build zed_wrapper
```

#### [dynamicEDT3d (my fork version)](https://github.com/icsl-Jeon/octomap)
```
git clone https://github.com/icsl-Jeon/octomap
cd dynamicEDT3D
mkdir build && cmake .. 
sudo make install
```

#### [octomap_server (my fork version)](https://github.com/icsl-Jeon/octomap_mapping) 
```
cd catkin_ws/src
git clone https://github.com/icsl-Jeon/octomap_mapping
catkin build octomap_server
```
#### This package 
```
cd catkin_ws/src
git clone /github.com/icsl-Jeon/zed2_client.git
catkin build zed2_client
```


### 2. Launch 

```
roslaunch zed2_client client.launch is_bag:=true
```
## Matching between the tracked objects and newly detected objects (TBD)

### 1. Matching score

#### Position 

#### Velocity

#### HSV color difference 


### 2. Assumptions 

* **Accept** : if there is o newly observed object with same zed labeling, match it.
* **Accept** : two objects with hsv color difference smaller than `target_detection/assumption/color_accept` and `target_detection/assumption/dist_accept`
  will have an advantage by having only the velocity cost.
* **Reject** : two objects having distance difference larger than `target_detection/assumption/dist_reject`.
* **Reject** : two objects having color difference larger than  `target_detection/assumption/color_reject`. 


