# zed2_client
![image](img/zed_client2.png)
## Installation 

### Dependencies
  * [compressed_depth_image_transport](http://wiki.ros.org/compressed_depth_image_transport)
  * [zed_interfaces](https://github.com/stereolabs/zed-ros-wrapper/tree/master/zed_interfaces)
    ```
    cd catkin_ws/src
    https://github.com/stereolabs/zed-ros-wrapper.git
    catkin build zed_interfaces
    ```
* [octomap_server (my fork version)](https://github.com/icsl-Jeon/octomap_mapping) 

## Diagram 
![diagram](img/diagram.png)

## Launch 

```
roslaunch zed2_client client.launch is_bag:=true
```

