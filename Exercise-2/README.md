# ROS perception pipeline
In this exercise you will start building up your perception pipeline in ROS.

Here, you are provided a gazebo world which contains a table with some random objects on top of it.

A simple stick robot with RGBD camera attached to its head via a pan-tilt joint is placed in front of the table.

First of all copy/move the `sensor_stick` package to /src directory of your active ros workspace. 

Next make sure you have all the dependencies resolved by using the rosdep install tool:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

Now, to start building your perception pipeline, you must perform following steps:

1. Test the simulation setup by launching the gazebo environment. The command stated below will open a gazebo world along with an rviz window. Play around with it and get familiar with different topics being published to.
```sh
$ roslaunch sensor_stick robot_spawn.launch
```
![sensor_stick](https://user-images.githubusercontent.com/9555001/27804170-51f5da76-5fe2-11e7-845c-130fcfd8cf3e.png)

2. Create a python ros node that subscribes to `/sensor_stick/point_cloud` topic. Use the `template.py` file found under /sensor_stick/scripts/ to get started.

3. Use your code from exercise-1 to apply various filters and segment the table in real-time

4. Create publishers and topics to publish the segmented table and tabletop objects as separate point clouds 

![inliers](https://user-images.githubusercontent.com/9555001/27804173-5693dcea-5fe2-11e7-828a-4113edd8649a.png)

![outliers](https://user-images.githubusercontent.com/9555001/27804174-59807db4-5fe2-11e7-9bd8-99a28ce1a737.png)

5. Apply euclidean clustering on the table-top objects (after table segmentation is successful)

6. Create a XYZRGB point cloud such that each cluster obtained from the previous step has its own unique color.

7. Finally publish your colored cluster cloud on a separate topic 
![clusters](https://user-images.githubusercontent.com/9555001/27804180-604d6e04-5fe2-11e7-9f33-d8d8da9a8bc0.png)

You will find `pcl_helper.py` file under `/sensor_stick/scripts`. This file contains various functions to help you build up your perception pipeline. 

Refer to the main Readme file for documentation on python-pcl and pcl_helper modules
