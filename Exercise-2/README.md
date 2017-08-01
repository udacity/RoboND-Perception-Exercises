# Euclidean Clustering with ROS and PCL

In this exercise you will start building up your perception pipeline in ROS.  Here, you are provided a gazebo world, which contains the same table and random objects from Exercise-1.

A simple stick robot with an RGB-D camera attached to its head via a pan-tilt joint is placed in front of the table.  For the detailed steps of how to carry out this exercise, please see the [Clustering for Segmentation](https://classroom.udacity.com/nanodegrees/nd209/parts/586e8e81-fc68-4f71-9cab-98ccd4766cfe/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/2cc29bbd-5c51-4c3e-b238-1282e4f24f42/concepts/02428d63-6f79-40dc-8105-31eda8e0def4) lesson in the RoboND classroom.

Here's a brief summary of how to get setup for the exercise:

1. First of all copy/move the `sensor_stick` package to `/src` directory of your active ros workspace. 

2. Make sure you have all the dependencies resolved by using the rosdep install tool and run `catkin_make`:  

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ catkin_make
```
3. Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/sensor_stick/models

source ~/catkin_ws/devel/setup.bash
```

4. Test the simulation setup by launching the gazebo environment. The command stated below will open a gazebo world along with an rviz window. Play around with it and get familiar with different topics being published to.

```sh
$ roslaunch sensor_stick robot_spawn.launch
```
![screen shot 2017-07-05 at 12 56 36 pm](https://user-images.githubusercontent.com/20687560/27895526-30da599c-61c8-11e7-80ab-4b4224cfbb10.png)
Now, to build your perception pipeline, you must perform following steps:

2. Create a python ros node that subscribes to `/sensor_stick/point_cloud` topic. Use the `template.py` file found under /sensor_stick/scripts/ to get started.

3. Use your code from Exercise-1 to apply various filters and segment the table using RANSAC. 

4. Create publishers and topics to publish the segmented table and tabletop objects as separate point clouds 

5. Apply Euclidean clustering on the table-top objects (after table segmentation is successful)

6. Create a XYZRGB point cloud such that each cluster obtained from the previous step has its own unique color.

7. Finally publish your colored cluster cloud on a separate topic 
![clusters](https://user-images.githubusercontent.com/9555001/27804180-604d6e04-5fe2-11e7-9f33-d8d8da9a8bc0.png)

You will find `pcl_helper.py` file under `/sensor_stick/scripts`. This file contains various functions to help you build up your perception pipeline. 

Refer to the main README file for documentation on `python-pcl` and `pcl_helper.py` modules
