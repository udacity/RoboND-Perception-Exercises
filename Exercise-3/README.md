# Object Recognition with Python, ROS and PCL

This exercise builds on what you've done in Exercises 1 and 2. If you haven't done those yet, you should start by completing the [Lesson for Exercise-1](https://classroom.udacity.com/nanodegrees/nd209/parts/586e8e81-fc68-4f71-9cab-98ccd4766cfe/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/8d51e0bf-0fa1-49a7-bd45-e062c4a2121f/concepts/02cbb56e-9e54-4c08-977b-df149cb0bca4) and the [Lesson for Exercise-2](https://classroom.udacity.com/nanodegrees/nd209/parts/586e8e81-fc68-4f71-9cab-98ccd4766cfe/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/2cc29bbd-5c51-4c3e-b238-1282e4f24f42/concepts/02428d63-6f79-40dc-8105-31eda8e0def4?contentVersion=1.0.0&contentLocale=en-us). 

In this exercise, you will continue building up your perception pipeline in ROS.  Here you are provided with a very simple gazebo world, where you can extract color and shape features from the objects that were sitting on the table from Exercise-1 and Exercise-2, in order to train a classifier to detect them.


## Setup
* If you completed Exercises 1 and 2 you will already have a `sensor_stick` folder in your `~/catkin_ws/src` directory.  You should replace that folder with the `sensor_stick` folder contained in this repository and add the Python script you wrote for Exercise-2 to the `scripts` directory. 

* If you do not already have a `sensor_stick` directory, first copy/move the `sensor_stick` folder to the `~/catkin_ws/src` directory of your active ros workspace. 

* Make sure you have all the dependencies resolved by using the `rosdep install` tool and running `catkin_make`:  
 
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ catkin_make
```

* If it's not already there, add the following lines to your `.bashrc` file  

```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/sensor_stick/models
source ~/catkin_ws/devel/setup.bash
```

## Preparing for training

Launch the `training.launch` file to bring up the Gazebo environment: 

```sh
$ roslaunch sensor_stick training.launch
```
You should see an empty scene in Gazebo with only the sensor stick robot.

## Capturing Features
Next, in a new terminal, run the `capture_features.py` script to capture and save features for each of the objects in the environment.  This script spawns each object in random orientations (default 5 orientations per object) and computes features based on the point clouds resulting from each of the random orientations.

```sh
$ rosrun sensor_stick capture_features.py
```

The features will now be captured and you can watch the objects being spawned in Gazebo. It should take 5-10 sec. for each random orientations (depending on your machine's resources) so with 7 objects total it takes awhile to complete. When it finishes running you should have a `training_set.sav` file.

## Training

Once your feature extraction has successfully completed, you're ready to train your model. First, however, if you don't already have them, you'll need to install the `sklearn` and `scipy` Python packages.  You can install these using `pip`:

```sh
pip install sklearn scipy
```

After that, you're ready to run the `train_svm.py` model to train an SVM classifier on your labeled set of features.

```sh
$ rosrun sensor_stick train_svm.py
```
**Note:  Running this exercise out of the box your classifier will have poor performance because the functions `compute_color_histograms()` and `compute_normal_histograms()` (within `features.py` in /sensor_stick/src/sensor_stick) are generating random junk.  Fix them in order to generate meaningful features and train your classifier!**

## Classifying Segmented Objects

If everything went well you now have a trained classifier and you're ready to do object recognition!  First you have to build out your node for segmenting your point cloud.  This is where you'll bring in your code from Exercises 1 and 2.

Make yourself a copy of the `template.py` file in the `sensor_stick/scripts/` directory and call it something like `object_recognition.py`.  Inside this file, you'll find all the TODO's from Exercises 1 and 2 and you can simply copy and paste your code in there from the previous exercises.  

The new code you need to add is listed under the Exercise-3 TODO's in the `pcl_callback()` function.  You'll also need to add some new publishers for outputting your detected object clouds and label markers.  For the step-by-step instructions on what to add in these Exercise-3 TODOs, see the [lesson in the classroom](https://classroom.udacity.com/nanodegrees/nd209/parts/586e8e81-fc68-4f71-9cab-98ccd4766cfe/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/81e87a26-bd41-4d30-bc8b-e747312102c6/concepts/dfab1b50-2efd-428d-bfd9-d1df0544541e).

