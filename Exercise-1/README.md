# Tabletop Segmentation
![tabletop](https://user-images.githubusercontent.com/20687560/27845160-34792f8e-60e1-11e7-9260-4bf22c317b27.png)
In this exercise you'll get a chance to apply some filtering techniques and use RANSAC plane fitting to segment the table in your point cloud.  

### Note: This exercise requires Python 2 due to the Python bindings to PCL being used here.  If you installed Python via Anaconda using the RoboND-Python-Starterkit, you have Python 3 installed in your RoboND conda environment so this exercise won't work.  

### The VM provided by Udacity comes with Python 2 installed as the default so if you are running the VM, or have Python 2 installed locally, you should have no problem.

To follow along with a detailed walkthrough of this project please see the [Calibration, Filtering and Segmentation]() lesson from RoboND.

In brief, the steps to complete this exercise are the following:

1. Downsample your point cloud by applying a Voxel Grid Filter.
2. Apply a Pass Through Filter to isolate the table and objects.
3. Perform RANSAC plane fitting to identify the table.
4. Use the ExtractIndices Filter to create new point clouds containing the table and objects separately.

### To view a .pcd file:

```
$ pcl_viewer filename.pcd 
```
