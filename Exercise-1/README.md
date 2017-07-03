# Tabletop Segmentation

### Note: This exercise requires Python 2.  If you installed Python via Anaconda using the RoboND-Python-Starterkit, you have Python 3 installed in your RoboND conda environment so this exercise won't work.  The VM provided by Udacity comes with Python 2 installed as the default so if you are running the VM, or have Python 2 installed locally, you should have no problem.

### Install cython
```
$ sudo pip install cython
```

### Build and Install pcl-python
```
$ cd ~/RoboND-Perception-Exercises/python-pcl
$ python setup.py build
$ sudo python setup.py install
```

### Install pcl-tools
```
$ sudo apt-get install pcl-tools
```

### To view a .pcd file:
```
$ pcl_viewer filename.pcd 
```
