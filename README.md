# RoboND-Perception-Exercises
# Add general instructions here


## pcl_helper.py

#### Functions:
`random_color_gen()` 
```
Generates a random set of r,g,b values
Return: a 3-tuple with r,g,b values (range 0-255)
```

`ros_to_pcl(sensor_msgs/PointCloud2)` 
```
Converts sensor_msgs/PointCloud2 to XYZRGB Point Cloud
Return: pcl.PointCloud_PointXYZRGB
```

`pcl_to_ros(pcl.PointCloud_PointXYZRGB)` 
```
Converts XYZRGB Point Cloud to sensor_msgs/PointCloud2
Return: sensor_msgs/PointCloud2
```

`XYZRGB_to_XYZ(XYZRGB_cloud)` 
```
Converts XYZRGB Point Cloud to XYZ Point CLoud
Return: pcl.PointCloud
```

`XYZ_to_XYZRGB(XYZ_cloud, color)` 
```
Takes a 3-tuple as color and adds it to XYZ Point Cloud
Return: pcl.PointCloud_PointXYZRGB
```

`rgb_to_float(color)`
```
Converts 3-tuple color to a single float32
Return: rgb packed as a single float32
```

`get_color_list(cluster_count)` 
```
Creates a list of 3-tuple (rgb) with length of the list = cluster_count
Return: get_color_list.color_list
```
