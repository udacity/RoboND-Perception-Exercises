# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load('table_scene_lms400.pcd')

# Much like the previous filters, we start by creating a filter object: 
outlier_filter = cloud.make_statistical_outlier_filter()

# Set the number of neighboring points to analyze for any given point
outlier_filter.set_mean_k(50)

# Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
x = 1
outlier_filter.set_std_dev_mul_thresh(x)

# Finally call the filter function for magic
cloud_filtered = outlier_filter.filter()

# pcl.save(cloud, filename)
filename = 'outlier_removed.pcd'
pcl.save(cloud_filtered, filename)

filename = 'outlier.pcd'
outlier_filter.set_negative(True)
pcl.save(outlier_filter.filter(), filename)