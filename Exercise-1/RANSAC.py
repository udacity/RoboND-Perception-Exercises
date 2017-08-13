# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')
# For students using Python 2.7 with Anaconda,
# PCL module can be installed from https://anaconda.org/ccordoba12/python-pcl
# And the documentation as follows:
# http://strawlab.github.io/python-pcl/
# In that case, update the code above to:
# cloud = pcl.PointCloud()
# cloud.from_file('tabletop.pcd')


# Voxel Grid filter


# PassThrough filter


# RANSAC plane segmentation


# Extract inliers

# Save pcd for table
# pcl.save(cloud, filename)


# Extract outliers


# Save pcd for tabletop objects


