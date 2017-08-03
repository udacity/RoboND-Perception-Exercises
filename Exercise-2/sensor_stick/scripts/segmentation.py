#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

	# TODO: Convert ROS msg to PCL data
	cloud = ros_to_pcl(pcl_msg)

	# TODO: Voxel Grid Downsampling
	vox = cloud.make_voxel_grid_filter()
	LEAF_SIZE = 0.01   
	# Set the voxel (or leaf) size  
	vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
	# Call the filter function to obtain the resultant downsampled point cloud
	cloud_filtered = vox.filter()
	# TODO: PassThrough Filter
	passthrough = cloud_filtered.make_passthrough_filter()

	# Assign axis and range to the passthrough filter object.
	filter_axis = 'z'
	passthrough.set_filter_field_name (filter_axis)
	axis_min = 0.77
	#axis_min = 0.6 # leave vertical side of the table
	axis_max = 1.1
	passthrough.set_filter_limits (axis_min, axis_max)

	# Finally use the filter function to obtain the resultant point cloud. 
	cloud_filtered = passthrough.filter()

	# TODO: RANSAC Plane Segmentation
	seg = cloud_filtered.make_segmenter()

	# Set the model you wish to fit 
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)

	# Max distance for a point to be considered fitting the model
	max_distance = 0.01
	seg.set_distance_threshold(max_distance)

	# Call the segment function to obtain set of inlier indices and model coefficients
	inliers, coefficients = seg.segment()


	# TODO: Extract inliers and outliers
	# Extract inliers
	extracted_inliers = cloud_filtered.extract(inliers, negative=False)
	# Extract outliers
	extracted_outliers = cloud_filtered.extract(inliers, negative=True)

	# TODO: Euclidean Clustering
	white_cloud =  XYZRGB_to_XYZ(extracted_outliers)# Apply function to convert XYZRGB to XYZ
	tree = white_cloud.make_kdtree()
	# Create a cluster extraction object
	ec = white_cloud.make_EuclideanClusterExtraction()
	# Set tolerances for distance threshold 
	# as well as minimum and maximum cluster size (in points)
	ec.set_ClusterTolerance(0.015)
	ec.set_MinClusterSize(20)
	ec.set_MaxClusterSize(1500)
	# Search the k-d tree for clusters
	ec.set_SearchMethod(tree)
	# Extract indices for each of the discovered clusters
	cluster_indices = ec.Extract()

	# TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
	#Assign a color corresponding to each segmented object in scene
	cluster_color = get_color_list(len(cluster_indices))

	color_cluster_point_list = []

	for j, indices in enumerate(cluster_indices):
	    for i, indice in enumerate(indices):
	        color_cluster_point_list.append([white_cloud[indice][0],
	                                        white_cloud[indice][1],
	                                        white_cloud[indice][2],
	                                         rgb_to_float(cluster_color[j])])

	#Create new cloud containing all clusters, each with unique color
	cluster_cloud = pcl.PointCloud_PointXYZRGB()
	cluster_cloud.from_list(color_cluster_point_list)
	# TODO: Convert PCL data to ROS messages
	ros_cloud_objects =  pcl_to_ros(cluster_cloud)
	ros_cloud_table = pcl_to_ros(extracted_inliers)

	# TODO: Publish ROS messages
	pcl_objects_pub.publish(ros_cloud_objects)
	pcl_table_pub.publish(ros_cloud_table)

if __name__ == '__main__':

	# TODO: ROS node initialization
	rospy.init_node('clustering', anonymous=True)

	# TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

	# TODO: Create Publishers
	pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
	pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)

	# Initialize color_list
	get_color_list.color_list = []

	# TODO: Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()
