# -*- coding: utf-8 -*-

### include ###
# common?
include "PointXYZtoPointXYZ.pxi"
# Segmentation
include "Segmentation/Segmentation_172.pxi"
include "Segmentation/SegmentationNormal_172.pxi"
include "Segmentation/EuclideanClusterExtraction_172.pxi"
# Filters
include "Filters/StatisticalOutlierRemovalFilter_172.pxi"
include "Filters/VoxelGridFilter_172.pxi"
include "Filters/PassThroughFilter_172.pxi"
include "Filters/ApproximateVoxelGrid_172.pxi"
include "Surface/MovingLeastSquares_172.pxi"
# include "KdTree/KdTree.pxi"
include "KdTree/KdTree_FLANN.pxi"
# Octree
include "Octree/OctreePointCloud_172.pxi"
include "Octree/OctreePointCloud2Buf_172.pxi"
include "Octree/OctreePointCloudSearch_172.pxi"
include "Octree/OctreePointCloudChangeDetector_172.pxi"

include "Vertices.pxi"
include "Filters/CropHull_172.pxi"
include "Filters/CropBox_172.pxi"
include "Filters/ProjectInliers_172.pxi"
include "Filters/RadiusOutlierRemoval_172.pxi"
include "Filters/ConditionAnd_172.pxi"
include "Filters/ConditionalRemoval_172.pxi"
include "Surface/ConcaveHull_172.pxi"
include "Common/RangeImage/RangeImages_172.pxi"

# Registration
include "registration/GeneralizedIterativeClosestPoint_172.pxi"
include "registration/IterativeClosestPoint_172.pxi"
include "registration/IterativeClosestPointNonLinear_172.pxi"
# SampleConsensus
include "SampleConsensus/RandomSampleConsensus_172.pxi"
include "SampleConsensus/SampleConsensusModelPlane_172.pxi"
include "SampleConsensus/SampleConsensusModelSphere_172.pxi"
include "SampleConsensus/SampleConsensusModelCylinder_172.pxi"
include "SampleConsensus/SampleConsensusModelLine_172.pxi"
include "SampleConsensus/SampleConsensusModelRegistration_172.pxi"
include "SampleConsensus/SampleConsensusModelStick_172.pxi"
# pcl 1.7.2?
# include "registration/NormalDistributionsTransform_172.pxi"

# include "Visualization/PointCloudColorHandlerCustoms_172.pxi"

# Features
include "Features/NormalEstimation_172.pxi"
include "Features/VFHEstimation_172.pxi"
include "Features/IntegralImageNormalEstimation_172.pxi"
# package ng
# include "Features/MomentOfInertiaEstimation_172.pxi"

# keyPoint
# include "KeyPoint/HarrisKeypoint3D_172.pxi"
# execute NG?
# include "KeyPoint/UniformSampling_172.pxi"

# Registration
include "registration/NormalDistributionsTransform_172.pxi"
###



