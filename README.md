# PointCloud2 to open3d
You can convert ROS message sensor_msgs/PointCloud2 to Open3D Point Cloud. 

I apply the techniques as follow: 
- Remove the Ground point 
- Clustering with DBSCAN
- set Region of Interest 
- republish Open3D point cloud as sensor_msgs/PointCloud2

# open3d_convert.py
I used Open3D library for clustering and downsampling, and numpy to set ROI and remove ground.

# pcl_convert.cpp
I used PCL library to convert sensor_msgs/PointCloud2 to pcl::PointCloud.

# xyz.py
I used numpy and sklearn libraries to remove the ground and to cluster data, respectively.
