# PointCloud2 to open3d
You can convert ROS message sensor_msgs/PointCloud2 to Open3D Point Cloud. 

I apply the techniques as follow: 
- Remove the Ground point 
- Clustering with DBSCAN
- set Region of Interest 
- republish Open3D point cloud as sensor_msgs/PointCloud2
