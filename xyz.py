#!/usr/bin/env python3

import ros_numpy, numpy as np, rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from sklearn.cluster import DBSCAN
from std_msgs.msg import Header
import struct, pcl

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgb', 16, PointField.UINT32, 1),
          ]
a = 255

def callback_pcl(pcl_msg):
    global fields, a, pub
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcl_msg)
 
    # Delete the ground Point Cloud
    xyz_array = np.delete(xyz_array, np.where(xyz_array[:,2]<-1.2), axis=0)

    # Clustering DBSCAN
    model = DBSCAN(eps=1, min_samples=2)
    cluster_id = model.fit_predict(xyz_array)
    num_cls = max(cluster_id) + 1

    clusters = []
    for _ in range(num_cls):
        temp = []
        clusters.append(temp)

    for i, xyz in enumerate(xyz_array):
        if cluster_id[i] == -1:
            continue
        clusters[cluster_id[i]].append([xyz[0], xyz[1], xyz[2]])

    # Revisualization
    points = []
    for cluster in clusters:
        for x, y, z in cluster:
            r = np.uint8(x)
            g = np.uint8(y)
            b = np.uint8(z)
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            point = [x, y, z, rgb]
            points.append(point)
    
    pc2 = point_cloud2.create_cloud(pcl_msg.header, fields, points)
    pub.publish(pc2)


rospy.init_node('convertor', anonymous=True)
rospy.Subscriber("/velodyne_points", PointCloud2, callback_pcl)
pub = rospy.Publisher("new_points", PointCloud2, queue_size=1)
rospy.spin()
