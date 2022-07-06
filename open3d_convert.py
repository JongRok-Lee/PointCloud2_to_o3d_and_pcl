#!/usr/bin/env python3

import numpy as np, rospy, open3d, struct
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgb', 16, PointField.UINT32, 1),
          ]
a = 255
GROUND_z = -1.2 # ground z threshold
ROI_x, ROI_y = [-5, 50], [-5, 5]

def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = np.asarray(list(point_cloud2.read_points(ros_cloud, skip_nans=True, field_names = field_names)))
    xyz = np.delete(cloud_data, (3,4), axis=1)
    xyz_1 = np.delete(xyz, np.where(xyz[:,2]<GROUND_z), axis=0)
    xyz_2 = np.delete(xyz_1, np.where(xyz_1[:,0]<ROI_x[0]), axis=0)
    xyz_3 = np.delete(xyz_2, np.where(xyz_2[:,0]>ROI_x[1]), axis=0)
    
    xyz_4 = np.delete(xyz_3, np.where(xyz_3[:,1]<ROI_y[0]), axis=0)
    xyz_5 = np.delete(xyz_4, np.where(xyz_4[:,1]>ROI_y[1]), axis=0)
    
    open3d_cloud = open3d.geometry.PointCloud()
    open3d_cloud.points = open3d.utility.Vector3dVector(xyz_5)

    return open3d_cloud

def callback_pcl(pcl_msg):
    global pub
    o3d_cloud = convertCloudFromRosToOpen3d(pcl_msg)
    down_cloud = open3d.geometry.PointCloud.voxel_down_sample(o3d_cloud, 0.1)
    labels = np.array(down_cloud.cluster_dbscan(eps=0.3, min_points=20, print_progress=False))

    pc2 = pc2pub(labels, down_cloud, pcl_msg)
    pub.publish(pc2)
    


def pc2pub(labels, down_cloud, pcl_msg):
    global fields, a
    clusters = []
    num_cls = labels.max() + 1
    xyz_array = np.asarray(down_cloud.points)
    for _ in range(num_cls):
        temp = []
        clusters.append(temp)

    for i, xyz in enumerate(xyz_array):
        if labels[i] == -1:
            continue
        clusters[labels[i]].append([xyz[0], xyz[1], xyz[2]])

    # Revisualization
    points = []
    for i, cluster in enumerate(clusters):
        for x, y, z in cluster:
            r, g, b = np.uint8(10*i), np.uint8(50*i), np.uint8(100*i)
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            point = [x, y, z, rgb]
            points.append(point)
    pc2 = point_cloud2.create_cloud(pcl_msg.header, fields, points)
    return pc2

rospy.init_node('convertor', anonymous=True)
rospy.Subscriber("/velodyne_points", PointCloud2, callback_pcl)
pub = rospy.Publisher("new_points", PointCloud2, queue_size=1)
rospy.spin()
