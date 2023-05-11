#!/usr/bin/env python
# roslaunch spot_viz view_model.launch
# this program requiers of plo_TFs.py for plotting the TF in RVIZ
import rospy
import numpy as np
from rospy import Time 
import time
from tf import TransformBroadcaster
from std_msgs.msg import String, Float32
import itertools
import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

import open3d as o3d
from sklearn.cluster import KMeans, DBSCAN
from sklearn.preprocessing import StandardScaler

from lib_plot_TFs_real import *
#from lib_plot_TFs_sim import *

from scipy.cluster.vq import kmeans,vq

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

from optimize_route import *


pcd_temp = o3d.io.read_point_cloud("/home/robot/christyan/PoinClouds/PoinCLoud_spot_five_cameras_30.pcd")
#o3d.visualization.draw_geometries([pcd])
################################################################################
##########     LIBRARIES
################################################################################


################################################################################
##########     VARIABLES
################################################################################

threshold    =     plt.axes([0.25, 0.05, 0.65, 0.02])
num_clusters =     plt.axes([0.25, 0.10, 0.65, 0.02])
axdownsample =     plt.axes([0.25, 0.25, 0.65, 0.02])
axradiusoutliers = plt.axes([0.25, 0.2, 0.65, 0.02])
axnboutliers =     plt.axes([0.25, 0.15, 0.65, 0.02])
z_max =            plt.axes([0.25, 0.35, 0.65, 0.02])
z_min =            plt.axes([0.25, 0.30, 0.65, 0.02])
y_max =            plt.axes([0.25, 0.45, 0.65, 0.02])
y_min =            plt.axes([0.25, 0.40, 0.65, 0.02])
x_max =            plt.axes([0.25, 0.55, 0.65, 0.02])
x_min =            plt.axes([0.25, 0.50, 0.65, 0.02])

axcut = plt.axes([0.25, 0.6, 0.3, 0.075])
axcut_2 = plt.axes([0.6, 0.6, 0.3, 0.075])
extract_soil = plt.axes([0.25, 0.7, 0.3, 0.075])
pub_mark = plt.axes([0.6, 0.7, 0.3, 0.075])
pub_arm_pose = plt.axes([0.6, 0.8, 0.3, 0.075])
get_optimal_order = plt.axes([0.25, 0.8, 0.3, 0.075])

# read values from slider
threshold_slider = Slider(threshold, 'threshold vol', 0.0, 2000.0, 1500.0)
clusters = Slider(num_clusters, 'num clusters', 0.0, 40.0, 3.0, color='cyan')
freq = Slider(axdownsample, 'downsample', 0.0, 1.0, 0.5)
amplitude = Slider(axradiusoutliers, 'radius outliers', 0, 5.0, 1.0)
neib = Slider(axnboutliers, 'neib outliers', 0.0, 100.0, 20.0)
zmax = Slider(z_max, 'Z Max', -4.0, 5.0,  -0.127)
zmin = Slider(z_min, 'Z Min', -5.0, 4.0, -3.0)
ymax = Slider(y_max, 'Y Max', -4.0, 5.0, 2.0)
ymin = Slider(y_min, 'Y Min', -5.0, 4.0, -2.0)
xmax = Slider(x_max, 'X Max', -4.0, 5.0, 2.0)
xmin = Slider(x_min, 'X Min', -5.0, 4.0, -2.0)

# START VALUE
clust_default = 3.0
val_downs = 0.1
val_rad_out = 0.8 
val_nb_out = 20.0
zmax_val = 3.0
zmin_val = 0.0
ymax_val = 10.0
ymin_val = -9.0
xmax_val = 10.0
xmin_val = -9.0
threshold_slider_val = 1500
counter_num_collect_obj = 0
counter_pose_arm = 0

#TF VROADCASTER
translation = (0.0, 0.0, 0.0)
rotation = (0.0, 0.0, 0.0, 1.0)
a = TransformBroadcaster()
b = TransformBroadcaster()

# VARIABLES NUBES DE PUNTOS
d = {}
# Radio Max for inspection
Rad = 0.65
z_alt = 0

################################################################################
##########     Traversability
################################################################################


################################################################################
##########     FUNCTIONS
################################################################################
def _get_optimal_order(event):
    global centroids
    
    test_funct(centroids)
    print("Its optimal")

def _pub_arm_pose(event):
    # Function to pub each one of the  goal points of clusters to inspect with the arm camera
    global poses_arm, centroids, point_pose_arm, counter_pose_arm

    coord_x = centroids[counter_pose_arm][0]
    coord_y = centroids[counter_pose_arm][1]
    angle = abs(math.atan(coord_y / coord_x))

    new_coord_x = (coord_x/abs(coord_x))*(Rad * math.cos(angle))
    new_coord_y = (coord_y/abs(coord_y))*(Rad * math.sin(angle))
    print(new_coord_x,new_coord_y)

    point_pose_arm.publish(new_coord_x,new_coord_y,z_alt)
    b = TransformBroadcaster()
    b.sendTransform((new_coord_x,new_coord_y,z_alt), (0,0,0,1), Time.now() - rospy.Duration(3) , 'arm_inspect_point', '/vision')

    counter_pose_arm += 1
    print ("Pub Goal Pose for Arm Inspect")

def _pub_mark(event):
    # Function to indicate the robot take an detectoed object
    global centroids, point, counter_num_collect_obj
    # create an interactive marker server on the topic namespace simple_marker

    """server = InteractiveMarkerServer("simple_marker")
    
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "body"
    int_marker.name = "object_1"

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.SPHERE
    box_marker.scale.x = 0.1; box_marker.scale.y = 0.1 ; box_marker.scale.z = 0.1
    box_marker.color.r = 0.0 ; box_marker.color.g = 1 ; box_marker.color.b = 0.0 ; box_marker.color.a = 0.7
    box_marker.pose.position.x = centroids[0][0]
    box_marker.pose.position.y = centroids[0][1]
    box_marker.pose.position.z = centroids[0][2]

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( box_marker )
    # add the control to the interactive marker
    int_marker.controls.append( box_control )

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker)
    server.applyChanges()"""
    
    point.publish(centroids[counter_num_collect_obj][0],centroids[counter_num_collect_obj][1],centroids[counter_num_collect_obj][2])
    print("Lets grasp object # ",counter_num_collect_obj + 1)
    #rospy.spin()

def _extract_soil(event):

    global pcd_4_pub, pcd_temp
    plane_model, inliers = pcd_4_pub.segment_plane(distance_threshold=0.02,
                                         ransac_n=4,
                                         num_iterations=1000)

    inlier_cloud = pcd_4_pub.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd_4_pub.select_by_index(inliers, invert=True)
    pcd_temp = outlier_cloud
    
def _save_param(event):
    global counter_pose_arm
    # get clusters
    global pcd_4_pub, clust_default, translation, rotation, centroids
    # Restart object reach using arm
    counter_pose_arm = 0

    clust_default = clusters.val
    threshold_slider_val = threshold_slider.val
    pcd_klusters = pcd_4_pub

    num_klusters = round(clust_default)
    points = np.asarray(pcd_klusters.points).copy()

    # ································································································
    # ................. KMEANS
    # ································································································
    
    # Clustering KMEANS:
    #model = KMeans(n_clusters= num_klusters, max_iter=100, init='random')
    model = KMeans(n_clusters= num_klusters, init= 'k-means++', max_iter=100, random_state=0)
    model.fit(points)
    # Get labels:
    labels = model.labels_
    # Get the number of colors:
    n_clusters = len(set(labels))

    # Mapping the labels classes to a color map:
    colors = plt.get_cmap("tab20")(labels / (n_clusters if n_clusters > 0 else 1))
    colors[labels < 0] = 0
    # Update points colors:
    pcd_klusters.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # getcentroids
    centroids  = model.cluster_centers_
    # ································································································
    # ................. KMEANS
    # ································································································
    

    # Obtain the position of each point regarding the label 
    # for i in range(len(set(labels))):
    #    print("radios...", np.where(labels == i)[0])

    # Evaluate the number of points on each cluster
    idx,_ = vq(points,centroids)
    print("Vol:", np.bincount(idx))

    # Delete the centroids woth values higer than threshold
    centroids_list= centroids.tolist()

    cont_centr_aux = 0
    for i in range(len(set(labels))):
        if np.bincount(idx)[i] > threshold_slider_val: 
            print ("PC zise", np.bincount(idx)[i])
            centroids_list.pop(i-cont_centr_aux)
            cont_centr_aux += 1
    
    # Convert the centroids in list again 
    centroids = np.asarray(centroids_list)
    print("Centroid",centroids)
    
    # Plot TF in Rviz
    plot_TF(len(centroids),centroids)
    o3d.visualization.draw_geometries([pcd_klusters])

def _save_point_cloud(event):

    global pcd_4_pub
    o3d.io.write_point_cloud("spot_360_t1.pcd", pcd_4_pub, write_ascii=True)
    o3d.visualization.draw_geometries([pcd_4_pub ])
    print("point CL")

def update(val):
    global val_downs, pub, pcl_pub, pcd_4_pub, pcd_temp, clust_default

    pcd = pcd_temp
    #read new values
    
    val_downs = freq.val
    val_rad_out = amplitude.val
    val_nb_out = neib.val
    zmax_val = zmax.val
    zmin_val = zmin.val
    ymax_val = ymax.val
    ymin_val = ymin.val
    xmax_val = xmax.val
    xmin_val = xmin.val
    #a = amplitude.val
    print(val_downs)

    # 1111111111111111111     create limits
    #bounds = [[-math.inf, math.inf], [-math.inf, math.inf], [zmin_val, zmax_val]]  # set the bounds
    bounds = [[xmin_val, xmax_val], [ymin_val, ymax_val], [zmin_val, zmax_val]]  # set the bounds
    bounding_box_points = list(itertools.product(*bounds))  # create limit points
    bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
        o3d.utility.Vector3dVector(bounding_box_points))  # create bounding box object

    pcd = pcd.crop(bounding_box)

    # 2222222222222222222 Downsample
    pcd = pcd.voxel_down_sample(voxel_size = val_downs)

    # 3333333333333333333     outliers removal
    cloud, ind  = pcd.remove_statistical_outlier(nb_neighbors = int(val_nb_out), 
                                             std_ratio = val_rad_out)
    pcd = pcd.select_by_index(ind)

    # Generate new pointCloud in order to publish
    xx=np.asarray(pcd.points)[:,0]
    yy=np.asarray(pcd.points)[:,1]
    zz=np.asarray(pcd.points)[:,2]

    pcd_4_pub = pcd
    
    pcd_g=[]
    pcd_g=[[xx[j], yy[j], zz[j]]for j in range(len(xx))]

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    # for simulation
    #header.frame_id = 'body'
    # for real
    header.frame_id = 'vision'
    scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, pcd_g)

    pcl_pub.publish(scaled_polygon_pcl)   
    pub.publish(val_downs)


# ································································································
# ................. ROS FUNCTIONS
# ································································································

def callback_finish_grasp(data):
    global counter_num_collect_obj
    print("Finished collection", counter_num_collect_obj + 1)
    counter_num_collect_obj += 1
    

def talker():
    global f, pub, pcl_pub, point, point_pose_arm
    pub = rospy.Publisher('slider', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    # publisher Poin Cloud
    pcl_pub = rospy.Publisher("/downsample_cloud2", PointCloud2, queue_size=1000000)
    
    # publisher point
    point = rospy.Publisher("detected_object",Point, queue_size = 1)
    point_pose_arm = rospy.Publisher("inspect_pose_arm",Point, queue_size = 1)
 
    # subscriber to float32
    rospy.Subscriber('finish_grasp', Float32, callback_finish_grasp)

    bcut = Button(axcut, 'Get Clusters', color='white', hovercolor='green')
    bcut.on_clicked(_save_param)

    bcut_2 = Button(axcut_2, 'Save Point Cloud', color='white', hovercolor='green')
    bcut_2.on_clicked(_save_point_cloud)

    bcut_3 = Button(extract_soil, 'Extract Soil', color='white', hovercolor='green')
    bcut_3.on_clicked(_extract_soil)

    bcut_4 = Button(pub_mark, 'Send Grasp Object', color='white', hovercolor='red')
    bcut_4.on_clicked(_pub_mark)

    bcut_5 = Button(pub_arm_pose, 'Pub Pose Arm Inspect', color='white', hovercolor='red')
    bcut_5.on_clicked(_pub_arm_pose)

    bcut_6 = Button(get_optimal_order, 'Get Optimal Sequence', color='white', hovercolor='yellow')
    bcut_6.on_clicked(_get_optimal_order)

    
    

    #header pointclouds
    rate = rospy.Rate(100) # 10hz
    
    while not rospy.is_shutdown():
        hello_str = val_downs
        
        pub.publish(hello_str)

        freq.on_changed(update)
        amplitude.on_changed(update)
        neib.on_changed(update)
        zmax.on_changed(update)
        zmin.on_changed(update)
        ymax.on_changed(update)
        ymin.on_changed(update)
        xmax.on_changed(update)
        xmin.on_changed(update)
        threshold_slider.on_changed(update)

        plt.show() 

        # plot centroids
        plot_TF(len(centroids),centroids)
        
        rate.sleep() 
         
################################################################################
##########     FUNCTIONS
################################################################################



################################################################################
##########     MAIN
################################################################################

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
