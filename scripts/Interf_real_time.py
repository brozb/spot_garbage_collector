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
from sklearn.cluster import DBSCAN
from sklearn import metrics

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from spot_msgs.srv import ArmGaze
from std_srvs.srv import Trigger

import open3d as o3d
from sklearn.cluster import KMeans, DBSCAN
from sklearn.preprocessing import StandardScaler
import sensor_msgs.point_cloud2 as pc2

from lib_plot_TFs_real import *
#from lib_plot_TFs_sim import *

from scipy.cluster.vq import kmeans,vq

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

from spot_garbage_collector.srv import MultiGrasp, MultiGraspRequest


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
threshold_slider = Slider(threshold, 'threshold vol', 0.0, 500.0, 80.0)
clusters = Slider(num_clusters, 'num clusters', 0.0, 40.0, 3.0, color='cyan')
freq = Slider(axdownsample, 'downsample', 0.001, 1.0, 0.5)
amplitude = Slider(axradiusoutliers, 'radius outliers', 0, 5.0, 1.0)
neib = Slider(axnboutliers, 'neib outliers', 0.0, 100.0, 20.0)
zmax = Slider(z_max, 'Z Max', 0.0, 1.0,  0.154)
zmin = Slider(z_min, 'Z Min', -1.0, 1.0, -0.077)
ymax = Slider(y_max, 'Y Max', -4.0, 5.0, 1.17)
ymin = Slider(y_min, 'Y Min', -5.0, 4.0, -0.11)
xmax = Slider(x_max, 'X Max', -4.0, 5.0, 1.625)
xmin = Slider(x_min, 'X Min', -5.0, 4.0, 0.192)

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
##########     New Clusters
################################################################################



###############################################
##########     FUNCTIONS
################################################################################
nube_local_recibida = PointCloud2()

def callback_L_PC(data):
    global nube_local_recibida
    nube_local_recibida = data

def _get_optimal_order(event):
    global centroids
    
    test_funct(centroids)
    print("Its optimal")

def _pub_arm_pose(event):
    # Function to pub each one of the  goal points of clusters to inspect with the arm camera
    global poses_arm, centroids, point_pose_arm, counter_pose_arm

    rospy.wait_for_service('/spot/arm_gaze')
    proxy = rospy.ServiceProxy('/spot/arm_gaze', ArmGaze)

    coord_x = centroids[counter_pose_arm][0]
    coord_y = centroids[counter_pose_arm][1]
    coord_z = centroids[counter_pose_arm][2]

    # send gaze command
    gaze_cmd = ArmGaze()
    gaze_cmd.frame_name = 'vision'
    gaze_cmd.point = Point(coord_x, coord_y, coord_z)
    res = proxy(gaze_cmd)  #returns the result once the arm has reached the position

    if not res.success:
        rospy.logerr("Gaze command was not successful")
    else:
        rospy.loginfo("Gaze completed")

    # the object should be now approximately in the center of the image of gripper camera

    # TODO: read gripper camera image and verify if the detected object is graspable
    #       wait is probably necessary, since the camera image can be a little bit delayed
    #       multiple objects can be detected in single image, use the P matrix from sensor_msgs/CameraInfo to project the 3d point into the image

    # gaze command leaves the arm at the selected pose, stow it before returning
    rospy.wait_for_service('/spot/arm_stow')
    proxy2 = rospy.ServiceProxy('/spot/arm_stow', Trigger)
    proxy2()
    # the gripper has to be closed
    rospy.wait_for_service('/spot/gripper_close')
    proxy3 = rospy.ServiceProxy('/spot/gripper_close', Trigger)
    proxy3()

    counter_pose_arm += 1
    print ("Pub Goal Pose for Arm Inspect")

def _pub_mark(event):
    # Function to indicate the robot take an detectoed object
    global centroids, point, counter_num_collect_obj, service
    # create an interactive marker server on the topic namespace simple_marker

    list = []

    for i in range(len(centroids)):
        p1 = Point()
        p1.x = centroids[i][0]
        p1.y = centroids[i][1]
        p1.z = centroids[i][2]
        list.append(p1)        

    #req = MultiGraspRequest('vision',[p1,p2,p3])
    req = MultiGraspRequest('vision',list)
    resp = service(req)
    
    if resp.fail_id != -1:
        #### Text Output
        plt.text(2.3, 2,"Grasp Fails", bbox={'facecolor':'r','pad':5},
                ha="right", va="top", transform=plt.gca().transAxes )
        #### Text Output
    else:
        #### Text Output
        plt.text(2.3, 2,"Grasp Fails", bbox={'facecolor':'w','pad':5},
                ha="right", va="top", transform=plt.gca().transAxes )
        #### Text Output

    print("ok")

def _extract_soil(event):

    global pcd_4_pub, pcd_temp
    plane_model, inliers = pcd_4_pub.segment_plane(distance_threshold=0.02,
                                         ransac_n=4,
                                         num_iterations=1000)

    inlier_cloud = pcd_4_pub.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd_4_pub.select_by_index(inliers, invert=True)
    pcd_temp = outlier_cloud
    
def generate_random_color():
    return np.random.uniform(0, 1, 3)

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
    
    # Kmeans constrained
    #model = KMeansConstrained(n_clusters= num_klusters, size_min=2, size_max=5, random_state=0  )
    #model.fit_predict(points)

    # Clustering KMEANS:
    #model = KMeans(n_clusters= num_klusters,  max_iter=100, init='random')
    model = KMeans(n_clusters= num_klusters, n_init=10, random_state=1)
    model.fit(points)

    # --------------------------------------------------------------------------------------------
    # Perform DBSCAN clustering
    # --------------------------------------------------------------------------------------------

    dbscan = DBSCAN(eps=0.1, min_samples=10)  # Adjust eps and min_samples according to your data
    labels = dbscan.fit_predict(points)

    # Process the clusters
    num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    rospy.loginfo("Number of clusters: %d", num_clusters)

    clustered_DBSCAN = []
    centroids_DBSCAN = []

    for i in range(num_clusters):
        cluster_points = points[labels == i]
        rospy.loginfo("Cluster %d: %d points", i, len(cluster_points))
        a = list(np.mean(cluster_points, axis=0))
        centroids_DBSCAN.append(a)

        # visualization
        pcd22 = o3d.geometry.PointCloud()
        pcd22.points = o3d.utility.Vector3dVector(cluster_points)
        color = generate_random_color()
        pcd22.paint_uniform_color(color)
        clustered_DBSCAN.append(pcd22)
       
    #o3d.visualization.draw_geometries(clustered_DBSCAN)


    # --------------------------------------------------------------------------------------------
    # Perform DBSCAN clustering
    # -----------------------------np.mean(cluster_points, axis=0)---------------------------------------------------------------

    # ································································································
    # ................. KMEANS
    # ································································································
    
    """# Get labels:
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
    
    # Evaluate the number of points on each cluster
    idx,_ = vq(points,centroids)
    print("Vol:", np.bincount(idx))

    # Delete the centroids woth values higer than threshold
    centroids_list= centroids.tolist()"""

    # ································································································
    # ................. KMEANS
    # ································································································
    centroids_list = centroids_DBSCAN

    cont_centr_aux = 0
    for i in range(len(set(labels))):
        #cluster_points = points[labels == i]
        #if np.bincount(idx)[i] > threshold_slider_val:
        if len(points[labels == i]) > threshold_slider_val:     
            #print ("PC zise", np.bincount(idx)[i])
            centroids_list.pop(i-cont_centr_aux)
            cont_centr_aux += 1
    
    # Convert the centroids in list again 
    centroids = np.asarray(centroids_list)
    print("centroides Kmeans",centroids)
    
    # Plot TF in Rviz
    plot_TF(len(centroids),centroids)

def _save_point_cloud(event):

    global pcd_4_pub
    o3d.io.write_point_cloud("spot_360_t1.pcd", pcd_4_pub, write_ascii=True)
    o3d.visualization.draw_geometries([pcd_4_pub ])
    print("point CL")

def update(val):
    global val_downs, pub, pcl_pub, pcd_4_pub, pcd_temp, clust_default
    global nube_local_recibida

    #pcd = pcd_temp
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

    
    # Generate new pointCloud in order to publish
    #xx=np.asarray(pcd.points)[:,0]
    #yy=np.asarray(pcd.points)[:,1]
    #zz=np.asarray(pcd.points)[:,2]

    pcd = pc2.read_points(nube_local_recibida, skip_nans=True, field_names=("x", "y", "z"))

    #pcd_4_pub = pcd

    nube_local_recibida_arr = pc2.read_points(nube_local_recibida, skip_nans=True)

    points = []
    for point in nube_local_recibida_arr:
        points.append([point[0], point[1], point[2]])

    # Create Open3D PointCloud object
    """pcd_g = o3d.geometry.PointCloud()
    pcd_g.points = o3d.utility.Vector3dVector(points)
   

    pcd_g = pcd_g.crop(bounding_box)

    # 2222222222222222222 Downsample
    pcd_g = pcd_g.voxel_down_sample(voxel_size = val_downs)

    # 3333333333333333333     outliers removal
    cloud, ind  = pcd_g.remove_statistical_outlier(nb_neighbors = int(val_nb_out), 
                                             std_ratio = val_rad_out)
    pcd_g = pcd_g.select_by_index(ind)

    pcd_4_pub = pcd_g """

    # DO IT HERE ALL THE OPERATIONS '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

    o3d_pointcloud = o3d.geometry.PointCloud()
    o3d_pointcloud.points = o3d.utility.Vector3dVector(points)

    o3d_pointcloud = o3d_pointcloud.crop(bounding_box)
    cloud, ind  = o3d_pointcloud.remove_statistical_outlier(nb_neighbors = int(val_nb_out), 
                                             std_ratio = val_rad_out)
    o3d_pointcloud = o3d_pointcloud.select_by_index(ind)

    pcd_4_pub = o3d_pointcloud

    # PUBLSH AGAIN TO RVIZ ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    # for simulation
    #header.frame_id = 'body'
    # for real
    header.frame_id = 'vision'
    scaled_polygon_pcl = pc2.create_cloud_xyz32(header, o3d_pointcloud.points)

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
    global f, pub, pcl_pub, point, point_pose_arm, service
    pub = rospy.Publisher('slider', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    # call service
    service = rospy.ServiceProxy('/multi_pick_and_place', MultiGrasp)

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

    sub_LPCL = rospy.Subscriber("/pcl_filtered", PointCloud2, callback_L_PC)
    

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
