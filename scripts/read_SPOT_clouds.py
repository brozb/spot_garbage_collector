#!/usr/bin/env python
# determina la localizacion mediante L-PC y G-PC
import open3d as o3d
import numpy as np
import copy
import rospy, time, math
import struct
from std_msgs.msg import Float64

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2



nube_local_recibida = PointCloud2()
nube_local_recibida2 = PointCloud2()
instante_inicial = time.time()
ok = True

def callback_L_PC(data):
    global nube_local_recibida
    nube_local_recibida = data
    
def callback_L_PC2(data):
    global nube_local_recibida2
    nube_local_recibida2 = data

def main():

    global nube_local_recibida, nube_local_recibida2

    # DEFINICON DE NODOS DE ROS PARA SUBSCRIBIR Y PUBLICAR

    rospy.init_node('SPOT_reader')
	#rospy.init_node("controller_manager", anonymous=True)

    #sub_LPCL = rospy.Subscriber("/camera/depth/points", PointCloud2, callback_L_PC)
    sub_LPCL = rospy.Subscriber("/points", PointCloud2, callback_L_PC)
    sub_LPCL2 = rospy.Subscriber("/points/frontright", PointCloud2, callback_L_PC2)
	#rate = rospy.Rate(100)
    msg = Float64()



    #while not rospy.is_shutdown():
    while not rospy.is_shutdown():
        print ('aqui')
        instante_actual = time.time()
        tiempo = instante_actual - instante_inicial
        print (tiempo)

        if tiempo > 4:
            
            pc = pc2.read_points(nube_local_recibida, skip_nans=True, field_names=("x", "y", "z"))
            pc_x = []
            pc_y = []
            pc_z = []

            for p in pc:
            #pc_list.append( [p[0],p[1],p[2]] )

                pc_x.append(p[0])
                pc_y.append(p[1])
                pc_z.append(p[2])

            pc_2 = pc2.read_points(nube_local_recibida2, skip_nans=True, field_names=("x", "y", "z"))
            pc_x_2 = []
            pc_y_2 = []
            pc_z_2 = []

            pcd_g = [[pc_x[c], pc_y[c], pc_z[c]]for c in range(len(pc_x))]
            
            for p in pc_2:
            #pc_list.append( [p[0],p[1],p[2]] )

                pc_x_2.append(p[0])
                pc_y_2.append(p[1])
                pc_z_2.append(p[2])

            pcd_g_2 = [[pc_x_2[c], pc_y_2[c], pc_z_2[c]]for c in range(len(pc_x_2))]

            pc = pcd_g + pcd_g_2
            pcd_tem = o3d.geometry.PointCloud()
            pcd_tem.points = o3d.utility.Vector3dVector(pcd_g)
            #if tiempo > 1:
            #o3d.visualization.draw_geometries([pcd_tem])

            o3d.visualization.draw_geometries([pcd_tem])
            o3d.io.write_point_cloud("/home/robot/christyan/PoinClouds/PoinCLoud_spot_five_cameras_30.pcd", pcd_tem, write_ascii=True)

		#rate.sleep()
	
	#rospy.spin()


if __name__ == "__main__":

	main()


