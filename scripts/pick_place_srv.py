#!/usr/bin/env python3

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from spot_msgs.srv import ArmCartesianTrajectory, ArmCartesianTrajectoryRequest
from geometry_msgs.msg import Point, Pose, Quaternion
from std_srvs.srv import Trigger, TriggerRequest
from spot_garbage_collector.srv import MultiGrasp, MultiGraspResponse
from spot_msgs.srv import Grasp3d, Grasp3dRequest
import time
import numpy as np


class PickPlace:
    def __init__(self):
        rospy.init_node("multi_pick_and_place")

        # generate circle arm trajectory from carry to container
        q = Quaternion(0.0, 0.7209864258766174, 0.0, 0.6929491758346558)
        p1 = np.array([[0.6494148373603821], [0.0], [0.36452290415763855]])
        p3 = np.array([[-0.06267781555652618], [0.020665884017944336], [0.36452290415763855]])
        self.traj = [Pose(Point(p1[0, 0], p1[1, 0], p1[2, 0]), q)]
        center = (p1+p3)/2
        center[1, 0] = 0.  # keep the center aligned with the body axis
        p1_s = p1 - center
        p3_s = p3 - center
        norm = np.array([[0], [0], [1]])
        angle = float(np.arctan2(np.matmul(np.cross(p1_s, p3_s, axis=0).T, norm), np.matmul(p1_s.T, p3_s)))
        angle_inc = angle/10
        s = np.sin(angle_inc)
        c = np.cos(angle_inc)
        rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        point = p1_s
        for i in range(10):
            point = np.matmul(rot, point)
            tmp = point + center
            self.traj += [Pose(Point(tmp[0, 0], tmp[1, 0], tmp[2, 0]), q)]

        self.pickup_proxy = rospy.ServiceProxy('/spot/grasp_3d', Grasp3d)
        self.trajectory_proxy = rospy.ServiceProxy('/spot/arm_cartesian_trajectory', ArmCartesianTrajectory)
        self.open_proxy = rospy.ServiceProxy('/spot/gripper_open', Trigger)
        self.close_proxy = rospy.ServiceProxy('/spot/gripper_close', Trigger)
        self.stow_proxy = rospy.ServiceProxy('/spot/arm_stow', Trigger)
        self.service = rospy.Service('/multi_pick_and_place', MultiGrasp, self.run)

    def run(self, req):
        points = req.coords
        frame = req.fixed_frame
        n_failed = 0
        rospy.loginfo('Received grasp request with %d objects' % (len(points)))
        for p in points:
            # build grasp request
            req_pick = Grasp3dRequest(frame, [p.x, p.y, p.z])
            # send pickup request
            rospy.loginfo('Sending pickup request')
            resp = self.pickup_proxy(req_pick)
            if resp.success:
                # move the hand above the container
                r = ArmCartesianTrajectoryRequest()
                r.root_frame = 'body'
                r.traj_time = 5
                r.poses = self.traj
                rospy.loginfo('Sending trajectory request')
                resp = self.trajectory_proxy(r)
                rospy.loginfo(resp)

                # open the gripper
                self.open_proxy(TriggerRequest())
                time.sleep(2)

                # move the arm back to front to avoid singular position
                r = ArmCartesianTrajectoryRequest()
                r.root_frame = 'body'
                r.traj_time = 5
                r.poses = self.traj[::-1]
                rospy.loginfo('Sending trajectory request')
                resp = self.trajectory_proxy(r)
                rospy.loginfo(resp)

                self.close_proxy()
                self.stow_proxy()
                time.sleep(2)
                rospy.loginfo('Grasp successful')
            else:
                rospy.logwarn('Grasp failed')
                n_failed += 1
        rospy.loginfo('Request finished with %d failures' % n_failed)
        if n_failed == 0:
            msg = 'All grasp successful'
            success = True
        elif n_failed < len(points):
            msg = 'Some grasps failed'
            success = True
        else:
            msg = 'All grasps failed'
            success = False
        return success, n_failed, msg


if __name__ == "__main__":
    pick_server = PickPlace()
    rospy.spin()
