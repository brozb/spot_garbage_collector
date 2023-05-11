#!/usr/bin/env python3

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from spot_msgs.srv import Grasp3d, Grasp3dRequest
from spot_msgs.srv import ArmCartesianTrajectory, ArmCartesianTrajectoryRequest
from geometry_msgs.msg import Point, Pose, Quaternion
from std_srvs.srv import Trigger, TriggerRequest


class PickPlace:
    def __init__(self):
        rospy.init_node("pick_and_place")
        self.pickup_proxy = rospy.ServiceProxy('/spot/grasp_3d',Grasp3d)
        self.trajectory_proxy = rospy.ServiceProxy('/spot/arm_cartesian_trajectory', ArmCartesianTrajectory)
        self.gripper_proxy = rospy.ServiceProxy('/spot/gripper_open', Trigger)

    def run(self):
        msg = rospy.wait_for_message('/detected_object', Point)

        # send pickup request
        req = Grasp3dRequest()
        req.frame_name = 'vision'
        req.object_rt_frame = [msg.x, msg.y, msg.z]
        rospy.loginfo('Sending pickup request')
        resp = self.pickup_proxy(req)
        if resp.success:
            # move the hand above the container
            p1 = Pose(Point(0.6494148373603821, 0.0, 0.36452290415763855), Quaternion(0.0, 0.7209864258766174, 0.0, 0.6929491758346558))
            p2 = Pose(Point(0.300723671913147, 0.42215684056282043, 0.3783569037914276), Quaternion(0.0, 0.7209864258766174, 0.0, 0.6929491758346558))
            p3 = Pose(Point(-0.06267781555652618, 0.020665884017944336, 0.3927745521068573), Quaternion(0.0, 0.7209864258766174, 0.0, 0.6929491758346558))
            traj = [p1, p2, p3]

            r = ArmCartesianTrajectoryRequest()
            r.root_frame = 'body'
            r.traj_time = 15
            r.poses = traj

            rospy.loginfo('Sending trajectory request')
            resp = self.trajectory_proxy(r)
            rospy.loginfo(resp)

            # open the gripper
            self.gripper_proxy(TriggerRequest())
        else:
            rospy.logerr('Grasp failed')

if __name__ == "__main__":
    p = PickPlace()
    p.run()
