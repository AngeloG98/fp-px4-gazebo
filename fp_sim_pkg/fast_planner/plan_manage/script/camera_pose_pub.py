#!/usr/bin/env python
# Imports
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped

class PoseTransformer:
    def __init__(self):
        rospy.init_node('camera_pose_pub')
        self.rate = rospy.Rate(30.0)
        self.local_t = [0,0,0]
        self.local_q = [1,0,0,0]
        self.local_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pose_callback)
        self.camera_pose_pub = rospy.Publisher('/camera/pose', PoseStamped, queue_size=1)

    def q2R(self,q):
        q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
        R = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                      [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                      [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])
        return R

    def R2q(self,R):
        q_w = ((np.trace(R)+1)**0.5)/2
        q_x = (R[2][1] - R[1][2]) / (4*q_w)
        q_y = (R[0][2] - R[2][0]) / (4*q_w)
        q_z = (R[1][0] - R[0][1]) / (4*q_w)
        return [q_w, q_x, q_y, q_z]

    def local_pose_callback(self, msg):
        self.localpose = msg
        self.local_t = [self.localpose.pose.position.x, self.localpose.pose.position.y, self.localpose.pose.position.z]
        self.local_q = [self.localpose.pose.orientation.w, self.localpose.pose.orientation.x, 
                                    self.localpose.pose.orientation.y, self.localpose.pose.orientation.z]

    def transform_local2camera(self):
        # R
        self.local_R = self.q2R(self.local_q)
        self.R_cb = np.array([[0,0,1],
                                                [-1,0,0],
                                                [0,-1,0]])
        self.camera_R = self.local_R.dot(self.R_cb)
        self.camera_q = self.R2q(self.camera_R)
        # t
        self.camera_t = self.local_t

    def pub_camera_pose(self):
        self.transform_local2camera()
        camera_pose_msg = PoseStamped()
        camera_pose_msg.header.frame_id = 'mycamera'
        camera_pose_msg.header.stamp = rospy.Time.now()
        camera_pose_msg.pose.position.x = self.camera_t[0]
        camera_pose_msg.pose.position.y = self.camera_t[1]
        camera_pose_msg.pose.position.z = self.camera_t[2]
        camera_pose_msg.pose.orientation.x = self.camera_q[1]
        camera_pose_msg.pose.orientation.y = self.camera_q[2]
        camera_pose_msg.pose.orientation.z = self.camera_q[3]
        camera_pose_msg.pose.orientation.w = self.camera_q[0]
        self.camera_pose_pub.publish(camera_pose_msg)
        self.rate.sleep()

if __name__ == '__main__':
    l2c = PoseTransformer()
    while True:
        l2c.pub_camera_pose()


