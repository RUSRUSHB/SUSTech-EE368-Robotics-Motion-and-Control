import math

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

class Link:
    def __init__(self, dh_params):
        self.dh_params_ = dh_params

    def transformation_matrix(self, theta):
        alpha = self.dh_params_[0]
        a = self.dh_params_[1]
        d = self.dh_params_[2]
        theta = theta+self.dh_params_[3]
        st = math.sin(theta)
        ct = math.cos(theta)
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        trans = np.array([[ct, -st, 0, a],
                          [st*ca, ct * ca, - sa, -sa * d],
                          [st*sa, ct * sa,   ca,  ca * d],
                          [0, 0, 0, 1]])
        return trans

    @staticmethod
    def basic_jacobian(trans, ee_pos):
        pos = np.array(
            [trans[0, 3], trans[1, 3], trans[2, 3]])
        z_axis = np.array(
            [trans[0, 2], trans[1, 2], trans[2, 2]])

        basic_jacobian = np.hstack(
            (np.cross(z_axis, ee_pos - pos), z_axis))
        return basic_jacobian


class NLinkArm:

    def __init__(self, dh_params_list) -> None:
        self.link_list = []
        for i in range(len(dh_params_list)):
            self.link_list.append(Link(dh_params_list[i]))

    def transformation_matrix(self, thetas):
        trans = np.identity(4)
        for i in range(len(self.link_list)):
            trans = np.dot(
                trans, self.link_list[i].transformation_matrix(thetas[i]))
        return trans

    def forward_kinematics(self, thetas):
        trans = self.transformation_matrix(thetas)
        x = trans[0, 3]
        y = trans[1, 3]
        z = trans[2, 3]
        
        alpha, beta, gamma = self.euler_angle(thetas)
        return [x, y, z, alpha, beta, gamma]

    def euler_angle(self, thetas):
        trans = self.transformation_matrix(thetas)

        alpha = math.atan2(trans[1][2], trans[0][2])
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) + math.pi
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) - math.pi
        beta = math.atan2(
            trans[0][2] * math.cos(alpha) + trans[1][2] * math.sin(alpha),
            trans[2][2])
        gamma = math.atan2(
            -trans[0][0] * math.sin(alpha) + trans[1][0] * math.cos(alpha),
            -trans[0][1] * math.sin(alpha) + trans[1][1] * math.cos(alpha))

        return alpha, beta, gamma

    def inverse_kinematics(self, ref_ee_pose):
        thetas = [0, 0, 0, 0, 0, 0]
        for cnt in range(500):
            ee_pose = self.forward_kinematics(thetas)
            diff_pose = np.array(ref_ee_pose) - ee_pose

            basic_jacobian_mat = self.basic_jacobian(thetas)
            alpha, beta, gamma = self.euler_angle(thetas)

            K_zyz = np.array(
                [[0, -math.sin(alpha), math.cos(alpha) * math.sin(beta)],
                 [0, math.cos(alpha), math.sin(alpha) * math.sin(beta)],
                 [1, 0, math.cos(beta)]])
            K_alpha = np.identity(6)
            K_alpha[3:, 3:] = K_zyz

            theta_dot = np.dot(
                np.dot(np.linalg.pinv(basic_jacobian_mat), K_alpha),
                np.array(diff_pose))
            thetas = thetas + theta_dot / 100.
        return thetas
    

    def basic_jacobian(self, thetas):
        ee_pos = self.forward_kinematics(thetas)[0:3]
        basic_jacobian_mat = []
        trans = np.identity(4)
        for i in range(len(self.link_list)):
            trans = np.dot(
                trans, self.link_list[i].transformation_matrix(thetas[i]))
            basic_jacobian_mat.append(
                self.link_list[i].basic_jacobian(trans, ee_pos))
        return np.array(basic_jacobian_mat).T

if __name__ == "__main__":
    rospy.init_node("jacobian_test")
    tool_pose_pub = rospy.Publisher("/tool_pose_cartesian",Point,queue_size=1)
    tool_velocity_pub = rospy.Publisher("/tool_velocity_cartesian",Point,queue_size=1)
    tool_force_pub = rospy.Publisher("/tool_force_cartesian",Point,queue_size=1)

    dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                               [math.pi/2, 0, 10/1000, 0+math.pi/2],
                               [math.pi, 280/1000, 0, 0+math.pi/2],
                               [math.pi/2, 0, 245/1000, 0+math.pi/2],
                               [math.pi/2, 0, 57/1000, 0],
                               [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
    gen3_lite = NLinkArm(dh_params_list)

    while not rospy.is_shutdown():
        feedback = rospy.wait_for_message("/my_gen3_lite/joint_states", JointState)
        thetas = feedback.position[0:6]
        velocities = feedback.velocity[0:6]
        torques = feedback.effort[0:6]

        tool_pose = gen3_lite.forward_kinematics(thetas)
        J = gen3_lite.basic_jacobian(thetas)
        tool_velocity = J.dot(velocities)
        tool_force = np.linalg.pinv(J.T).dot(torques)

        tool_pose_msg = Point()
        tool_pose_msg.x = tool_pose[0]
        tool_pose_msg.y = tool_pose[1]
        tool_pose_msg.z = tool_pose[2]

        tool_velocity_msg = Point()
        tool_velocity_msg.x = tool_velocity[0]
        tool_velocity_msg.y = tool_velocity[1]
        tool_velocity_msg.z = tool_velocity[2]

        tool_force_msg = Point()
        tool_force_msg.x = tool_force[0]
        tool_force_msg.y = tool_force[1]
        tool_force_msg.z = tool_force[2]

        tool_pose_pub.publish(tool_pose_msg)
        tool_velocity_pub.publish(tool_velocity_msg)
        tool_force_pub.publish(tool_force_msg)

        print(f"joint position: {thetas}")
        print(f"joint velocity: {velocities}")
        print(f"joint torque: {torques}")

        print(f"tool position: {tool_pose}")
        print(f"tool velocity: {tool_velocity}")
        print(f"tool torque: {tool_force}")