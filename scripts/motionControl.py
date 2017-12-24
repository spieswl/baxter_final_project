#!/usr/bin/env python


import rospy
import baxter_interface
import math
import numpy as np
import tf

from ar_track_alvar_msgs.msg import AlvarMarker
from baxter_core_msgs.msg import EndpointState
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
from container_manipulator.srv import (TrigDirective, TrigObjMove, TrigOffsetMove)


class motionControls():

    def __init__(self):

        # Publishers and Subscribers
        rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self.cb_update_lfarm_state)
        rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.cb_update_rtarm_state)
        rospy.Subscriber('/robot/joint_states', JointState, self.cb_update_wrist_states)
        rospy.Subscriber('object_poses', AlvarMarker, self.cb_set_obj_positions) # TODO: MAY NEED TO BE CHANGED TO NEW MSG TYPE

        # Baxter interface components
        self.left_arm = baxter_interface.Limb('left')
        self.left_wrist = self.left_arm.joint_names()[6]
        self.right_arm = baxter_interface.Limb('right')
        self.right_wrist = self.right_arm.joint_names()[6]

        # Service Definitions
        rospy.Service('motion_controller/move_to_object', TrigObjMove, self.svc_move_to_object)
        rospy.Service('motion_controller/move_to_offset', TrigOffsetMove, self.svc_move_to_offset)
        rospy.Service('motion_controller/twist_wrist', TrigDirective, self.svc_twist_wrist)

        # Local 'object' pose data containers, and joint and arm positions
        self.lid_pose = Pose()
        self.bottle_pose = Pose()
        self.table_pose = Pose()

        self.left_ee_pose = Pose()
        self.right_ee_pose = Pose()

        self.left_wrist_pos = np.float64(0.0)
        self.right_wrist_pos = np.float64(0.0)


    def cb_update_lfarm_state(self, data):

        self.left_ee_pose = data.pose

        return


    def cb_update_rtarm_state(self, data):

        self.right_ee_pose = data.pose

        return


    def cb_update_wrist_states(self, data):

        for joint in range(len(data.name)):

            if (data.name[joint] == 'left_w2'):
                self.left_wrist_state = data.position[joint]

            elif (data.name[joint] == 'right_w2'):
                self.right_wrist_state = data.position[joint]

        return


    def cb_set_obj_positions(self, data):
        '''
        Cache new pose values that are obtained from the detected object's AR tag, centroid position, or other data source.
        '''

        # Check the ID value for each object presented, update the related pose information (with rotations applied as necessary)
        if (data.id == 0):      # Lid object ID

            # Initial assignment of object pose
            self.lid_pose = data.pose.pose

            # Update desired orientation of gripper based on rotation of object frame to an orientation identical to Baxter's EE state
            lid_q_old = [self.lid_pose.orientation.x, self.lid_pose.orientation.y, self.lid_pose.orientation.z, self.lid_pose.orientation.w]
            lid_q_rot = tf.transformations.quaternion_from_euler(0, math.pi, 0)
            lid_q_new = tf.transformations.quaternion_multiply(lid_q_rot, lid_q_old)

            self.lid_pose.orientation = lid_q_new

        elif (data.id == 5):    # Bottle object ID

            # Initial assignment of object pose
            self.bottle_pose = data.pose.pose

            # Update desired orientation of gripper based on rotation of object frame to an orientation identical to Baxter's EE state
            bot_q_old = [self.bottle_pose.orientation.x, self.bottle_pose.orientation.y, self.bottle_pose.orientation.z,
                         self.bottle_pose.orientation.w]
            bot_q_rot = tf.transformations.quaternion_from_euler(0, math.pi, 0)
            bot_q_new = tf.transformations.quaternion_multiply(bot_q_rot, bot_q_old)

            self.bottle_pose.orientation = bot_q_new

        elif (data.id == 8):    # Table object ID

            # Initial assignment of object pose
            self.table_pose = data.pose.pose

            # Update desired orientation of gripper based on rotation of object frame to an orientation identical to Baxter's EE state
            table_q_old = [self.table_pose.orientation.x, self.table_pose.orientation.y, self.table_pose.orientation.z,
                           self.table_pose.orientation.w]
            table_q_rot = tf.transformations.quaternion_from_euler(0, math.pi, 0)
            table_q_new = tf.transformations.quaternion_multiply(table_q_rot, table_q_old)

            self.table_pose.orientation = table_q_new

        return


    def svc_move_to_object(self, data):
        '''
        '''

        # Local scope data containers
        object_pose = Pose()

        # Get the pose information for the object Baxter is about to move towards
        if (data.object_id == 0):
            rospy.loginfo("MOTION CTRL - Getting location information for container lid.")
            object_pose = self.lid_pose

        elif (data.object_id == 5):
            rospy.loginfo("MOTION CTRL - Getting location information for container body.")
            object_pose = self.bottle_pose

        elif (data.object_id == 8):
            rospy.loginfo("MOTION CTRL - Getting location information for table.")
            object_pose = self.table_pose

        else:
            return (False, "MOTION CTRL - Invalid object specified in motion service request.")

        # Establish connection to specific limb's IKSolver service
        ns = '/ExternalTools/' + data.side + '/PositionKinematicsNode/IKService'
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Update header information based on current time and with reference to base frame
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        # Object-specific poses (typ. for marker, bottle, or table) plugged into new PoseStamped data type
        obj_coords = PoseStamped(header = hdr, pose = object_pose)

        # Set the desired pose in the service request message
        ikreq.pose_stamp.append(obj_coords)

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call fault: %s" % (e,))
            return (False, "MOTION CTRL - Service call to Baxter's IK solver failed.")

        if (resp.isValid[0]):
            rospy.loginfo("IK SOLVER - Success! Valid joint solution found.")

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

            # Print Info message to alert users of impending motion
            rospy.loginfo("MOTION CTRL - WARNING! Moving Baxter's " + data.side + " arm to object position.")

            arm = baxter_interface.Limb(data.side)
            arm.move_to_joint_positions(limb_joints)

            return (True, "MOTION CTRL - Move to object position complete.")

        else:
            return (False, "MOTION CTRL - Motion planner failure.")


    def svc_move_to_offset(self, data):
        '''
        This service function takes pose information from data contained in a service request passed from the main
        sequencer and generates a new desired end-effector pose. That pose is sent as the target pose for the purpose
        of generating a new IK solution. This service will then trigger a move to the calculated joint positions.
        '''

        # Local scope data containers
        arm_pose = Pose()

        # Get the pose information for the specified robot arm
        if (data.side == 'left'):
            rospy.loginfo("MOTION CTRL - Getting endpoint state information for left arm.")
            arm_pose = self.left_ee_pose

        elif (data.side == 'right'):
            rospy.loginfo("MOTION CTRL - Getting endpoint state information for right arm.")
            arm_pose = self.right_ee_pose

        else:
            return (False, "MOTION CTRL - Invalid side specified in motion service request.")

        # Establish connection to specific limb's IKSolver service
        ns = '/ExternalTools/' + data.side + '/PositionKinematicsNode/IKService'
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Update header information based on current time and with reference to base frame
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        # Add offsets to currently stored end-effector position and orientation
        offset_pose = PoseStamped(
            header = hdr,
            pose = Pose(
                position = Point(
                    x = data.offset.position.x + arm_pose.position.x,
                    y = data.offset.position.y + arm_pose.position.y,
                    z = data.offset.position.z + arm_pose.position.z
                ),
                orientation = Quaternion(
                    x = data.offset.orientation.x + arm_pose.orientation.x,
                    y = data.offset.orientation.y + arm_pose.orientation.y,
                    z = data.offset.orientation.z + arm_pose.orientation.z,
                    w = data.offset.orientation.w + arm_pose.orientation.w
                )
            )
        )

        # Set the desired pose in the service request message to pose information pulled from the object pose topic
        ikreq.pose_stamp.append(offset_pose)

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call fault: %s" % (e,))
            return (False, "MOTION CTRL - Service call to Baxter's IK solver failed.")

        if (resp.isValid[0]):
            rospy.loginfo("IK SOLVER - Success! Valid joint solution found.")

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

            # Print Info message to alert users of impending motion
            rospy.loginfo("MOTION CTRL - WARNING! Moving Baxter's " + data.side + " arm to offset position.")

            arm = baxter_interface.Limb(data.side)
            arm.move_to_joint_positions(limb_joints)

            return (True, "MOTION CTRL - Move to offset position complete.")
        else:
            return (False, "MOTION CTRL - Motion planner failure.")


    def svc_twist_wrist(self, data):

        if (data.side == 'left'):

            if (data.directive == 'CW'):
                if (self.left_wrist_pos < 3.0):
                    self.left_arm.move_to_joint_positions({self.left_wrist: 3})
                return (True, "MOTION CTRL - Left wrist turned fully clockwise.")

            elif (data.directive == 'CCW'):
                if (self.left_wrist_pos > -3.0):
                    self.left_arm.move_to_joint_positions({self.left_wrist: -3})
                return (True, "MOTION CTRL - Left wrist turned fully counterclockwise.")

            elif (data.directive == 'Home'):
                self.left_arm.move_to_joint_positions({self.left_wrist: 0})
                return (True, "MOTION CTRL - Left wrist returned to home position.")

            else:
                return (False, "MOTION CTRL - Please specify either 'CW', 'CCW', or 'Home' as wrist directives.")

        elif (data.side == 'right'):
            if (data.directive == 'CW'):
                if (self.right_wrist_pos < 3.0):
                    self.right_arm.move_to_joint_positions({self.right_wrist: 3})
                return (True, "MOTION CTRL - Right wrist turned fully clockwise.")

            elif (data.directive == 'CCW'):
                if (self.right_wrist_pos > -3.0):
                    self.right_arm.move_to_joint_positions({self.right_wrist: -3})
                return (True, "MOTION CTRL - Right wrist turned fully counterclockwise.")

            elif (data.directive == 'Home'):
                self.right_arm.move_to_joint_positions({self.right_wrist: 0})
                return (True, "MOTION CTRL - Right wrist returned to home position.")

            else:
                return (False, "MOTION CTRL - Please specify either 'CW', 'CCW', or 'Home' as wrist directives.")

        else:
            return (False, "MOTION CTRL - Please specify either 'left' or 'right' when triggering this action.")


# ========== #


def main():

    # Node initialization
    rospy.init_node('motion_controller')

    # Class initialization
    motionControls()

    while not rospy.is_shutdown():
        rospy.spin()

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("MOTION CTRL - Shutting down motion controller node.")
        pass
