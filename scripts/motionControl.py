#!/usr/bin/env python


import rospy
import baxter_interface
import math
import tf

from ar_track_alvar_msgs.msg import AlvarMarker
from baxter_core_msgs.msg import EndpointState
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
from container_manipulator.srv import (TrigDirective, TrigObjMove, TrigOffsetMove)
from std_srvs.srv import Trigger


class motionControls():

    def __init__(self):

        # Publishers and Subscribers
        rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self.cb_update_lhand_position)
        rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.cb_update_rhand_position)
        rospy.Subscriber('object_poses', AlvarMarker, self.cb_set_obj_positions) # TODO: MAY NEED TO BE CHANGED TO NEW MSG TYPE

        # Baxter interface components
        self.left_arm = baxter_interface.Limb('left')
        self.left_wrist = self.left_arm.joint_names()[6]
        self.right_arm = baxter_interface.Limb('right')
        self.right_wrist = self.right_arm.joint_names()[6]

        # Service Definitions
        rospy.Service('motion_controller/store_object_poses', Trigger, self.svc_store_object_poses)
        rospy.Service('motion_controller/move_to_object', TrigObjMove, self.svc_move_to_object)
        rospy.Service('motion_controller/move_to_offset', TrigOffsetMove, self.svc_move_to_offset)
        rospy.Service('motion_controller/twist_wrist', TrigDirective, self.svc_twist_wrist)

        # Local 'object' pose data containers & end-effector positions
        self.lid_pose = Pose()
        self.bottle_pose = Pose()
        self.table_pose = Pose()

        self.left_ee_point = Point()
        self.left_ee_orientation = Quaternion()
        self.right_ee_point = Point()
        self.right_ee_orientation = Quaternion()


    def cb_update_lhand_position(self, data):
        '''
        Cache pose information for Baxter's left end effector. Useful for offset moves based on current position.
        '''

        self.left_ee_point = data.pose.position
        self.left_ee_orientation = data.pose.orientation

        return


    def cb_update_rhand_position(self, data):
        '''
        Cache pose information for Baxter's right end effector. Useful for offset moves based on current position.
        '''

        self.right_ee_point = data.pose.position
        self.right_ee_orientation = data.pose.orientation

        return

# TODO

    def cb_set_obj_positions(self, data): # TODO: REVIEW THIS FUNCTION'S OPERATION
        '''
        Cache new pose values that are obtained from the detected object's AR tag, centroid position, or other data source.
        '''

        # Check the ID value for each object presented, update the related pose container
        if (data.id == 0):
            self.lid_pose = data.pose.pose
        elif (data.id == 5):
            self.bottle_pose = data.pose.pose
        elif (data.id == 8):
            self.table_pose = data.pose.pose

        return


    def svc_store_object_poses(self, data): # TODO: CLEAN UP THIS FUNCTION
        '''
        This service will copy the last stored AR tag value and cache it for some other purpose. This sequence assumes
        that the tag for the lid (while ON the bottle) is read, allowing for this function to store a location for where
        we assume the bottle to be.
        '''

        # Correct the orientation for the bottle from the cached marker orientation
        bottle_q_old = [self.lid_pose.orientation.x, self.lid_pose.orientation.y, self.lid_pose.orientation.z, self.lid_pose.orientation.w]
        bottle_q_rot = tf.transformations.quaternion_from_euler(0, math.pi, 0)
        bottle_q_new = tf.transformations.quaternion_multiply(bottle_q_rot, bottle_q_old)

        self.bottle_pose.position = self.lid_pose.position

        self.bottle_pose.orientation = Quaternion(
            x = bottle_q_new[0],
            y = bottle_q_new[1],
            z = bottle_q_new[2],
            w = bottle_q_new[3]
        )

        return (True, "MOTION CTRL - Bottle location cached.")


    def svc_move_to_object(self, data): # TODO: CLEAN UP THIS FUNCTION
        '''
        This service function takes the cached pose information for the lid's AR marker, calls for an IK solution that
        brings the designated end effector into alignment with the lid, and executes a move to the calculated joint
        positions.
        '''

        # Establish connection to specific limb's IKSolver service
        ns = '/ExternalTools/' + self.limb + '/PositionKinematicsNode/IKService'
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Update header information based on current time and with reference to base frame
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        # Update orientation of gripper based on rotation of AR tag frame to a frame compatible with Baxter's EE state
        tag_q_old = [self.lid_pose.orientation.x, self.lid_pose.orientation.y, self.lid_pose.orientation.z, self.lid_pose.orientation.w]
        tag_q_rot = tf.transformations.quaternion_from_euler(0, math.pi, 0)
        tag_q_new = tf.transformations.quaternion_multiply(tag_q_rot, tag_q_old)

        # Locally relevant AR tag derived poses (typ. for marker, bottle, or table)
        AR_coords = PoseStamped(
            header = hdr,
            pose = Pose(
                position = self.lid_pose.position,
                orientation = Quaternion(
                    x = tag_q_new[0],
                    y = tag_q_new[1],
                    z = tag_q_new[2],
                    w = tag_q_new[3]
                )
            )
        )

        # Set the desired pose in the service request message to pose information pulled from the object pose topic
        ikreq.pose_stamp.append(AR_coords)

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
            rospy.loginfo("MOTION CTRL - WARNING! Moving Baxter's " + self.limb + " arm to lid pounce position.")

            if (self.limb == 'left'):
                left = baxter_interface.Limb('left')
                left.move_to_joint_positions(limb_joints)
            elif (self.limb == 'right'):
                right = baxter_interface.Limb('right')
                right.move_to_joint_positions(limb_joints)

            return (True, "MOTION CTRL - Move to lid pounce position complete.")
        else:
            return (False, "MOTION CTRL - Motion planner failure.")


    def svc_move_to_bottle(self, data): # TODO: REMOVE THIS FUNCTION
        '''
        This service function takes the cached pose information for the bottle, based on the original AR marker's pose,
        calls for an IK solution that brings the designated end effector into position, and executes a move to the
        calculated joint positions.
        '''

        # Establish connection to specific limb's IKSolver service
        ns = '/ExternalTools/' + self.limb + '/PositionKinematicsNode/IKService'
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Update header information based on current time and with reference to base frame
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        bottle_coords = PoseStamped(
            header = hdr,
            pose = Pose(
                position = self.bottle_pose.position,
                orientation = self.bottle_pose.orientation
            )
        )


        # Set the desired pose in the service request message to pose information pulled from the object pose topic
        ikreq.pose_stamp.append(bottle_coords)

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
            rospy.loginfo("MOTION CTRL - WARNING! Moving Baxter's " + self.limb + " arm to bottle pounce position.")

            if (self.limb == 'left'):
                left = baxter_interface.Limb('left')
                left.move_to_joint_positions(limb_joints)
            elif (self.limb == 'right'):
                right = baxter_interface.Limb('right')
                right.move_to_joint_positions(limb_joints)

            return (True, "MOTION CTRL - Move to bottle pounce position complete.")
        else:
            return (False, "MOTION CTRL - Motion planner failure.")



    def svc_move_to_offset(self, data): # TODO: CLEAN UP THIS FUNCTION
        '''
        This service function takes pose information from data contained in a service request passed from the main
        sequencer and generates a new desired end-effector pose. That pose is sent as the target pose for the purpose
        of generating a new IK solution. This service will then trigger a move to the calculated joint positions.
        '''

        # Establish connection to specific limb's IKSolver service
        ns = '/ExternalTools/' + data.side + '/PositionKinematicsNode/IKService'
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Update header information based on current time and with reference to base frame
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        # Add input offsets to currently stored end-effector positions
        left_offset = PoseStamped(
            header = hdr,
            pose = Pose(
                position = Point(
                    x = data.offset.position.x + self.left_ee_point.x,
                    y = data.offset.position.y + self.left_ee_point.y,
                    z = data.offset.position.z + self.left_ee_point.z
                ),
                orientation = Quaternion(
                    x = data.offset.orientation.x + self.left_ee_orientation.x,
                    y = data.offset.orientation.y + self.left_ee_orientation.y,
                    z = data.offset.orientation.z + self.left_ee_orientation.z,
                    w = data.offset.orientation.w + self.left_ee_orientation.w
                )
            )
        )

        right_offset = PoseStamped(
            header = hdr,
            pose = Pose(
                position = Point(
                    x = data.offset.position.x + self.right_ee_point.x,
                    y = data.offset.position.y + self.right_ee_point.y,
                    z = data.offset.position.z + self.right_ee_point.z
                ),
                orientation = Quaternion(
                    x = data.offset.orientation.x + self.right_ee_orientation.x,
                    y = data.offset.orientation.y + self.right_ee_orientation.y,
                    z = data.offset.orientation.z + self.right_ee_orientation.z,
                    w = data.offset.orientation.w + self.right_ee_orientation.w
                )
            )
        )

        offset_poses = { 'left' : left_offset,
                         'right': right_offset }

        # Set the desired pose in the service request message to pose information pulled from the object pose topic
        ikreq.pose_stamp.append(offset_poses[data.side])

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


    def svc_twist_wrist(self, data): # TODO: FINISH THIS FUNCTION

        if (data.side == 'left'):

            if (data.directive == 'CCW'):

            # DO SOMETHING
            self.left_arm.move_to_joint_positions({self.left_wrist: 3})

        elif (data.side == 'right'):

            # DO SOMETHING
            self.right_arm.move_to_joint_positions({self.right_wrist: 3})

        else:
            return (False, "MOTION CTRL - Please specify either 'left' or 'right' when triggering this action.")

# END TODO

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
