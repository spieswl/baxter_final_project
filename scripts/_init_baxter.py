#!/usr/bin/env python


import rospy
import baxter_interface

from baxter_interface import CHECK_VERSION
from baxter_interface.camera import CameraController

from std_msgs.msg import Header
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
from std_srvs.srv import Empty


class BaxterCtrls():

    def __init__(self):

        # Initialize actions of note
        self.cmds = baxter_interface.RobotEnable(CHECK_VERSION)

        # Reset the cameras and establish camera controller connections
        self.reset_srv = rospy.ServiceProxy('cameras/reset', Empty)
        rospy.wait_for_service('cameras/reset', timeout = 10)
        self.reset_srv()

        # Home position definitions - Determined by moving Baxter to safe positions
        self.hdr = Header
        self.home_arm_poses = {
            'left': Pose(
                position = Point(
                    x = 0.76053,
                    y = 0.30539,
                    z = 0.27095
                ),
                orientation = Quaternion(
                    x = 0.01397,
                    y = 0.99479,
                    z = -0.05797,
                    w = 0.08265
                )
            ),
            'right': Pose(
                position = Point(
                    x = 0.52721,
                    y = -0.86806,
                    z = -0.09085
                ),
                orientation = Quaternion(
                    x = 0.31333,
                    y = 0.94930,
                    z = 0.00165,
                    w = 0.02574
                )
            )
        }


    def enable_baxter(self):

        # Print Info message detailing that Baxter is now enabled (unless there are other issues)
        rospy.loginfo("INIT - Issuing 'Enable' command to Baxter.")

        # Issue enabling command to Baxter
        # Kick out error message if an exception is encountered
        try:
            self.cmds.enable()
        except Exception, e:
            rospy.logerr(e.strerror)

        return


    def disable_baxter(self):

        # Print Info message detailing that Baxter is now disabled
        rospy.loginfo("INIT - Issuing 'Disable' command to Baxter.")

        # Issue disabling command to Baxter
        # Kick out error message if an exception is encountered
        try:
            self.cmds.disable()
        except Exception, e:
            rospy.logerr(e.strerror)

        return


    def calibrate_grippers(self):

        # Loop through available grippers and calibrate each
        for side in ['left', 'right']:
            rospy.loginfo("INIT - Calibrating " + side + " gripper.")
            side = baxter_interface.Gripper(side)
            side.calibrate()

        return


    def camera_setup_head_LH(self):

        # Print Info message describing Camera actions
        rospy.loginfo("Enabling 'head' and 'left_hand' cameras w/ output image resolution of 1280x800.")

        # Start camera setup after camera/list service is available
        rospy.wait_for_service('cameras/list', timeout = 10)

        head_cam = CameraController('head_camera')
        lh_cam = CameraController('left_hand_camera')

        # Open "head" camera with 1280x800 resolution
        head_cam.resolution = (1280,800)
        head_cam.open()

        # Open "left_hand" camera with 1280x800 resolution
        lh_cam.resolution = (1280,800)
        lh_cam.open()

        return


    def move_arm_to_home(self, limb):

        # Set up IK Solver service connection
        ns = '/ExternalTools/' + limb + '/PositionKinematicsNode/IKService'
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Build header information
        self.hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        # Assemble pose request based on "limb" argument passed to function
        self.pose_requested = PoseStamped(
            header = self.hdr,
            pose = self.home_arm_poses[limb]
        )

        ikreq.pose_stamp.append(self.pose_requested)

        # Call the IK Solver Request with the assembled pose information
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("IK SOLVER - Request service call failed: %s" % (e,))
            return 1

        # If a solution is found, initiate motion to that position
        if (resp.isValid[0]):
            rospy.loginfo("IK SOLVER - Success! Valid joint solution found!")

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

            # Print Info message to alert users of impending motion
            rospy.loginfo("INIT - WARNING! Moving Baxter's " + limb + " arm to home position.")

            if (limb == 'left'):
                left = baxter_interface.Limb('left')
                left.move_to_joint_positions(limb_joints)
            elif (limb == 'right'):
                right = baxter_interface.Limb('right')
                right.move_to_joint_positions(limb_joints)
            else:
                pass
        else:
            rospy.loginfo("IK SOLVER - Invalid pose - No valid joint solution found.")

        return

    def set_display_image(self, imagePath):

        # PLACEHOLDER FOR NOW

        return