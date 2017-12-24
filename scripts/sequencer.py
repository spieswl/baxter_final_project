#!/usr/bin/env python


import rospy

import _init_baxter

from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)

from container_manipulator.srv import (TrigDirective, TrigObjMove, TrigOffsetMove)
from std_srvs.srv import Trigger



def main():
    '''
    This script acts solely in a state machine-like manner. The sequence is set up ahead of time, and leverages several
    defined services to command Baxter to execute the desired behavior. Each of the separate service providers have
    various topics they subscribe or publish to in order to accomplish their directed goals.

    Change the sequence as desired (inside the execution loop) to get the intended effect.
    '''

    ##################
    # INITIALIZATION #
    ##################
    # Start master controller ('director') node
    rospy.init_node('director')

    # Service initializations (IMPORTANT!!!)
    update_obj_pose = rospy.ServiceProxy('pose_relay/update_obj_pose', Trigger)
    rospy.wait_for_service('pose_relay/update_obj_pose', 3.0)

    store_bottle_pose = rospy.ServiceProxy('motion_controller/store_bottle_pose', Trigger)
    move_to_AR_tag = rospy.ServiceProxy('motion_controller/move_to_AR_tag', Trigger)
    move_to_offset = rospy.ServiceProxy('motion_controller/move_to_offset', OffsetMove)
    move_to_bottle = rospy.ServiceProxy('motion_controller/move_to_bottle', Trigger)
    rospy.wait_for_service('motion_controller/move_to_bottle', 3.0)

    close_grip = rospy.ServiceProxy('gripper_controller/close_grip', Trigger)
    open_grip = rospy.ServiceProxy('gripper_controller/open_grip', Trigger)
    unscrew_lid = rospy.ServiceProxy('gripper_controller/unscrew_lid', Trigger)
    screw_lid = rospy.ServiceProxy('gripper_controller/screw_lid', Trigger)
    rospy.wait_for_service('gripper_controller/screw_lid', 3.0)

    # Stored offset pose information
    os_down1 = Pose(
        position = Point(
            x = 0.01,
            y = 0.0,
            z = -0.06
        ),
        orientation = Quaternion()
    )

    os_up1 = Pose(
        position = Point(
            x = 0.0,
            y = 0.0,
            z = 0.20
        ),
        orientation = Quaternion()
    )

    os_left = Pose(
        position = Point(
            x = 0.0,
            y = 0.30,
            z = 0.0
        ),
        orientation = Quaternion()
    )

    os_up2 = Pose(
        position=Point(
            x = 0.0,
            y = 0.0,
            z = 0.30
        ),
        orientation = Quaternion()
    )

    os_down2 = Pose(
        position = Point(
            x = 0.0,
            y = 0.0,
            z = -0.40
        ),
        orientation = Quaternion()
    )

    ##################
    # EXECUTION LOOP #
    ##################
    # Initialize Baxter to nominal state
    baxCtrl = _init_baxter.BaxterCtrls()

    baxCtrl.enable_baxter()
    baxCtrl.camera_setup_head_LH()
    baxCtrl.calibrate_grippers()

    # Move Baxter's arms to home
    baxCtrl.move_arm_to_home('left')
    baxCtrl.move_arm_to_home('right')

    # Display info message communicating initialization state
    rospy.loginfo("Baxter initialization complete.")

    # Main process loop
    while (True):

        # Update the published position of the lid
        update_obj_pose()
        rospy.sleep(1)
        store_bottle_pose()
        rospy.sleep(1)

        # Move to the pounce position over the detected AR tag
        move_to_AR_tag()

        # Extra motion to get the lid between the fingers
        move_to_offset(os_down1)

        # Unscrewing lid subsequence
        unscrew_lid()

        # Series of motions off to the side of the bottle
        move_to_offset(os_up1)
        move_to_offset(os_left)
        move_to_offset(os_down2)

        # Opening the gripper to place the cup on the table
        open_grip()
        rospy.sleep(1)

        # Move Baxter's left arm a raised position (good view of the table area)
        move_to_offset(os_up2)

        # Find the lid after it has been set (and moved!) on the table
        update_obj_pose()
        rospy.sleep(1)

        # Move to the pounce position over the detected AR tag
        move_to_AR_tag()

        # Extra motion to get the lid between the fingers
        move_to_offset(os_down1)

        # Close the grippers
        close_grip()
        rospy.sleep(1)

        # Move up and over to stored bottle location
        move_to_offset(os_up2)
        move_to_bottle()
        rospy.sleep(1)

        # Extra motion to seat the lid in the threads on the bottles
        move_to_offset(os_down1)

        # Screw lid down subsequence
        screw_lid()

        # Release the lid
        open_grip()
        rospy.sleep(1)

        # Move clear of the bottle.
        move_to_offset(os_up2)
        rospy.sleep(1)

        rospy.loginfo("Sequence complete.")

        break

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
