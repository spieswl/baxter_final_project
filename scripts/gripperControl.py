#!/usr/bin/env python


import rospy
import baxter_interface

from baxter_core_msgs.msg import EndEffectorState

from container_manipulator.srv import TrigDirective


class gripperControls():

    def __init__(self):

        # Publishers and Subscribers
        rospy.Subscriber('/robot/end_effector/left_gripper/state', EndEffectorState, self.cb_update_lgrip_state)
        rospy.Subscriber('/robot/end_effector/right_gripper/state', EndEffectorState, self.cb_update_rgrip_state)

        # Baxter Interface Components
        self.left_gripper = baxter_interface.Gripper('left')
        self.right_gripper = baxter_interface.Gripper('right')

        # Service Definitions
        rospy.Service('gripper_controller/close_grip', TrigDirective, self.srv_close_grip)
        rospy.Service('gripper_controller/open_grip', TrigDirective, self.srv_open_grip)

        # Local class data containers
        self.lgrip_state = EndEffectorState()
        self.rgrip_state = EndEffectorState()


    def cb_update_lgrip_state(self, data):
        '''
        Cache gripper state information for Baxter's left gripper.
        '''

        self.lgrip_state = data

        return


    def cb_update_rgrip_state(self, data):
        '''
        Cache gripper state information for Baxter's right gripper.
        '''

        self.rgrip_state = data

        return


    def srv_open_grip(self, data):
        '''
        '''

        if (data.side == 'left'):

            self.left_gripper.open()

            while True:
                if (self.lgrip_state.position > 90 and self.lgrip_state.moving != 1):
                    return (True, "GRIP - Left Gripper OPEN.")

                elif (self.lgrip_state.error == 1):
                    return (False, "GRIP - Left Gripper failed to reach OPEN state.")

        elif (data.side == 'right'):

            self.right_gripper.open()

            while True:
                if (self.rgrip_state.position > 90 and self.rgrip_state.moving != 1):
                    return (True, "GRIP - Right Gripper OPEN.")

                elif (self.rgrip_state.error == 1):
                    return (False, "GRIP - Right Gripper failed to reach OPEN state.")

        else:
            return (False, "GRIP - Please specify either 'left' or 'right' when triggering this action.")


    def srv_close_grip(self, data):
        '''
        '''

        if (data.side == 'left'):

            self.left_gripper.close()

            while True:
                if (self.lgrip_state.gripped == 1 and self.lgrip_state.moving != 1):
                    return (True, "GRIP - Left Gripper CLOSED to object.")

                elif (self.lgrip_state.missed == 1 and self.lgrip_state.moving != 1):
                    return (False, "GRIP - Left Gripper MISSED closing to object.")

                elif (self.lgrip_state.error == 1):
                    return (False, "GRIP - Left Gripper failed to reach CLOSED state.")

        elif (data.side == 'right'):

            self.right_gripper.close()

            while True:
                if (self.rgrip_state.gripped == 1 and self.rgrip_state.moving != 1):
                    return (True, "GRIP - Right Gripper CLOSED to object.")

                elif (self.rgrip_state.missed == 1 and self.rgrip_state.moving != 1):
                    return (False, "GRIP - Right Gripper MISSED closing to object.")

                elif (self.rgrip_state.error == 1):
                    return (False, "GRIP - Right Gripper failed to reach CLOSED state.")

        else:
            return (False, "GRIP - Please specify either 'left' or 'right' when triggering this action.")


# ========== #


def main():

    # Node initialization
    rospy.init_node('gripper_controller')

    # Class initialization
    gripperControls()

    while not rospy.is_shutdown():
        rospy.spin()

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("GRIP - Shutting down gripper controller node.")
        pass
