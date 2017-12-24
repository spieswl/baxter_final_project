#!/usr/bin/env python


import rospy

from ar_track_alvar_msgs.msg import (AlvarMarkers, AlvarMarker)
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)

from std_srvs.srv import Trigger


class poseHandler():

    def __init__(self):

        # Publishers and Subscribers
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.cb_register_obj_pose)
        self.object_pose = rospy.Publisher('object_poses', AlvarMarker, queue_size = 1)

        # Service Definitions
        self.update_obj_pose = rospy.Service('pose_relay/update_obj_pose', Trigger, self.svc_update_published_pose)

        # Static configuration variables
        self.update_rate = rospy.Rate(5)

        # Variables for latest subscribed poses and actively published poses
        self.last_obj_pose = []
        self.static_obj_pose = []


    def cb_register_obj_pose(self, data):
        '''
        This callback function reads the last set of markers data passed across the "ar_pose_marker" topic and stores
        the last marker(s) to be read.
        '''

        if (data.markers == []):
            pass
        else:
            self.last_obj_pose = data.markers

        return


    def svc_update_published_pose(self, data):
        '''
        This service acts on a trigger from the sequencer to update the static object pose being broadcast across the
        "object_pose" topic. Ideally, this 'locks' the marker position and orientation information to an unchanging
        value for stability purposes. This service can be triggered at any time.
        '''

        if (self.last_obj_pose == []):
            return (False, "POSE HANDLER - Update Failed. No AR Tags in view.")
        else:
            # Clear out old pose information before storing new poses
            self.static_obj_pose = []

            for index in range(len(self.last_obj_pose)):
                self.static_obj_pose.append(self.last_obj_pose[index])

            return (True, "POSE HANDLER - Update Complete.")

# ========== #


def main():

    # Node initialization
    rospy.init_node('pose_relay')

    # Class initialization
    pose_relay = poseHandler()

    while not rospy.is_shutdown():
        if len(pose_relay.static_obj_pose) != 0:
            for tag in range(len(pose_relay.static_obj_pose)):
                pose_relay.object_pose.publish(pose_relay.static_obj_pose[tag])
        else:
            pass

        # Publish new pose values at 5 Hz (provided poses have been 'captured')
        pose_relay.update_rate.sleep()

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass