#!/usr/bin/env python

import rospy
import smach
import math
import tf
import smach_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent

# publishers
twist_pub = None

forward_twist = Twist()
forward_twist.angular.z = 0
forward_twist.linear.x = 1

rate = rospy.Rate(10)

position = None
orientation = None

bumper_index = None
is_bumped = False


class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=["done", "bump", "finish"],
                             input_keys=["previous_output",
                                         "forward_distance"],
                             output_keys=["bumper_index",
                                          "previous_output", "previous_state"]
                             )

    def execute(self, userdata):
        global twist_pub
        global forward_twist
        global rate
        global position
        global orientation
        global bumper_index
        global is_bumped
        DISTANCE_TO_GOAL = 3

        if not userdata.previous_output or userdata.previous_output == "finish":
            # continue the journey without any problems

            while not rospy.is_shutdown():
                if position:
                    # we reached the goal!! Yaaayyyy
                    if position.x > DISTANCE_TO_GOAL:
                        return 'done'

                if is_bumped:
                    userdata.bumper_index = bumper_index
                    userdata.previous_output = "bump"
                    userdata.previous_state = "FORWARD"
                    return "bump"

                twist_pub.publish(forward_twist)
                rate.sleep()

        if userdata.previous_output == "go_forward":
            start_pos = position
            while not rospy.is_shutdown():
                diffs = [position.x - start_pos.x, position.y -
                         start_pos.y, position.z - start_pos.z]

                if max(diffs) > userdata.forward_distance:
                    userdata.previous_output = "finish"
                    userdata.previous_state = "FORWARD"
                    return "finish"

                if is_bumped:
                    userdata.bumper_index = bumper_index
                    userdata.previous_output = "bump"
                    userdata.previous_state = "FORWARD"
                    return "bump"

                twist_pub.publish(forward_twist)
                rate.sleep()


class Backward(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=["bump", "finish"],
                             input_keys=["previous_output"],
                             output_keys=["previous_state"]
                             )

    def execute(self, userdata):
        pass


class Turn(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=["finish"],
                             input_keys=["previous_output"],
                             output_keys=["previous_state"]
                             )

    def execute(self, userdata):
        pass


class Resolve(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=["finish", "go_forward",
                                       "go_back", "go_turn"],
                             input_keys=["previous_output",
                                         "previous_state", "bumper_index"],
                             )

    def execute(self, userdata):
        pass


def odom_callback(data):
    global position
    global orientation

    position = data.pose.pose.position
    orientation_quat = data.pose.pose.orientation
    orientation = tf.transformations.euler_from_quaternion(
        [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])


def bump_callback(data):
    global bumper_index
    global is_bumped

    is_bumped = data.state == data.PRESSED
    bumper_index = data.bumper


if __name__ == "__main__":
    rospy.init_node("demo2part2")

    sm = smach.StateMachine(outcomes=["exit"])
    sm.userdata.previous_output = None

    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.Subscriber('mobile_base/events/bumper', BumperEvent, bump_callback)

    with sm:
        smach.StateMachine.add("FORWARD", Forward(), transitions={
            "done": "exit",
            "bump": "RESOLVE",
            "finish": "RESOLVE"
        })

        smach.StateMachine.add("BACKWARD", Backward(), transitions={
            "bump": "RESOLVE",
            "finish": "RESOLVE"
        })

        smach.StateMachine.add("TURN", Turn(), transitions={
            "finish": "RESOLVE"
        })

        smach.StateMachine.add("RESOLVE", Resolve(), transitions={
            "finish": "FORWARD", "go_forward": "FORWARD", "go_back": "BACKWARD", "go_turn": "TURN"
        })

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()
