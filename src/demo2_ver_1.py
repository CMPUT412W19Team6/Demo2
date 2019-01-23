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

# twist messages
forward_twist = Twist()
forward_twist.angular.z = 0
forward_twist.linear.x = 1

backward_twist = Twist()
backward_twist.angular.z = 0
backward_twist.linear.x = -1

left_twist = Twist()
left_twist.angular.z = 1
left_twist.linear.x = 0

right_twist = Twist()
right_twist.angular.z = -1
right_twist.linear.x = 0

rate = None

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

        elif userdata.previous_output == "go_forward":
            start_pos = position
            while not rospy.is_shutdown():
                diffs = [abs(position.x - start_pos.x), abs(position.y -
                                                            start_pos.y), abs(position.z - start_pos.z)]

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

        return "done"


class Backward(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=["finish"],
                             input_keys=["previous_output",
                                         "backward_distance"],
                             output_keys=["previous_state", "previous_output"]
                             )

    def execute(self, userdata):
        global twist_pub
        global backward_twist
        global rate
        global position

        if userdata.previous_output == "go_back":
            start_pos = position
            while not rospy.is_shutdown():
                diffs = [abs(position.x - start_pos.x), abs(position.y -
                                                            start_pos.y), abs(position.z - start_pos.z)]

                if max(diffs) > userdata.backward_distance:
                    userdata.previous_output = "finish"
                    userdata.previous_state = "BACKWARD"
                    return "finish"

                twist_pub.publish(backward_twist)
                rate.sleep()

        else:
            # you shouldn't be here
            return None


class Turn(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=["finish"],
                             input_keys=["previous_output", "turn_angle"],
                             output_keys=["previous_state", "previous_output"]
                             )

    def execute(self, userdata):
        global orientation
        global twist_pub
        global rate

        if userdata.previous_output == "go_turn":
            if userdata.turn_angle > 0:
                turn_twist = left_twist
            else:
                turn_twist = right_twist

            start_orientation = math.degrees(orientation[2])

            while not rospy.is_shutdown():
                turned = abs(math.degrees(orientation[2])-start_orientation)
                if turned > abs(userdata.turn_angle):
                    userdata.previous_output = "finish"
                    userdata.previous_state = "TURN"
                    return "finish"

                twist_pub.publish(turn_twist)
                rate.sleep()
        else:
            # you shouldn't be here
            print "in TURN, with wrong previous_output"
            return None


class Resolve(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=["finish", "go_forward",
                                       "go_back", "go_turn"],
                             input_keys=["previous_output",
                                         "previous_state", "bumper_index"],
                             output_keys=["previous_output",
                                          "forward_distance", "backward_distance", "turn_angle"]
                             )

        self.reset()

    def reset(self):
        self.current_round = 1
        self.default_turn_angle = 90
        self.default_moving_distance = 1
        self.current_task = None
        self.task_list = ["go_back", "turn_left", "go_forward_one", ]

    def execute(self, userdata):
        if userdata.previous_state == "FORWARD" and userdata.previous_output == "bump":
            if not self.current_task:
                self.current_task = "go_back"
                userdata.previous_output = "go_back"
                userdata.backward_distance = self.default_moving_distance
                return self.current_task

        # if userdata.previous_state == ""


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
    rate = rospy.Rate(10)
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
