#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from hippocampus_common.node import Node


class JoyMapperNode(Node):


    def __init__(self, name):
        self.left_analogstick_right = 0
        self.left_analogstick_up = 1
        self.left_trigger = 2
        self.right_analogstick_right = 3
        self.right_analogstick_up = 4
        self.right_trigger = 5
        self.cross_right = 6
        self.cross_up = 7

        self.but_A = 0
        self.but_B = 1
        self.but_X = 2
        self.but_Y = 3
        self.but_LB = 4
        self.but_RB = 5
        self.but_back = 6
        self.but_start = 7
        self.but_logitech = 8
        self.but_left_stick = 9
        self.but_right_stick = 10

        self.roll_stab = 0
        self.pitch_stab = 0
        self.yaw_stab = 0
        self.thrust_stab = 0
        self.lateral_thrust_stab = 0
        self.vertical_thrust_stab = 0

        self.yaw_pad = 0
        self.thrust_pad = 0
        self.vertical_thrust_pad = 0
        self.lateral_thrust_pad = 0
        self.speed = True
        self.manual_mode = 0

        # factor how strong the input is linear between 0 and 1
        self.factor = 1
        self.factor_thrust = self.factor
        self.factor_vertical_thrust = self.factor
        self.factor_lateral_thrust = self.factor
        self.factor_roll = self.factor
        self.factor_pitch = self.factor
        self.factor_yaw = self.factor

        super(JoyMapperNode, self).__init__(name)

        rospy.logwarn("hello")
        self.mapping = self.read_params()

        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust", Float64, queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust", Float64, queue_size=1)
        self.thrust_pub = rospy.Publisher("thrust", Float64, queue_size=1)
        self.roll_pub = rospy.Publisher("roll", Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher("pitch", Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher("yaw", Float64, queue_size=1)
        self.manual_pub = rospy.Publisher("manual_mode", Float64, queue_size=1)
        self.gripper_pub = rospy.Publisher("gripper_cmd", Float64, queue_size=1)

        rospy.Subscriber("joy", Joy, self.on_joy, queue_size=10)

        self.vertical_thrust_sub = rospy.Subscriber("vertical_thrust_stab", Float64, self.on_vertical_thrust_stab, queue_size=1)
        self.lateral_thrust_sub = rospy.Subscriber("lateral_thrust_stab", Float64, self.on_lateral_thrust_stab, queue_size=1)
        self.thrust_sub = rospy.Subscriber("thrust_stab", Float64, self.on_thrust_stab, queue_size=1)
        self.roll_sub = rospy.Subscriber("roll_stab", Float64, self.on_roll_stab, queue_size=1)
        self.pitch_sub = rospy.Subscriber("pitch_stab", Float64, self.on_pitch_stab, queue_size=1)
        self.yaw_sub = rospy.Subscriber("yaw_stab", Float64, self.on_yaw_stab, queue_size=1)

    def read_params(self):
        mapping = {}
        mapping["thrust"] = self.get_param("thrust", self.left_analogstick_up)
        mapping["lateral_thrust"] = self.get_param("lateral", self.right_analogstick_right)
        mapping["vertical_thrust"] = self.get_param("vertical", self.right_analogstick_up)
        mapping["yaw"] = self.get_param("yaw", self.left_analogstick_right)
        return mapping

    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            self.roll_cmd = self.roll_stab
            self.roll_pub.publish(self.roll_cmd)

            self.pitch_cmd = self.pitch_stab
            self.pitch_pub.publish(self.pitch_cmd)

            self.yaw_cmd = self.yaw_pad + self.yaw_stab
            self.yaw_pub.publish(self.yaw_cmd)

            self.thrust_cmd = self.thrust_pad + self.thrust_stab
            self.thrust_pub.publish(self.thrust_cmd)

            self.vertical_thrust_cmd = self.vertical_thrust_pad + self.vertical_thrust_stab
            self.vertical_thrust_pub.publish(self.vertical_thrust_cmd)

            self.lateral_thrust_cmd = self.lateral_thrust_pad + self.lateral_thrust_stab
            self.lateral_thrust_pub.publish(self.lateral_thrust_cmd)    
            rate.sleep()


    def on_joy(self, msg):
        #for axis in self.mapping:
        #    msg_out = Float64()
        #    pub = self.get_pub(axis)
        #    factor = self.get_factor(axis)
        #    index = self.mapping[axis]
        #    msg_out.data = msg.axes[index]*factor
        #    pub.publish(msg_out)

        self.yaw_pad = msg.axes[self.left_analogstick_right]*self.factor_yaw
        self.thrust_pad = msg.axes[self.left_analogstick_up]*self.factor_thrust
        self.vertical_thrust_pad = msg.axes[self.right_analogstick_up]*self.factor_vertical_thrust
        self.lateral_thrust_pad = msg.axes[self.right_analogstick_right]*self.factor_lateral_thrust

        if msg.buttons[self.but_A]:
            if self.speed == True:
                self.set_factor(0.5)
                self.speed = False
            else:
                self.set_factor(1)
                self.speed = True

        if msg.buttons[self.but_LB] == msg.buttons[self.but_RB]:
            self.gripper_pub.publish(0)
        elif msg.buttons[self.but_LB]:
            self.gripper_pub.publish(-1)
        elif msg.buttons[self.but_RB]:
            self.gripper_pub.publish(1)

        if msg.buttons[self.but_Y]:
            if self.manual_mode == 4:
                self.manual_mode = 0
            else:
                self.manual_mode = self.manual_mode + 1
            print("mode: ", self.manual_mode)
            self.manual_pub.publish(self.manual_mode)


    def set_factor(self,msg):
                # factor how strong the input is linear between 0 and 1
        self.factor_thrust = msg
        self.factor_vertical_thrust = msg
        self.factor_lateral_thrust = msg
        self.factor_roll = 0
        self.factor_pitch = 0
        self.factor_yaw = msg * 0.5

    def get_pub(self, axis):
        if axis == "thrust":
            return self.thrust_pub
        elif axis == "vertical_thrust":
            return self.vertical_thrust_pub
        elif axis == "lateral_thrust":
            return self.lateral_thrust_pub
        elif axis == "roll":
            return self.roll_pub
        elif axis == "pitch":
            return self.pitch_pub
        elif axis == "yaw":
            return self.yaw_pub
        else:
            return None
   
    def get_factor(self, axis):
        if axis == "thrust":
            return self.factor_thrust
        elif axis == "vertical_thrust":
            return self.factor_vertical_thrust
        elif axis == "lateral_thrust":
            return self.factor_lateral_thrust
        elif axis == "roll":
            return self.factor_roll
        elif axis == "pitch":
            return self.factor_pitch
        elif axis == "yaw":
            return self.factor_yaw
        else:
            return None

    def on_lateral_thrust_stab(self, msg):
        self.lateral_thrust_stab = msg.data

    def on_vertical_thrust_stab(self, msg):
        self.vertical_thrust_stab = msg.data

    def on_thrust_stab(self, msg):
        self.thrust_stab = msg.data

    def on_roll_stab(self, msg):
        self.roll_stab = msg.data

    def on_pitch_stab(self, msg):
        self.pitch_stab = msg.data

    def on_yaw_stab(self, msg):
        self.yaw_stab = msg.data
        
def main():
    node = JoyMapperNode("joystick_mapper")
    node.run()


if __name__ == "__main__":
    main()
