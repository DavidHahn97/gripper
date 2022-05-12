#!/usr/bin/env python
from multiprocessing.connection import wait
import rospy
from std_msgs.msg import Float64


class gripperpospub():

    def __init__(self, name):
        rospy.init_node(name=name)

        self.start_time = rospy.get_time()
        self.i = 0
        self.wait = 3

        self.gripper_pub = rospy.Publisher("set_gripper_position",
                                            Float64,
                                            queue_size=1)


    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            self.set_manipulator_state(0)
            rospy.sleep(self.wait)
            self.set_manipulator_state(0.2)
            rospy.sleep(self.wait)
            self.set_manipulator_state(0.4)
            rospy.sleep(self.wait)
            self.set_manipulator_state(0.6)
            rospy.sleep(self.wait)
            self.set_manipulator_state(0.8)
            rospy.sleep(self.wait)
            self.set_manipulator_state(1)
            rospy.sleep(self.wait)
            self.set_manipulator_state(0.8)
            rospy.sleep(self.wait)
            self.set_manipulator_state(0.6)
            rospy.sleep(self.wait)
            self.set_manipulator_state(0.4)
            rospy.sleep(self.wait)
            self.set_manipulator_state(0.2)
            rospy.sleep(self.wait)
            self.set_manipulator_state(0)
            rospy.sleep(self.wait)

            rate.sleep()

    def set_manipulator_state(self, value):
        self.gripper_state = value
        self.gripper_pub.publish(Float64(data=self.gripper_state))

def main():
    node = gripperpospub("gripperpospub")
    node.run()

if __name__ == "__main__":
    main()

