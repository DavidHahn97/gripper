#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Int64
from apriltag_ros.msg import AprilTagDetectionArray
import yaml
import os
import numpy as np
import enum
from copy import copy

#   this node manages everything around the gripper, it initializes and saves config files for different settings.
#   you can give it relative, absolute or manual movements for the gripper
#   it gives out the distance in the gripper, the state, and the midpoint.
#   also it is the node that directly communicates with the gripper hardware node on the raspi

class GripperState(enum.Enum):
    OPEN = enum.auto()
    CLOSED = enum.auto()
    UNDEFINED = enum.auto()

    def __str__(self):
        return f"{self.name}"

class gripperinit():

    def __init__(self, name):
        rospy.init_node(name=name)

        self.gripper_is_vector_left = np.zeros(3)
        self.gripper_is_vector_right = np.zeros(3)
        self.gripper_is_distance= 0

        #self.gripper_endpoint_LO = np.zeros(3)
        #self.gripper_endpoint_LC = np.zeros(3)
        #self.gripper_endpoint_RO = np.zeros(3)
        #self.gripper_endpoint_RC = np.zeros(3)

        self.gripper_distance_open = 0
        self.gripper_distance_close = 0
        
        self.gripper_offset_percent = 0.10
        self.set_gripper_percent = 0.5
        self.gripper_move_per_button = 0.1

        self.manipulator_state = 0
        self.moving = 0
        self.message_timeout = 3
        self.gripper_state = GripperState.UNDEFINED
        self.open_left_position_detected = False
        self.open_right_position_detected = False
        self.closed_left_position_detected = False
        self.closed_right_position_detected = False
        self.LB_pressed = False
        self.RB_pressed = False
        self.manual_control = True
        self.config_set = True
        self.tag_left = 200
        self.tag_right = 201

        self.manipulator_pub = rospy.Publisher("gripper_hardware", Int64, queue_size=1)
        self.position_percent_pub = rospy.Publisher("gripper_position_percent",Float64,queue_size=1)
        self.distance_pub = rospy.Publisher("gripper_distance",Float64,queue_size=1)

        self.key_sub = rospy.Subscriber("front_camera/tag_detections",AprilTagDetectionArray,self.distance_callback,queue_size=1)
        self.gripper_cmd_sub = rospy.Subscriber("gripper_cmd",Float64,self.on_gripper_cmd,queue_size=1)
        self.manual_sub = rospy.Subscriber("manual_mode",Float64,self.on_manual_mode,queue_size=1)
        self.gripper_position_sub = rospy.Subscriber("set_gripper_position",Float64,self.get_position,queue_size=1)

    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            if self.config_set == False:
                rospy.sleep(1)
                self.write_config()
                self.config_set = True

            #self.calc_distance()

            #if self.manual_control == False:
            #    self.set_position()
            #else:
            #    self.set_and_publish_manipulator_state(self.moving)
            self.set_and_publish_manipulator_state(self.moving)
            #self.calc_and_pub_position_percent()                                             
            rate.sleep()                                         

    def on_manual_mode(self, msg):                                             
        if msg.data ==0:                                             
            self.manual_control = True                                           
        else:                                            
            self.manual_control = False                                                                                    

    def on_gripper_cmd(self, msg):   
        #if self.manual_control == True:
            self.moving = msg.data                                                                                    

    def get_position(self, msg):
        if msg.data <= 1.00 and msg.data >= 0.00:
            self.set_gripper_percent = msg.data
            print (self.set_gripper_percent)
        else:
            print('got invalid gripper percent')

    def set_position(self):
        self.max_movement = self.gripper_distance_open - self.gripper_distance_close
        self.set_distance = self.gripper_distance_close + self.max_movement * self.set_gripper_percent
        self.offset = self.max_movement * self.gripper_offset_percent

        if self.gripper_is_distance > self.set_distance + self.offset:
            self.set_and_publish_manipulator_state(1)
        elif self.gripper_is_distance < self.set_distance - self.offset:
            self.set_and_publish_manipulator_state(-1)
        else:
            self.set_and_publish_manipulator_state(0)
        self.calc_and_pub_position_percent()

    def calc_and_pub_position_percent(self):
        self.is_distance_percent = (self.gripper_is_distance - self.gripper_distance_close) / self.max_movement
        self.position_percent_pub.publish(self.is_distance_percent)

    def write_config(self):
        self.init_move(-1)  #brings the arm in an open position
        self.calc_distance()
        self.gripper_distance_open = self.gripper_is_distance

        self.init_move(1)  #brings the arm in a closed position
        self.calc_distance()
        self.gripper_distance_close = self.gripper_is_distance
        #self.yaml_write('src/tutorial_package/config/yamltest.yaml')

    def init_move(self, value):  #moves the arm for message timeout seconds in one direction, should make sure its in on of its endpoints
        rospy.loginfo(f"Waiting {self.message_timeout} seconds for tag detection.")
        self.set_and_publish_manipulator_state(value)
        rospy.sleep(self.message_timeout)
        print('Sleep finished.')
        if value == -1:
            self.gripper_state = GripperState.OPEN
        elif value == 1:
            self.gripper_state = GripperState.CLOSED
        else:
            self.gripper_state = GripperState.UNDEFINED
        self.set_and_publish_manipulator_state(0)

        rospy.loginfo(
            f"Waiting {self.message_timeout} seconds for tag detection.")
        rospy.sleep(self.message_timeout)
        print('Sleep finished.')        

    def yaml_write(self, filename):  #creates and saves a .yaml file
        with open(filename, "w") as file_handle:
            data = self.yaml_content()
            yaml.dump(data, file_handle)
            print("Created file '{}'".format(
                os.path.join(os.getcwd(), filename)))

    def yaml_content(self):  #content for the .yaml file
        data = {}
        data["gripper_poses"] = []
        data["gripper_poses"].append({
            "max distance":
            float(self.gripper_distance_open),
            "min distance":
            float(self.gripper_distance_close)
        })
        return data

    def distance_callback(self, msg):
        for idx, tag in enumerate(msg.detections):
            if tag.id[0] == self.tag_left:  #tag on gripper
                self.gripper_is_vector_left[0] = tag.pose.pose.pose.position.x
                self.gripper_is_vector_left[1] = tag.pose.pose.pose.position.y
                self.gripper_is_vector_left[2] = tag.pose.pose.pose.position.z

            if tag.id[0] == self.tag_right:  #tag on gripper
                self.gripper_is_vector_right[0] = tag.pose.pose.pose.position.x
                self.gripper_is_vector_right[1] = tag.pose.pose.pose.position.y
                self.gripper_is_vector_right[2] = tag.pose.pose.pose.position.z

    def set_and_publish_manipulator_state(self, value):
        #rospy.loginfo("Publishing manipulator command: %s", value)
        self.manipulator_state = int(value)
        self.manipulator_pub.publish(self.manipulator_state)
    
    def calc_distance(self):
        self.gripper_is_distance = np.linalg.norm(self.gripper_is_vector_right - self.gripper_is_vector_left)
        self.distance_pub.publish(self.gripper_is_distance)

def main():
    node = gripperinit("gripperinit")
    node.run()

if __name__ == "__main__":
    main()