#!/usr/bin/env python
from re import L
from unittest import skip
import rospy
from std_msgs.msg import Float64
import numpy as np
from nav_msgs.msg import Odometry
import tf.transformations
import math 
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from hippocampus_msgs.msg import DepthEKFStamped
from dynamic_reconfigure.server import Server
from gripper.cfg import PathFinderConfig
import threading

class pathfinder():

    def __init__(self, name):
        rospy.init_node(name=name)
        self.data_lock = threading.RLock()

        self.roll_to_origin = 0
        self.pitch_to_origin = 0
        self.yaw_to_origin = 0

        self.depth_P_gain = 0
        self.depth_I_gain = 0
        self.depth_D_gain = 0
        self.vertical_thrust_max = 0
        self.depth_integrator = 0 

        self.thrust_P_gain = 0
        self.thrust_I_gain = 0
        self.thrust_D_gain = 0
        self.thrust_max = 0
        self.thrust_integrator = 0

        self.lateral_thrust_P_gain = 0
        self.lateral_thrust_I_gain = 0
        self.lateral_thrust_D_gain = 0
        self.lateral_thrust_max = 0
        self.lateral_thrust_integrator = 0

        self.roll_P_gain = 0
        self.roll_I_gain = 0
        self.roll_D_gain = 0
        self.roll_max = 0
        self.roll_integrator = 0

        self.pitch_P_gain = 0
        self.pitch_I_gain = 0
        self.pitch_D_gain = 0
        self.pitch_max = 0
        self.pitch_integrator = 0

        self.yaw_P_gain = 0
        self.yaw_I_gain = 0
        self.yaw_D_gain = 0
        self.yaw_max = 0
        self.yaw_integrator = 0
        

        self.checkedallflags = True
        self.manual_mode = 0
        self.angle_acceptable = False
        self.simulation = False

        self.beer = np.zeros(3)
        self.bluerov_pos = np.zeros(3)
        self.bluerov_pos_twist = np.zeros(3)
        self.bluerov_orientation = np.zeros(4)
        self.bluerov_orientation_twist = np.zeros(3)
        self.v_bluerov_beer = np.zeros(2)
        self.v_norm_x = [1, 0]

        self.start_time = rospy.get_time()

        self.distance_pub = rospy.Publisher("distance_beer_bluerov",Float64,queue_size=1)
        self.yaw_to_origin_pub = rospy.Publisher("yaw_to_origin", Float64,queue_size=1)
        self.roll_to_origin_pub = rospy.Publisher("roll_to_origin", Float64,queue_size=1)
        self.pitch_to_origin_pub = rospy.Publisher("pitch_to_origin", Float64,queue_size=1)
        self.angle_path_origin_pub = rospy.Publisher("angle_path_to_origin", Float64,queue_size=1)
        self.angle_offset_pub = rospy.Publisher("angle_offset", Float64,queue_size=1)

        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust_stab", Float64,queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust_stab", Float64,queue_size=1)
        self.yaw_pub = rospy.Publisher("yaw_stab", Float64, queue_size=1)
        self.roll_pub = rospy.Publisher("roll_stab", Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher("pitch_stab", Float64, queue_size=1)
        self.thrust_pub = rospy.Publisher("thrust_stab", Float64, queue_size=1)

        self.is_position_pub = rospy.Publisher("is_position", PoseStamped, queue_size=1)
        self.is_twist_pub = rospy.Publisher("is_twist", TwistStamped, queue_size=1)


        #tuning publisher
        self.P_pub = rospy.Publisher("P", Float64, queue_size=1)
        self.I_pub = rospy.Publisher("I", Float64, queue_size=1)
        self.D_pub = rospy.Publisher("D", Float64, queue_size=1)
        self.offset_pub = rospy.Publisher("offset", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("speed", Float64, queue_size=1)
        self.integrator_pub = rospy.Publisher("integrator", Float64, queue_size=1)
        self.output_pub = rospy.Publisher("output", Float64, queue_size=1)

        self.reconfigure_server = Server(PathFinderConfig,self.on_reconfigure)

        self.manual_mode_sub = rospy.Subscriber("manual_mode", Float64, self.on_manual_mode, queue_size=1)
        self.beer_x_sub = rospy.Subscriber("beer_x_pos", Float64, self.on_beer_x, queue_size=1)
        self.beer_y_sub = rospy.Subscriber("beer_y_pos", Float64, self.on_beer_y, queue_size=1)
        self.beer_z_sub = rospy.Subscriber("beer_z_pos", Float64, self.on_beer_z, queue_size=1)
        if self.simulation:
            self.bluerov_pos_sub = rospy.Subscriber("ground_truth/state", Odometry, self.on_bluerov_pos, queue_size=1)
            self.bluerov_twist_sub = rospy.Subscriber("ground_truth/twist_body_frame", TwistStamped, self.on_bluerov_twist, queue_size=1)
        else:
            self.bluerov_pos_sub = rospy.Subscriber("mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, self.on_bluerov_real_pos, queue_size=1)
            self.bluerov_orientation_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.on_bluerov_real_orientation, queue_size=1)
            self.bluerov_depth_sub = rospy.Subscriber("depth",DepthEKFStamped , self.on_bluerov_real_depth, queue_size=1)

    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            while self.checkedallflags == True:
                self.calc_distance()
                self.calc_angles()
                self.pub_is_position()
                self.pub_is_twist()
                #self.roll_stability()
                self.pitch_stability()
                self.depth_stability()
                #self.lateral_thrust_stability()
                self.yaw_stability()
                if self.angle_acceptable:
                   self.minimize_distance()
            rate.sleep()

    def on_reconfigure(self, config, _):
        with self.data_lock:
            self.depth_P_gain = config["depth_P_gain"]
            self.depth_I_gain = config["depth_I_gain"]
            self.depth_D_gain = config["depth_D_gain"]
            self.vertical_thrust_max = config["vertical_thrust_max"]

            self.thrust_P_gain = config["thrust_P_gain"]
            self.thrust_I_gain = config["thrust_I_gain"]
            self.thrust_D_gain = config["thrust_D_gain"]
            self.thrust_max = config["thrust_max"]

            self.lateral_thrust_P_gain = config["lateral_thrust_P_gain"]
            self.lateral_thrust_I_gain = config["lateral_thrust_I_gain"]
            self.lateral_thrust_D_gain = config["lateral_thrust_D_gain"]
            self.lateral_thrust_max =  config["lateral_thrust_max"]

            self.roll_P_gain = config["roll_P_gain"]
            self.roll_I_gain = config["roll_I_gain"]
            self.roll_D_gain = config["roll_D_gain"]
            self.roll_max = config["roll_max"]

            self.pitch_P_gain = config["pitch_P_gain"]
            self.pitch_I_gain = config["pitch_I_gain"]
            self.pitch_D_gain = config["pitch_D_gain"]
            self.pitch_max = config["pitch_max"]

            self.yaw_P_gain = config["yaw_P_gain"]
            self.yaw_I_gain = config["yaw_I_gain"]
            self.yaw_D_gain = config["yaw_D_gain"]
            self.yaw_max = config["yaw_max"]
            self.max_integrator = config["max_integrator"]
        return config

    def on_manual_mode(self, msg):
        self.manual_mode = msg.data

    def on_beer_x(self, msg):
        self.beer[0] = msg.data

    def on_beer_y(self, msg):
        self.beer[1] = msg.data

    def on_beer_z(self, msg):
        self.beer[2] = msg.data

    def on_bluerov_pos(self, msg):
        self.bluerov_pos[0] = msg.pose.pose.position.x
        self.bluerov_pos[1] = msg.pose.pose.position.y
        self.bluerov_pos[2] = msg.pose.pose.position.z

        self.bluerov_orientation[0] = msg.pose.pose.orientation.x        
        self.bluerov_orientation[1] = msg.pose.pose.orientation.y
        self.bluerov_orientation[2] = msg.pose.pose.orientation.z
        self.bluerov_orientation[3] = msg.pose.pose.orientation.w

    def on_bluerov_twist(self, msg):
        self.bluerov_pos_twist[0] = msg.twist.linear.x
        self.bluerov_pos_twist[1] = msg.twist.linear.y
        self.bluerov_pos_twist[2] = msg.twist.linear.z

        self.bluerov_orientation_twist[0] = msg.twist.angular.x        
        self.bluerov_orientation_twist[1] = msg.twist.angular.y
        self.bluerov_orientation_twist[2] = msg.twist.angular.z

    def on_bluerov_real_pos(self, msg):
        self.bluerov_pos[0] = msg.pose.pose.position.x
        self.bluerov_pos[1] = msg.pose.pose.position.y
        #self.bluerov_pos[2] = msg.pose.pose.position.z   # z axis via pressure

        self.bluerov_orientation[0] = msg.pose.pose.orientation.x        
        self.bluerov_orientation[1] = msg.pose.pose.orientation.y
        self.bluerov_orientation[2] = msg.pose.pose.orientation.z
        self.bluerov_orientation[3] = msg.pose.pose.orientation.w

        _, _, self.yaw_to_origin = tf.transformations.euler_from_quaternion(self.bluerov_orientation)

    def on_bluerov_real_depth(self, msg):
        self.bluerov_pos[2] = msg.depth
        self.bluerov_pos_twist[2] = msg.z_vel

    def on_bluerov_real_orientation(self,msg):
        self.bluerov_orientation[0] = msg.pose.orientation.x        
        self.bluerov_orientation[1] = msg.pose.orientation.y
        self.bluerov_orientation[2] = msg.pose.orientation.z
        self.bluerov_orientation[3] = msg.pose.orientation.w

        self.roll_to_origin, self.pitch_to_origin, _ = tf.transformations.euler_from_quaternion(self.bluerov_orientation)

    def pub_is_position(self):
        msg = PoseStamped()                         
        msg.header.frame_id = "body"     
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.bluerov_pos[0]
        msg.pose.position.y = self.bluerov_pos[1]
        msg.pose.position.z = self.bluerov_pos[2]
        msg.pose.orientation.x = self.bluerov_orientation[0]
        msg.pose.orientation.y = self.bluerov_orientation[1]
        msg.pose.orientation.z = self.bluerov_orientation[2]
        msg.pose.orientation.w = self.bluerov_orientation[3]
        self.is_position_pub.publish(msg)

    def pub_is_twist(self):
        msg = TwistStamped()
        msg.header.frame_id = "body"
        msg.twist.linear.x = self.bluerov_pos_twist[0]
        msg.twist.linear.y = self.bluerov_pos_twist[1]
        msg.twist.linear.z = self.bluerov_pos_twist[2]
        msg.twist.angular.x = self.bluerov_orientation_twist[0]
        msg.twist.angular.y = self.bluerov_orientation_twist[1]
        msg.twist.angular.z = self.bluerov_orientation_twist[2]
        self.is_twist_pub.publish(msg)


    def calc_distance(self):
        self.distance_bluerov_beer = np.linalg.norm(self.beer - self.bluerov_pos)
        self.distance_pub.publish(self.distance_bluerov_beer)

    def minimize_distance(self):
        if self.manual_mode > 3:
            with self.data_lock:
                self.thrust_offset = (self.distance_bluerov_beer - 0.3)
                self.thrust, self.thrust_integrator = self.PID(self.thrust_P_gain, self.thrust_I_gain, self.thrust_D_gain, self.thrust_offset, self.bluerov_pos_twist[0],self.thrust_integrator, self.thrust_max)
                self.thrust_pub.publish(self.thrust)
        else:
            self.thrust_pub.publish(0)

    def depth_stability(self):
        if self.manual_mode > 2:
            with self.data_lock:
                self.depth_offset = self.beer[2] - self.bluerov_pos[2]
                #print("offset ", self.depth_offset, "beer z ", self.beer[2], "bluerov z ", self.bluerov_pos[2])
                self.vertical_thrust, self.depth_integrator = self.PID(self.depth_P_gain, self.depth_I_gain, self.depth_D_gain, self.depth_offset, self.bluerov_pos_twist[2],self.depth_integrator, self.vertical_thrust_max)
                self.vertical_thrust_pub.publish(self.vertical_thrust)
        else:
            self.vertical_thrust_pub.publish(0)

    def lateral_thrust_stability(self):
        with self.data_lock:
            self.lateral_thrust_offset = 0
            self.lateral_thrust, self.lateral_thrust_integrator = self.PID(self.lateral_thrust_P_gain, self.lateral_thrust_I_gain, self.lateral_thrust_D_gain, self.lateral_thrust_offset, self.bluerov_pos_twist[1],self.lateral_thrust_integrator, self.lateral_thrust_max)
            self.lateral_thrust_pub.publish(self.lateral_thrust)

    def calc_angles(self):
        self.yaw_to_origin_pub.publish(self.yaw_to_origin)
        self.roll_to_origin_pub.publish(self.roll_to_origin)
        self.pitch_to_origin_pub.publish(self.pitch_to_origin)

        self.v_bluerov_beer[0] = self.beer[0] - self.bluerov_pos[0]
        self.v_bluerov_beer[1] = self.beer[1] - self.bluerov_pos[1]

        self.angle_path_origin = np.math.atan2(np.linalg.det([self.v_norm_x, self.v_bluerov_beer]),np.dot(self.v_norm_x, self.v_bluerov_beer))
        self.angle_path_origin_pub.publish(self.angle_path_origin)
        #print("xdist", self.v_bluerov_beer[0], "ydist", self.v_bluerov_beer[1], "angle", self.angle_path_origin)

        self.yaw_offset = self.angle_path_origin - self.yaw_to_origin + math.pi
        self.yaw_offset = self.yaw_offset % (2*math.pi)
        self.yaw_offset = self.yaw_offset - math.pi
        #print("angle bluerov origin", self.yaw_to_origin, "angle path origin", self.angle_path_origin, "angle offset", self.yaw_offset)
        if abs(self.yaw_offset) < math.pi / 6:
            self.angle_acceptable = True
        else:
            self.angle_acceptable = False
        self.angle_offset_pub.publish(self.yaw_offset)
   
    def yaw_stability(self):
        if self.manual_mode > 4:
            with self.data_lock:
                self.yaw_cmd, self.yaw_integrator = self.PID(self.yaw_P_gain, self.yaw_I_gain, self.yaw_D_gain, self.yaw_offset, self.bluerov_orientation_twist[2], self.yaw_integrator, self.yaw_max)
                self.yaw_pub.publish(self.yaw_cmd)
        else:
            self.yaw_pub.publish(0)

    def roll_stability(self):
        with self.data_lock:
            self.roll_set = 0
            self.roll_offset = self.roll_set - self.roll_to_origin
            self.roll_cmd, self.roll_integrator = self.PID(self.roll_P_gain, self.roll_I_gain, self.roll_D_gain, self.roll_offset, self.bluerov_orientation_twist[0], self.roll_integrator, self.roll_max)
            self.roll_pub.publish(self.roll_cmd)

    def pitch_stability(self):
        if self.manual_mode > 0:
            with self.data_lock:
                self.pitch_set = 0
                self.pitch_offset = self.pitch_set - self.pitch_to_origin
                self.pitch_cmd, self.pitch_integrator = self.PID(self.pitch_P_gain, self.pitch_I_gain, self.pitch_D_gain, self.pitch_offset, self.bluerov_orientation_twist[1], self.pitch_integrator, self.pitch_max)
                self.pitch_pub.publish(self.pitch_cmd)
        else:
            self.pitch_pub.publish(0)

    def PID(self, P_gain, I_gain, D_gain, offset, speed, integrator, max_value):
        self.P_gain = P_gain
        self.I_gain = I_gain
        self.D_gain = D_gain
        self.offset = offset
        self.speed = speed
        self.integrator = integrator
        self.max_value = max_value

        P = self.P_gain * offset

        self.integrator = self.integrator + self.offset * 0.0001
        if self.integrator > self.max_integrator:
            self.integrator = self.max_integrator
        elif abs(self.integrator) > self.max_integrator:
            self.integrator = self.max_integrator * -1
        I = self.I_gain * self.integrator

        D = self.D_gain * self.speed * -1

        output = P + I + D

        if output > self.max_value:
            output = self.max_value
        elif abs(output) > self.max_value:
            output = -self.max_value

        self.P_pub.publish(P)
        self.I_pub.publish(I)
        self.D_pub.publish(D)
        self.offset_pub.publish(offset)
        self.speed_pub.publish(speed)
        self.output_pub.publish(output)
        self.integrator_pub.publish(integrator)
        return(output, self.integrator)

def main():
    node = pathfinder("pathfinder")
    node.run()

if __name__ == "__main__":
    main()

