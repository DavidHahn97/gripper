#!/usr/bin/env python
from multiprocessing.connection import wait
import rospy
from std_msgs.msg import Float64
import numpy as np


class beerpublisher():

    def __init__(self, name):
        rospy.init_node(name=name)

        self.beer_x_pos = rospy.get_param('pos_x')
        self.beer_y_pos = rospy.get_param('pos_y')
        self.beer_z_pos = rospy.get_param('pos_z')
        

        self.start_time = rospy.get_time()
    

        self.beer_x_pub = rospy.Publisher("beer_x_pos",
                                            Float64,
                                            queue_size=1)
        self.beer_y_pub = rospy.Publisher("beer_y_pos",
                                            Float64,
                                            queue_size=1)
        self.beer_z_pub = rospy.Publisher("beer_z_pos",
                                            Float64,
                                            queue_size=1)


    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            
            self.beer_x_pub.publish(self.beer_x_pos)
            
            self.beer_y_pub.publish(self.beer_y_pos)
            
            self.beer_z_pub.publish(self.beer_z_pos)
            
            rate.sleep()

def main():
    node = beerpublisher("beerpublisher")
    node.run()

if __name__ == "__main__":
    main()

