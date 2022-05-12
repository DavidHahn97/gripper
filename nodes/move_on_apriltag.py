#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray


class moveonapriltag():

    def __init__(self, name):
        rospy.init_node("move_on_apriltag")

        self.manipulator_state = 'neutral'
        self.start_time = rospy.get_time()
        self.duration = 1

        self.manipulator_pub = rospy.Publisher("manipulator",
                                            String,
                                            queue_size=1)

        self.key_sub = rospy.Subscriber("front_camera/tag_detections", AprilTagDetectionArray, self.distance_callback, queue_size=1)

    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():

            rate.sleep()

    def set_manipulator_state(self, value):
        self.manipulator_state = value
        self.manipulator_pub.publish(String(data=self.manipulator_state))

    def distance_callback(self,msg):
        
        for idx, tag in enumerate(msg.detections):
            print('ID: ', tag.id[0])
            print('tag pose: ', tag.pose.pose.pose.position.z)

            if tag.id[0] == 71:
                if tag.pose.pose.pose.position.z < 0.25:
                    self.set_manipulator_state('open')
                    print('sendopen')
                elif tag.pose.pose.pose.position.z > 0.25:
                    self.set_manipulator_state("close")
                    print('sendclose')
                else:
                    self.set_manipulator_state('neutral')
                    print('sendneutral')                



    def publish_message(self):
        msg = String()
        #msg.header.stamp = rospy.Time.now()
        msg.data = self.manipulator_state
        self.manipulator_pub.publish(msg)
        rospy.loginfo("Manipulator State: %s", msg.data)


def main():
    node = moveonapriltag("moveonapriltag")
    node.run()

if __name__ == "__main__":
    main()

