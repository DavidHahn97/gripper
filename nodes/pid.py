#!/usr/bin/env python
import rospy
from gripper.msg import pid


class pathfinder():

    def __init__(self, name):
        rospy.init_node(name=name)

        self.pid_sub = rospy.Subscriber("pid", pid, self.on_pid queue_size=1)

    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():

            rate.sleep()

    def on_pid(self, P_gain, I_gain, D_gain, offset, speed, integrator, max_value):
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
    node = pid("pid")
    node.run()

if __name__ == "__main__":
    main()

