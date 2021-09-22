#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from bobi_msgs.msg import MotorVelocities
from bobi_msgs.srv import EnableIR, EnableTemp


class WingmanJoy:
    def __init__(self):
        rospy.wait_for_service('enable_ir')
        rospy.wait_for_service('enable_temp')

        rospy.Subscriber("joy_fishbot", Joy, self.joy_cb)
        self._vel_pub = rospy.Publisher(
            "set_velocities", MotorVelocities, queue_size=1)

        self._linear_scaler = rospy.get_param('linear_scaler', 0.6)
        self._angular_scaler = rospy.get_param('angular_scaler', 4.0)
        self._l = 0.0451

        self._enable_ir = False
        self._enable_temp = False

    def joy_cb(self, data):
        mv = MotorVelocities()
        if (sum(data.buttons) > 0):
            if data.buttons[8]:
                mv.left = 0
                mv.right = 0
                self._vel_pub.publish(mv)
                rospy.loginfo("Emergency stop")

            if data.buttons[5]:
                self._linear_scaler += 0.05
                rospy.loginfo("Linear scaler is now: %f", self._linear_scaler)
            if data.buttons[2]:
                self._linear_scaler -= 0.05
                rospy.loginfo("Linear scaler is now: %f", self._linear_scaler)

            if data.buttons[4]:
                self._angular_scaler += 0.1
                rospy.loginfo("Angular scaler is now: %f",
                              self._angular_scaler)
            if data.buttons[1]:
                self._angular_scaler -= 0.1
                rospy.loginfo("Angular scaler is now: %f",
                              self._angular_scaler)

            if data.buttons[3]:
                self._enable_temp = not self._enable_temp
                try:
                    enable_temp = rospy.ServiceProxy('enable_temp', EnableTemp)
                    enable_temp.enable = self._enable_temp
                except rospy.ServiceException as e:
                    rospy.loginfo("Service call failed: %s" % e)
                rospy.loginfo("Enable Temp: %d", self._enable_temp)
            if data.buttons[0]:
                self._enable_ir = not self._enable_ir
                try:
                    enable_ir = rospy.ServiceProxy('enable_ir', EnableIR)
                    enable_ir.enable = self._enable_ir
                except rospy.ServiceException as e:
                    rospy.loginfo("Service call failed: %s" % e)
                rospy.loginfo("Enable IR: %d", self._enable_ir)

        elif (sum(data.axes) > 0):
            v = data.axes[1] * self._linear_scaler
            w = -data.axes[3] * self._angular_scaler
            mv.left = (2. * v - w * self._l) / 2.
            mv.right = (2. * v + w * self._l) / 2.
            self._vel_pub.publish(mv)
            rospy.loginfo(
                "Received (v, w) = (%f, %f) | Sending (vl, vr) = (%f, %f)", v, w, mv.left, mv.right)


def main():
    rospy.init_node('joy_controller')
    _ = WingmanJoy()
    rospy.spin()


if __name__ == '__main__':
    main()
