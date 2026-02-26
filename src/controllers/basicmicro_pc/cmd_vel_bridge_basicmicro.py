#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

# Adjust this import to wherever your class lives
# e.g. from my_driver_pkg.basicmicro_driver import VentionBase
from vention_basicmicro_py3_controller import VentionBase


class CmdVelBridgeBasicmicro:
    """
    Subscribes to /cmd_vel (m/s, rad/s) and drives your base using:
      - base.translate(speed_units)
      - base.rotate(speed_units)

    You DON'T handle encoder control here.
    You only tune:
      - linear_scale:  speed_units per (m/s)
      - angular_scale: speed_units per (rad/s)
    """
    def __init__(self):
        rospy.init_node("cmd_vel_bridge_basicmicro")

        self.linear_scale  = rospy.get_param("~linear_scale", 1000.0)   # speed_units per 1.0 m/s (tune)
        self.angular_scale = rospy.get_param("~angular_scale", 800.0)   # speed_units per 1.0 rad/s (tune)
        self.max_speed_units = rospy.get_param("~max_speed_units", 3000) # clamp for safety
        self.timeout_s = rospy.get_param("~timeout_s", 0.3)

        rospy.loginfo("Connecting VentionBase...")
        self.base = VentionBase(port_id="unused")  # your class uses defaults internally

        self.last_cmd = rospy.Time.now()
        rospy.Subscriber("/cmd_vel", Twist, self.cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.watchdog)  # 20 Hz safety stop
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("cmd_vel bridge running. Waiting for /cmd_vel...")

    def cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Convert to your driver "speed units"
        lin_units = int(v * self.linear_scale)
        rot_units = int(w * self.angular_scale)

        # Clamp
        lin_units = max(min(lin_units, self.max_speed_units), -self.max_speed_units)
        rot_units = max(min(rot_units, self.max_speed_units), -self.max_speed_units)

        # SIMPLE logic (matches what you said):
        # - If rotation commanded, rotate in place (same speed magnitude, opposite direction)
        # - Else translate
        if abs(rot_units) > 0:
            self.base.rotate(rot_units)
        else:
            self.base.translate(lin_units)

        self.last_cmd = rospy.Time.now()

    def watchdog(self, _evt):
        if (rospy.Time.now() - self.last_cmd).to_sec() > self.timeout_s:
            self.base.stop()

    def on_shutdown(self):
        try:
            self.base.stop()
            self.base.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    CmdVelBridgeBasicmicro()
    rospy.spin()