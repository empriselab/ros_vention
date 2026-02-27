#!/usr/bin/env python3
import os
import sys
import traceback

import rospy
from geometry_msgs.msg import Twist


def add_ros_vention_src_to_path():
    try:
        import rospkg
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("ros_vention")
        src_path = os.path.join(pkg_path, "src")
        if src_path not in sys.path:
            sys.path.insert(0, src_path)
        return True
    except Exception as e:
        rospy.logwarn(f"Failed to add ros_vention/src to PYTHONPATH via rospkg: {e}")
        return False


class CmdVelBridgeBasicmicro:
    def __init__(self, dev="arduino"):
        rospy.init_node("cmd_vel_bridge_basicmicro")

        self.linear_scale  = rospy.get_param("~linear_scale", 1000.0)
        self.angular_scale = rospy.get_param("~angular_scale", 800.0)

        # Caps and minimum magnitude
        self.max_speed_units = rospy.get_param("~max_speed_units", 800)   # cap ±800
        self.min_speed_units = rospy.get_param("~min_speed_units", 250)   # min |speed| when nonzero

        self.timeout_s = rospy.get_param("~timeout_s", 0.3)

        self.arduino_port = rospy.get_param("~arduino_port", "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351603052-if00")
        self.arduino_baud = rospy.get_param("~arduino_baud", 115200)

        rospy.loginfo("Connecting VentionBase...")

        add_ros_vention_src_to_path()

        try:
            if dev == "arduino":
                from controllers.basicmicro_arduino.vention_arduino_control import VentionBase
                if not self.arduino_port:
                    raise RuntimeError("~arduino_port is empty. Set it to /dev/serial/by-id/... in your launch file.")
                self.base = VentionBase(port_id=self.arduino_port, baud=self.arduino_baud)
            else:
                from controllers.basicmicro_pc.vention_basicmicro_py3_controller import VentionBase
                self.base = VentionBase()
        except Exception:
            rospy.logerr("Failed to import/construct VentionBase:\n" + traceback.format_exc())
            raise

        self.last_cmd = rospy.Time.now()
        self._last_stop_attempt = rospy.Time(0)
        rospy.Subscriber("/cmd_vel", Twist, self.cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.watchdog)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("cmd_vel bridge running. Waiting for /cmd_vel...")

    def clamp_with_min(self, x: int) -> int:
        """
        Enforce:
          - x == 0 -> 0
          - min_speed_units <= |x| <= max_speed_units
        """
        if x == 0:
            return 0

        sign = 1 if x > 0 else -1
        ax = abs(x)

        if ax < self.min_speed_units:
            ax = self.min_speed_units
        if ax > self.max_speed_units:
            ax = self.max_speed_units

        return sign * ax

    def cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        lin_units = int(v * self.linear_scale)
        rot_units = int(w * self.angular_scale)

        lin_units = self.clamp_with_min(lin_units)
        rot_units = self.clamp_with_min(rot_units)

        try:
            if abs(rot_units) > 0:
                self.base.rotate(rot_units)
            else:
                self.base.translate(lin_units)
        except Exception:
            rospy.logerr("Motor command failed:\n" + traceback.format_exc())

        self.last_cmd = rospy.Time.now()

    def watchdog(self, _evt):
        if (rospy.Time.now() - self.last_cmd).to_sec() > self.timeout_s:
            # rate-limit stop attempts (e.g., 1 Hz)
            if (rospy.Time.now() - self._last_stop_attempt).to_sec() < 1.0:
                return
            self._last_stop_attempt = rospy.Time.now()

            try:
                self.base.stop()
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"Stop failed (will keep trying slowly): {e}")

    def on_shutdown(self):
        try:
            self.base.stop()
            self.base.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    CmdVelBridgeBasicmicro(dev=rospy.get_param("~dev", "arduino"))
    rospy.spin()