#!/usr/bin/env python3
import rospy
import time, math
from collections import deque
import numpy as np
import tf
import tf.transformations as tf_trans
import serial


class VentionPositionControl:
    """
    Vention position control:
    - Base frame: /camera_link
    - Serial commands: f/b/l/r/s
    - Waypoints:
        ("drive", meters, timeout_s)
        ("turn", degrees_ccw, timeout_s)
        ("idle", seconds, timeout_s)
    - "translate" is treated as "drive"
    """

    def __init__(self):
        # TF
        self.tf_listener = tf.TransformListener()
        self.pose_history = deque(maxlen=2)

        self.x_current = self.y_current = self.z_current = 0.0
        self.roll_current = self.pitch_current = self.yaw_current = 0.0

        # Frames
        self.map_frame  = rospy.get_param("~map_frame", "/map")
        self.base_frame = rospy.get_param("~base_frame", "/camera_link")

        # Tolerances
        self.tol_dist_m     = float(rospy.get_param("~tol_dist_m", 0.03))
        self.tol_angle_deg  = float(rospy.get_param("~tol_angle_deg", 2.0))
        self.settle_time_s  = float(rospy.get_param("~settle_time_s", 0.2))

        self.loop_min_hz    = float(rospy.get_param("~loop_min_hz", 60.0))
        self.status_hz      = float(rospy.get_param("~status_hz", 2.0))

        # Serial
        self.serial_port = rospy.get_param("~serial_port", "/dev/ttyACM0")
        self.baud_rate   = int(rospy.get_param("~baud_rate", 9600))
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0)
        time.sleep(2.0)

        self.last_cmd = None

        rospy.loginfo(
            "[VentionPositionControl] Ready.\n"
            f"  TF: {self.map_frame} -> {self.base_frame}\n"
            f"  tol_dist_m={self.tol_dist_m:.3f}, tol_angle_deg={self.tol_angle_deg:.1f}"
        )

        self.stop()
        time.sleep(0.2)

    # --------------------------------------------------
    # SERIAL
    # --------------------------------------------------
    def _send(self, c: str):
        if c not in ("f", "b", "l", "r", "s"):
            c = "s"
        if c != self.last_cmd:
            self.ser.write(c.encode("utf-8"))
            self.last_cmd = c

    def stop(self):
        self._send("s")

    # --------------------------------------------------
    # TF
    # --------------------------------------------------
    def _normalize_angle(self, angle_rad):
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

    def _state_update(self):
        now = rospy.Time.now()
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.map_frame, self.base_frame, rospy.Time(0)
            )
            self.pose_history.append((now.to_sec(), trans, rot))
            self.x_current, self.y_current, self.z_current = trans
            self.roll_current, self.pitch_current, self.yaw_current = tf_trans.euler_from_quaternion(rot)
            return True
        except:
            rospy.logwarn_throttle(2.0, "[TF] Unavailable")
            return False

    def _settle(self):
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < self.settle_time_s:
            time.sleep(0.01)

    # --------------------------------------------------
    # DRIVE
    # --------------------------------------------------
    def drive(self, meters: float, timeout_s: float):
        if abs(meters) < 1e-6 or timeout_s <= 0:
            return False

        cmd = "l" if meters > 0 else "r"
        target = abs(meters)

        if not self._state_update():
            return False

        x0, y0 = self.x_current, self.y_current
        start_time = time.time()
        prev_remaining = target

        while not rospy.is_shutdown():

            if time.time() - start_time > timeout_s:
                rospy.logwarn("[Drive] Timeout")
                self.stop()
                self._settle()
                return False

            self._state_update()

            dx = self.x_current - x0
            dy = self.y_current - y0
            prog = math.hypot(dx, dy)
            remaining = target - prog

            # zero-cross guard
            if prev_remaining > 0 and remaining <= 0:
                break

            if remaining <= self.tol_dist_m:
                break

            prev_remaining = remaining
            self._send(cmd)
            time.sleep(1.0 / self.loop_min_hz)

        self.stop()
        self._settle()
        return True

    # --------------------------------------------------
    # TURN
    # --------------------------------------------------
    def turn(self, degrees_ccw: float, timeout_s: float):
        if abs(degrees_ccw) < 1e-6 or timeout_s <= 0:
            return False

        cmd = "f" if degrees_ccw > 0 else "b"
        target = degrees_ccw

        if not self._state_update():
            return False

        yaw0 = self.yaw_current
        start_time = time.time()
        prev_remaining = target

        while not rospy.is_shutdown():

            if time.time() - start_time > timeout_s:
                rospy.logwarn("[Turn] Timeout")
                self.stop()
                self._settle()
                return False

            self._state_update()

            yaw_prog = math.degrees(self._normalize_angle(self.yaw_current - yaw0))
            remaining = target - yaw_prog

            if (prev_remaining > 0 and remaining <= 0) or \
               (prev_remaining < 0 and remaining >= 0):
                break

            if abs(remaining) <= self.tol_angle_deg:
                break

            prev_remaining = remaining
            self._send(cmd)
            time.sleep(1.0 / self.loop_min_hz)

        self.stop()
        self._settle()
        return True

    # --------------------------------------------------
    # IDLE
    # --------------------------------------------------
    def idle(self, seconds: float, timeout_s: float):
        if timeout_s <= 0:
            return False
        t_idle = min(seconds, timeout_s)
        self.stop()
        time.sleep(t_idle)
        return True

    # --------------------------------------------------
    # SEQUENCE
    # --------------------------------------------------
    def execute_sequence(self, sequence):

        rospy.loginfo(f"[Sequence] {sequence}")

        for i, wp in enumerate(sequence):

            if len(wp) != 3:
                rospy.logwarn(f"[Seq] Waypoint {i} must include timeout.")
                continue

            action = wp[0].lower()
            value = float(wp[1])
            timeout_s = float(wp[2])

            if action == "drive" or action == "translate":
                self.drive(value, timeout_s)

            elif action == "turn":
                self.turn(value, timeout_s)

            elif action == "idle":
                self.idle(value, timeout_s)

            else:
                rospy.logwarn(f"[Seq] Unknown action {action}")

        self.stop()
        rospy.loginfo("[Sequence] Done.")


def main():
    rospy.init_node("vention_position_control")

    node = VentionPositionControl()

    raw_seq = rospy.get_param("~sequence", [
        ["drive", 1, 20.0],
        ["idle", 2.0, 3.0],
        ["turn", -30.0, 15.0],
        ["idle", 2.0, 3.0],
        ["drive", -0.2, 10.0],
        ["idle", 2.0, 3.0],
    ])

    sequence = [tuple(x) for x in raw_seq]
    node.execute_sequence(sequence)


if __name__ == "__main__":
    main()