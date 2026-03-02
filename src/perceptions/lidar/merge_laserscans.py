#!/usr/bin/env python3
import math
import rospy
import tf2_ros
import tf2_py as tf2
import tf2_geometry_msgs  # noqa: F401 (needed by tf2 sometimes)

from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
from geometry_msgs.msg import TransformStamped
import sensor_msgs.point_cloud2 as pc2

import message_filters

try:
    # Noetic usually has this
    from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
except Exception:
    do_transform_cloud = None


class LaserScanMerger:
    def __init__(self):
        self.scan_topic_1 = rospy.get_param("~scan_topic_1", "/lidar_l/scan")
        self.scan_topic_2 = rospy.get_param("~scan_topic_2", "/lidar_r/scan")
        self.output_topic = rospy.get_param("~output_topic", "/scan_merged")

        # Frame in which the merged scan will be expressed (should be base_link for nav)
        self.target_frame = rospy.get_param("~target_frame", "vention_base_link")

        # Output scan geometry
        self.angle_min = rospy.get_param("~angle_min", -math.pi)
        self.angle_max = rospy.get_param("~angle_max",  math.pi)
        self.angle_increment = rospy.get_param("~angle_increment", math.radians(1.0))  # 1 deg default

        # Range limits (if not set, we’ll infer from inputs each callback)
        self.range_min_param = rospy.get_param("~range_min", None)
        self.range_max_param = rospy.get_param("~range_max", None)

        # TF + projection
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.projector = LaserProjection()

        self.pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=10)

        sub1 = message_filters.Subscriber(self.scan_topic_1, LaserScan)
        sub2 = message_filters.Subscriber(self.scan_topic_2, LaserScan)

        slop = rospy.get_param("~sync_slop", 0.05)  # seconds
        queue_size = rospy.get_param("~sync_queue", 20)
        ats = message_filters.ApproximateTimeSynchronizer([sub1, sub2], queue_size=queue_size, slop=slop)
        ats.registerCallback(self.cb)

        rospy.loginfo("laser_scan_merger: subscribing to [%s] and [%s], publishing [%s] in frame [%s]",
                      self.scan_topic_1, self.scan_topic_2, self.output_topic, self.target_frame)

    def lookup_transform(self, target_frame: str, source_frame: str, stamp: rospy.Time) -> TransformStamped:
        # Use a small timeout; if TF is late, we fail gracefully for that pair
        return self.tf_buffer.lookup_transform(
            target_frame, source_frame, stamp, timeout=rospy.Duration(0.2)
        )

    def cloud_from_scan(self, scan: LaserScan) -> PointCloud2:
        # Project LaserScan to PointCloud2 in the scan's frame
        return self.projector.projectLaser(scan)

    def transform_cloud(self, cloud: PointCloud2, tf: TransformStamped) -> PointCloud2:
        if do_transform_cloud is not None:
            return do_transform_cloud(cloud, tf)

        # Fallback: manual transform of xyz points (rarely needed on Noetic)
        # Convert TransformStamped to matrix and apply; keep it simple.
        import numpy as np
        from tf.transformations import quaternion_matrix

        t = tf.transform.translation
        q = tf.transform.rotation
        T = quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z

        fields = cloud.fields
        pts_out = []
        for p in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            v = np.array([p[0], p[1], p[2], 1.0], dtype=float)
            v2 = T.dot(v)
            pts_out.append((float(v2[0]), float(v2[1]), float(v2[2])))

        header = cloud.header
        header.frame_id = tf.header.frame_id  # target frame
        return pc2.create_cloud_xyz32(header, pts_out)

    def merge_clouds_to_scan(self, header, points_xyz, range_min, range_max) -> LaserScan:
        n_bins = int(math.floor((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        ranges = [float("inf")] * n_bins

        for (x, y, z) in points_xyz:
            # 2D projection
            r = math.hypot(x, y)
            if r < range_min or r > range_max:
                continue
            ang = math.atan2(y, x)
            if ang < self.angle_min or ang > self.angle_max:
                continue
            idx = int((ang - self.angle_min) / self.angle_increment)
            if 0 <= idx < n_bins and r < ranges[idx]:
                ranges[idx] = r

        out = LaserScan()
        out.header = header
        out.angle_min = self.angle_min
        out.angle_max = self.angle_max
        out.angle_increment = self.angle_increment
        out.time_increment = 0.0
        out.scan_time = 0.0
        out.range_min = range_min
        out.range_max = range_max
        out.ranges = ranges
        # intensities left empty (you can extend this if you need)
        return out

    def cb(self, scan1: LaserScan, scan2: LaserScan):
        # Decide range min/max
        range_min = self.range_min_param if self.range_min_param is not None else max(scan1.range_min, scan2.range_min)
        range_max = self.range_max_param if self.range_max_param is not None else min(scan1.range_max, scan2.range_max)

        # Use a common timestamp (latest of the two) for TF lookup
        stamp = scan1.header.stamp if scan1.header.stamp >= scan2.header.stamp else scan2.header.stamp

        try:
            tf1 = self.lookup_transform(self.target_frame, scan1.header.frame_id, stamp)
            tf2t = self.lookup_transform(self.target_frame, scan2.header.frame_id, stamp)
        except (tf2.LookupException, tf2.ExtrapolationException, tf2.ConnectivityException) as e:
            rospy.logwarn_throttle(2.0, "laser_scan_merger: TF lookup failed: %s", str(e))
            return

        # Project to clouds
        c1 = self.cloud_from_scan(scan1)
        c2 = self.cloud_from_scan(scan2)

        # Transform to target frame
        try:
            c1t = self.transform_cloud(c1, tf1)
            c2t = self.transform_cloud(c2, tf2t)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "laser_scan_merger: cloud transform failed: %s", str(e))
            return

        # Collect points
        pts = []
        for p in pc2.read_points(c1t, field_names=("x", "y", "z"), skip_nans=True):
            pts.append((p[0], p[1], p[2]))
        for p in pc2.read_points(c2t, field_names=("x", "y", "z"), skip_nans=True):
            pts.append((p[0], p[1], p[2]))

        # Output scan header in target frame
        header = scan1.header
        header.stamp = stamp
        header.frame_id = self.target_frame

        merged = self.merge_clouds_to_scan(header, pts, range_min, range_max)
        self.pub.publish(merged)


if __name__ == "__main__":
    rospy.init_node("laser_scan_merger")
    LaserScanMerger()
    rospy.spin()