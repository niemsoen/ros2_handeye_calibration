# !/usr/bin/env python3
"""
Collect poses and perform calibration
"""

import rclpy

from rclpy.node import Node
from rclpy.time import Duration, Time
from geometry_msgs.msg import TransformStamped, Transform
from scipy.spatial.transform import Rotation as Rot
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .calibration_backend import CalibrationBackend


def get_transform(tf_message: Transform):
    tr = tf_message.translation
    qt = tf_message.rotation
    out = [tr.x, tr.y, tr.z, qt.x, qt.y, qt.z, qt.w]
    return out

def tf_list_to_string(mlist: list):
    return "tx, ty, tz, qx, qy, qz, qw: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]" % tuple(mlist)

def urdf_list_to_string(mlist: list):
    return "translation: %.4f, %.4f, %.4f   rpy: %.4f, %.4f, %.4f" % tuple(mlist)

def tf_to_urdf_tf(mlist: list):
    """
    Transform tx, ty, tz, qx, qy, qz, qw into tx, ty, tz, r, p, y
    """
    res = mlist[0:3]

    e = list(Rot.from_quat(mlist[3:]).as_euler(seq="ZYX"))
    """
    The roll-pitchRyaw axes in a typical URDF are defined as a
    rotation of ``r`` radians around the x-axis followed by a rotation of
    ``p`` radians around the y-axis followed by a rotation of ``y`` radians
    around the z-axis. These are the Z1-Y2-X3 Tait-Bryan angles. See
    Wikipedia_ for more information.
    .. _Wikipedia: https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix
    """
    r, p, y = e[2], e[1], e[0]
    res += [r, p, y]
    return res

def tf_to_string(tf_stamped_message: TransformStamped):
    tfl = get_transform(tf_stamped_message.transform)
    out = str(tf_stamped_message.child_frame_id) + " -> " + str(tf_stamped_message.header.frame_id) + ":"
    out += "\n\t" + tf_list_to_string(tfl)
    return str(out)


class DataCollector(Node):

    def __init__(self):
        mname = "hand_eye_calibration"
        super().__init__(mname)

        self.declare_parameter('tracking_base_frame', "")
        self.declare_parameter('tracking_marker_frame', "")
        self.declare_parameter('robot_base_frame', "")
        self.declare_parameter('robot_effector_frame', "")
        # options are eye-in-hand or eye-on-base
        self.declare_parameter('calibration_type', "eye-on-base")

        self.tracking_base_frame = str(self.get_parameter('tracking_base_frame').value)
        self.tracking_marker_frame = str(self.get_parameter('tracking_marker_frame').value)
        self.robot_base_frame = str(self.get_parameter('robot_base_frame').value)
        self.robot_effector_frame = str(self.get_parameter('robot_effector_frame').value)
        self.calibration_type = str(self.get_parameter('calibration_type').value)

        self.capture_point_service_name = mname + "/capture_point"
        self.capture_point_service = self.create_service(
            Trigger, 
            self.capture_point_service_name, 
            self.capture_point_service_callback)

        # Transform listener.
        self.tf_buffer = Buffer()
        self._listener = TransformListener(self.tf_buffer, self)

        self.robot_samples = list()
        self.tracking_samples = list()

    def capture_point_service_callback(self, req: Trigger.Request, resp: Trigger.Response):
        # get transforms 
        time = Time(nanoseconds=0)  # get the latest data

        try:
            # here we trick the library (it is actually made for eye_in_hand only). Trust me, I'm an engineer
            if self.calibration_type == "eye-in-hand":
                robot = self.tf_buffer.lookup_transform(self.robot_base_frame,
                                                    self.robot_effector_frame, time,
                                                    Duration(seconds=2))
            elif self.calibration_type == "eye-on-base":
                robot = self.tf_buffer.lookup_transform(self.robot_effector_frame,
                                                    self.robot_base_frame, time,
                                                    Duration(seconds=2))
            else:
                msg = "Invalid calibration_type: " + self.calibration_type + ". Options are eye-in-hand or eye-on-base"
                self.get_logger().error(msg)

            tracking = self.tf_buffer.lookup_transform(self.tracking_base_frame,
                                                    self.tracking_marker_frame, time,
                                                    Duration(seconds=2))
        except TransformException as ex:
            self.get_logger().error("Could not get transforms")
            self.get_logger().error(str(ex))

        self.get_logger().info("robot: " + tf_to_string(robot))
        self.get_logger().info("tracking: " + tf_to_string(tracking))

        self.robot_samples.append(get_transform(robot.transform))
        self.tracking_samples.append(get_transform(tracking.transform))

        cal = self.get_calibration()
        if cal is None:
            msg = "Not enough samples yet..."
        else:
            self.get_logger().info("Current estimate of: " + self.tracking_base_frame + " -> " + self.robot_base_frame)
            self.get_logger().info("transform: " + tf_list_to_string(cal))
            self.get_logger().info("as euler: " + urdf_list_to_string(tf_to_urdf_tf(cal)))
            msg = "Current estimate: " + tf_list_to_string(cal) + " as euler: " + urdf_list_to_string(tf_to_urdf_tf(cal))
        resp.success = True
        resp.message = msg
        return resp

    def get_calibration(self):
        if len(self.robot_samples) < 4:
            self.get_logger().info("Not enough samples yet...")
            return None
        else:
            self.get_logger().info("Estimating ...")
            cal = CalibrationBackend.compute_calibration(samples_robot=self.robot_samples, 
                                                         samples_tracking=self.tracking_samples)
            return cal


def main():
    rclpy.init()
    node = DataCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()