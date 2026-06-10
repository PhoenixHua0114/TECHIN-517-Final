"""Save pose service node.

Provides the /follower/save_pose service (soa_interfaces/srv/SavePose)
to capture the current pose of the gripper_link in the base_link frame
and optionally append it to a CSV file.

Usage:
    ros2 run soa_functions save_pose

Services:
    /follower/save_pose (soa_interfaces/srv/SavePose)
        request:  csv_path — path to CSV file; if empty, pose is not saved
        response: success, pose
"""

import csv
import os

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros
from soa_interfaces.srv import SavePose


class SavePoseNode(Node):

    def __init__(self):
        super().__init__('save_pose')

        self._cb_group = ReentrantCallbackGroup()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_service(
            SavePose,
            '/follower/save_pose',
            self._handle_save_pose,
            callback_group=self._cb_group,
        )

        self.get_logger().info('SavePose service ready.')

    def _handle_save_pose(self, req, res):
        """Handle the /follower/save_pose service request."""
        try:
            transform = self._tf_buffer.lookup_transform(
                'follower/base_link',
                'follower/gripper_link',
                rclpy.time.Time(),
            )
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')
            res.success = False
            return res

        t = transform.transform.translation
        r = transform.transform.rotation

        res.success = True
        res.pose.position.x = t.x
        res.pose.position.y = t.y
        res.pose.position.z = t.z
        res.pose.orientation.x = r.x
        res.pose.orientation.y = r.y
        res.pose.orientation.z = r.z
        res.pose.orientation.w = r.w

        if req.csv_path:
            try:
                self._append_to_csv(req.csv_path, t, r)
            except OSError as e:
                self.get_logger().error(f'Failed to write CSV: {e}')
                res.success = False

        return res

    def _append_to_csv(self, path, translation, rotation):
        """Append a single row of pose to a CSV file."""
        file_exists = os.path.exists(path)
        fieldnames = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        with open(path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerow({
                'x':  translation.x,
                'y':  translation.y,
                'z':  translation.z,
                'qx': rotation.x,
                'qy': rotation.y,
                'qz': rotation.z,
                'qw': rotation.w,
            })
        self.get_logger().info(f'Saved pose to {path}')


def main(args=None):
    rclpy.init(args=args)
    node = SavePoseNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()