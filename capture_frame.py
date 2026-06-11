import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class FrameCapture(Node):
    def __init__(self):
        super().__init__('frame_capture')
        self.done = False
        self.sub = self.create_subscription(
            Image,
            '/static_camera/static_camera/color/image_raw',
            self.callback, 1)
        self.get_logger().info('Waiting for image...')

    def callback(self, msg):
        if self.done:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img_bgr = img[:, :, ::-1].copy()
        cv2.imwrite('/home/ubuntu/techin517/dice_test.jpg', img_bgr)
        self.get_logger().info('Saved: dice_test.jpg')
        self.done = True

def main():
    rclpy.init()
    node = FrameCapture()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
