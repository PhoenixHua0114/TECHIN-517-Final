import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2

BASELINE_PATH = "/home/ubuntu/techin517/bet_baseline.jpg"
YELLOW_ROI = (31, 193, 150, 376)   # SMALL
BLUE_ROI   = (170, 213, 242, 395)  # BIG
THRESHOLD  = 0.2

class BetDetector(Node):
    def __init__(self):
        super().__init__('bet_detector')
        self.baseline = cv2.imread(BASELINE_PATH)
        self.pub = self.create_publisher(String, '/bet_result', 10)
        self.sub = self.create_subscription(
            Image,
            '/static_camera/static_camera/color/image_raw',
            self.callback, 1)
        self.get_logger().info('Bet detector running...')

    def has_chip(self, img, roi):
        x1, y1, x2, y2 = roi
        region = cv2.cvtColor(img[y1:y2, x1:x2], cv2.COLOR_BGR2GRAY)
        ref    = cv2.cvtColor(self.baseline[y1:y2, x1:x2], cv2.COLOR_BGR2GRAY)
        diff   = cv2.absdiff(region, ref)
        _, mask = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)
        return np.sum(mask > 0) / mask.size

    def callback(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img_bgr = img[:, :, ::-1].copy()

        small_cov = self.has_chip(img_bgr, YELLOW_ROI)
        big_cov   = self.has_chip(img_bgr, BLUE_ROI)

        small = small_cov > THRESHOLD
        big   = big_cov   > THRESHOLD

        if small and not big:
            bet = "SMALL"
        elif big and not small:
            bet = "BIG"
        elif small and big:
            bet = "BOTH"
        else:
            bet = "NONE"

        msg_out = String()
        msg_out.data = bet
        self.pub.publish(msg_out)
        self.get_logger().info(f'Bet: {bet}  (small={small_cov:.3f}, big={big_cov:.3f})')

def main():
    rclpy.init()
    node = BetDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
