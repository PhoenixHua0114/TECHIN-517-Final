import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2
from ultralytics import YOLO

MODEL_PATH = "/home/ubuntu/techin517/dice-recognition/runs/detect/train/weights/best.pt"
ROI_X1, ROI_Y1, ROI_X2, ROI_Y2 = 285, 206, 381, 286
SCALE = 8

class DiceDetector(Node):
    def __init__(self):
        super().__init__('dice_detector')
        self.model = YOLO(MODEL_PATH)
        self.sub = self.create_subscription(
            Image,
            '/static_camera/static_camera/color/image_raw',
            self.callback, 1)
        self.pub = self.create_publisher(String, '/dice_result', 10)
        self.get_logger().info('Dice detector running...')

    def callback(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img_bgr = img[:, :, ::-1].copy()

        roi = img_bgr[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]
        roi_large = cv2.resize(roi, (roi.shape[1]*SCALE, roi.shape[0]*SCALE),
                               interpolation=cv2.INTER_CUBIC)

        results = self.model(roi_large, conf=0.1, iou=0.3, verbose=False)

        faces = []
        for r in results:
            for box in r.boxes:
                faces.append(int(box.cls) + 1)

        if len(faces) == 2:
            total = sum(faces)
            outcome = "BIG" if total >= 7 else "SMALL"
            msg_out = String()
            msg_out.data = f"{faces[0]}+{faces[1]}={total} {outcome}"
            self.pub.publish(msg_out)
            self.get_logger().info(msg_out.data)
        else:
            self.get_logger().info(f'Detected {len(faces)} dice, waiting...')

def main():
    rclpy.init()
    node = DiceDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
