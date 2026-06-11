#!/usr/bin/env python3
"""Quick bet-detection accuracy test (same logic as orchestrator.py).

Counts ORANGE pixels (chip color) in each ROI via HSV, and "locks" a bet once one
zone stays above threshold continuously for BET_STABLE_SEC. Use it to tune the
orange range / threshold against the yellow tray without running the whole game.

Run (needs the overhead camera publishing CAM_TOPIC, e.g. bringup or a standalone
RealSense launch):
    python3 test_bet_detection.py

GUI mode: trackbars tune H_low/H_high/S_low/V_low/threshold live; ROIs turn green
when over threshold; LOCKED shows when a zone held long enough. Press q to quit.
Headless: falls back to console-only using the constants below.

Copy the values you settle on back into orchestrator.py.
"""

import time
import threading

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# ── keep in sync with orchestrator.py ──
CAM_TOPIC       = '/static_camera/overhead_cam/color/image_raw'
YELLOW_ROI      = (31, 193, 150, 376)   # SMALL bet zone
BLUE_ROI        = (170, 213, 242, 395)  # BIG bet zone
ORANGE_HSV_LOW  = (5, 120, 80)
ORANGE_HSV_HIGH = (18, 255, 255)
BET_THRESHOLD   = 0.03
BET_STABLE_SEC  = 2.0


class FrameGrabber(Node):
    def __init__(self):
        super().__init__('bet_detection_test')
        self.latest = None
        self.create_subscription(Image, CAM_TOPIC, self._cb, 1)

    def _cb(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self.latest = img[:, :, ::-1].copy()   # RGB8 -> BGR


def orange_frac(bgr, roi, lo, hi):
    x1, y1, x2, y2 = roi
    region = bgr[y1:y2, x1:x2]
    if region.size == 0:
        return 0.0, None
    hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return float(np.count_nonzero(mask)) / mask.size, mask


def main():
    rclpy.init()
    node = FrameGrabber()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    win = 'bet detection test'
    gui = True
    try:
        cv2.namedWindow(win)
        cv2.createTrackbar('H_low',     win, ORANGE_HSV_LOW[0],       179, lambda v: None)
        cv2.createTrackbar('H_high',    win, ORANGE_HSV_HIGH[0],      179, lambda v: None)
        cv2.createTrackbar('S_low',     win, ORANGE_HSV_LOW[1],       255, lambda v: None)
        cv2.createTrackbar('V_low',     win, ORANGE_HSV_LOW[2],       255, lambda v: None)
        cv2.createTrackbar('thr_x1000', win, int(BET_THRESHOLD*1000), 300, lambda v: None)
    except cv2.error:
        gui = False
        print('No GUI available; console-only mode (tune the constants in this file).')

    candidate = None
    candidate_since = None
    last_print = 0.0
    print(f'Waiting for frames on {CAM_TOPIC} ...')

    try:
        while rclpy.ok():
            frame = node.latest
            if frame is None:
                time.sleep(0.05)
                continue

            if gui:
                lo = (cv2.getTrackbarPos('H_low', win),
                      cv2.getTrackbarPos('S_low', win),
                      cv2.getTrackbarPos('V_low', win))
                hi = (cv2.getTrackbarPos('H_high', win), 255, 255)
                thr = cv2.getTrackbarPos('thr_x1000', win) / 1000.0
            else:
                lo, hi, thr = ORANGE_HSV_LOW, ORANGE_HSV_HIGH, BET_THRESHOLD

            s_frac, s_mask = orange_frac(frame, YELLOW_ROI, lo, hi)
            b_frac, b_mask = orange_frac(frame, BLUE_ROI, lo, hi)
            small, big = s_frac > thr, b_frac > thr

            if small and big:
                current, state = None, 'DOUBLE'
            else:
                current = 'SMALL' if small else 'BIG' if big else None
                state = current or '-'

            if current != candidate:
                candidate = current
                candidate_since = time.time() if current is not None else None
            held = (time.time() - candidate_since) if candidate_since else 0.0
            locked = candidate is not None and held >= BET_STABLE_SEC

            if time.time() - last_print > 0.5:
                print(f'SMALL={s_frac:.3f}  BIG={b_frac:.3f}  thr={thr:.3f}  '
                      f'{state}  held={held:.1f}/{BET_STABLE_SEC}s'
                      f'{"  >>> LOCKED " + candidate if locked else ""}')
                last_print = time.time()

            if gui:
                vis = frame.copy()
                for roi, name, on in ((YELLOW_ROI, 'SMALL', small), (BLUE_ROI, 'BIG', big)):
                    x1, y1, x2, y2 = roi
                    color = (0, 255, 0) if on else (0, 0, 255)
                    cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(vis, name, (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.putText(vis, f'S={s_frac:.3f} B={b_frac:.3f} thr={thr:.3f} '
                                 f'{state} held={held:.1f}/{BET_STABLE_SEC}',
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                if locked:
                    cv2.putText(vis, f'LOCKED: {candidate}', (10, 55),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow(win, vis)
                if s_mask is not None:
                    cv2.imshow('SMALL mask', s_mask)
                if b_mask is not None:
                    cv2.imshow('BIG mask', b_mask)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if gui:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()