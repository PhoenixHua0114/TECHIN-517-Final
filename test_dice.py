from ultralytics import YOLO
import cv2

MODEL_PATH = "/home/ubuntu/techin517/dice-recognition/runs/detect/train/weights/best.pt"
IMAGE_PATH = "/home/ubuntu/techin517/dice_test.jpg"
ROI_X1, ROI_Y1, ROI_X2, ROI_Y2 = 285, 206, 381, 286

model = YOLO(MODEL_PATH)
img = cv2.imread(IMAGE_PATH)
roi = img[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]
roi_large = cv2.resize(roi, (roi.shape[1]*8, roi.shape[0]*8), interpolation=cv2.INTER_CUBIC)
cv2.imwrite("/home/ubuntu/techin517/roi_large.jpg", roi_large)

results = model(roi_large, conf=0.1, iou=0.3, verbose=False)

faces = []
for r in results:
    for box in r.boxes:
        face = int(box.cls) + 1
        conf = float(box.conf)
        faces.append(face)
        print(f"Dice face: {face}  conf={conf:.2f}")

print(f"\nDetected {len(faces)} dice, total: {sum(faces)}")
img_out = results[0].plot()
cv2.imwrite("/home/ubuntu/techin517/dice_result.jpg", img_out)
