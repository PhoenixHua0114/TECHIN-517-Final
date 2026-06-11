import cv2

img = cv2.imread("/home/ubuntu/techin517/dice_test.jpg")
roi = cv2.selectROI("Select dice area - press ENTER to confirm", img, False)
cv2.destroyAllWindows()

x, y, w, h = roi
print(f"ROI_X1, ROI_Y1, ROI_X2, ROI_Y2 = {x}, {y}, {x+w}, {y+h}")

crop = img[y:y+h, x:x+w]
cv2.imwrite("/home/ubuntu/techin517/roi_check.jpg", crop)
print("Saved: roi_check.jpg")
