import cv2
import numpy as np
from apriltag import apriltag

imagepath = 'test.jpg'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
detector = apriltag("tagStandard41h12")

detections = detector.detect(image)

print(type(detections))

print(detections)
