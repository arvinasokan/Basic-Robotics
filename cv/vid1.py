import cv2
import cv
import numpy as np
#cv2.namedWindow("preview")
vc = cv2.VideoCapture(0)
while(1):
    thresh=0
    W = 120
    H = 240
    _, frame = vc.read()
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    (thresh, bw_image) = cv2.threshold(gray_image, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    newx,newy = frame.shape[1]/2,frame.shape[0]/2 #new size (w,h)
    smallerimage = cv2.resize(frame,(newx,newy))
    blur = cv2.blur(bw_image,(5,5))
    cv2.imshow("original", frame)
    cv2.imshow("blur", blur)
    cv2.imshow("smaller", smallerimage)
    cv2.imshow("bw", bw_image)
    k = cv2.waitKey(5) & 0xFF	 
    if k == 27:
        break
cv2.destroyWindow("preview")
