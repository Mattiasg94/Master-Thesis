from cv2 import cv2
import numpy as np
from openCV_pos import get_angle_and_pos

cap = cv2.VideoCapture(0)

def nothing(x):
    pass

cv2.namedWindow("Tracking")
cv2.createTrackbar("LH", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LS", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LV", "Tracking", 0, 255, nothing)
cv2.createTrackbar("UH", "Tracking", 255, 255, nothing)
cv2.createTrackbar("US", "Tracking", 255, 255, nothing)
cv2.createTrackbar("UV", "Tracking", 255, 255, nothing)
LB=[]
UB=[]
#------ Blue
l_b=np.array([50,75,40])
u_b=np.array([160,255,125])
LB.append(l_b)
UB.append(u_b)
#------ Red 
l_b=np.array([60,135,100])
u_b=np.array([255,255,255])
LB.append(l_b)
UB.append(u_b)

images=['blue','red']
def get_res_of_moving_obj():
    images=[]
    for i in range(2):
        l_b=LB[i]
        u_b=UB[i]
        mask = cv2.inRange(hsv, l_b, u_b)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        images.append(res)
    return images

while True:
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("LH", "Tracking")
    l_s = cv2.getTrackbarPos("LS", "Tracking")
    l_v = cv2.getTrackbarPos("LV", "Tracking")

    u_h = cv2.getTrackbarPos("UH", "Tracking")
    u_s = cv2.getTrackbarPos("US", "Tracking")
    u_v = cv2.getTrackbarPos("UV", "Tracking")

    images =get_res_of_moving_obj()
    xs,ys,angle,orig_img=get_angle_and_pos(frame,[images[0],images[1]])
    # static obstacle
    # l_b = np.array(0,0,0)
    # u_b = np.array(0,0,0)
    # mask = cv2.inRange(hsv, l_b, u_b)
    # res = cv2.bitwise_and(frame, frame, mask=mask)
    # static_obs_xs,static_obs_ys,_,orig_img=get_angle_and_pos(frame,res)
    cv2.imshow("frame", orig_img)

    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()