import cv2

# literally only need one marker for now
# small dict
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# make marker
marker_id = 42
marker_size = 200 #px
marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

cv2.imwrite('marker_42.png', marker_img)

