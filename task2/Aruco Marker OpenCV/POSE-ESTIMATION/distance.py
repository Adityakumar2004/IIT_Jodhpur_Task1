import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import math

markerLength = 0.25

cap = cv2.VideoCapture(0)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

objpoints = [] 
imgpoints = []

images = glob.glob('calib_images/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners((7,6),None)

    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)
        img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints,None,None)

calibrationFile = "calibrationFileName.xml"
calibrationParams = cv2.FileStorage(calibrationFile, cv2.FILE_STORAGE_READ) 
camera_matrix = calibrationParams.getNode("cameraMatrix").mat() 
dist_coeffs = calibrationParams.getNode("distCoeffs").mat() 
count = 1
while(True):
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParameters =  aruco.DetectorParameters_create()
    aruco_list = {}
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)
    if np.all(ids != None):
        if len(corners):
            for k in range(len(corners)):
                temp_1 = corners[k]
                temp_1 = temp_1[0]
                temp_2 = ids[k]
                temp_2 = temp_2[0]
                aruco_list[temp_2] = temp_1
        key_list = aruco_list.keys()
        font = cv2.FONT_HERSHEY_SIMPLEX
        for key in key_list:
            dict_entry = aruco_list[key]    
            centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]
            centre[:] = [int(x / 4) for x in centre]
            orient_centre = centre + [0.0,5.0]
            centre = tuple(centre)  
            orient_centre = tuple((dict_entry[0]+dict_entry[1])/2)
            cv2.circle(frame,centre,1,(0,0,255),8)
        display = aruco.drawDetectedMarkers(frame, corners)
        display = np.array(display)
    else:
        display = frame

    cv2.imshow('Display',display)
    if cv2.waitKey(1) & 0xFF == ord('q'):
            break
cap.release()
cv2.destroyAllWindows()
