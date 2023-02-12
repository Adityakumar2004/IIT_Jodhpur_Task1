# Inter_IIT

## Prerequisites
- Ubuntu 20.04/Ubuntu 22.04
- Python3 

## Installation
- OpenCv installation 
```
$ sudo apt update
```
```
$ sudo apt install libopencv-dev python3-opencv
```
- Install numpy
```
pip3 install numpy
```
- Install aruco
```
pip3 install aruco
```

## Program Files
- [rectangle_2](https://github.com/Adityakumar2004/inter_iit/blob/main/rectangle_2.py) : Performs ```task 2```, moving the drone in a rectangle. 
- [Aruco Marker OpenCV](https://github.com/Adityakumar2004/team_ID_51_Drona/tree/main/task2/Aruco%20Marker%20OpenCV): Performs the task 2 to detect the ```Aruco marker``` and returns the respective position coordinates.
    - [CAMERA-CALIBARTION](https://github.com/Adityakumar2004/team_ID_51_Drona/tree/main/task2/Aruco%20Marker%20OpenCV/CAMERA-CALIBARTION):Camera calibration helps                                        in obtaining the camera intrinsic parameters and distortion coefficients. This parameters remain fixed unless the camera optic                                        is modified, thus camera calibration only need to be done once. For the same we have uploaded chessboard images in different-                                        different plane and angle. 
    - [POSE-ESTIMATION](https://github.com/Adityakumar2004/team_ID_51_Drona/tree/main/task2/Aruco%20Marker%20OpenCV/POSE-ESTIMATION): The camera pose relative to                                       the marker is a 3d transformation from the marker coordinate system to the camera coordinate system. It is specified by rotation                                     and translation vectors.
    - [MARKER-DETECTION](https://github.com/Adityakumar2004/team_ID_51_Drona/tree/main/task2/Aruco%20Marker%20OpenCV/MARKER-DETECTION): We will apply the ArUco                                         detector with OpenCV’s cv2.aruco.detectMarkers function.
    - [Pattern-PNG](https://github.com/Adityakumar2004/team_ID_51_Drona/tree/main/task2/Aruco%20Marker%20OpenCV/Pattern-PNG): Pattern png for camera calibration.
    - [calib_data](https://github.com/Adityakumar2004/team_ID_51_Drona/tree/main/task2/Aruco%20Marker%20OpenCV/calib_data):As a result of the calibration, we get a                                          camera matrix: a matrix of 3x3 elements with the focal distances and the camera center coordinates, and the distortion c                                              coefficients: a vector of 5 or more elements that models the distortion produced by your camera.
              

