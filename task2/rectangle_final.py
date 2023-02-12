
#import msp_define as msp
#import command_func as cf
import socket
import threading
import time
#from pid import keyboard_control,getKey,settings
#import sys, select, termios, tty

# Define MSP commands
MSP_RAW_IMU = 102
MSP_SET_RAW_RC = 200

import cv2 as cv
from cv2 import aruco
import numpy as np
import time 


HOST = '192.168.4.1'  # The server's hostname or IP address
PORT = 23             # The port used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))


class Msp_packet:        #creating a class

    def __init__(self):  #initialising the values
    
        self.roll=1500
        self.pitch=1500
        self.throttle=1500
        self.yaw=1500
        self.AUX1=1500
        self.AUX2=1500
        self.AUX3=1500
        self.AUX4=1000
        self.pitch_trim=10
        self.roll_trim=10  
        self.hex_form=''
        self.msp_trim=''   
        self.pose=[0,0,0]
        self.setpoint = [0, 0, 20]
        self.Kp = [21          ,21       ,45     ,0]  
        self.Ki = [0.6         ,0.2      ,0     ,0]
        self.Kd = [35          ,35       ,3       ,0]
        self.correct_roll = 0.0
        self.correct_pitch = 0.0
        self.correct_yaw = 0.
        self.correct_throt = 0.0
        self.last_time = 0.0
        self.loop_time = 0.025 	#the time interval after which the pid is computed again
        self.previous_error = [0.0,0.0,0.0,0.0]  #[pitch_previous_error, roll_previous_error, throttle_preious_error, yaw__pre_error]
        self.errors_sum = [0.0,0.0,0.0,0.0]   
        self.Current_error = [0.0,0.0,0.0,0.0]
        self.Current_Traversal = 0


    def tohex(self,val):   #converting to hex
        nbits = 16
        hex_value=hex((val + (1 << nbits)) % (1 << nbits))[2:]
        if(len(hex_value)==1):
            return "0"+hex_value
        else:
            return hex_value
    
    def checksum(self,hextr):  #creating checksum 
        l=hextr.split()
        xor=0
        for i in l[3:]:
            try:
                str_int=int(i,16)
                xor^=str_int
            except:
                pass

        if(len(hex(xor)[2:])==1):
            return "0"+hex(xor)[2:]
        return hex(xor)[2:] 
    
    def break_into_onebyte(self,hextr): #splitting the hex values to one byte
        out=""
        if(len(hextr)>1):
            if(len(hextr)%2==0):
                if(len(hextr)==2 and hextr[0]=="0"):
                    out=out+hextr+" "+"00"
                else:
                    out=out+hextr[len(hextr)-2:]+" "+hextr[0:len(hextr)-2]
            else:
                out=out+hextr[len(hextr)-2:]+" "+"0"+hextr[0:len(hextr)-2]
        else:
            out=out+"0"+hextr[0]+" "+"00"
        return out

    def msp_raw_rc_create_packet(self): #creating msp_raw_rc package
        hex_form='24 4d 3c '+hex(16)[2:]+" "+'c8'+" "+self.break_into_onebyte(hex(self.roll)[2:])+" "+self.break_into_onebyte(hex(self.pitch)[2:])+" "+self.break_into_onebyte(hex(self.throttle)[2:])+" "+self.break_into_onebyte(hex(self.yaw)[2:])+" "+self.break_into_onebyte(hex(self.AUX1)[2:])+" "+self.break_into_onebyte(hex(self.AUX2)[2:])+" "+self.break_into_onebyte(hex(self.AUX3)[2:])+" "+self.break_into_onebyte(hex(self.AUX4)[2:])
        hex_form=hex_form+" "+self.checksum(hex_form)
        self.hex_form=hex_form        

    def msp_trim_create_packet(self):   #creating msp trim package
        msp_trim='24 4d 3c '+self.tohex(4)+" "+'ef'+" "+self.break_into_onebyte(self.tohex(self.pitch_trim))+" "+self.break_into_onebyte(self.tohex(self.roll_trim))
        msp_trim=msp_trim+" "+self.checksum(msp_trim)
        self.msp_trim=msp_trim        

    def take_off(self): #take off the drone
	# while(True):
        
        self.msp_raw_rc_create_packet() #form the packets

        #disarm command msp_set_raw_rc
        for i in range(20):
            #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 e8 03 ea'
            
            data = bytes.fromhex(self.hex_form)  ##default is aux4 being 1000
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received disarm {dat}")
        
        
        ##take_off command msp_set_raw_rc
        for i in range(50):
            # data = bytes.fromhex('24 4d 3c 10 c8 dc 05 dc 05 08 07 dc 05 dc 05 dc 05 08 07 dc 05 d8')	
            data = bytes.fromhex('24 4d 3c 02 d9 01 00 da')
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received take_off {dat}")
        
        

        #arm the drone
        self.AUX4=1500  ##all are 1500
        self.throttle = 1900
        self.msp_raw_rc_create_packet()
        for i in range(100):
            #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 d8'
            data = bytes.fromhex(self.hex_form) # msp_set_raw_rc 
            s.sendall(data)
            dat = s.recv(1024)	
            print(f"Received arm {dat}")


    def limit(self, input_value, max_value, min_value):
        if input_value > max_value:
            return max_value
        if input_value < min_value:
            return min_value
        else:
            return input_value
    
    def calc_pid(self):
        
        self.seconds = time.time()
        current_time = self.seconds - self.last_time
        
        if(current_time >= self.loop_time):
            
            self.correct_pitch = self.pid_calculator(0)  #0 pitch
            self.correct_roll = self.pid_calculator(1) * -1	#1 roll 
            self.correct_throt = self.pid_calculator(2)  #2 throtle
            # self.correct_yaw = self.pid_calculator(3)    #3 yaw
            self.Check_Current_Error()
            self.last_time = self.seconds	
    
    def pid_calculator(self, index):
        
            self.Current_error[index] = self.setpoint[index] - self.pose[index]      
            self.errors_sum[index] =self.errors_sum[index] + self.Current_error[index] * self.loop_time 
            self.errDiff = (self.Current_error[index] - self.previous_error[index]) / self.loop_time
            self.Proportional_term = self.Kp[index] * self.Current_error[index]
            self.Derivative_term = self.Kd[index] * self.errDiff
            self.Intergral_term = self.Ki[index] * self.errors_sum[index] 
            self.Computed_pid = self.Proportional_term + self.Derivative_term + self.Intergral_term 
            self.previous_error[index] = self.Current_error[index]
            
            return self.Computed_pid
    
    def Check_Current_Error(self):
        if(self.Current_Traversal < 10):
                if(self.abs(self.Current_error[1]) <= 2):						#change destination when abs value of current error in y-axis is below 0.2
                    if(self.abs(self.Current_error[0]) <= 2):					#change destination when abs value of current error in x-axis is below 0.2
                        self.Current_Traversal = self.Current_Traversal + 1		#increase value of current traversal.
                        self.change_coordinates(self.Current_Traversal)

    def change_coordinates(self, Current_Traversal):	
        if(Current_Traversal == 1):
            self.setpoint = [50, 0, 20]
        elif(Current_Traversal == 2):
            self.setpoint = [ 50, 25, 20]	
        elif(Current_Traversal == 3):
            self.setpoint = [0, 25, 20]		
        elif(Current_Traversal == 4):
            self.setpoint = [0, 0, 20]			

    def hover(self,iter,init_pose):  #hover the drone
        print("Init pose :",init_pose[0],",",init_pose[1],",",init_pose[2])
        self.roll=1500
        self.pitch=1500
        self.throttle=1700
        self.yaw=1500
        self.AUX1=1500
        self.AUX2=1500
        self.AUX3=1500
        self.AUX4=1500
        self.msp_raw_rc_create_packet()
        dec = False
        for i in range(iter):

            self.calc_pid()
            pitch_value = int(1500 - self.correct_pitch)              #adding/subtracting Computed pid to give drone motion in x direction
            self.pitch = self.limit (pitch_value, 1530, 1470)   #clamping pitch value between 1550 - 1450. 
            roll_value = int(1500 + self.correct_roll)				  #adding/subtracting Computed pid to give drone motion in y direction
            self.roll = self.limit(roll_value, 1525,1475)		  #clamping roll value between 1600 - 1400.
            # self.throttle = 1820 - int(i/50)
            # throt_value = int(1500 - self.correct_throt)			  #adding/subtracting Computed pid to give drone motion in z direction
            # self.throttle = self.limit(throt_value, 1800,1250)  #clamping throttle value between 1600 - 1400.
            
            # yaw_value = int(1500 + self.correct_yaw)				  #adding/subtracting Computed pid to make drone stay at a specific orientation
            # self.yaw = self.limit(yaw_value, 1700,1300)	
            # if(i%1000==0):
            #     if(dec==False):
            #         dec = True
            #         self.throttle = 1750
            #     else:
            #         dec = False
            #         self.throttle = 1820

            self.msp_raw_rc_create_packet()
            data = bytes.fromhex(self.hex_form)  ##default is aux4 being 1000
            s.sendall(data)
            time.sleep(0.01)

        # s.close()
        #if self.pose[2] <= 220:
            #    self.throttle=1400
            #    print("below 220")
            #elif self.pose[2] >= 215:
            #    self.throttle=1750
            #    print("above 215")
            #self.msp_raw_rc_create_packet()    
            

   
    def pose_estimate(self):


        calib_data_path ='Aruco_Marker_OpenCV/calib_data/MultiMatrix.npz'

        calib_data = np.load(calib_data_path)
        # print(calib_data.files)

        cam_mat = calib_data["camMatrix"]
        dist_coef = calib_data["distCoef"]
        r_vectors = calib_data["rVector"]
        t_vectors = calib_data["tVector"]

        MARKER_SIZE = 8  # centimeters
        marker_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)

        param_markers = aruco.DetectorParameters_create()

        cap = cv.VideoCapture(1)
        cap.set(cv.CAP_PROP_FPS,30)
        prev_frame_time=0
        scale = 10

            
        while True:
            
            ret, frame = cap.read()
            if not ret:
                break


            new_frame_time = time.time()
            fps = 1/(new_frame_time - prev_frame_time)
            prev_frame_time = new_frame_time

            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, reject = aruco.detectMarkers(
                gray_frame, marker_dict, parameters=param_markers
            )
            if marker_corners:
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners, MARKER_SIZE, cam_mat, dist_coef
                )
                total_markers = range(0, marker_IDs.size)
                for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                    cv.polylines(
                        frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                    )
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = corners[0].ravel()
                    top_left = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    bottom_left = corners[3].ravel()

                    # distance = np.sqrt(
                    #     tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                    # )
                    point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                    x=tVec[i][0][0]
                    y= tVec[i][0][1]
                    z=tVec[i][0][2]
                    # print(tVec)
                    # print(ids, " x= ",x , "y = ",y, "z = ", distance)
                    # print(rVec)
                    self.pose[0]=x
                    self.pose[1]=y
                    self.pose[2]=321-z
                    # print(self.pose[2])

            cv.imshow("frame", frame)
            key = cv.waitKey(1)
            if key == ord("q"):
                break

                
    def rectangle(self):
        self.take_off()
 
        self.hover(2000,self.pose)

        print("ok")

        # self.land()
    

    def trim(self):
        while True:
            self.msp_trim_create_packet()    
            data = bytes.fromhex(self.msp_trim)	##trim msp_set_trim
            s.sendall(data)
            dat = s.recv(1024)
        # for i in range(2000):
        #     data = bytes.fromhex(self.msp_trim)	##trim msp_set_trim
        #     s.sendall(data)
        #     dat = s.recv(1024)
        #     # print(f"Received trim{dat}")
    
    # def land(self):
    #     self.AUX3=1500
    #     self.roll=1500
    #     self.pitch=1500
    #     self.yaw=1500
    #     self.throttle=1000
    #     self.msp_raw_rc_create_packet()

    #     for i in range(50):
    #         self.hex_form
    #         data = bytes.fromhex(self.hex_form) # msp_set_raw_rc 
    #         s.sendall(data)
    #         dat = s.recv(1024)	
    #         print(f"Received land :{dat}")
        
    #     for i in range(20):
    #         data = bytes.fromhex('24 4d 3c 02 d9 02 00 d9')
    #         s.sendall(data)
    #         dat = s.recv(1024)
    #         print(f"Received take_off {dat}")


       

if __name__ =="__main__":
        Order=Msp_packet()
    
        t1 = threading.Thread(target=Order.pose_estimate, name="t1")
        t2 = threading.Thread(target=Order.rectangle,name = "t2")
        # t3 = threading.Thread(target=Order.trim, name="t3")
        # # t4=threading.Thread(target=Order.imu_1, name='t4')
        # Order.imu_1()
        # # # starting thread 1
        t1.start()
        # # # starting thread 2
        t2.start()
        # # time.sleep(5)
        # t3.start()
        # # t4.start()
    
        # # wait until thread 1 is completely executed
        t1.join()
        t2.join()
        # t3.join()
        # t4.join()
        # both threads completely executed
        print("Done!")    