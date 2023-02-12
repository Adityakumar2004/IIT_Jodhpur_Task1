#import msp_define as msp
#import command_func as cf
import socket
import threading
import time
#from pid import keyboard_control,getKey,settings
#import sys, select, termios, tty
HOST = '192.168.4.1'  # The server's hostname or IP address
PORT = 23 # The port used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))


class Msp_packet:

    def __init__(self):
    
        self.roll=1500
        self.pitch=1500
        self.throttle=1500
        self.yaw=1500
        self.AUX1=1500
        self.AUX2=1500
        self.AUX3=1500
        self.AUX4=1000
        self.pitch_trim=0
        self.roll_trim=4
        self.command_type=1    
        self.hex_form=''
        self.msp_trim=''   
        self.key='' 
        self.pose=[]
    
    def tohex(self,val):
        nbits = 16
        hex_value=hex((val + (1 << nbits)) % (1 << nbits))[2:]
        if(len(hex_value)==1):
            return "0"+hex_value
        else:
            return hex_value
    
    def checksum(self,hextr):
        l=hextr.split()
        xor=0
        for i in l[3:]:
            str_int=int(i,16)
            xor^=str_int

        if(len(hex(xor)[2:])==1):
            return "0"+hex(xor)[2:]
        return hex(xor)[2:] 
    
    def break_into_onebyte(self,hextr):
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

    def msp_raw_rc_create_packet(self):
        hex_form='24 4d 3c '+hex(16)[2:]+" "+'c8'+" "+self.break_into_onebyte(hex(self.roll)[2:])+" "+self.break_into_onebyte(hex(self.pitch)[2:])+" "+self.break_into_onebyte(hex(self.throttle)[2:])+" "+self.break_into_onebyte(hex(self.yaw)[2:])+" "+self.break_into_onebyte(hex(self.AUX1)[2:])+" "+self.break_into_onebyte(hex(self.AUX2)[2:])+" "+self.break_into_onebyte(hex(self.AUX3)[2:])+" "+self.break_into_onebyte(hex(self.AUX4)[2:])
        hex_form=hex_form+" "+self.checksum(hex_form)
        self.hex_form=hex_form        

    def msp_trim_create_packet(self):
        msp_trim='24 4d 3c '+self.tohex(4)+" "+'ef'+" "+self.break_into_onebyte(self.tohex(self.pitch_trim))+" "+self.break_into_onebyte(self.tohex(self.roll_trim))
        msp_trim=msp_trim+" "+self.checksum(msp_trim)
        self.msp_trim=msp_trim        

    def arm(self):
        self.msp_raw_rc_create_packet()
        hex_form = self.hex_form
        
        for i in range(150):
            data = bytes.fromhex(hex_form)
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received new {dat}")

        for i in range(150):
            data = bytes.fromhex('24 4d 3c 10 c8 dc 05 dc 05 e8 03 dc 05 dc 05 dc 05 dc 05 dc 05 ea')	
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received {dat}")

        while(True):
            data = bytes.fromhex('24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 d8') # msp_set_raw_rc 
            s.sendall(data)
            dat = s.recv(1024)	
            print(f"Received new {dat}") 

    def take_off(self):
	# while(True):
        
        self.msp_raw_rc_create_packet() #form the packets

        #disarm command msp_set_raw_rc
        for i in range(150):
            #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 e8 03 ea'
            
            data = bytes.fromhex(self.hex_form)  ##default is aux4 being 1000
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received disarm {dat}")
        
        
        ##take_off command msp_set_raw_rc
        for i in range(150):
            # data = bytes.fromhex('24 4d 3c 10 c8 dc 05 dc 05 08 07 dc 05 dc 05 dc 05 08 07 dc 05 d8')	
            data = bytes.fromhex('24 4d 3c 02 d9 01 00 da')
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received take_off {dat}")
        
        

        #arm the drone
        self.AUX4=1500  ##all are 1500
        self.throttle = 1700
        self.msp_raw_rc_create_packet()
        for i in range(150):
            #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 d8'
            data = bytes.fromhex(self.hex_form) # msp_set_raw_rc 
            s.sendall(data)
            dat = s.recv(1024)	
            print(f"Received arm {dat}")

    
    def forward(self,iter):
        self.pitch=1600
        self.msp_raw_rc_create_packet()
        for i in range(iter):
                #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 e8 03 ea'
                
            data = bytes.fromhex(self.hex_form)  ##default is aux4 being 1000
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received forward {dat}")
    def backward(self,iter):
        self.pitch=1400
        self.msp_raw_rc_create_packet()

        for i in range(iter):
            #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 e8 03 ea'
            
            data = bytes.fromhex(self.hex_form)  ##default is aux4 being 1000
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received backward {dat}")
            # print('backward')


    def left(self,iter):
        self.roll=1400
        self.msp_raw_rc_create_packet()

        for i in range(iter):
            #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 e8 03 ea'
        
            data = bytes.fromhex(self.hex_form)  ##default is aux4 being 1000
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received left {dat}") 
        # print('left')

    def right(self,iter):
        self.roll=1600
        self.msp_raw_rc_create_packet()

        for i in range(iter):
                #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 e8 03 ea'
                
            data = bytes.fromhex(self.hex_form)  ##default is aux4 being 1000
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received right {dat}")
            # print('backward')
    # print('right')
    # def hover(self,iter,init_pose):
    def hover(self,iter):
        self.roll=1500
        self.pitch=1500
        self.throttle=1550
        self.yaw=1500
        self.AUX1=1500
        self.AUX2=1500
        self.AUX3=1500
        self.AUX4=1500
        self.msp_raw_rc_create_packet()
        for i in range(iter):
                #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 e8 03 ea'
                
            data = bytes.fromhex(self.hex_form)  ##default is aux4 being 1000
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received hover {dat}")

        # for i in range(iter):
        #         #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 e8 03 ea'
            
        #     if(i%20==0 and self.pose!=None and self.pose[0]-init_pose[0] > 10):
        #         self.roll_trim -= 1
        #     if(i%20==0 and self.pose!=None and self.pose[0]-init_pose[0] < 10):
        #         self.roll_trim += 1
        #     if(i%20==0 and self.pose!=None and self.pose[1]-init_pose[1] > 10):
        #         self.pitch_trim -= 1
        #     if(i%20==0 and self.pose!=None and self.pose[1]-init_pose[1] > 10):
        #         self.pitch_trim += 1
        #     if(i%20==0 and self.pose!=None and self.pose[2]-init_pose[2] > 10):
        #         self.decrease_height(150)
        #     if(i%20==0 and self.pose!=None and self.pose[2]-init_pose[2] < 10):
        #         self.increase_height(150)
            
        #     data = bytes.fromhex(self.hex_form)  ##default is aux4 being 1000
        #     s.sendall(data)
        #     dat = s.recv(1024)
        #     print(f"Received hover {dat}")
    
    def increase_height(self,iter):
        self.throttle=1800
        self.msp_raw_rc_create_packet()
        for i in range(iter):
                #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 e8 03 ea'
                
            data = bytes.fromhex(self.hex_form)  ##default is aux4 being 1000
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received right {dat}")
        # print("increase_height")

    def decrease_height(self,iter):
        self.throttle=1300
        self.msp_raw_rc_create_packet()
        for i in range(iter):
                #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 e8 03 ea'
                
            data = bytes.fromhex(self.hex_form)  ##default is aux4 being 1000
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received right {dat}")
        # print('decrease_height')

    def rectangle(self):
        self.take_off()
        
        self.forward(150)
        self.backward(20)
        self.hover(20)
        # self.decrease_height(50)
        # self.hover(150)
        self.right(150)
        self.left(20)
        self.hover(20)
        self.backward(150)
        self.forward(20)
        self.hover(20)
        # self.decrease_height(50)
        # self.hover(150)
        self.left(500)
        self.right(20)
        self.hover(20)
        # self.decrease_height(50)
       
        # self.hover(150,self.pose)
        self.land()
    

    def trim(self):
        self.msp_trim_create_packet()
        while self.key!='e':    
            data = bytes.fromhex(self.msp_trim)	##trim msp_set_trim
            s.sendall(data)
            dat = s.recv(1024)
        for i in range(2000):
            data = bytes.fromhex(self.msp_trim)	##trim msp_set_trim
            s.sendall(data)
            dat = s.recv(1024)
            # print(f"Received trim{dat}")
    def land(self):
        self.AUX3=1500
        self.roll=1500
        self.pitch=1500
        self.yaw=1500
        self.throttle=1000
        self.msp_raw_rc_create_packet()

        for i in range(50):
            self.hex_form
            data = bytes.fromhex(self.hex_form) # msp_set_raw_rc 
            s.sendall(data)
            dat = s.recv(1024)	
            print(f"Received land :{dat}")
        
        for i in range(20):
            data = bytes.fromhex('24 4d 3c 02 d9 02 00 d9')
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received land {dat}")
    
if __name__ =="__main__":
        Order=Msp_packet()
    
        t1 = threading.Thread(target=Order.rectangle, name="t1")
        # t2 = threading.Thread(target=Order.trim, name = "t2")
    
        # # starting thread 1
        t1.start()
        # # starting thread 2
        # t2.start()
    
        # wait until thread 1 is completely executed
        t1.join()
  
        # t2.join()

        # both threads completely executed
        print("Done!")    