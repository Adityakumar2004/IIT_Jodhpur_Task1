import msp_define as msp
#import command_func as cf
import socket
import threading
import time
from pid import keyboard_control,getKey,settings
import sys, select, termios, tty
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
        self.pitch_trim=5
        self.roll_trim=5
        self.command_type=1    
        self.hex_form=''
        self.msp_trim=''   
        self.key='' 
    
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
        
        for i in range(100):
            data = bytes.fromhex(hex_form)
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received new {dat}")

        for i in range(100):
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
        self.command_type=1
        
        self.msp_raw_rc_create_packet() #form the packets

        #disarm command msp_set_raw_rc
        for i in range(100):
            #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 e8 03 ea'
            
            data = bytes.fromhex(self.hex_form)  ##default is aux4 being 1000
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received disarm {dat}")
        
        
        ##take_off command msp_set_raw_rc
        for i in range(20):
            # data = bytes.fromhex('24 4d 3c 10 c8 dc 05 dc 05 08 07 dc 05 dc 05 dc 05 08 07 dc 05 d8')	
            data = bytes.fromhex('24 4d 3c 02 d9 01 00 da')
            s.sendall(data)
            dat = s.recv(1024)
            print(f"Received take_off {dat}")
        
        

        #arm the drone
        self.AUX4=1500  ##all are 1500
        self.throttle = 1500
        self.msp_raw_rc_create_packet()
        for i in range(200):
            #'24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 d8'
            data = bytes.fromhex(self.hex_form) # msp_set_raw_rc 
            s.sendall(data)
            dat = s.recv(1024)	
            print(f"Received arm {dat}")

        self.command_type=0

    #self.land()
        #     data = bytes.fromhex('24 4d 3c 04 ef ff ff f4 ff e0')	##trim msp_set_trim
        #     s.sendall(data)
        #     dat = s.recv(1024)
        #     print(f"Received trim{dat}")    

            # self.throttle=1300
            # #print(self.hex_form)
            # data = bytes.fromhex(self.hex_form) # msp_set_raw_rc 
            # dat = s.recv(1024)	
            # print(f"Received 1500's all :{dat}")
            
# Continuous throttle

            # self.throttle=1500
            # #print(self.hex_form)
            # for i in range(self.throttle, self.throttle-200):
            #     self.throttle=i
            #     data = bytes.fromhex(self.hex_form) # msp_set_raw_rc 
            #     s.sendall(data)
            #     dat = s.recv(1024)	
            #     print(f"Received 1500's all :{dat}")


            #     data = bytes.fromhex('24 4d 3c 04 ef ff ff f4 ff e0')	##trim msp_set_trim
            #     s.sendall(data)
            #     dat = s.recv(1024)
            #     print(f"Received trim{dat}")
                

            # self.throttle=1460
            # #print(self.hex_form)
            # data = bytes.fromhex(self.hex_form) # msp_set_raw_rc 
            # s.sendall(data)
            # dat = s.recv(1024)	
            # print(f"Received 1500's all :{dat}")

            #self.left_yaw()

            #data = bytes.fromhex(self.hex_form) # msp_set_raw_rc 
            #s.sendall(data)
            #dat = s.recv(1024)	
            #print(f"Received 1500's all :{dat}")


        #pass
        
    def forward(self):
        self.pitch=1550
        self.msp_raw_rc_create_packet()
        print("forward")

    def trim(self):
        self.msp_trim_create_packet()
        while self.key!='e':  
            self.msp_trim_create_packet()  
            data = bytes.fromhex(self.msp_trim)	##trim msp_set_trim
            s.sendall(data)
            dat = s.recv(1024)
            print(self.pitch_trim)
        for i in range(2000):
            data = bytes.fromhex(self.msp_trim)	##trim msp_set_trim
            s.sendall(data)
            dat = s.recv(1024)
            # print(f"Received trim{dat}")
        # pass

    def backward(self):
        self.pitch=1425   
        self.msp_raw_rc_create_packet()
        print('backward')


    def left(self):
        self.roll=1450
        self.msp_raw_rc_create_packet() 
        print('left')

    def right(self):
        self.roll=1550
        self.msp_raw_rc_create_packet()
        print('right')

    def left_yaw(self):
        self.yaw=1300
        self.msp_raw_rc_create_packet()
        print('left_yaw')

    def right_yaw(self):
        self.yaw=1700
        self.msp_raw_rc_create_packet()
        print('rigth_yaw')

    def increase_height(self):
        self.throttle=1700
        self.msp_raw_rc_create_packet()
        print("increase_height")

    def decrease_height(self):
        self.throttle=1400
        self.msp_raw_rc_create_packet()
        print('decrease_height')

    def altitude_decode(self,en):
        try:
            out=""
            for i in en[2:]:
                if(i!="\\" and i!="x" and i!="'"):
                    out=out+i
                if(i=="\\"):
                    out=out+" "
            #print(out)
            l=out.split()
            # print(l)
            if(l[0]=="$M>"):
                msg_len=int("0x"+l[1][0:2],0)
                req_msg_len=2
                out=""
                for i in range(msg_len+2-req_msg_len, msg_len+2):
                    if(len(l[i])>2):
                        out=l[i][0:2]+out
                        for j in range(2,len(l[i])):
                            out=out+hex(ord(l[i][j]))[2:]
                    else:
                        out=l[i]+out
                print(out)
                print(int(out,16))
            # print(int.from_bytes(bytes.fromhex(hex(int(out,16))[2:]), byteorder='big', signed=True))
        except:
            pass
    
    def altitude(self):
        # '24 4d 3c 10 c8 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 d8'
        while 1:
            data = bytes.fromhex('24 4d 3c 00 6d 6d') # msp_set_raw_rc 
            s.sendall(data)
            dat = s.recv(1024)	
            print(f"Received altitude data {dat}")
            self.altitude_decode(str(dat))
            # en=str(b'$M>\x06m\xce\x00\x00\x00\x10i\x00m\xb5')
            # self.altitude_decode(en)

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
            print(f"Received landed {dat}")


    def trim_left_roll_key(self):
        self.roll_trim=self.roll_trim-5
        print('trimed left')

    def trim_right_roll_key(self):
        self.roll_trim=self.roll_trim+5   
        print('trimed right')

    def trim_back_pitch_key(self):
        self.pitch_trim=self.pitch_trim-5
        print('trimed back')

    def trim_front_pitch_key(self):
        self.pitch_trim=self.pitch_trim+5
        print('trimed front')



    #def set_of_commands(self):
    #    msp_command={msp.MSP_SET_RAW_RC:self.arm, msp.MSP_SET_COMMAND:self.take_off, msp.MSP_ATTITUDE:self.attitude, msp.MSP_ALTITUDE:self.altitude, msp.MSP_RAW_IMU:self.imu}
    #    return msp_command
    
    def hover(self):
        self.roll=1500
        self.pitch=1500
        self.throttle=1500
        self.yaw=1500
        self.AUX1=1500
        self.AUX2=1500
        self.AUX3=1500
        self.AUX4=1500
        self.msp_raw_rc_create_packet()

    # def balance(self,prev):
    #     if(prev==10):
    #         for i in range(100):
    #             self.backward()
        
    #     elif(prev==110):


    
    def keyboard_input_control(self): #runs parallel in a thread
        prev=0
        # self.msp_trim_create_packet()
        while 1:
            
            self.key=getKey()
            prev = keyboard_control[self.key]

            self.key_handling(keyboard_control[self.key])
            
            if self.key=='\x03' or self.key=='e':
                break

            if self.key=="":
                # self.balance(prev)
                self.hover()
            
            if self.command_type==0:
                print("here on default")
                data = bytes.fromhex(self.hex_form) # msp_set_raw_rc 
                s.sendall(data)
                dat = s.recv(1024)
                
                # data = bytes.fromhex(self.msp_trim)	##trim msp_set_trim
                # s.sendall(data)
                # dat = s.recv(1024)
            
            else:
                pass    
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    def key_handling(self,key):
        if key==10:
            self.forward()

        elif key==None:
            pass;
        
        elif key==30:
            self.left()
        
        elif key==40:
            self.right()    
        
        elif key==90:
            if self.auto_pilot() ==1:
                self.auto_pilot=0      
            else:
                 self.auto_pilot=1
        
        elif key==50:
            self.increase_height()
        
        elif key==60:
            self.decrease_height()   
        
        elif key==110:
            self.backward()     
        
        elif key==130:
            self.take_off()    
        
        elif key==140:
            self.land()
        
        elif key==150:
            self.left_yaw()  
        
        elif key==160:
            self.right_yaw()
         
        elif key==105:
            self.trim_left_roll_key()


        elif key==106:
            self.trim_right_roll_key() 

        elif key==205:
            self.trim_front_pitch_key()

        elif key==206:
            self.trim_back_pitch_key()     

    def control(self):##runs parallel in thread
        while 1:
            if self.command_type==0:
                data = bytes.fromhex(self.hex_form) # msp_set_raw_rc 
                s.sendall(data)
                dat = s.recv(1024)  
            else:
                pass;    
    
# Order=Msp_packet()
# Order.trim()
if __name__ =="__main__":
    Order=Msp_packet()
    # Order.keyboard_input_control()
    # creating thread
    #Order.take_off()
    t1 = threading.Thread(target=Order.keyboard_input_control, name="t1")
    #t2 = threading.Thread(target=Order.trim, name = "t2")
 
    # # starting thread 1
    t1.start()
    # # starting thread 2
    #t2.start()
 
    # wait until thread 1 is completely executed
    t1.join()
    # wait until thread 2 is completely executed
   # t2.join()

    # both threads completely executed
    print("Done!")
