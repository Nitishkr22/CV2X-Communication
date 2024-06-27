#!/usr/bin/python
from     gps import *
from     time import *
from     os import path
import   threading, thread
import   zmq, sys, serial, time
import   smbus
import   obu_config as config
import   obu_id as node_id
import   numpy as np
from datetime import datetime

#################################   Global variables    #################################
### Configurable parameters
OBU_ID            = node_id.obu_id

### Initialising bluetooth port##
ser = serial.Serial('/dev/ttymxc3', 115200, timeout=0.50)
ser.write("K\n")
time.sleep(0.05)
ser.write("A\n")
time.sleep(0.02)


### Logging
LOGLVL_ERR        =  1     ### 1 : Enables log updates with log_level = ERROR
LOGLVL_WARN       =  1     ### 1 : Enables log updates with log_level = WARNING
LOGLVL_INFO       =  0     ### 1 : Enables log updates with log_level = INFO
DEBUG_LOG_PATH    =  "/home/guest/app_logs/"        ### File : case1-applog__obu-*

# Packet Fields
APP_PKT_TYPE = "01"    # 01 for CASE1; 02 for CASE2
SOURCE_TYPE  = "00"    # 00 for Source packet; SourceID for relay packet                                    
                                                                                         
# GPS                                                                                    
gpsd = None                                                                              
                                                                                                            
# Accelerometer                                                                   
Device_Addr  = 0x68   # MPU9250 device address                                    
# MPU9250 Registers and their Address                                                    
SMPLRT_DIV   = 0x19                                                                      
CONFIG       = 0x1A                                                                                         
GYRO_CONFIG  = 0x1B                                                                                         
INT_ENABLE   = 0x38                                                                                         
ACCEL_XOUT_H = 0x3B                                                                                         
ACCEL_YOUT_H = 0x3D                                                                                         
ACCEL_ZOUT_H = 0x3F                                                                   
GYRO_XOUT_H  = 0x43                                                                                         
GYRO_YOUT_H  = 0x45                                                                                         
GYRO_ZOUT_H  = 0x47                                                                                         
PWR_MGMT_1   = 0x6B                                                                   
                                                                                         
                                                                                      
########################   Global Function Definitions   #############################
                                                                                      
                                                                                  
###############################  Boot Logging #################################   
### Function Description : Log boot information to a file                      
### Arguments :                                                       
###   data        -> Information to be updated to log in string format                                  
###   dbg_level   -> Level of debug information (DEBUG/DEFAULT)                                         
def update_boot_log(data, dbg_level):                                                                   
   if (dbg_level == 1):                                                                  
      global utc_date                                                                                   
      global utc_time                                                                                   
      todays_date = utc_date                                                                            
      time_now = utc_time                                                                               
      #time_now = datetime.now()                                                                        
      debug_log_fullpath  = DEBUG_LOG_PATH + "case1-applog__obu-" + OBU_ID + "__" + todays_date + ".txt"
      if path.exists(debug_log_fullpath):                                                               
         debug_logfile = open(debug_log_fullpath,"a+")                                
      else:                                                                           
         debug_logfile = open(debug_log_fullpath,"w+")                                
                                                                                      
      debug_logfile.write("/-------------------------------------------------/" +"\n")
      debug_logfile.write("      OBU ID : " + data +"\n")                             
      debug_logfile.write("/-------------------------------------------------/" +"\n")
      debug_logfile.write(time_now + " : Test Application 1 Booting..." +"\n")                          
      debug_logfile.close()                                                                             
                                                                                  
                                                                                      
###############################  Debug Logging #################################      
### Function Description : Log debug information to a file                      
### Arguments :                                                                       
###   data        -> Information to be updated to log in string format                                      
###   dbg_level   -> Level of debug information (DEBUG/DEFAULT)                          
def update_debug_log(data, dbg_level):                                                   
   if (dbg_level == 1):                                                                  
      global utc_date                                                                                       
      global utc_time                                                                 
      todays_date = utc_date                                                          
      time_now = utc_time                                                                
      #time_now = datetime.now()                                                         
      debug_log_fullpath  = DEBUG_LOG_PATH + "case1-applog__obu-" + OBU_ID + "__" + todays_date + ".txt"    
      if path.exists(debug_log_fullpath):                                                                   
         debug_logfile = open(debug_log_fullpath,"a+")                                                      
      else:                                                                                                 
         debug_logfile = open(debug_log_fullpath,"w+")                                                      
      debug_logfile.write("case1-obu @ " + time_now + " >> " + data +"\n")            
      debug_logfile.close()                                                                                 
                                                                                                            
                                                                                                            
###############################  GPS Polling thread #################################                   
### Function Description : Class to poll for GPS updates                                                
class GpsPoller(threading.Thread):                                                                      
   def __init__(self):                                                                                  
      threading.Thread.__init__(self)                                                                   
      global gpsd #bring it in scope                                                                    
      gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info                                        
      self.current_value = None                                                                         
      self.running       = True                                                      
   def run(self):                                                                                       
      global gpsd                                                                                       
      while gpsp.running:                                                                               
         gpsd.next()                                                                     
                                                                                                        
###############################  Accelerometer Functions ###########################                    
### Function Description : Initialize Accelerometer                                                     
def accelerometer_init():                                                                               
   bus.write_byte_data(Device_Addr, SMPLRT_DIV, 7)                                                      
   bus.write_byte_data(Device_Addr, PWR_MGMT_1, 1)                                                      
   bus.write_byte_data(Device_Addr, CONFIG, 0)                                                          
   bus.write_byte_data(Device_Addr, GYRO_CONFIG, 24)                                  
   bus.write_byte_data(Device_Addr, INT_ENABLE, 1)                                    
                                                                                      
### Function Description : Read accelerometer readings                                
def accelerometer_read(addr):                                                         
   high  = bus.read_byte_data(Device_Addr, addr)                                      
   low   = bus.read_byte_data(Device_Addr, addr+1)                                    
   value = ((high << 8) | low)                                                                          
   if(value > 32768):                                                                                   
          value = value - 65536                                                                         
   return value                                                                       
                                                                                                           
#####conversion from LAT-LONG to X,Y,Z coordinate####                                 
def get_cartesian(lat=None,lon=None):                                                 
    lat, lon = np.deg2rad(lat), np.deg2rad(lon)                                                         
    R = 6371 # radius of the earth                                                                          
    x = R * np.cos(lat) * np.cos(lon)                                                 
    y = R * np.cos(lat) * np.sin(lon)                                                 
    z = R *np.sin(lat)                                                              
    return x,y,z                                                                     
#####calculating heading angle#####                                                                         
                                                                                                            
def get_heading(aLocation):                                                                                 
       off_x = aLocation[-1][1] - aLocation[-2][1]                                                          
       off_y = aLocation[-1][0] - aLocation[-2][0]                                                          
       heading = 90.00 + math.atan2(-off_y, off_x) * 57.2957795                                         
       if heading < 0:                                                                                      
           heading += 360.00                                                                                
       return heading                                                                                       
                                                                                     
                                                                                         
                                                                                                        
#################################### Main code #####################################                    
                                                                                                        
# Accelerometer                                                                                         
bus = smbus.SMBus(4)    # Accelerometer is connected to I2C-4                       
accelerometer_init()    # Initiate accelerometer module                                                 
                                                                                                        
                                                                                                        
if __name__ == '__main__':                                                                              
                                                                                                        
   global utc_date                                                                       
   utc_date = "000000"                                                                                  
   global utc_time                                                                       
   utc_time       = "000000"                                                          
   ser.write("A\n")                                                                   
   try:                                                                               
                                                                                      
      ################################  Transmit Function ######################################
      def dsrc_send(threadName):                                                                
            global gpsp                                                                                 
            global utc_date                                                                     
            global utc_time                                                                     
            global x1,y1,z1                                                                     
            global head_ang                                                                             
            port = "8888"                                                                       
            context = zmq.Context()                                                             
            tx_socket  = context.socket(zmq.PAIR)                                   
            k =[]                                                                               
            alocation=[[0,0]]                                                                   
            cnt =0                                                                    
            x1=0                                                                                        
            y1=0                                                                                        
            z1=0                                                                                            
            try:                                                                                
               tx_socket.connect("tcp://127.0.0.1:%s" % port)                                   
               gpsp = GpsPoller() # Main Thread - Polling GPS packets                                   
               gpsp.start()                                                                                 
               gpsd.utc = 0 # initialization                                                    
               seqno = 1                                                                        
                                                                                                
               utc_prev_time = "chk" #initial value                                             
                                                                                                            
               while True:                                                                                  
                     #####################  Accelerometer Reading #######################                   
                     # Read Accelerometer raw value                                                         
                     acc_x = accelerometer_read(ACCEL_XOUT_H)                                               
                     acc_y = accelerometer_read(ACCEL_YOUT_H)                                           
                     acc_z = accelerometer_read(ACCEL_ZOUT_H)                                               
                                                                                                            
                     # Read Gyroscope raw value                                                             
                     gyro_x = accelerometer_read(GYRO_XOUT_H)                                   
                     gyro_y = accelerometer_read(GYRO_YOUT_H)                                   
                     gyro_z = accelerometer_read(GYRO_ZOUT_H)                                           
                                                                                                        
                     # Full scale range +/- 250 degree/C as per sensitivity scale factor                
                     Ax = acc_x/16384.0                                                                 
                     Ay = acc_y/16384.0                                                         
                     Az = acc_z/16384.0                                                                 
                     Gx = gyro_x/131.0                                                                  
                     Gy = gyro_y/131.0                                                                  
                     Gz = gyro_z/131.0                                                                  
                                                                                                        
                     # Data formatting                                                          
                     acce_data=str(Ax)[:4]+","+str(Ay)[:4]+","+str(Az)[:4]                              
                     gyro_data=str(Gx)[:4]+","+str(Gy)[:4]+","+str(Gz)[:4]                      
                   ######################## abrupt data not required##################          
                    # if(Gz>3.8):                                                               
                    #    ser.write("shw,0018,34\n")                                             
                    #    print("abrupt left")                                                   
                    #    time.sleep(0.7)                                                        
                    #    ser.write("shw,0018,30\n")                                             
                    # if(Gz<-3.8):####################################################  ABRUPT RIGHT OF MY CAR
                    #    ser.write("shw,0018,35\n")                                                           
                    #    print("abrupt right")                                                                
                    #    time.sleep(0.7)                                                                      
                    #    ser.write("shw,0018,30\n")                                                           
                                                                                                              
                     ######################  GPS Reading and Packet Encoding  ################                
                     utc = str(gpsd.utc)                                                                      
                     E=0                                                                                      
                     # If GPS packet is valid                                                                 
                     if utc != "0" and utc != "None":                                                         
                        #utc = str(gpsd.utc)                                                                  
                        utc_date_full = utc.split("T",1)[0]  #Extracting date                                 
                        year, month, date = utc_date_full.split('-')                                          
                        utc_date = date+month+year[2:]                                                        
                        utc_time_full = utc.split("T",1)[1] #Extracting time                    
                        utc_time_seg = utc_time_full[:8]                                                      
                        hrs, mins, secs = utc_time_seg.split(':')                                             
                        utc_time = hrs+mins+secs                                                              
                                                                                                              
                     if utc_time != "000000" and utc_time != utc_prev_time:                                   
                        # Data Encoding                                                                       
                        speed = gpsd.fix.speed * (18/5)                                                       
                                                                                                              
                      ####################################### SLOW DOWN ###############                     
                                                                                                              
                        if(speed>20):                                                                         
                           ser.write("shw,0018,32\n")                                                         
                           print("speed limit exceeded")                                                      
                           time.sleep(0.7)                                                                    
                           ser.write("shw,0018,30\n")                                                       
                        k.append(speed)                                                                       
                        #print(k)                                                                             
                        if(cnt<6):                                                                      
                          cnt += 1                                                                            
                        else:                                                                                 
                          diff1 = abs((k[-2]) - (k[-1]))                                                      
                          diff2 = abs((k[-3]) - (k[-1]))                                                      
                          diff3 = abs((k[-4]) - (k[-1]))                                                      
                          diff4 = abs((k[-5]) - (k[-1]))                                                      
                          diff5 = abs((k[-6]) - (k[-1]))                                                      
                          #print("SPEED DIFF: ",diff1,diff2,diff3,diff4,diff5)                                
                          if (diff1>6 or diff2>6 or diff3>6 or diff4>6 or diff5>6) and (speed<1.5):           
                             E=1                                                                   
                                                                                                        
                       ################################### emergency break not required ######################################################
                                                                                                                                              
                          #   print("Emergency break applied")                                                                                
                          #   ser.write("shw,0018,32\n")                                                                                      
                          #   time.sleep(0.7)                                                                                                 
                          #   ser.write("shw,0018,30\n")                                                                                      
                        x1,y1,z1 = get_cartesian(gpsd.fix.latitude,gpsd.fix.longitude)                                                        
                                                                                                                                              
                        alocation.append([gpsd.fix.latitude,gpsd.fix.longitude])                                                              
                       # print(alocation)                                                                                                     
                        head_ang= get_heading(alocation)                                                                                      
                        print(get_heading(alocation))                                                                                         
                       # print(len(alocation))                                                                                                
                                                                                                                                              
                        current_date = datetime.now()                                                                                         
                        #data_info= "UTCTIME,"+utc_time+",A,"+str(gpsd.fix.latitude)[:11]+","+str(gpsd.fix.longitude)[:11]+","
+"S,"+str(speed)[:6]+","+str(gpsd.fix.track)[:4]+",acc"+acce_data+",gyro"+gyro_data+",UTCDATE,"+utc_date
                        data_info=utc_time+",A,"+str(gpsd.fix.latitude)[:11]+","+str(gpsd.fix.longitude)[:11]+","
+str(gpsd.fix.speed)[:6]+","+str(gpsd.fix.track)[:4]+","+acce_data+","+gyro_data+","+utc_date
                        # Packet Encoding
                        hdr_payload  = OBU_ID +","+SOURCE_TYPE+","+APP_PKT_TYPE
                        data_payload = data_info+","+str(seqno)
                        chksum       = "TIHAN"
                        #full_pkt= "V2I,"+hdr_payload+","+str(len(data_payload))+","+data_payload+","+str(head_ang)+","+str(x1)+","+str(y1)+","+str(z1)+","+str(E)+","+str(current_date)+","+chksum
                        full_pkt= "V2I,"+hdr_payload+","+str(len(data_payload))+","+data_payload+","+str(head_ang)+","+str(x1)+","+str(y1)+","+str(z1)+","+str(E)+","+chksum

                        utc_prev_time = utc_time

                        time.sleep(0.2)
                        # Send packet
                        tx_socket.send(full_pkt , flags=zmq.NOBLOCK)
                        print(full_pkt)
                        update_debug_log(full_pkt + '\n', LOGLVL_INFO)
                        seqno=seqno+1
                        if seqno>=65536:
                          seqno=1

                        #utc_prev_time = utc_time
                        #time.sleep(0.20)
                     else:
                        # Zero pad for GPS fields
                        print("no GPS")
                        #data_info="000000,V,0.000000000,00.00000000,00.000,00.0"+","+acce_data+","+gyro_data+","+"000000"
                        #data_info="A"+acce_data+",G"+gyro_data
                        time.sleep(0.20)  ###### may be removed once valid gps has been tested


                     #seqno=seqno+1

            finally:
                tx_socket.close()




      ################################  Receive Function ######################################
      def dsrc_recv(threadName): ### Not currently in use
         port = "8889"
         context = zmq.Context()
         rx_socket = context.socket(zmq.SUB)
         rx_socket.connect("tcp://127.0.0.1:%s" % port)
         rx_socket.setsockopt(zmq.SUBSCRIBE, "")
         global dist
         dist = 0
         global out
         m=[]
         count=0
         while True:
            try:
               msg = rx_socket.recv()
               time.sleep(0.1)
               out = msg.split(",")
               #print("rx: ",out)


               if out[1]=='D1' or out[1]=='D2' or out[1]=='D3':

                 ############## EMERGENCY BREAK ####################################

                 if(out[-2]==str(1)):
                      ser.write("shw,0018,36\n")
                      time.sleep(0.7)
                      ser.write("shw,0018,30\n")

                ############## ABNORMAL BEHAVIOR ####################################

                 if(float(out[-9])>3.8 or float(out[-9])<-3.8):
                      ser.write("shw,0018,37\n")
                      time.sleep(0.7)
                     # print("abnormal")
                      ser.write("shw,0018,30\n")

                ############# VEHICLE 30m AWAY #####################################

                 dist = np.sqrt((float(out[-5]) - x1)**2 + (float(out[-4]) - y1)**2 + (float(out[-3]) - z1)**2)
                 time.sleep(0.1)
                 differ= (float(out[-6]) - head_ang)
                 time.sleep(0.1)
                 print(dist)
                 #print("speed ",out[9])
                 if(dist<0.03):
                   ser.write("shw,0018,39\n")
                   time.sleep(0.7)
                   ser.write("shw,0018,30\n")

                ################## collision avoidance comm vehicle ##############

                 m.append(dist)
                 if (count<6):
                   count+=1
                 elif((m[-4]>m[-3]>m[-2]>m[-1]) and (abs(differ)>85 and abs(differ)<105)):
                    ser.write("shw,0018,34\n")
                    time.sleep(0.7)
                    ser.write("shw,0018,30\n")

                ##################  emergency vehicle alert comm vehicle   ##############

                 elif((m[-4]>m[-3]>m[-2]>m[-1]>30) and(out[1]=='D2') and abs(differ)>350 and abs(differ)<10):

                    ser.write("shw,0018,33\n")
                    time.sleep(0.7)
                    ser.write("shw,0018,30\n")

                ##################  FORWARD COLLISION   ##############

                 elif((m[-4]>m[-3]>m[-2]>m[-1]>20) and out[-2]==str(1) and  abs(differ)>350 and abs(differ)<10):

                    ser.write("shw,0018,38\n")
                    time.sleep(0.7)
                    ser.write("shw,0018,30\n")

               ##################  PEDESTRIAN DETECTED   ##############

               if (out[1]=='R1' or out[1]=='R2' or out[1]=='R3'):
                 print("rx_rsu : ", out)
                 if (out[-1]=='1'):
                    print("pedestrian_detected")
                    ser.write("shw,0018,35\n")
                    time.sleep(0.7)
                    ser.write("shw,0018,30\n")

               #print("Rx:",msg)
               #update_debug_log("Rx : " + msg, LOGLVL_ERR)

               # Add logic to handle configurtaion updates from Server................

            except Exception as e:
               update_debug_log("Error receiving data: ", LOGLVL_ERR)
              # rx_socket.close()


      #################################   Thread Handling   ###################################
      # Create two threads for transmission and reception
      try:
         update_boot_log(OBU_ID, LOGLVL_ERR)
         time.sleep(1)
         thread.start_new_thread( dsrc_send, ("Thread-1__DSRC_Tx", ) )
         thread.start_new_thread( dsrc_recv, ("Thread-2__DSRC_Rx", ) )
      except:
        update_debug_log("Error: unable to start thread\n", LOGLVL_ERR)
         print "Error: unable to start thread\n"

      while True:
         time.sleep(1)

   except (KeyboardInterrupt, SystemExit): # for KeyboardInterrupts
      update_debug_log("Killing Thread...\n", LOGLVL_ERR)
      print "Closing application due to user interrupt."

########################################## obu_case1 : EOF   ##########################################



