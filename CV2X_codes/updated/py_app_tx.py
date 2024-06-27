import threading, zmq
from enum import Enum
import time
from gps import *

class Results(Enum):
    Failure = 0
    Success = 1

def decoded(s):
    return int.from_bytes(s,'little')

def encoded(value, len):
    return value.to_bytes(len,'little')


class Integer8():
    def __init__(self):
        self.value = None

    def encode(self):
        if self.value == None:
            return None
        return encoded(self.value,1)

    def decode(self,s):
        self.value = decoded(s[:1])
        return s[1:]

class Integer16():
    def __init__(self):
        self.value = None

    def encode(self):
        return encoded(self.value,2)

    def decode(self,s):
        self.value = decoded(s[:2])
        return s[2:]


class Integer32():
    def __init__(self):
        self.value = None

    def encode(self):
        out = encoded(self.value,4)
        #print("4 byte encoded data: ",out)
        return out
    
    def decode(self,s):
        self.value = decoded(s[:4])
        return s[4:]
   
def sdecoded(s):
    return int.from_bytes(s,'little',signed=True)

def sencoded(value, len):
    return value.to_bytes(len,'little',signed = True)

class SInteger8():
    def __init__(self):
        self.value = None

    def encode(self):
        return sencoded(self.value,1)
    
    def decode(self,s):
        self.value = sdecoded(s[:1])
        return s[1:]

class Integer48():
    def __init__(self):
        self.value = None

    def encode(self):
        return encoded(self.value,6)

    def decode(self,s):
        self.value = s[:6].hex()
        return s[6:]

class Opaque():
    def __init__(self):
        self.value = None

    def encode(self):
        out = self.value.encode('utf-8')
        return out

class mode(Enum):
    SPS_MODE = 1
    ADHOC_MODE = 2

class hle_wsmp():
    def __init__(self):
        self.mode = Integer8()
        self.ch_id = Integer8()
        self.time_slot = Integer8()
        self.data_rate = Integer8()
        self.tx_pow = SInteger8()
        self.ch_ld = Integer8()
        self.info = Integer8()
        self.usr_prio = Integer8()
        self.expiry_time = Integer8()
        self.mac = Integer48()
        self.psid = Integer32()
        self.dlen = Integer16()
        self.data = None

    def encode(self):
        out = self.mode.encode() + self.ch_id.encode() + self.time_slot.encode() + self.data_rate.encode() + self.tx_pow.encode() + self.ch_ld.encode() + self.info.encode(
                ) + self.usr_prio.encode() + self.expiry_time.encode() + self.mac.encode() + self.psid.encode() + self.dlen.encode() + self.data
        return out

def getPositionData(gps):
    nx = gpsd.next()
    if nx['class'] == 'TPV':
        latitude = getattr(nx,'lat', "Unknown")
        longitude = getattr(nx,'lon', "Unknown")
        speed = getattr(nx,'speed',"unknown")
        #print("Your position: lon = " + str(longitude) + ", lat = " + str(latitude)+", track: "+str(track))
        #print("aaaaaaaaaaaaaaa", type(track))
        gps_data = [latitude,longitude,speed]
        return gps_data
gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

#####conversion from LAT-LONG to X,Y,Z coordinate####
def get_cartesian(lat=None,lon=None):
    lat, lon = math.radians(lat), math.radians(lon)
    R = 6371 # radius of the earth
    x = R * math.cos(lat) * math.cos(lon)
    y = R * math.cos(lat) * math.sin(lon)
    z = R *math.sin(lat)
    return x,y,z

#####calculating heading angle#####

def get_heading(aLocation):
       off_x = aLocation[-1][1] - aLocation[-2][1]
       off_y = aLocation[-1][0] - aLocation[-2][0]
       heading = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
       if heading < 0:
           heading += 360.00
       return heading

def FillWsmpContent(data):

    hle_msg = hle_wsmp()

    hle_msg.mode.value = mode.SPS_MODE.value
    hle_msg.ch_id.value = 172
    hle_msg.time_slot.value = 0
    hle_msg.data_rate.value = 12
    hle_msg.tx_pow.value = -98
    hle_msg.ch_ld.value = 0
    hle_msg.info.value = 0
    hle_msg.expiry_time.value = 0
    hle_msg.usr_prio.value = 0
    hle_msg.mac.value = 16557351571215
    hle_msg.psid.value = 32
    hle_msg.dlen.value = len(data)
    hle_msg.data = bytes(data,'utf-8')

    out = hle_msg.encode()
    return out

def wsmp_operation():

    wsmp_context = zmq.Context()
    wsmp_socket = wsmp_context.socket(zmq.REQ)
    wsmp_socket.connect("tcp://localhost:5555")
    

    #application_data = "APPLICATION DATA"
    #result = FillWsmpContent(application_data)
    #print("data before sending wsmp and len: \n",result,len(result))
    k = []
    cnt = 0
    E = 0
    head_ang = 0
    speed=0
    alocation=[[0,0]]
    while True:
        
        file1 = open("OBU_TX.txt","a")       

        gps_data = getPositionData(gpsd)
        if gps_data!= None:
            speed = gps_data[2]
            latitude = gps_data[0]
            longitude = gps_data[1]

            alocation.append([latitude,longitude])
            head_ang= get_heading(alocation)
            print("qqqqqqqqqqqqq",head_ang)
            x1,y1,z1 = get_cartesian(latitude,longitude)
            print("x_cord: {}, y_coord: {}, z_coord: {} ".format(x1,y1,z1))

            

  ########################################### Emergency Break ###################################
            k.append(speed)
            print(k)
            if(cnt<6):
                cnt += 1
            else:
                diff1 = abs((k[-2]) - (k[-1]))
                diff2 = abs((k[-3]) - (k[-1]))
                diff3 = abs((k[-4]) - (k[-1]))
                diff4 = abs((k[-5]) - (k[-1]))
                diff5 = abs((k[-6]) - (k[-1]))
                print("SPEED DIFF: ",diff1,diff2,diff3,diff4,diff5)

                if (diff1>5 or diff2>5 or diff3>5 or diff4>5 or diff5>5) and (speed<1):
                    E=1
                    print("Emergency break applied")
                else:
                    E=0
####################################################################################################        
        #print("aaaaaaaaa: ",speed)
            application_data = "latitude,"+str(latitude)+","+"longitude,"+str(longitude)+","+"Speed,"+str(speed)+",heading_angle,"+str(head_ang)+","+"Emeregency_break,"+str(E)+"\n"
            file1.write(application_data)
            file1.close()
            print("length: ",len(application_data))

            result = FillWsmpContent(application_data)
            print("data before sending wsmp and len: \n",result,len(result))

            wsmp_socket.send(result)
            msg = wsmp_socket.recv()
            print(msg)
            time.sleep(.1)
    

class Action(Enum):
    Add = 1
    Delete = 2


class wme_sub():
    def __init__(self):
        self.action = Integer8()
        self.psid = Integer32()
        self.appname = Opaque()

    def encode(self):
        out = self.action.encode() + self.psid.encode() + self.appname.encode()
        return out

def Wme_operation():
    wme_context = zmq.Context()
    wme_socket = wme_context.socket(zmq.REQ)
    wme_socket.connect("tcp://localhost:9999")


    psid_sub_mag = wme_sub()
    psid_sub_mag.action.value = Action.Add.value
    psid_sub_mag.psid.value = 32
    psid_sub_mag.appname.value = "TX_APPLICATION"
    out = psid_sub_mag.encode()  
    wme_socket.send(out)
    cmh_recv_msg = wme_socket.recv()
    print("psid 32 subscribed to wme")   


if __name__ == "__main__":

    Wme_operation()
    app_operation_th = threading.Thread(target=wsmp_operation)
    app_operation_th.start()
    

    

