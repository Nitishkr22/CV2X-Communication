import threading, zmq
from enum import Enum
import time
from gps import *
from     time import *
from     os import path
import   zmq, sys
from datetime import datetime


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
        latitude = getattr(nx,'lat','Unknown')
        longitude = getattr(nx,'lon',"Unknown")
        track = getattr(nx,"track","Unknown")
        #print("position:  "+str(longitude))
        return longitude

gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

                    
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
    
    #getPositionData(gpsd)
    #logt = getPositionData(gpsd)
    #print("ppppppppppppp",logt)
    #application_data = "APPLICATION DATA"
    #application_data = getPositionData(gpsd)
    #result = FillWsmpContent(application_data)
    #print("data before sending wsmp and len: \n",result,len(result))
    while True:
        longt = getPositionData(gpsd)
        application_data = "App_data: "+str(longt)
        result = FillWsmpContent(application_data)
        print("data before sending wsmp and len: \n",result,len(result))



        #print("rrrrrrrrrrr",result)
        #longt = getPositionData(gpsd)
        print("llllllllll",longt)
        wsmp_socket.send(result)
        msg = wsmp_socket.recv()
        print(msg)
        time.sleep(0.1)

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
    

    

