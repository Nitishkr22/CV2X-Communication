import zmq
from enum import Enum
import threading


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

class Integer48():
    def __init__(self):
        self.value = None

    def encode(self):
        return encoded(self.value,6)

    def decode(self,s):
        self.value = s[:6].hex()
        return s[6:]

def sdecoded(s):
    return int.from_bytes(s,'little',signed=True)

class SInteger8():
    def __init__(self):
        self.value = None

    def decode(self,s):
        self.value = sdecoded(s[:1])
        return s[1:]


class Opaque():
    def __init__(self):
        self.value = None

    def encode(self):
        out = self.value.encode('utf-8')
        return out


class wsmp_hle():
    def __init__(self):
        self.wsmp_version = Integer8()
        self.channel_no = Integer8()
        self.data_rate = Integer8()
        self.tx_pow_level = SInteger8()
        self.channel_load = Integer8()
        self.user_priority = Integer8()
        self.peer_mac_addr = Integer48()
        self.psid = Integer32()
        self.dlen = Integer16()
        self.data = None

    def decode(self, s):
        ret_ver = self.wsmp_version.decode(s)
        ret_chh = self.channel_no.decode(ret_ver)
        ret_dr = self.data_rate.decode(ret_chh)
        ret_txpow = self.tx_pow_level.decode(ret_dr)
        ret_chld = self.channel_load.decode(ret_txpow)
        ret_usr_prio = self.user_priority.decode(ret_chld)
        ret_peer = self.peer_mac_addr.decode(ret_usr_prio)
        ret_psid = self.psid.decode(ret_peer)
        ret_len = self.dlen.decode(ret_psid)
        self.data = ret_len[:self.dlen.value]

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
    psid_sub_mag.appname.value = "RX_APPLICATION"
    out = psid_sub_mag.encode()  
    wme_socket.send(out)
    cmh_recv_msg = wme_socket.recv()
    print("psid 32 subscribed to wme")


def Wsmp_operation():
    wsmp_context = zmq.Context()
    wsmp_socket = wsmp_context.socket(zmq.SUB)
    wsmp_socket.connect("tcp://localhost:4444")
    wsmp_socket.setsockopt(zmq.SUBSCRIBE,b"32")

    while True:
        message = wsmp_socket.recv()
        if message!=b'32':

            #print("Received: ", message)
            rx = message.decode('utf-8')
            print("decoded: ",rx)
            file1 = open("OBU_RX.txt","a") 
            if int(rx[-2])==1:
                file1.write(rx[:-1]+" "+"Detected"+"\n")
            else:
                file1.write(rx[:-1]+" "+"Not_detected"+"\n")
            file1.close()
            wsmp_msg = wsmp_hle()
            wsmp_msg.decode(message)
        

if __name__ == "__main__":
    Wme_operation()
    Wsmp_operation_th = threading.Thread(target=Wsmp_operation)
    Wsmp_operation_th.start()
