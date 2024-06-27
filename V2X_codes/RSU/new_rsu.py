
#!/usr/bin/python
from     gps import *
from     time import *
from     datetime import datetime
from     itertools import islice
from     os import path
import   threading, thread
import   zmq, time, os, socket, glob
import   rsu_config as config
import   rsu_id as node_id
import   re


###########################   Global Variables   #############################################################
### Configuration parameters
RSU_ID            = node_id.rsu_id
GPRS_HOST         = config.case1_gprs_server_ip
GPRS_PORT         = config.case1_gprs_server_port
CASE1_LOG_TX_RATE = config.case1_log_tx_rate
CONF_UPDATE_RATE  = config.case1_conf_update_rate
HEARTBEAT_RATE    = config.case1_heartbeat_rate

### Terminal prints
EN_TRMNL_PRINT    =  0     ### 1 : Enable prints to appear in terminal window


##GPS
gpsd = None

# Packet Fields
APP_PKT_TYPE = "01"    # 01 for CASE1; 02 for CASE2
SOURCE_TYPE  = "00"    # 00 for Source packet; SourceID for relay packet
#OBU_ID = node_id.rsu_id
### Logging
#LOGLVL_ERR        =  1     ### 1 : Enables log updates with log_level = ERROR # commented by N on 19052022
LOGLVL_ERR        =  0     ### 1 : Enables log updates with log_level = ERROR

# LOGLVL_WARN       =  1     ### 1 : Enables log updates with log_level = WARNING
LOGLVL_WARN       =  0     ### 1 : Enables log updates with log_level = WARNING
LOGLVL_INFO       =  0     ### 1 : Enables log updates with log_level = INFO
DEBUG_LOG_PATH    =  "/var/prove/case1_applog/"        ### File : case1-applog__rsu-*
DATA_LOG_PATH     =  "/var/prove/case1_datalog/"       ### File : case1-log__rsu-*

# Packet Fields
ALIVE_PKT_TYPE    = "aa"    # aa representing RSU alive pkt to Server
LOG_PKT_TYPE      = "bb"    # bb representing RSU log pkt to Server
SOURCE_TYPE       = "00"    # 00 for Source packet; SourceID for relay packet
CHK_SUM           = "abcd"

### Application parameters
NUM_PKTS_TO_LOG         = 10
NUM_PKTS_IN_SINGLE_READ = 50
NUM_PKTS_FROM_PREVLOG   = 500
ONE_DAY_IN_SEC          = 86400

### Generic
chk_logdata       = False  ### To check CASE1_info_log for updates
update_prsnt      = False  ### To indicate new updates are available in current log file
prev_log_prsnt    = False  ### To indicate a previous log file was present
prev_log_file     = ""     ### Filename of previous log
rxd_pkt           = ""     ### String to save current CASE1_info
read_index        = 0      ### Read pointer

### sckt_cmnd
CONN_CSKT   = 1   ### Sckt cmnd to try socket reconn for sending current CASE1 pkt
CONN_LSKT   = 2   ### Sckt cmnd to try socket reconn for sending logged CASE1 pkt
CONN_HSKT   = 3   ### Sckt cmnd to try socket reconn for sending HRTBT pkt
SKT_ERR     = 4   ### Sckt cmnd to indicate ERROR to sckt retry
IDLE_SKT    = 5   ### Sckt cmnd to stay idle as ther is NO pkts to update
HAULT_SKT   = 6   ### Sckt cmnd to hault sckt reconn for a while
CLOSE_SKT   = 7   ### Sckt cmnd to close sckt


### sckt_state
OPEN     = 1
CLOSED   = 0

### pkt_type
NO_PKT   = 1
CURR_PKT = 2
LOG_PKT  = 3
HBT_PKT  = 4
######################   Global Function Definitions   ##########################################################

###############################  GPS Polling thread #################################
## Function Description : Class to poll for GPS updates
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

###############################  Boot Log #################################
### Function Description : Log debug information to a file
### Arguments :
###   data        -> Information to be updated to log in string format
###   dbg_level   -> Level of debug information (LOGLVL_ERR/LOGLVL_WARN/LOGLVL_INFO)

def update_boot_log(data, dbg_level):
    if (dbg_level == 1):
        time_now = datetime.now()
        debug_log_fullpath  = DEBUG_LOG_PATH + "case1-applog__rsu-" + RSU_ID + "__" + time_now.strftime("%d%m%Y") + ".txt"
        if path.exists(debug_log_fullpath):
            debug_logfile = open(debug_log_fullpath,"a+")
        else:
            debug_logfile = open(debug_log_fullpath,"w+")
        debug_logfile.write("/-------------------------------------------------/" +"\n")
        debug_logfile.write("      RSU ID : " + data +"\n")
        debug_logfile.write("/-------------------------------------------------/" +"\n")
        debug_logfile.write("case1-rsu @ " + time_now.strftime("%d%m%Y_%H:%M:%S") + " : RSU CASE1 Application Booting..." +"\n")
        debug_logfile.close()

###############################  Debug Logging #################################
### Function Description : Log debug information to a file
### Arguments :
###   data        -> Information to be updated to log in string format
###   dbg_level   -> Level of debug information (LOGLVL_ERR/LOGLVL_WARN/LOGLVL_INFO)
def update_debug_log(data, dbg_level): 
    if (dbg_level == 1):
        time_now = datetime.now()
        debug_log_fullpath  = DEBUG_LOG_PATH + "case1-applog__rsu-" + RSU_ID + "__" + time_now.strftime("%d%m%Y") + ".txt"
        if path.exists(debug_log_fullpath):
            debug_logfile = open(debug_log_fullpath,"a+")
        else:
            debug_logfile = open(debug_log_fullpath,"w+")         
        debug_logfile.write("case1-rsu @ " + time_now.strftime("%d%m%Y_%H:%M:%S") + " >> " + data +"\n")
        debug_logfile.close()

###############################  Read logged CASE1 information ############################
### Function Description : Application to fetch log data
###   1. Explore the latest log file
###   2. If a new log file present, open prev log file on this run
###      Else, open latest log file
###   3. Read 'n' number of log data and format it to our packet structure
###   4. Concatenate to a single string
###   5. Return the string data

def read_case1_log():
    ### Variables
    v2i_in   = ""    ### Input packet  (Rxd CASE1 pkt)
    i2s_out  = ""    ### Output packet (Formatted GPRS packet to be txd)
    global read_index
    global prev_log_file
    global lock_flag
    global update_prsnt
    global prev_log_prsnt
    global todays_log

    latest_log_file   = todays_log


    ### Check whther file is already accessed for write
    ### If so, wait till lock is released
    while (lock_flag == True):
        time.sleep(0.025)

    ### Read log file
    ### Case : New log file created, still, recheck previous log file for any updates
    if (latest_log_file != prev_log_file) and (prev_log_prsnt == True):
        prev_log_prsnt = False
        if EN_TRMNL_PRINT:
            print("Log Read : Last updates from prev log file...")
        with open(prev_log_file,"r",os.O_NONBLOCK) as read_file:
            v2i_in = islice(read_file,read_index,read_index+NUM_PKTS_FROM_PREVLOG)   #islice(iterable, start, stop, step)
            for line in v2i_in:
                if line != "":
                    i2s_out += line
                else:  ### Case : End of line in previous log
                    if EN_TRMNL_PRINT:
                        print("Log Read : Reached EOL in previous log file ..!!")
            read_index = 0
            prev_log_file = latest_log_file
            return i2s_out
    else:
        prev_log_prsnt = True
        if EN_TRMNL_PRINT:
            print("Log Read : Reading curr log file...")
        with open(latest_log_file,"r",os.O_NONBLOCK) as read_file:
            v2i_in = islice(read_file,read_index,read_index+NUM_PKTS_IN_SINGLE_READ)   
            for line in v2i_in:
                if line != "":
                    i2s_out += line
                    read_index = read_index + 1
                else:  ### Case : End of line in curr log
                    update_prsnt = False
                    if EN_TRMNL_PRINT:
                        print("Log Read : Reached EOL in curr log file ..!!")
            prev_log_file = latest_log_file
            return i2s_out

###############################  Server updation #################################
### Function Description : RSU Client application to forward I2S packet to Server
### It handles
###   1. Establish socket connection with Server via modem
###   2. Update case1 current and log data to Server

def update_server():
    update_debug_log("TCP : Starting server update fn ...", LOGLVL_INFO)
    if EN_TRMNL_PRINT:
        print("TCP : Starting server update fn ...")
   
    global i2s_pkt
    global pkt_type 

    global sckt_reconn
    global sckt_cmnd
    global retry_cnt
    idle_cnt = 0
    global sckt_state
    sckt_state = CLOSED
    pkt_fields = []
    cmp_pkt  = '0'
    old_pkt = '1'
    old_cnt = -4
    new_pkt  = 0

    try:
        client_socket = socket.socket()
        time.sleep(1)
        client_socket.connect((GPRS_HOST, GPRS_PORT))
        update_debug_log("TCP : Established connection with Server !!!", LOGLVL_ERR)
        if EN_TRMNL_PRINT:
            print("TCP : Established connection with Server !!!")

        while True:                            
            retry_cnt = 0  
            sckt_state = OPEN
            ###########################  Server transmission  ############################
            #if ((pkt_type == CURR_PKT) or (pkt_type == LOG_PKT) or (pkt_type == HBT_PKT)):
            if ((pkt_type == CURR_PKT) or (pkt_type == HBT_PKT)):
                idle_cnt =  0
            
                if (i2s_pkt != ""):
               ### Check if its not repeated packet, except in case of sequence number
                    new_pkt = 0
                    pkt_fields = re.compile(r',')
                    if (pkt_fields.split(i2s_pkt)[0:][2] != '00'): ## Ensuring its NOT ALIVE_PKT
                        print("DSRC : DATA pkt...")
                        cmp_pkt = pkt_fields.split(i2s_pkt)[:18]
                        if (cmp_pkt != old_pkt):
                            old_pkt = cmp_pkt 
                            new_pkt = 1
                        else:
                            if (pkt_fields.split(i2s_pkt)[0:][6] == 'V'): ## For the INVALID GPS case
                                if ( int(pkt_fields.split(i2s_pkt)[0:][18]) >= (old_cnt+4) ):
                                    old_cnt = int(pkt_fields.split(i2s_pkt)[0:][18])
                                    new_pkt = 1
                                else : 
                                    new_pkt = 0
                            else :
                                new_pkt = 0 
                    else: ## If ALIVE_PKT, it can be updated to Server
                        print("DSRC : ALIVE pkt...")
                        new_pkt = 1   

                    if (new_pkt == 1):  
                        if client_socket.send(i2s_pkt) <= 0:  ### Send to server
                            if EN_TRMNL_PRINT:
                                print("TCP_Error :Socket send failed...")
                            client_socket.close()  ### Close the connection
                            time.sleep(1)
                            sckt_cmnd   = SKT_ERR
                            sckt_reconn = 1
                            break
                        else:
                            if EN_TRMNL_PRINT:
                                print(i2s_pkt)
                pkt_type = NO_PKT
            ##########################  Close socket connection  ########################
            elif (sckt_cmnd == CLOSE_SKT): ### NOT currently used
                idle_cnt =  0
                if EN_TRMNL_PRINT:
                    print("TCP : Closing Server socket for periodic reconnect...")
                client_socket.close()  ### Close the connection
                time.sleep(1)
                sckt_cmnd   = SKT_ERR
                sckt_reconn = 1
                sckt_state  = CLOSED
                break    
            else :
                idle_cnt = idle_cnt+1
                if idle_cnt >= 65536:
                    idle_cnt =  1
                if ((idle_cnt % 700) == 0): ### 700 confirms above 1 min of idle time
                    if EN_TRMNL_PRINT:
                        print("TCP : Closing Server socket due to idle state ...")
                    client_socket.close()  ### Close the connection
                    time.sleep(1)
                    sckt_state = CLOSED
                    sckt_cmnd = IDLE_SKT
                    sckt_reconn  = 1
                    break
                  
            time.sleep(0.1)

    ###############  Exception handling of GSM Txn Thread #################
    ### For catching socket errors
    except socket.error as exc:
        sckt_state = CLOSED
        update_debug_log("TCP_Error : Caught exception socket.error : %s" % exc, LOGLVL_ERR)
        if EN_TRMNL_PRINT:
            print("TCP_Error : Caught exception socket.error : %s" % exc)
        #client_socket.close()  ### Close the connection
        time.sleep(1)
        sckt_cmnd   = SKT_ERR
        sckt_reconn = 1

    except Exception as err:
        sckt_state = CLOSED
        update_debug_log("TCP_Error : Exception --> Trying to reconnect...: %s" % err, LOGLVL_ERR)
        if EN_TRMNL_PRINT:
            print("TCP_Error : Exception --> Trying to reconnect...: %s ",err)
        client_socket.close()  ### Close the connection
        time.sleep(1)
        sckt_cmnd   = SKT_ERR
        sckt_reconn = 1   

###############################  Main loop #########################################################################
if __name__ == '__main__':

    global utc_date
    utc_date = "000000"
    global utc_time
    utc_time = "000000"
    global sckt_reconn
    sckt_reconn = 0      ### To trigger GSM thread recall, in case of any errors
    global sckt_cmnd
    sckt_cmnd = 0
    global sckt_state
    sckt_state = CLOSED

    ###############################  DSRC Rxn Thread #################################
    ### Function Description : RSU application which handles
    ###   1. Reception of DSRC packets from OBU and
    ###   2. Log the received packets to variable, and later to a file
    def dsrc_recv(threadName):
        ### WAVE_ZM Rx socket
        port     = "8889"
        context  = zmq.Context()
        socket   = context.socket(zmq.SUB)
        socket.connect("tcp://127.0.0.1:%s" % port)
        socket.setsockopt(zmq.SUBSCRIBE, "")

        global case1_pkt_array
        case1_pkt_array  = []  # Log list
        global rxd_pkt

        while  True:
            msg = socket.recv()
	    print("Rx_at_RSU",msg)
            if ( msg[0:3]== 'V2I'): # needs to be tested
                ####################  Copy to variable ####################
                            
                rxd_pkt = "I2S," + RSU_ID + "," + msg[4:6] + msg[9:] 
                log_rxdpkt = "I2S," + RSU_ID + "," + msg[4:7] + LOG_PKT_TYPE + msg[12:]
              #  print ("Rx at rsu from OBU",rxd_pkt) #1          
                case1_pkt_array.append(log_rxdpkt)
            #msg = socket.recv()
            #print("Rx_at_RSU",msg)
            
            else:
                if EN_TRMNL_PRINT:
                    print("DSRC_Rx : Received an invalid pkt")
                time.sleep(0.001)

    
    ###############################  Packet Handling ########################
    ### Function Description : Application fro selecting packet for txn and Logging
    ### It handles
    ###   1. Selection of packet to tx
    ###   2. Log rxd CASE1 information to log file
    def pkt_handling(threadName):
        global rxd_pkt
        global case1_pkt_array
        global chk_logdata
        global update_prsnt            
        global todays_log
        global lock_flag      
        global pkt_type
        global i2s_pkt
        global send_hbt

        global sckt_state
        global sckt_cmnd
        global sckt_reconn

        prev_pkt    = ""            
        logger_date = ""      
        pkt_type    = NO_PKT 
        store_flag  = False     ### Flag to indicate whether to append stored pkt or NOT
        hbt_seqno   = 0         ### Seq no. for RSU alive pkts
        num_pkts    = 0         ### Number of pkts saved when store_flag=True
        print_once  = False     ### Flag to control print for CASE1 (May be removed after testing)

        port = "8888"
        context = zmq.Context()
        tx_socket  = context.socket(zmq.PAIR)

        tx_socket.connect("tcp://127.0.0.1:%s" % port)

        while True:
            #####################  Selection of packet to tx #######################
            ### If timer fired,fetch log packet
            ### Else,fetch current packet
            
            ### Case-1: New OBU pkt received
            if prev_pkt != rxd_pkt: ### prv_pkt : to prevent sending last pkt continuously
                prev_pkt = rxd_pkt            
                pkt_type = CURR_PKT
                        
                ### if socket is closed, try to reconn server 
                #if (sckt_state == CLOSED) and (sckt_cmnd == IDLE_SKT):
                if (sckt_state == CLOSED):         
                    sckt_cmnd   = CONN_CSKT
                    sckt_reconn = 1
                    store_flag  = True
                    print_once  = True  
                    i2s_pkt     = rxd_pkt             
            
                elif (sckt_state == OPEN):                  
                    i2s_pkt     = rxd_pkt  ### Append i2s_pkt with unsend pkts
                    store_flag  = False
                    print_once  = False
                    num_pkts = 0
         
                else:
                    ### store upto 10 pkts till socket is open
                    if (store_flag == True): 
                        num_pkts    += 1
                        if (num_pkts < 10):                        
                            i2s_pkt  += rxd_pkt
                            print("num_pkts:"+str(num_pkts))
                        else:
                            num_pkts = 0
                            i2s_pkt  = ""
                                 
                    if (print_once==True):
                        print_once  = False
                        update_debug_log("CURR_PKT: state-" +str(sckt_state)+ " & cmd-" +str(sckt_cmnd), LOGLVL_ERR)
                    if (EN_TRMNL_PRINT and (print_once==True)):
                        print("CURR_PKT: state-" +str(sckt_state)+ " & cmd-" +str(sckt_cmnd))
            
            ### Case-2: Time to update log info      
            elif(chk_logdata == True) and (update_prsnt == True):               
                ### if socket is closed, try to reconn server 
                #if (sckt_state == CLOSED) and (sckt_cmnd == IDLE_SKT): 
                if (sckt_state == CLOSED): 
                    sckt_cmnd   = CONN_LSKT
                    sckt_reconn = 1
                else:
                    chk_logdata = False
                if EN_TRMNL_PRINT:
                    print("Fetching CASE1 log data")
                pkt_frm_log  = read_case1_log()
                if pkt_frm_log == "":   ### Added to stop log read after reaching EOF
                    update_prsnt = False
                    update_debug_log("Reached EOF for log file & completed log read successfully..!! ", LOGLVL_ERR)
                    if EN_TRMNL_PRINT:
                        print("Reached EOF for log file & completed log read successfully..!!  ")
                else: 
                    i2s_pkt = pkt_frm_log
                    pkt_type = LOG_PKT
                
                    ### This print not reqd for EOF case
                    update_debug_log("LOG_PKT: state-" +str(sckt_state)+ " & cmd-" +str(sckt_cmnd), LOGLVL_ERR)
                    if EN_TRMNL_PRINT:
                        print("LOG_PKT: state-" +str(sckt_state)+ " & cmd-" +str(sckt_cmnd))

            ### Case-3: Time to update hbt pkt
            elif (send_hbt == True): 
                
                ### if socket is closed, try to reconn server 
                #if (sckt_state == CLOSED) and (sckt_cmnd == IDLE_SKT): 
                if (sckt_state == CLOSED):
                    sckt_cmnd   = CONN_HSKT
                    sckt_reconn = 1
                    hbt_seqno = hbt_seqno+1
                    if hbt_seqno >= 65536:
                        hbt_seqno =  1      
            
                    i2s_pkt  = "I2S," + RSU_ID + ",HBT," + SOURCE_TYPE + "," + ALIVE_PKT_TYPE + ",00,"+str(hbt_seqno)+"," + CHK_SUM
                    #i2s_pkt  = "I2S," + RSU_ID + ",00,aa,00,"+str(hbt_seqno)+",abcd" 
                    pkt_type = HBT_PKT
                    update_debug_log("HBT_PKT: state-" +str(sckt_state)+ " & cmd-" +str(sckt_cmnd), LOGLVL_ERR)
                    
                    #send packet
                    print("HBT Tx s close")
                    tx_socket.send(i2s_pkt , flags=zmq.NOBLOCK)
                    print (i2s_pkt) #1
                    #update_debug_log(i2s_pkt + '\n', LOGLVL_INFO)
                    
                    if EN_TRMNL_PRINT:
                        print("HBT_PKT: state-" +str(sckt_state)+ " & cmd-" +str(sckt_cmnd))
                else:
                    send_hbt = False
                    hbt_seqno = hbt_seqno+1
                    if hbt_seqno >= 65536:
                        hbt_seqno =  1  

                    i2s_pkt  = "I2S," + RSU_ID + ",HBT," + SOURCE_TYPE + "," + ALIVE_PKT_TYPE + ",00,"+str(hbt_seqno)+"," + CHK_SUM
                    #i2s_pkt  = "I2S," + RSU_ID + ",00,aa,00,"+str(hbt_seqno)+",abcd" 
                    pkt_type = HBT_PKT
                    update_debug_log("HBT_PKT: state-" +str(sckt_state)+ " & cmd-" +str(sckt_cmnd), LOGLVL_ERR)
                    
                    #send packet
                    tx_socket.send(i2s_pkt , flags=zmq.NOBLOCK)
                    print (i2s_pkt) #2
                    print("HBT S open")
                    #update_debug_log(i2s_pkt + '\n', LOGLVL_INFO)
                    
                    if EN_TRMNL_PRINT:
                        print("HBT_PKT: state-" +str(sckt_state)+ " & cmd-" +str(sckt_cmnd))
         
                
            time.sleep(0.05)
        # tx_socket.close()

            #####################  Log to file  #######################
            ### Case-1: New OBU pkt received, log it to file
            if pkt_type  == CURR_PKT:
                ### Log 'n' number of CASE1 packet to Log file
                if (len(case1_pkt_array) >= NUM_PKTS_TO_LOG):
                    update_prsnt = True
                    lock_flag = True
                    ### Log to file based on date
                    curr_date = datetime.now().strftime("%d%m%Y")
                    if (curr_date != logger_date):
                        ### Open a new log file
                        logger_date = curr_date
                        todays_log  = DATA_LOG_PATH + "case1-log__rsu-" + RSU_ID + "__" + logger_date + ".txt"

                    if os.path.exists(todays_log):
                        log_file = open(todays_log,"a+")
                    else:
                        log_file = open(todays_log,"w+")

                    for line in case1_pkt_array:
                        log_file.write(line + '\n')   ### Write to file
                    log_file.close()
                    del case1_pkt_array[:]   ### Clear log list
                    time.sleep(0.1)
                    lock_flag = False
            else:
                time.sleep(0.01) 
        # tx_socket.close()

################################  Transmit Function ######################################
    def dsrc_send(threadName):
        global gpsp
        global utc_date
        global utc_time
        port = "8888"
        context = zmq.Context()
        tx_socket  = context.socket(zmq.PAIR)

        try:
            tx_socket.connect("tcp://127.0.0.1:%s" % port)
            gpsp = GpsPoller() # Main Thread - Polling GPS packets
            gpsp.start()
            gpsd.utc = 0 # initialization
            seqno = 1
            utc_prev_time = "chk" #initial value
		   
            while True:
                ######################  GPS Reading and Packet Encoding  ################
                utc = str(gpsd.utc)

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
                    data_info= utc_time+",A,"+str(gpsd.fix.latitude)[:11]+","+str(gpsd.fix.longitude)[:11]+","+utc_date
                    utc_prev_time = utc_time
		    # Packet Encoding
                   #hdr_payload  = OBU_ID +","+SOURCE_TYPE+","+APP_PKT_TYPE
                    hdr_payload  = RSU_ID

                    data_payload = data_info+","+str(seqno)
                    chksum       = "tihan"

                    f1  = open('/home/guest/rsu_apps/predict1.csv', "r")
                    last_line = int(f1.readlines()[-1])
                    f1.close()


                    full_pkt= "V2I,"+hdr_payload+","+str(len(data_payload))+","+data_payload+","+chksum+","+str(last_line)
                    time.sleep(0.80)
                   # Send packet
                    tx_socket.send(full_pkt , flags=zmq.NOBLOCK)
                    print ("tx from RSU",full_pkt) #3


                    time.sleep(0.20)
                else:
                    # Zero pad for GPS fields
                    #data_info="000000,V,0.000000000,00.00000000,00.000,00.0"+","+","+","+"000000"
		    print("no GPS")
                    time.sleep(0.20)  ###### may be removed once valid gps has been tested

                # Packet Encoding
               #hdr_payload  = OBU_ID +","+SOURCE_TYPE+","+APP_PKT_TYPE
		#hdr_payload  = RSU_ID

                #data_payload = data_info+","+str(seqno)
                #chksum       = "tihan"

		#f1 = open('/home/guest/rsu_apps/predict1.csv', "r")
		#last_line = int(f1.readlines()[-1])
		#f1.close()
		

                #full_pkt= "V2I,"+hdr_payload+","+str(len(data_payload))+","+data_payload+","+chksum+","+str(last_line)                     
                #time.sleep(0.60)
                # Send packet
                #tx_socket.send(full_pkt , flags=zmq.NOBLOCK)
                #print ("tx from RSU",full_pkt) #3
                #update_debug_log(full_pkt + '\n', LOGLVL_INFO)
        finally:
            tx_socket.close()

    ###############################  Log Timer  #################################
    ### Function Description : A periodic timer to trigger log data txn
    ### Once triggered sets a global variable to TRUE
    def log_timer(threadName):
        global chk_logdata
        while True:
            chk_logdata = False
            time.sleep(CASE1_LOG_TX_RATE)
            chk_logdata = True
            time.sleep(0.02)	

    ###########################  Configuration Checker ###########################
    ### Function Description : A periodic timer to update configuration parameters
    ### For first time alone, triggers for transmission of hrtbt pkt
    ### Else, triggers socket close, which inturn triggers hrtbt txn
    def config_update(threadName):
        global send_hbt
        send_hbt    = False
        first_time  = True
        while True:
            if first_time == True:
                if EN_TRMNL_PRINT:
                    print("Updating conf file...")
                ### Update configuration parameters
                RSU_ID            = node_id.rsu_id
                CASE1_LOG_TX_RATE = config.case1_log_tx_rate
                CONF_UPDATE_RATE  = config.case1_conf_update_rate
                HEARTBEAT_RATE    = config.case1_heartbeat_rate

                first_time = False
                time.sleep(0.01)
                #time.sleep(CONF_UPDATE_RATE)
                send_hbt    = True
                time.sleep(HEARTBEAT_RATE)
            else:  
                send_hbt    = True
                time.sleep(HEARTBEAT_RATE)

    ###############################  Main loop #################################
    ### Function Description : Main controller funtion
    ### It handles
    ###   1. Start of all threads
    ###   2. If any error occurs, restarts the thread based on a global variable
    try:
        thread.start_new_thread( config_update,   ("Thread-1__Conf-Chk", ) )
        time.sleep(0.1)
        update_boot_log(RSU_ID, LOGLVL_ERR)  # LOGLVL_ERR : Start information of high importance
        thread.start_new_thread( dsrc_recv,       ("Thread-3__DSRC-Rx", ) )
        thread.start_new_thread( log_timer,       ("Thread-4__Log-Timer", ) )
        #thread.start_new_thread( pkt_handling,    ("Thread-2__DSRC-Tx", ) )
        thread.start_new_thread( dsrc_send, ("Thread-2__DSRC_Tx", ) )
        
        retry_cnt   = 0
        prev_cmnd   = 0
        time.sleep(5)  ### Time to get GSM connection                
        update_server()


        while True:
            if sckt_reconn  == 1:
                sckt_reconn  = 0 
                if (sckt_cmnd == SKT_ERR) or (sckt_cmnd == prev_cmnd):     
                    ### SKT_ERR is the first error to occur (so mostly code enters this loop only)       
                    actual_cmnd = sckt_cmnd ## Saving curr sckt_cmnd to local variable
                    ### Incremental delay if socket connection fails
                    sckt_cmnd = HAULT_SKT
                    retry_cnt += 1
                    update_debug_log("Haulting for "+str(retry_cnt)+" mins (cmnd:"+str(sckt_cmnd)+")",LOGLVL_ERR)
                    if EN_TRMNL_PRINT:
                        print("Haulting for "+str(retry_cnt)+" mins (cmnd:"+str(sckt_cmnd)+")")   
                    time.sleep(60*retry_cnt)
                    if (retry_cnt >= 10):
                        retry_cnt = 0
                    
                    ### Go back to prev_state  
                    sckt_cmnd = actual_cmnd ## Retreiving curr sckt_cmnd from local variable
                    prev_cmnd = sckt_cmnd
                    if (sckt_cmnd == CONN_HSKT) and (retry_cnt >= 2):
                        update_debug_log("Case : Hrtbt tx failed 2 times --> Stop Server retry", LOGLVL_WARN)
                        if EN_TRMNL_PRINT:
                            print("Case : Hrtbt tx failed 2 times --> Stop Server retry")
                    else:
                        update_debug_log("Case : Hault over --> Retry ServerConn", LOGLVL_ERR)
                        if EN_TRMNL_PRINT:
                            print("Case : Hault over --> Retry ServerConn")
                        update_server()   ### Trying to reconnect with Server      
                    
                elif (sckt_cmnd == CLOSE_SKT):
                    time.sleep(1)   
                    update_debug_log("Case : Periodic thread restart --> Retry ServerConn", LOGLVL_WARN)
                    if EN_TRMNL_PRINT:
                        print("Case : Periodic thread restart --> Retry ServerConn")              
                    update_server()   ### Trying to reconnect with Server
                    
                elif (sckt_cmnd == CONN_CSKT):
                    time.sleep(1)   
                    update_debug_log("Case : Pkt rxd from OBU --> Retry ServerConn", LOGLVL_WARN)
                    if EN_TRMNL_PRINT:
                        print("Case : Pkt rxd from OBU --> Retry ServerConn")              
                    update_server()   ### Trying to reconnect with Server
                    
                elif (sckt_cmnd == CONN_LSKT):
                    time.sleep(1)   
                    update_debug_log("Case : LogSend timer fired --> Retry ServerConn", LOGLVL_WARN)
                    if EN_TRMNL_PRINT:
                        print("Case : Pkt rxd from OBU --> Retry ServerConn")              
                    update_server()   ### Trying to reconnect with Server
                    
                elif (sckt_cmnd == CONN_HSKT):
                    time.sleep(1)   
                    update_debug_log("Case : HrtbtPkt timer fired --> Retry ServerConn", LOGLVL_WARN)
                    if EN_TRMNL_PRINT:
                        print("Case : HrtbtPkt timer fired --> Retry ServerConn")              
                    update_server()   ### Trying to reconnect with Server
                    
                elif (sckt_cmnd == IDLE_SKT):
                    update_debug_log("Case : Idle Socket --> Stay idle !!!", LOGLVL_WARN)
                    if EN_TRMNL_PRINT:
                        print("Case : Idle Socket --> Stay idle !!!")                             
            else:                                    
                time.sleep(1)

    except Exception as err:
        update_debug_log("Error: Unable to start thread", LOGLVL_ERR)
   
    except (KeyboardInterrupt, SystemExit): ### When you press ctrl+c
        update_debug_log("Killing Thread : User interrupt...", LOGLVL_ERR)
        if EN_TRMNL_PRINT:
            print("Killing Thread : User interrupt...")


################################ rsu_case1 : EOF   #####################################

      



