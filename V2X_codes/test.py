from gps import *
import time
import math

running = True

def getPositionData(gps):
    nx = gpsd.next()
    # For a list of all supported classes and fields refer to:
    # https://gpsd.gitlab.io/gpsd/gpsd_json.html
    if nx['class'] == 'TPV':
        latitude = getattr(nx,'lat', "Unknown")
        longitude = getattr(nx,'lon', "Unknown")
        track = getattr(nx,'track',"unknown")
        #print("Your position: lon = " + str(longitude) + ", lat = " + str(latitude)+", track: "+str(track))
        #print("aaaaaaaaaaaaaaa", type(track))
        gps_data = [latitude,longitude,track]
        return gps_data

gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

#abc = getPositionData(gpsd)
#print(getPositionData(gpsd))

try:
    print("Application started!")
    while running:
        gps_data = getPositionData(gpsd)
        if gps_data!=None:
            
            print("rrrrrrrrr",gps_data[0])
        
        time.sleep(1.0)

except (KeyboardInterrupt):
    running = False
    print("Applications closed!")
