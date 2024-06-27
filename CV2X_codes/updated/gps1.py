from     gps import *
import math
gpsn = None

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




gpsp = GpsPoller() # Main Thread - Polling GPS packets
gpsp.start()
gpsd.utc = 0 
#gps.connect()

packet = gps.get_current()

latl =str(gpsd.fix.latitude)



gps.close()


