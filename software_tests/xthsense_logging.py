# -*- coding: utf-8 -*-
import serial
import time
import OSC

from optparse import OptionParser

### settings ###
serial_port = "/dev/tty.usbmodem1a1221"
#serial_port = "/dev/ttyUSB0"
hostport = 57121 ## where the OSC goes
cfg_mmg = 1 ## MMG sensing on (off=0)
cfg_mpu = 0 ## MPU sensing on (off=0)
cfg_temp = 1 ## temperature sension on (off=0)
###


baudrate = 115200
hostip = "127.0.0.1"

myip = "127.0.0.1"
myport = 57400

verbose = False

incMsg = []
incType = 'n'
escape = False



############### LOGGING ##################


import os
import sys
import logging
import logging.handlers as handlers

class LogFile(object):
    """File-like object to log text using the `logging` module."""

    def __init__(self, options, name=None):
        self.logger = logging.getLogger(name)
	#formatter = logging.Formatter('%(asctime)s %(levelname)s\t%(message)s')
	formatter = logging.Formatter('%(asctime)s\t%(message)s')
	level = logging.__dict__.get(options.loglevel.upper(),logging.DEBUG)
	self.logger.setLevel(level)
	 # Output logging information to screen
	if not options.quiet:
	  hdlr = logging.StreamHandler(sys.stderr)
	  hdlr.setFormatter(formatter)
	  self.logger.addHandler(hdlr)

	# Output logging information to file
	logfile = os.path.join(options.logdir, options.logname )
	if options.clean and os.path.isfile(logfile):
	    os.remove(logfile)
	hdlr2 = handlers.RotatingFileHandler(logfile, maxBytes=5242880, backupCount=20)  #max 5 megabytes per file
	#hdlr2 = logging.FileHandler(logfile)
	#hdlr2.setFormatter(formatter)
	self.logger.addHandler(hdlr2)


    def write(self, msg, level=logging.INFO):
        self.logger.log(level, msg)

    def flush(self):
        for handler in self.logger.handlers:
            handler.flush()


##########################################

#def sendOSCMessage( path, args ):
  #msg = OSC.OSCMessage()
  #msg.setAddress( path )
  ##print args
  #for a in args:
    #msg.append( a )
  #try:
    #oschost.send( msg )
    #if verbose:
      #print( "sending message", msg )
  #except OSC.OSCClientError:
    #if verbose:
      #print( "error sending message", msg )

import itertools

def split_seq(iterable, size):
    it = iter(iterable)
    item = list(itertools.islice(it, size))
    while item:
        yield item
        item = list(itertools.islice(it, size))

def twos_comp(val, bits):
    """compute the 2's compliment of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

import struct;

def display_mpu():
  # 0: msg type
  # 1: msg id
  # 2: msb time interval
  # 3: lsb time interval
  # 4 + 5: battery
  # 5: 
  msgtype = chr( incMsg[0] )
  #print incMsg
  if len( incMsg ) > 20:
    sourceAddress = incMsg[1]*256 + incMsg[2];
    linkQuality = incMsg[3]
    rssi = incMsg[4]
    msgid = incMsg[5]    
    timevalue = incMsg[6]*256 + incMsg[7]
    mpucount = incMsg[8]
    #batteryvalue = incMsg[8]*256 + incMsg[9]
    #temperature = incMsg[10]*256 + incMsg[11]
    mpuvalues_parsed = [];
    mpuvalues = list( split_seq( incMsg[9:21], 2 ) );
    #print "mpu", len( incMsg),  len(mpuvalues)
    for seq in mpuvalues:
      mpuvalstruct = struct.pack( 'BB', seq[0], seq[1] )
      mpuval = struct.unpack( 'h', mpuvalstruct )
      #print seq, mpuval
      #print type(mpuval)
      mpuvalues_parsed.append( mpuval[0] )
    mpuvalues = list( split_seq( incMsg[21:27], 2 ) );
    for seq in mpuvalues:
      mpuvalstruct = struct.pack( 'BB', seq[1], seq[0] ) # compass is the other way around
      mpuval = struct.unpack( 'h', mpuvalstruct )
      #print seq, mpuval
      #print type(mpuval)
      mpuvalues_parsed.append( mpuval[0] )
    #mpuvalues = list( split_seq( incMsg[8:24], 4 ) )
    #print len(mpuvalues)
    #for seq in mpuvalues:
      #mpuvalstruct = struct.pack( 'BBBB', seq[0], seq[1], seq[2], seq[3] )
      #mpuval = struct.unpack( 'i', mpuvalstruct );
      #mpuvalues_parsed.append( mpuval[0] ) 
      #print seq, mpuval
            
    #print "mpu", msgtype, msgid, timevalue, mpuvalues_parsed
    #print msgtype, msgid, timevalue, batteryvalue, temperature, millis
    mpusendData = [ time.clock(), msgid, timevalue ]
    mpusendData.extend( mpuvalues_parsed )
    #sendOSCMessage( "/xth/mpu", mpusendData )    
    #sendOSCMessage( "/xth/mpu-update", [ msgid, timevalue] )
    #sendOSCMessage( "/xth/mpu", mpuvalues_parsed )
    #print mpusendData
    global logfile
    logfile.write( mpusendData )
  else:
    print "mpu", msgtype, len(incMsg), "too short"
    
    
#16244	-98	-620	0	0	-1	-27	-203	-143    
#pretime: 10327 count: 137, time: 10335; 
#MPU data: 16270	-144	-624	-7	3	-3	-30	-207	-146	749314792	-19653318	-768594920	-18012328
#232 162 169 44 58 29 212 254 24 44 48 210 88 39 237 254 142 63 112 255 144 253 249 255 3 0 253 255 226 255 49 255 110 255 112 
#232 162 169 44 58 29 212 254 24 44 48 210 88 39 237 254 142 63 112 255 144 253 249 255 3 0 253 255 226 255 49 255 110 255 112 
    
def display_mmg():
  # 0: msg type
  # 1: msg id
  # 2: msb time interval
  # 3: lsb time interval
  # 4 + 5: battery
  # 5: 
  msgtype = chr( incMsg[0] )
  #print incMsg
  if len( incMsg ) > 11:
    sourceAddress = incMsg[1]*256 + incMsg[2];
    linkQuality = incMsg[3]
    rssi = incMsg[4]
    msgid = incMsg[5]
    timevalue = incMsg[6]*256 + incMsg[7]
    #batteryvalue = incMsg[8]*256 + incMsg[9]
    prev_mmg_index = incMsg[8]
    mmg_index = incMsg[9]
    #print incMsg[0:9]
    if len( incMsg ) > 80:
      mmgvalues = list( split_seq( incMsg[10:], 5 ) );
      mmgvalues_parsed = [];
      #print "mmg", msgtype, msgid, timevalue, mmg_index, len(incMsg[10:]), len( mmgvalues_parsed )
      for seq in mmgvalues:
	#print seq
	if len(seq) == 5:
	  value_parsed = seq[0]*4 + ( (seq[4]/64)%4);
	  mmgvalues_parsed.append( value_parsed );
	  value_parsed = seq[1]*4 + ( (seq[4]/16)%4); # last 6 bits
	  mmgvalues_parsed.append( value_parsed );
	  value_parsed = seq[2]*4 + ( (seq[4]/4)%4);  # last four bits
	  mmgvalues_parsed.append( value_parsed );
	  value_parsed = seq[3]*4 + ( (seq[4]%4) ); # last two bits
	  mmgvalues_parsed.append( value_parsed );
	#if len(seq) == 5:
	  #value_parsed = seq[0] + ( (seq[4]/64)%4 * 256);
	  #mmgvalues_parsed.append( value_parsed );
	  #value_parsed = seq[1] + ( (seq[4]/16)%4 * 256 ); # last 6 bits
	  #mmgvalues_parsed.append( value_parsed );
	  #value_parsed = seq[2] + ( (seq[4]/4)%4 * 256 );  # last four bits
	  #mmgvalues_parsed.append( value_parsed );
	  #value_parsed = seq[3] + ( (seq[4]%4) * 256 ); # last two bits
	  #mmgvalues_parsed.append( value_parsed );
      #print "mmg", msgtype, msgid, timevalue, mmg_index, len(incMsg[10:]), len( mmgvalues_parsed )
      #print mmgvalues_parsed
      #sendOSCMessage( "/xth/battery", [ batteryvalue ] )
      mmgsendData = [ time.clock(), msgid, timevalue, prev_mmg_index, mmg_index ]
      mmgsendData.extend( mmgvalues_parsed )
      #sendOSCMessage( "/xth/mmg", mmgsendData )
      #print mmgsendData
      global logfile
      logfile.write( mmgsendData )
    else:
      print "mmg", msgtype, msgid, timevalue, mmg_index, len(incMsg[10:])
  else:
    print "mmg", msgtype, len(incMsg), "too short"

def display_battery():
    msgtype = chr( incMsg[0] )
    sourceAddress = incMsg[1]*256 + incMsg[2];
    linkQuality = incMsg[3]
    rssi = incMsg[4]
    msgid = incMsg[5]
    timevalue = incMsg[6]*256 + incMsg[7]
    batteryvalue = incMsg[8]*256 + incMsg[9]
    #sendOSCMessage( "/xth/battery", [ batteryvalue ] )
    if len(incMsg) > 10:
      temperature = incMsg[10]*256 + incMsg[11]
      #sendOSCMessage( "/xth/temperature", [ temperature ] )
      print "battery/temperature", msgtype, msgid, timevalue, batteryvalue, temperature
    else:
      print "battery", msgtype, msgid, timevalue, batteryvalue

def display_serial_number():
  msgtype = chr( incMsg[0] )
  #print incMsg
  if len( incMsg ) > 6:
    sourceAddress = incMsg[1]*256 + incMsg[2];
    linkQuality = incMsg[3]
    rssi = incMsg[4]
    msgid = incMsg[5]
    #serial = struct.pack( 'BBBBBBBB', incMsg[6], incMsg[7], incMsg[8], incMsg[9], incMsg[10], incMsg[11], incMsg[12], incMsg[13] )
    myserial = "".join( map(chr, incMsg[6:14] ) )
    print "serial", msgtype, sourceAddress, linkQuality, rssi, msgid, "-", myserial, "-", incMsg
    serial_send_address( myserial, 4 )
  else:
    print "serial", msgtype, len(incMsg), "too short"

def display_address():
  msgtype = chr( incMsg[0] )
  #print incMsg
  if len( incMsg ) > 6:
    sourceAddress = incMsg[1]*256 + incMsg[2];
    linkQuality = incMsg[3]
    rssi = incMsg[4]
    msgid = incMsg[5]
    serial = "".join( map(chr, incMsg[6:13] ) )    
    print "address", msgtype, sourceAddress, linkQuality, rssi, msgid, "-", serial, "-", incMsg
    #serial_send_config( sourceAddress, 1, 1, 1 ) # all
    #serial_send_config( sourceAddress, 0, 1, 1 ) # mpu, temp
    #serial_send_config( sourceAddress, 1, 0, 1 ) # mmg, temp
    serial_send_config( sourceAddress, cfg_mmg, cfg_mpu, cfg_temp ) # mmg, temp
  else:
    print "address", msgtype, msgid, "too short"

def display_config():
  msgtype = chr( incMsg[0] )
  #print incMsg
  if len( incMsg ) > 6:
    sourceAddress = incMsg[1]*256 + incMsg[2];
    linkQuality = incMsg[3]
    rssi = incMsg[4]
    msgid = incMsg[5]
    print "config", msgtype, sourceAddress, linkQuality, rssi, msgid, incMsg
  else:
    print "config", msgtype, msgid, "too short"

def display_loopback():
  msgtype = chr( incMsg[0] )
  print "loopback", msgtype, incMsg


def parse_message():
  global incMsg, incType

  if type( incType ) == int:
    incTypeChr = chr( incType )
  else:
    incTypeChr = 'u' # unknown message

  if incTypeChr == 'm' : # mpu data
    display_mpu()
  elif incTypeChr == 'M' : # mmg data
    display_mmg()
  elif incTypeChr == 'b' : # battery and optional temperature data
    display_battery()    
  elif incTypeChr == 'S' : # serial number
    display_serial_number()
  elif incTypeChr == 'A' : # address confirmation
    display_address()
  elif incTypeChr == 'C' : # configuration confirmation
    display_config()
  else:
    display_loopback()

def read_byte( nrbytes ):
  global escape, incMsg, incType
  b = serial.read( nrbytes )
  if len( b ) > 0 :
    for byt in b:
      newbyte = ord( byt )
      #print len(b), byt, newbyte
      if escape:
	if newbyte in [ 10, 13, 92 ] :
	  incMsg.append( newbyte )
	else :
	  incMsg.append( newbyte )
	  incType = newbyte
	escape = False
      else :
	if newbyte == 92:
	  escape = True
	elif newbyte == 10:
	  #end of line
	  parse_message()
	  #print incType, incMsg
	  incMsg = []
	  incType = 'n'
	else :
	  incMsg.append( newbyte )

def read_data( ):
  bytes_toread = serial.inWaiting()  
  read_byte( bytes_toread )

def serial_send_address( ser, address ):
  print ser, address
  msg = chr( 92 )+"A"
  msg += msg.join( ser.split() )
  msg = appendToMsg( msg, address/256 )
  msg = appendToMsg( msg, address%256 )
  msg += b"\n"
  serial.write( msg )
  msgints = [];
  for b in msg:
    msgints.append( b )
  print len(msg), msgints, msg

def serial_send_config( address, c1, c2, c3 ):
  print address, c1, c2, c3
  msg = chr( 92 )+"C"
  #msg += msg.join( ser.split() )
  msg = appendToMsg( msg, address/256 )
  msg = appendToMsg( msg, address%256 )
  msg = appendToMsg( msg, c1 )
  msg = appendToMsg( msg, c2 )
  msg = appendToMsg( msg, c3 )
  msg += b"\n"
  serial.write( msg )
  msgints = [];
  for b in msg:
    msgints.append( b )
  print len(msg), msgints, msg

def serial_send_led( address, c1, c2, c3 ):
  print address, c1, c2, c3
  msg = chr( 92 )+"L"
  #msg += msg.join( ser.split() )
  msg = appendToMsg( msg, address/256 )
  msg = appendToMsg( msg, address%256 )
  msg = appendToMsg( msg, c1/256 )
  msg = appendToMsg( msg, c1%256 )  
  msg = appendToMsg( msg, c2/256 )
  msg = appendToMsg( msg, c2%256 )  
  msg = appendToMsg( msg, c3/256 )
  msg = appendToMsg( msg, c3%256 )  
  msg += b"\n"
  serial.write( msg )
  msgints = [];
  for b in msg:
    msgints.append( b )
  print len(msg), msgints, msg  

def appendToMsg( msg, dat ):
  dat = int( dat )
  if dat == 10:
    msg += chr( 92 )
  if dat == 13:
    msg += chr( 92 )
  if dat == 92:
    msg += chr( 92 )
  msg += chr( dat )
  return msg

####################### main ################

# Setup command line options
parser = OptionParser("usage: %prog [options]")
parser.add_option("-l", "--logdir", dest="logdir", default=".", help="log DIRECTORY (default ./)")
parser.add_option("-n", "--logname", dest="logname", default="xthsense.log", help="log name (default xthsense)")
parser.add_option("-v", "--loglevel", dest="loglevel", default="debug", help="logging level (debug, info, error)")
parser.add_option("-q", "--quiet", action="store_true", dest="quiet", help="do not log to console")
parser.add_option("-c", "--clean", dest="clean", action="store_true", default=False, help="remove old log file")

# Process command line options
(options, args) = parser.parse_args()

serial = serial.Serial()  # open first serial port
serial.baudrate = baudrate
serial.port = serial_port

serial.open()

logfile = LogFile( options, 'mmgdata')
#sys.stdout = logfile
#sys.stderr = logfile

#oschost = OSC.OSCClient()
#send_address = ( hostip, hostport )
#oschost.connect( send_address )

#receive_address = ( myip, myport )
#osc = OSC.OSCServer( receive_address )
#add_handlers()

while True:
  read_data()
  #osc.handle_request()
  time.sleep(0.001)
