# If you need to import/define BrickPi or grovepi, do that here.

#import brickpi
#import grovepi

# BP = brickpi.BrickPi()

'''
  First, assign labels to the data you will be sending and 
  and sample rate (samples per second) used to collect data.
  
  The labels are defined in the return statement as a list of strings
  The order of the data labels should be the same as the order defined
  in the getData() function below.
'''

def setParameters( sampleRate=5):            # samples = samples/second
  return ['x', 'y', 'z'], sampleRate

'''
  Define here how to measure and calculate each data point to plot
  Return data as a list of ints, floats, or booleans in the same 
  order defined above.
'''
def getData( prevData):
  ### Begin Example
  # This example will produce 3 sawtooth waves
  limit = 10

  x = prevData[0] + 1
  y = prevData[1] + 1
  z = prevData[2] + 1
  
  if (x > limit):
    x = 0
  if (y > limit):
    y = 0
  if (z > limit):
    z = 0

  ### End Example
  return [x, y, z]  # Make sure this matches the labels above


# Do not edit anything past this line.
######################### Main program #########################
from subprocess import check_output
from threading import Thread
from numpy import array
import piTalk as pt
import time
import sys
showSendData = False


def sampleValues( d):
  t, sampleRate = setParameters()
  sampleRate = 1 / sampleRate

  time.clock()
  lastTime = time.clock()
  global dataBuffer
  while True:
    if (time.clock() - lastTime >= sampleRate):
      lastTime = time.clock()
      d = getData(d)
      dataBuffer.append(d)


def sendBuffer():
  global dataBuffer
  try:
    while True:
      data = dataBuffer
      dataBuffer = []
    
      if not data: continue

      computer.sendData(data, showRawData=showSendData)
      time.sleep(0.001)
  except:
    sys.exit('Closing...')

    
params, sRate = setParameters()

npy = params
dim = list(array(npy).shape)
data = []
if len(dim) == 1:
  data = [0] * dim[0]
elif len(dim) == 2:
  temp = [0] * dim[1]
  for i in range(dim[0]):
    data.append(temp)
else:
  print('Invalid data structure recieved. Only 1 or 2 dimensional lists acceptable')


dataBuffer = []
sampleThread = Thread(target=sampleValues, args=[data])
sampleThread.daemon = True
sampleThread.start()

baseIP = str( check_output(['hostname', '-I']).decode('utf-8').strip().split()[-1][:-1])
ipAddress = str(baseIP + input('Enter the IPv4 Address of the connected computer: ' + baseIP))
computer = pt.PiTalk(ipAddress)

computer.buffer = 1024
computer.sendData(params)
computer.sendData(sRate)

computer.sendData(data)
sendBuffer()

