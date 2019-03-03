'''
  This file produces 6 equivalent sawtooth signals and transmits them
  to a computer for plotting in a 2-Dimensional plot layout (multiple
  columns of plots)
  
  Use this file to test the connection between your computer and the Pi.
'''

def setParameters( sampleRate=5):            # samples = samples/second
    return [['x', 'y', 'z'], ['j', 'k', 'l']], sampleRate

    
def getData( prevData):
  ### Begin Example
  # This example will produce 6 sawtooth waves
  limit = 10
  
  x = prevData[0][0] + 1
  y = prevData[0][1] + 1
  z = prevData[0][2] + 1
  
  j = prevData[1][0] - 1
  k = prevData[1][1] - 1
  l = prevData[1][2] - 1
  
  if (x > limit):
    x = 0
  if (y > limit):
    y = 0
  if (z > limit):
    z = 0
    
  if (j < 0):
    j = limit
  if (k < 0):
    k = limit
  if (l < 0):
    l = limit

  ### End Example
  return [[x, y, z], [j, k, l]]  # Make sure this matches the labels above


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
