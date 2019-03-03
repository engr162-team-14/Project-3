#import brickpi
import grovepi

# BP = brickpi.BrickPi()

# Connect the Grove Ultrasonic Ranger to digital port D4
# SIG,NC,VCC,GND
ultrasonic_ranger = 4

def setParameters( sampleRate=5):
  return ['x'], sampleRate


def getData( prevData):
  # Begin Example
  x = grovepi.ultrasonicRead(ultrasonic_ranger)
  x = 1

  return [x]
  # End Example



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

