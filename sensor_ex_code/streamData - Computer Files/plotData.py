from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from numpy import array, linspace
import compTalk as ct
import datetime
import csv
import sys

saveToCSV = True
windowSize = 10 # seconds
refreshTime = 250 # milliseconds

# Define the filename
if saveToCSV:
  print('\nEnter the filename you would like to save the data under.\nOr, press [Enter] for automatic file name.\n')
  filename = input('Filename:  ')
  print()
  # If no filename entered, use a default one defining the data and time the file was started
  if (filename == ''):
    timeStamp = datetime.datetime.now()
    filename = str(timeStamp.year) + '.' + str(timeStamp.month) +  '.' + str(timeStamp.day) +  '.' + str(timeStamp.hour) +  '.' + str(timeStamp.minute) +  '.' + str(timeStamp.second) + '_' + 'RawData.csv'
  # Assure a .csv extension is used
  if (filename[-4:] != '.csv'):
    filename = filename + '.csv'

# Define the connection
pi = ct.CompTalk()
pi.buffer = 1024

# First three communications will define parameters
params = pi.getData()
sRate = pi.getData()
dataFmt = pi.getData()

# Determine if plot should be 1 dimension or 2 dimension
npy = params
dim = list(array(npy).shape)
if len(dim) == 1:
  oneDim = True
elif len(dim) == 2:
  oneDim = False
else:
  sys.exit('Invalid list structure recieved')

# Define the csv writer
writeData = csv.writer(open(filename,'w',newline=''), delimiter=',')
data = []

# Initialize  variables in the correct format based on parameters set by Pi
# Also, create the plot windows
if oneDim:
  numSamples = dim[0]
  data = [[0] * (windowSize*sRate) for i in range(dim[0])]
  
  with open(filename, 'a'):
    writeData.writerow(params)
  
  fig, axarr = plt.subplots(dim[0], 1, sharex=True)
  #plt.subplots_adjust(top=0.99, right=0.91, left=0.15, bottom=0.09)
  if(dim[0] == 1):
    onePlot = True
  else:
    onePlot = False
else:
  numSamples = [dim[0], dim[1]]
  for i in range(dim[0]):
    data.append( [[0]*windowSize*sRate for j in range(dim[1])] )
  
  with open(filename, 'a'):
    header = []
    for label in params:
      [header.append(x) for x in label]
    writeData.writerow(header)
  
  fig, axarr = plt.subplots(dim[0], dim[1], sharex=True)
  #plt.subplots_adjust(top=0.95, right=0.91, left=0.09)

# Define a variable of the correct dimensions to use as the x axis array
xdata = []
[xdata.append(i) for i in linspace(0, windowSize, windowSize*sRate)]

plotting = True

# Called continuously to animate the display
def animate(i):
    global params, data, numSamples, oneDim, onePlot, plotting, filename, writeData, saveToCSV
    
    # Get the new data from the data stream and clear it for new data
    newData = pi.dataStream
    pi.dataStream = []
    
    if not newData: return 0

    newDisplay = []
    if oneDim:
      # Create an empty 1D array, with one entry for each stream
      newDisplay = [[] for i in range(numSamples)]
      
      # For each sample in each packet recieved
      for packet in newData:
        for samples in packet:
          # Separate each element for it's respective data stream
          for i, value in enumerate(samples):
            newDisplay[i].append(value)
      
      # Shift the old data and add the new data on the right for each display window
      for s in range(len(newDisplay)):
        data[s] = list( data[s][len(newDisplay[s]):] + newDisplay[s] )
      
      # Update the plots and labels
      if plotting and (len(data[0]) == len(xdata)):
        if not(onePlot):
          for i, plot in enumerate(axarr):
            plot.clear()
            plot.set_ylabel(params[i], fontweight='bold')
          axarr[-1].set_xlabel('Time (sec)', fontweight='bold')
      
          # Plot each data stream
          for i, ydata in enumerate(data):
            axarr[i].plot(xdata, ydata)

        else:
          # If only 1 sample is sent, axarr is not iterable, must address directly (not through loops)
          axarr.clear()
          axarr.set_ylabel(params[i], fontweight='bold')
          axarr.set_xlabel('Time (sec)', fontweight='bold')
          axarr.plot(xdata, data[0])
        
        fig.tight_layout()
      
      # If saving to a csv, append it to the file
      if saveToCSV:
        with open(filename, 'a'):
          [writeData.writerow(list(i)) for i in zip(*newDisplay)]
        
    else: #2 Dimensional display
      # Create an empty 2D array, with one entry for each stream
      for i in range(numSamples[0]):
        newDisplay.append( [[] for j in range(numSamples[1])] )
      
      # For each sample in each packet recieved
      for packet in newData:
        for samples in packet:
          # separate the data for each row and column into it's respective data stream array
          for row, values in enumerate(samples):
            for col, value in enumerate(values):
                newDisplay[row][col].append(value)
      
      # Shift the old data and add the new data on the right for each display window
      for i in range(numSamples[0]):
        for j in range(numSamples[1]):
          data[i][j] = data[i][j][len(newDisplay[i][j]):] + newDisplay[i][j]
      
      # Update the plots and labels
      if plotting and (len(data[0][0]) == len(xdata)):
        for i in range(len(axarr)):
          for j in range(len(axarr[0])):
            axarr[i][j].clear()
            axarr[i][j].set_title(params[i][j], fontweight='bold')
      
        for i in range(len(axarr[-1])):
          axarr[-1][i].set_xlabel('Time (sec)', fontweight='bold')
      
        # Plot each data stream
        for i, row in enumerate(data):
          for j, ydata in enumerate(row):
            if (len(ydata) > len(xdata)):
              print("overflow!")
              ydisp = ydata[-1*len(xData):]
            else:
              ydisp = ydata
            axarr[i][j].plot(xdata, ydisp)
          
        fig.tight_layout()

      # If saving to a csv, append it to the file
      if saveToCSV:
        csvData = []
        for time in range(len(newDisplay[0][0])):
          timePoint = []
          for row in newDisplay:
            for sample in row:
              timePoint.append(sample[time])
          csvData.append(timePoint)
            
        with open(filename, 'a'):
          writeData.writerows([x for x in csvData])
          

def pausePlot(event):
  global plotting
  if plotting:
    plotting = False
  else:
    plotting = True


try:
  # Initiate the data stream
  content = pi.streamData(showRawData=False)
  # Start the animation
  cid = fig.canvas.mpl_connect('key_press_event', pausePlot)
  ani = FuncAnimation(fig, animate, interval=refreshTime)
  plt.show()

except:
  print('Ending Stream...')
  pi.streaming = False
  sys.exit()

finally:
  print('Stopping Plot...')
  pi.streaming = False
  sys.exit()