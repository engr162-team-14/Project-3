import numpy as np
import csv
import matplotlib.pyplot as plt

file = open("DC3_Data.csv", "r")
data = csv.reader(file)

img = []

for row in data:
    img.append(float(row[0]))
    
i = np.reshape(img, (25,25))
data1 = np.array(i, dtype=float)
plt.figure(1)
plt.gray()
plt.imshow(data1)
plt.show()