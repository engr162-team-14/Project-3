import numpy as np
import csv
import matplotlib.pyplot as plt

file = open("DC3_Data.csv", "r")
data = csv.reader(file)

pixel_data = []

for row in data:
    pixel_data.append(float(row[0]))
    
image = np.reshape(pixel_data, (25,25))
image = np.array(image, dtype=float)

plt.figure(1)
plt.gray()
plt.imshow(image)
plt.show()