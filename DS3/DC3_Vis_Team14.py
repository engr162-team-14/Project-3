import numpy as np
import csv
import matplotlib.pyplot as plt

file = open("DC3_Data.csv", "r")
data = csv.reader(file)
image = []
img = []
img1 = []
img2 = []
img3 = []
img4 = []

for row in data:
    img.append(float(row[0]))
    img1.append(float(row[1]))
    img2.append(float(row[2]))
    img3.append(float(row[3]))
    img4.append(float(row[4]))
for x in range(0,5):
    image.append(img[x])
for x in range(0,5):
    image.append(img1[x])
for x in range(0,5):
    image.append(img2[x])
for x in range(0,5):
    image.append(img3[x])
for x in range(0,5):
    image.append(img4[x])
image = np.arange(25)
i = image.reshape(5,5)
data1 = np.array(i, dtype=float)
plt.figure(1)
plt.gray()
plt.imshow(data1)
plt.show()