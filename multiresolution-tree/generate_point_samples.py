# generate point samples from an image

from PIL import Image
from random import *
import matplotlib.pyplot as plt

output = open("teapot.xyz", "w")

im = Image.open('D:/Downloads/teapot.jpg', 'r')
width, height = im.size
pixel_values = list(im.getdata())
count = 0
arr_x = []
arr_y = []
arr   = []
for i in range(0, 100000):
  x = int(random() * width)
  y = int(random() * height)
  if (x >= width) or (y >= height):
    continue
  #print(pixel_values[width * y + x])
  if (pixel_values[width*y+x] == 255):
    continue
  else:
    count = count + 1
    #print(x, y)
    #arr_x.append(x)
    #arr_y.append(y)
    arr.append((x, y))

arr = list(dict.fromkeys(arr))

output.write("%s\n" % len(arr))
output.write("teapot\n")
for i in range(0, len(arr)):
  output.write("C %s %s 0\n" % (arr[i]))

output.close()

plt.scatter(arr_x, arr_y, s=1)
plt.show()
print(count)