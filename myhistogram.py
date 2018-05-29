# Comments
import matplotlib.pyplot as plt
import numpy as np
import sys

if len(sys.argv) != 3:
  print(sys.argv)
  print('Args: [text file with data] [output text file]')
  exit()

arr = np.loadtxt(sys.argv[1], dtype='float64')
h = np.histogram(arr, bins=1024)
#print(h)
fig, ax = plt.subplots()
plt.plot(h[1][1:], h[0])
plt.show()
fig.tight_layout()
fig.savefig(sys.argv[2])
print('hello')
