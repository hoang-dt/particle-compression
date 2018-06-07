# Comments
import math
import matplotlib.pyplot as plt
import numpy as np
import sys

if len(sys.argv) != 3:
  print(sys.argv)
  print('Args: [text file with data] [output text file]')
  exit()

def integrate_h(m, l, a, b):
  return (math.exp(l*a)-math.exp(l*b)) / (2-2*math.exp(l*m))

def integrate_exp(l, a, b):
  return math.exp(-l*a) - math.exp(-l*b);

arr = np.loadtxt(sys.argv[1], dtype='float64')
#h = np.histogram(arr, bins=512)
h = np.histogram(arr, bins=[-8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
#print(h)
fig, ax = plt.subplots()
s = np.sum(h[0])
plt.plot(h[1][:-1], h[0][:]/s)
#plt.plot(h[1][1:50], h[0][:49]/s)
#print(h[1][49])
m = 8
#l = 0.0647637
l = 0.000817401
#l = 0.00295633
#l = 0.0529976
x = np.linspace(-m, m, 17)
#x = np.linspace(0, 286.630859375, 512)
#x = np.linspace(0, 18362.0800781, 512)
y = np.copy(x)
#sum = 0
#for i in range(0, len(y)):
#  y[i] = integrate_exp(l, x[i], x[i]+18362.0800781/512)
#  sum += integrate_exp(l, x[i], x[i]+18362.0800781/512)
#print(sum)

sum = 0
for i in range(0, len(y)):
  if x[i] == 0:
    y[i] = 2*integrate_h(m+0.5, l, x[i], x[i]+0.5)
    sum += 2*integrate_h(m+0.5, l, x[i], x[i]+0.5)
  elif x[i] < 0:
    y[i] = integrate_h(m+0.5, l, -x[i]-0.5, -x[i]+0.5)
    sum += integrate_h(m+0.5, l, -x[i]-0.5, -x[i]+0.5)
  else:
    y[i] = integrate_h(m+0.5, l, x[i]-0.5, x[i]+0.5)
    sum += integrate_h(m+0.5, l, x[i]-0.5, x[i]+0.5)
#plt.plot(x, l/2 * np.exp(l*abs(x))/(math.exp(l*m)-1))
print(sum)

plt.plot(x, y)
#plt.plot(x, l * np.exp(-l*x))
plt.show()
fig.tight_layout()
fig.savefig(sys.argv[2])
print('hello')
