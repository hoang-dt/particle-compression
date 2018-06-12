# Comments
import math
import matplotlib.pyplot as plt
import numpy as np
import scipy.special
import sys

if len(sys.argv) != 3:
  print(sys.argv)
  print('Args: [text file with data] [output text file]')
  exit()

def integrate_h(m, l, a, b):
  return (math.exp(l*a)-math.exp(l*b)) / (2-2*math.exp(l*m))

def integrate_exp(l, a, b):
  return math.exp(-l*a) - math.exp(-l*b);

def integrate_gamma(a, b, beg, end):
  y = 0
  delta = (end-beg)/32
  x = beg
  while x < end:
    y += delta*pow(x, (a-1))/(scipy.special.gamma(a)*pow(b,a))*math.exp(-x/b)
    x += delta
  return y

arr = np.loadtxt(sys.argv[1], dtype='float64')
h = np.histogram(arr, bins=1024)
#h = np.histogram(arr, bins=[-8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
#print(h)
fig, ax = plt.subplots()
s = np.sum(h[0])
print(np.average(h[0]))
#plt.plot(h[1][:-1], h[0][:]/s)
plt.plot(h[1][1:40], h[0][:39]/s, label='actual data')
print(h[1][1])
print(h[1][39])
m = 8
#l = 0.0647637
l = 0.000817401
l = 0.00073118
#l = 0.00295633
#l = 0.0529976
x = np.linspace(h[1][1], h[1][39], 40)
delta = h[1][39]-h[1][1]
#x = np.linspace(0, 286.630859375, 512)
#x = np.linspace(0, 18362.0800781, 512)
y = np.copy(x)
z = np.copy(x)
#sum = 0
for i in range(0, len(y)):
  z[i] = integrate_exp(l, x[i]-delta/39, x[i])
  #sum += integrate_exp(l, x[i], x[i]+18362.0800781/512)
#print(sum)

#sum = 0
#for i in range(0, len(y)):
#  if x[i] == 0:
#    y[i] = 2*integrate_h(m+0.5, l, x[i], x[i]+0.5)
#    sum += 2*integrate_h(m+0.5, l, x[i], x[i]+0.5)
#  elif x[i] < 0:
#    y[i] = integrate_h(m+0.5, l, -x[i]-0.5, -x[i]+0.5)
#    sum += integrate_h(m+0.5, l, -x[i]-0.5, -x[i]+0.5)
#  else:
#    y[i] = integrate_h(m+0.5, l, x[i]-0.5, x[i]+0.5)
#    sum += integrate_h(m+0.5, l, x[i]-0.5, x[i]+0.5)
#plt.plot(x, l/2 * np.exp(l*abs(x))/(math.exp(l*m)-1))
#print(sum)

a = 0.336963
#a = 0.142655
b = 4058.76
#b = 9000
#b = 267694.08536
#b = 0.255292 * 1.04858e+06
sum = 0
for i in range(0, len(y)):
  #y[i] = pow(x[i], (a-1))/(scipy.special.gamma(a)*pow(b,a))*math.exp(-x[i]/b)
  y[i] = integrate_gamma(a,b,x[i]-delta/39,x[i])
  sum += y[i]
  #print(y[i])
#print(x)
plt.plot(x[:], y[:], label='gamma')
plt.plot(x, z, label='exponential')
#plt.plot(x, l * np.exp(-l*x))
plt.legend()
plt.yscale('log')
plt.show()
fig.tight_layout()
fig.savefig(sys.argv[2])
print('hello')
