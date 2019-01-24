import matplotlib.pyplot as plt
import math
import numpy

f = open("../random-result.txt")

random = []
for line in f:
    random.append(list(line.strip('\n').split(' ')))

x = []
y = []

for i in range(100):
    x.append(eval(random[i][0]))
    y.append(eval(random[i][1]))

abcd = []

for i in range(4):
    abcd.append(eval(random[100][i]))

plt.scatter(x,y)
#plt.show()

x_e = numpy.linspace(0,1,1000)
y_e = [math.exp(abcd[0]*(i**3)+abcd[1]*(i**2)+abcd[2]*(i**1)+abcd[3]) for i in x_e]

x_r = numpy.linspace(0,1,1000)
y_r = [math.exp(1*(i**3)+2*(i**2)+1*(i**1)+1.5) for i in x_r]

plt.plot(x_e,y_e)
plt.plot(x_r,y_r)
plt.show()