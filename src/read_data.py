# -*- coding: utf-8 -*-
"""
Created on Fri Nov 24 10:29:51 2017

@author: juan
"""

f = open("/home/juan/catkin_ws/src/adaptor001/src/images")
images = f.read().split(']')
f.close()

g = open("/home/juan/catkin_ws/src/adaptor001/src/labels")
lab = g.read().split()
g.close()

print(lab)
print(type(lab))

Y = []

for y in lab:
    Y.append(int(y))

print(Y)
print(type(Y))
print(type(Y[0]))

print(len(images))

data = []
data1 = []
data2 = []
    
for l in images:
    data1.append(l.replace('[',''))

for d in range(len(data1)):
    data2.append(data1[d].split())
    
for d in range(len(data2)):
    l = []
    for i in range(len(data2[d])):
        l.append(float(data2[d][i].replace(',','')))
    data.append(l)
    
print(data[0][0:10])
print(type(data))
print(type(data[0]))
print(type(data[0][0]))

print(len(data))
print(len(images))

#return data, Y



