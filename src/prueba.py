# -*- coding: utf-8 -*-
"""
Created on Wed Nov 29 12:43:37 2017

@author: juan
"""
import numpy as np
import math
import random
import tensorflow as tf

def convert_to_one_hot(labels, C):
    """
    Creates a matrix where the i-th row corresponds to the ith
    class number and the jth column
    corresponds to the jth training example. So
    if example j had a label i. Then entry (i,j) 
    will be 1. 
                     
    Arguments:
    labels -- vector containing the labels 
    C -- number of classes, the depth of the one hot dimension
    
    Returns: 
    one_hot -- one hot matrix
    """    
    # Create a tf.constant equal to C (depth), name it 'C'. (approx. 1 line)
    C = tf.constant(C)
    
    # Use tf.one_hot, be careful with the axis (approx. 1 line)
    one_hot_matrix = tf.one_hot(labels, C, axis=0) 
    
    # Create the session (approx. 1 line)
    sess = tf.Session()
    
    # Run the session (approx. 1 line)
    one_hot = sess.run(one_hot_matrix)
    
    # Close the session (approx. 1 line). See method 1 above.
    sess.close()
    
    return one_hot

i_name = '/home/juan/catkin_ws/src/adaptor001/src/images'
data = []
with open(i_name) as i_file:
    images = i_file.read().split(']')
    
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

print(type(data))    
print(len(data))
print(data[0][0:10])
print(type(data[0]))

l_name = '/home/juan/catkin_ws/src/adaptor001/src/labels'    
with open(l_name) as l_file:
    Y = l_file.read().split()
    
    Y = list(map(int, Y))

print(Y)   
print(len(Y))
print(type(Y))
    
combined = list(zip(data,Y))
random.shuffle(combined)
data[:], Y[:] = zip(*combined)

length = len(data)
cut = math.floor(2*length/3)

train_set_x_orig = np.array(data[0:cut])
train_set_y_orig = np.array(Y[0:cut])
test_set_x_orig = np.array(data[cut:-1])
test_set_y_orig = np.array(Y[cut:-1])

classes = []
for i in range(6):
    classes.append(i)
    
X_train_orig = train_set_x_orig
Y_train_orig = train_set_y_orig
X_test_orig = test_set_x_orig
Y_test_orig = test_set_y_orig
    
print(X_train_orig.shape)

# Flatten the training and test images
X_train_flatten = X_train_orig.T
X_test_flatten = X_test_orig.T

print(X_train_flatten.shape)

# Normalize image vectors
X_train = X_train_flatten
X_test = X_test_flatten

print(X_train.shape)
    
# Convert training and test labels to one hot matrices
Y_train = convert_to_one_hot(Y_train_orig, 6)
Y_test = convert_to_one_hot(Y_test_orig, 6)    
   
print ("number of training examples = " + str(X_train.shape[1]))
print ("number of test examples = " + str(X_test.shape[1]))
print ("X_train shape: " + str(X_train.shape))
print ("Y_train shape: " + str(Y_train.shape))
print ("X_test shape: " + str(X_test.shape))
print ("Y_test shape: " + str(Y_test.shape))  

