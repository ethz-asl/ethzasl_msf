#!/usr/bin/env python

#to test transfromation functions
import numpy as np
from transformation_functions import estimate_transformation


n=100
pa=np.zeros((3,n))
pb=np.zeros((3,n))
randmat=np.random.rand((3,3))
U,s,V=np.linalg.svd(randmat)
rotation_true=np.matmul(np.transpose(V), np.transpose(U))
print(rotation_true)
translation_true=np.array([1,-2,3])
for i in range(n):
  pa[:,i]=np.random.uniform(-5,5,(3))
  #print(np.matmul(rotation_true, pa[:,i])+translation_true)
  pb[:,i]=np.matmul(rotation_true, pa[:,i])+translation_true

#print(pa)
#print(pb)
translation, rotation=estimate_transformation(pa, pb)
#print(translation)
print(rotation)
