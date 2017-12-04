#!/usr/bin/env python


#helperfunctions for coordinate transformation (i was to stupid to find existing ones)
#formulas taken from: http://nghiaho.com/?page_id=671
#(c) Yannick Huber (huberya)

import numpy as np


def quaternion_to_matrix(quaternion):
  assert len(quaternion)==4
  q=np.reshape(quaternion/np.linalg.norm(quaternion), (4,1))
  qquad=np.matmul(q,np.transpose(q))
  return np.array([
    [1.0-qquad[2,2]-qquad[3, 3],     qquad[1, 2]-qquad[3, 0],     qquad[1, 3]+qquad[2, 0]],
    [    qquad[1, 2]+qquad[3, 0], 1.0-qquad[1, 1]-qquad[3, 3],     qquad[2, 3]-qquad[1, 0]],
    [    qquad[1, 3]-qquad[2, 0],     qquad[2, 3]+qquad[1, 0], 1.0-qquad[1, 1]-qquad[2, 2]]])
    
#estimates the best transformation based on 2 point clouds represented by (3,n) matrices
def estimate_transformation(pointsfrom, pointsto):
  #make sure matrices have same dimensions
  assert pointsfrom.shape==pointsto.shape

  #make sure points have 3 coordinates
  assert pointsfrom.shape[0]==3

  npoints=pointsfrom.shape[1]
  #print(npoints)
  comfrom=np.sum(pointsfrom, axis=1)/npoints
  #print(comfrom)
  comto=np.sum(pointsto, axis=1)/npoints
  #print(comto)
  #print(pointsfrom[:,0])
  #print(comfrom)
  #print(pointsto[:,0])
  #print(comto)
  #print(np.matmul(np.reshape(pointsfrom[:,0]-comfrom,(3,1)), np.reshape(pointsto[:,0]-comto,(1,3))))
  #H_test=np.zeros((3,3))
  #for i in range(npoints):
  #  H_test+=np.matmul(np.reshape(pointsfrom[:,i]-comfrom,(3,1)), np.reshape(pointsto[:,i]-comto,(1,3)))
  #print(H_test)
  #comfrommat=np.matmul(np.reshape(comfrom, (1,3)),np.ones((3,npoints)))
  #comtomat=np.matmul(np.reshape(comto,(1,3)),np.ones((3,npoints)))
  #H=np.matmul((pointsfrom-comfrommat), np.transpose(pointsto-comtomat))
  H=sum(np.matmul(np.reshape(pointsfrom[:,i]-comfrom,(3,1)), np.reshape(pointsto[:,i]-comto,(1,3))) for i in range(npoints))
  #print(H)
  U,s,V=np.linalg.svd(H)
  rotation=np.matmul(np.transpose(V), np.transpose(U))
  if np.linalg.det(rotation)<-0.9:
    V[:,2]*=-1
    rotation=np.matmul(np.transpose(V), np.transpose(U))
  #print(np.linalg.det(rotation))
  translation=comto-np.matmul(rotation, comfrom)
  return translation, rotation

#computes the transformed point
#translation is a 3 vector and rotation is 3x3 rotation matrix
def transform_point(point, translation, rotation):
  return np.matmul(rotation, point)+translation
  
  
  
  
