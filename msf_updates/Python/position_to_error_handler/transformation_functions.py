#!/usr/bin/env python


#helperfunctions for coordinate transformation (i was to stupid to find existing ones)
#formulas taken from: http://nghiaho.com/?page_id=671
#(c) Yannick Huber (huberya)

import numpy as np
from math import sqrt


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
  if np.linalg.det(rotation)<0.0:
    V[:,2]*=-1
    rotation=np.matmul(np.transpose(V), np.transpose(U))
  #print(np.linalg.det(rotation))
  translation=comto-np.matmul(rotation, comfrom)
  return translation, rotation

#computes the transformed point
#translation is a 3 vector and rotation is 3x3 rotation matrix
def transform_point(point, translation, rotation):
  return np.matmul(rotation, point)+translation
  

#algorithm taken from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
#returns the quaternion (w, x, y, z) according to rotation (3x3 orthogonal matrix)
def rot_to_quat(rotation):
  m00=rotation[0,0]
  m01=rotation[0,1]
  m02=rotation[0,2]
  m10=rotation[1,0]
  m11=rotation[1,1]
  m12=rotation[1,2]
  m20=rotation[2,0]
  m21=rotation[2,1]
  m22=rotation[2,2]
  tr = m00 + m11 + m22
  tol=0.001
  if tr > 0+tol:
    S = sqrt(tr+1.0) * 2; # S=4*qw 
    qw = 0.25 * S;
    qx = (m21 - m12) / S;
    qy = (m02 - m20) / S; 
    qz = (m10 - m01) / S; 
  elif m00 > m11+tol and m00 > m22+tol: 
    S = sqrt(1.0 + m00 - m11 - m22) * 2; # S=4*qx 
    qw = (m21 - m12) / S;
    qx = 0.25 * S;
    qy = (m01 + m10) / S; 
    qz = (m02 + m20) / S; 
  elif m11+tol > m22+tol: 
    S = sqrt(1.0 + m11 - m00 - m22) * 2; # S=4*qy
    qw = (m02 - m20) / S;
    qx = (m01 + m10) / S; 
    qy = 0.25 * S;
    qz = (m12 + m21) / S; 
  else:
    S = sqrt(1.0 + m22 - m00 - m11) * 2; # S=4*qz
    qw = (m10 - m01) / S;
    qx = (m02 + m20) / S;
    qy = (m12 + m21) / S;
    qz = 0.25 * S;
  result=np.array([qw, qx, qy, qz])
  return result
