#!/usr/bin/env python


#helperfunctions for coordinate transformation (i was to stupid to find existing ones)
#formulas taken from: http://nghiaho.com/?page_id=671
#(c) Yannick Huber (huberya)

import numpy as np
from math import sqrt


def quaternion_to_matrix(quaternion):
  assert len(quaternion)==4
  """
  q=np.reshape(quaternion/np.linalg.norm(quaternion), (4,1))
  qquad=np.matmul(q,np.transpose(q))
  return np.array([
    [1.0-qquad[2,2]-qquad[3, 3],     qquad[1, 2]-qquad[3, 0],     qquad[1, 3]+qquad[2, 0]],
    [    qquad[1, 2]+qquad[3, 0], 1.0-qquad[1, 1]-qquad[3, 3],     qquad[2, 3]-qquad[1, 0]],
    [    qquad[1, 3]-qquad[2, 0],     qquad[2, 3]+qquad[1, 0], 1.0-qquad[1, 1]-qquad[2, 2]]])
  """
  rotation=np.zeros((3,3))
  sqw=quaternion[0]*quaternion[0]
  sqx=quaternion[1]*quaternion[1]
  sqy=quaternion[2]*quaternion[2]
  sqz=quaternion[3]*quaternion[3]
  invs=1/(sqw+sqx+sqy+sqz)
  rotation[0,0]=(sqx-sqy-sqz+sqw)*invs
  rotation[1,1]=(-sqx+sqy-sqz+sqw)*invs
  rotation[2,2]=(-sqx-sqy+sqz+sqw)*invs
   
  tmp1=quaternion[1]*quaternion[2]
  tmp2=quaternion[3]*quaternion[0]
  rotation[1,0]=2*(tmp1+tmp2)*invs
  rotation[0,1]=2*(tmp1-tmp2)*invs
  
  tmp1=quaternion[1]*quaternion[3]
  tmp2=quaternion[2]*quaternion[0]
  rotation[2,0]=2*(tmp1-tmp2)*invs
  rotation[0,2]=2*(tmp1+tmp2)*invs
  
  tmp1=quaternion[2]*quaternion[3]
  tmp2=quaternion[1]*quaternion[0]
  rotation[2,1]=2*(tmp1+tmp2)*invs
  rotation[1,2]=2*(tmp1-tmp2)*invs
   
  return rotation
   
#estimates the best transformation based on 2 point clouds represented by (3,n) matrices
def estimate_transformation(pointsfrom, pointsto):
  #make sure matrices have same dimensions
  assert pointsfrom.shape==pointsto.shape

  #make sure points have 3 coordinates
  assert pointsfrom.shape[0]==3

  npoints=pointsfrom.shape[1]
  #print(npoints)
  comfrom=np.mean(pointsfrom, axis=1)
  print(comfrom)
  comto=np.mean(pointsto, axis=1)
  print(comto)

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
  #print("n")
  #print(point)
  #print(np.matmul(rotation, point)+translation)
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
  tr1=1+m00-m11-m22
  tr2=1-m00+m11-m22
  tr3=1-m00-m11+m22
  tol=0.00001
  if tr1>tr2 and tr2>tr3: 
    S = sqrt(1.0 + m00 - m11 - m22) * 2; # S=4*qx 
    qw = (m21 - m12) / S;
    qx = 0.25 * S;
    qy = (m01 + m10) / S; 
    qz = (m02 + m20) / S; 
  elif tr2>tr1 and tr2>tr3: 
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
