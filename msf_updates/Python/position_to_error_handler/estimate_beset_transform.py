#!/usr/bin/env python



#This code is used to estimate the best transform for data comming from 2 csv files
#they are supposed to have the following structure: timestamp, pos_x, pos_y, pos_z
#it goes throug the whole file matching points according to their timestamp and outputs the 
#transfromation as: translation, quaternion, rotationmatrix (3x3)
#(c) Yannick Huber (huberya)




import numpy as np
import csv
import os
import sys
import math
#sys.path.insert(0, '~/catkin_ws/src/ethzasl_msf/msf_core')
from transformation_functions import estimate_transformation, transform_point, quaternion_to_matrix, rot_to_quat

      
if __name__ == '__main__':
  if len(sys.argv)!=3:
    print("usage: python estimate_best_transform.py <csvfrom> <csvto>")
    sys.exit()
  fromframe=sys.argv[1]
  toframe=sys.argv[2]
  fromcsv=open(fromframe, "r")
  tocsv=open(toframe, "r")
  #print(len(list(csv.reader(fromcsv))))
  fromreader=np.transpose(np.asarray(list(csv.reader(fromcsv))[1:], dtype=float))
  toreader=np.transpose(np.asarray(list(csv.reader(tocsv))[1:], dtype=float))
  #start reading at line 1 (i.e. second line) since first is expected to be header
  #search for matching starttime
  fromline=1
  toline=1
  frommax=fromreader.shape[1]-2
  tomax=toreader.shape[1]-2
  if fromreader[0, 1]<toreader[0, 1]:
    while fromreader[0, fromline]<toreader[0, toline]:
      fromline+=1
      if fromline>=fromreader.shape[1]:
        print("no matching times found. Do your files agree on timestamp?")
        sys.exit()
  else:
     while toreader[0, toline]<fromreader[0, fromline]:
      toline+=1
      if toline>=toreader.shape[1]:
        print("no matching times found. Do your files agree on timestamp?")
        sys.exit()
  #do same from back to front so we have a window were times match (somehow)
  if fromreader[0, frommax]<toreader[0, tomax]:
    while fromreader[0, frommax]<toreader[0, tomax]:
      tomax-=1
      #here no strange exception can happen or we would already have discovered it above
  else:
     while toreader[0, tomax]<fromreader[0, frommax]:
      frommax-=1
      #same
  #now for [fromline, frommax] and [toline, tomax] the times match approximately (not 1 by 1)
  #set stepsize according to their sizes
  print("using data from to")
  print(fromline)
  print(frommax)
  tostep=1.0
  fromstep=(frommax-fromline)/(tomax-toline)
  #initialize big zero matrices
  frompoints=np.zeros((3,tomax-toline))
  topoints=np.zeros((3,tomax-toline))
  for i in range(tomax-toline):
    topoints[:,i]=toreader[1:4, toline+i]
    evalpoint=fromline+i*fromstep
    fraction=evalpoint-math.floor(evalpoint)
    frompoints[:,i]=(1-fraction)*fromreader[1:4, int(math.floor(evalpoint))]-fraction*fromreader[1:4, int(math.ceil(evalpoint))]
  #gives best transform from frompoints to topoints
  print("printing points")
  print(frompoints.shape)
  print("done")
  [translation, rotation]=estimate_transformation(frompoints, topoints)
  print(translation)
  print(rot_to_quat(rotation))
  print(rotation)
