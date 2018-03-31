import os
import sys
import numpy as np

if len(sys.argv)!=3:
	print("usage python postprocess_ts features_file label_file")
	sys.exit()
featfile=sys.argv[1]
labelfile=sys.argv[2]

ffeat = open(featfile, "r")
flabel = open(labelfile, "r")

linesfeat = ffeat.readlines()
lineslabel = flabel.readlines()

ffeat.close()
flabel.close()

#this is separating line "starting new training sequence\n"
#make sure that between any two separators or the end there are equal number of lines
#replace number by "deleted" to remove (to maintain indecis) and dont write these in the very end
featidx = 0
labelidx = 0

while featidx<len(linesfeat) and labelidx<len(lineslabel):
	#count to next separator
	featidxt = featidx
	labelidxt = labelidx
	while featidxt<len(linesfeat):
		if linesfeat[featidxt]=="starting new training sequence\n":
			#print("found sep")
			break
		featidxt+=1
	while labelidxt<len(lineslabel):
		if lineslabel[labelidxt]=="starting new training sequence\n":
			#print("found sep")
			break
		labelidxt+=1
	#check which one is larger
	if featidxt-featidx>labelidxt-labelidx:
		for i in range(featidx,featidxt+labelidx-labelidxt):
			linesfeat[i]="deleted"
	elif labelidxt-labelidx>featidxt-featidx:
		for i in range(labelidx,labelidxt+featidx-featidxt):
			lineslabel[i]="deleted"
	#if equal we do not need to do anything
	featidx=featidxt+1
	labelidx=labelidxt+1

ffeat = open(featfile, "w")

for i in range(len(linesfeat)):
	if linesfeat[i]!="deleted":
		ffeat.write(linesfeat[i])
ffeat.close()

flabel = open(labelfile, "w")
for i in range(len(lineslabel)):
	if lineslabel[i]!="deleted":
		flabel.write(lineslabel[i])
flabel.close()
