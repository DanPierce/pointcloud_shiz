#!/usr/bin/env python

import roslib; roslib.load_manifest('pointcloud_shiz')

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells
import tf
from math import sqrt

def callback(msg):
   
    pub = rospy.Publisher("object1", Point)
    
    block1=Point()
    block2=Point()
    cellSize=.05 #meters/cell
    Dmin=.08 #equal to the largest possible diagonal of desired object
    Dmax=.17 # equal to the minimum distance from object to wall
    D=[0]*len(msg.cells)
    objListx=[0]
    objListy=[0]
    for i in range(len(msg.cells)): # iterate through all the cells
        x=msg.cells[i].x
        y=msg.cells[i].y
        boolObject = 0
        for j in range(len(msg.cells)): #check if any periphery cells lay within radius
            D[j]=sqrt(pow((x-msg.cells[j].x),2)+pow((y-msg.cells[j].y),2)) # calc distance
            if (D[j]<Dmax and D[j]>Dmin): # if there is a cell within band, cell[i] is not object
                boolObject=1
        if (boolObject==0): # if boolObject remains at zero, cell[i] is object
            countAdd=0
            for k in range(len(objMat)): # add to list of 
                if (int(x*1000+.1)==int(objMat[k][0]*1000+.1) and int(y*1000+.1)==int(objMat[k][1]*1000+.1)):
                    objMat[k][2]=objMat[k][2]+1
                    countAdd=1
            if (countAdd==0):
                objMat.append([x,y,1])
    hiCountPose = [0, 0, 0]
    for k2 in range(len(objMat)):
        if (objMat[k2][2]>hiCountPose[2]):
            hiCountPose=[objMat[k2][0], objMat[k2][1], objMat[k2][2]]
    block1.x=hiCountPose[0]
    block1.y=hiCountPose[1]
    block1.z=5.0
    pub.publish(block1)
            #objX[i]=int(x*1000+.1)
            #objY[i]=int(y*1000+.1)



def listener():
    rospy.init_node('cmap_block_detect')
    
    rospy.Subscriber("/move_base/global_costmap/obstacles", GridCells, callback)
    
    rospy.spin()

if __name__ == '__main__':
    objMat=[[0]*3]*1
    try:
        listener()
    except rospy.ROSInterruptException: pass

        #for nx in range(msg.cells[i].x-cellSize, msg.cells[i].x+cellSize, cellSize):


    #for nx in [float(temp)/100 for temp in range(0, 100, 1)]: 
    #print(msg.cells[i].x)
    #xx = [2, 4, -11]
    #test=xx[0]
    #print(test)



