#!/usr/bin/env python
# coding: utf-8

# In[1]:


import math
import sys
import numpy as np
import matplotlib.pyplot as plt


# In[2]:


def obstacle(node):
    x= node[0]
    y= node[1]
    c=0
    d=0.3
    if(1-d<x and x<2+d and 2-d<y and y<8+d):
        c=1
    elif(5-d<x and x<7+d and 2-d<y and y<8+d):
        c=1
    elif(10-d<x and x<11+d and 2-d<y and y<8+d):
        c=1
    elif (0+d>x or x>12-d or 0+d>y or y>10-d):
        c=1
    return c

def obstacleOut(node):
    x= node[0]
    y= node[1]
    c=0
    d=0.3
    if(1-d<x and x<11+d and 2-d<y and y<8+d):
        c=1
    elif (0+d>x or x>12-d or 0+d>y or y>10-d):
        c=1
    return c

def obstacleNode(node):
    x= node[0]
    y= node[1]
    c=0
    d=0
    if(1-d<x and x<2+d and 2-d<y and y<8+d):
        c=1
    elif(5-d<x and x<7+d and 2-d<y and y<8+d):
        c=1
    elif(10-d<x and x<11+d and 2-d<y and y<8+d):
        c=1
    elif (0>x or x>12 or 0>y or y>10):
        c=1
    return c


# In[3]:


r = 0.076/2
l = 0.230
rpm1 = 5
rpm2 = 10

def movement(node,ul,ur,theta):
    n_nd = [0,0]
    x = node[0]
    y = node[1]
    for i in range(0,200):
        d_theta = (r/l)*(ur-ul)*0.005
        dx = (r/2)*(ul+ur)*(math.cos(theta))*0.005
        dy = (r/2)*(ul+ur)*(math.sin(theta))*0.005
        x = x + dx
        y = y + dy
        theta = theta + d_theta
    n_nd[0] = (n_nd[0]+x)
    n_nd[1] = (n_nd[1]+y)
    n_nd = [ ((math.floor(n_nd[0] * 100)) / 100.0), ((math.floor(n_nd[1] * 100)) / 100.0) ]
    return n_nd,theta

def distance( node1 , node2 ):
    h = math.sqrt ( (node1[0] - node2[0])**2 +  (node1[1] - node2[1])**2 )
    return h

def checkPoint(node, points):
    flag = 0
    for point in points:
        if (((node[0] - point[0])**2 + (node[1] - point[1])**2) - 0.1**2 < 0):
            return True
    return False


# In[4]:


def Astar(ndi,goal,theta_i):
    p_nd=[ndi]
    c_nd=[ndi]
    theta=[theta_i]
    cst=[distance(ndi,ndi)]
    hst=[distance(goal,ndi)]
    act=[[0,0]]
    vp_nd=[]
    vc_nd=[]
    v_theta=[]
    v_hst=[]
    v_cst=[]
    v_act=[]
    ndx = ndi
    flag=0
    
    def goal_achieved(node):
        c=0
        if (node[0]-goal[0])**2+(node[1]-goal[1])**2<(0.4)**2:
            c=1
        return c
    
    i=0
    while(flag!=1):

        new_node,new_theta=movement(ndx, 0,rpm1,theta[i])
        if (obstacle(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [0,rpm1]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([0,rpm1])

        new_node,new_theta=movement(ndx, 0,rpm2,theta[i])
        if (obstacle(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [0,rpm2]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([0,rpm2])

        new_node,new_theta=movement(ndx, rpm1,rpm2,theta[i])
        if (obstacle(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [rpm1,rpm2]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([rpm1,rpm2])

        new_node,new_theta=movement(ndx, rpm1,rpm1,theta[i])
        if (obstacle(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [rpm1,rpm1]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([rpm1,rpm1])

        new_node,new_theta=movement(ndx, rpm2,rpm2,theta[i])
        if (obstacle(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [rpm2,rpm2]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([rpm2,rpm2])

        new_node,new_theta=movement(ndx, rpm2,rpm1,theta[i])
        if (obstacle(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [rpm2,rpm1]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([rpm2,rpm1])

        new_node,new_theta=movement(ndx, rpm2,0,theta[i])
        if (obstacle(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [rpm2,0]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([rpm2,0]) 

        new_node,new_theta=movement(ndx, rpm1,0,theta[i])
        if (obstacle(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [rpm1,0]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([rpm1,0]) 

        vp_nd.append(p_nd.pop(i))
        vc_nd.append(c_nd.pop(i))
        v_theta.append(theta.pop(i))
        v_hst.append(hst.pop(i))
        v_act.append(act.pop(i))
        v_cst.append(cst.pop(i))

        if(flag==0 and c_nd==[]):         
            sys.exit("Path not found")

        if(goal_achieved(vc_nd[-1])==1):
            flag=1
        #print(vc_nd[-1])
        if(flag!=1):
            i=hst.index(min(hst))
            ndx=c_nd[i][:]


    seq=[v_act[-1]]
    x=vp_nd[-1]
    i=1
    while(x!=ndi):
        if(vc_nd[-i]==x):
            seq.append(v_act[-i])
            x=vp_nd[-i]
        i=i+1     

    seqn=[]
    seqn.append(vc_nd[-1])
    seqn.append(vp_nd[-1])
    x=vp_nd[-1]
    i=1
    while(x!=ndi):
        if(vc_nd[-i]==x):
            seqn.append(vp_nd[-i])
            x=vp_nd[-i]
        i=i+1
    return vc_nd,seqn


# In[5]:


def AstarOUT(ndi,goal,theta_i):
    p_nd=[ndi]
    c_nd=[ndi]
    theta=[theta_i]
    cst=[distance(ndi,ndi)]
    hst=[distance(goal,ndi)]
    act=[[0,0]]
    vp_nd=[]
    vc_nd=[]
    v_theta=[]
    v_hst=[]
    v_cst=[]
    v_act=[]
    ndx = ndi
    flag=0
    
    def goal_achieved(node):
        c=0
        if (node[0]-goal[0])**2+(node[1]-goal[1])**2<(0.4)**2:
            c=1
        return c
    
    i=0
    while(flag!=1):

        new_node,new_theta=movement(ndx, 0,rpm1,theta[i])
        if (obstacleOut(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [0,rpm1]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([0,rpm1])

        new_node,new_theta=movement(ndx, 0,rpm2,theta[i])
        if (obstacleOut(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [0,rpm2]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([0,rpm2])

        new_node,new_theta=movement(ndx, rpm1,rpm2,theta[i])
        if (obstacleOut(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [rpm1,rpm2]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([rpm1,rpm2])

        new_node,new_theta=movement(ndx, rpm1,rpm1,theta[i])
        if (obstacleOut(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [rpm1,rpm1]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([rpm1,rpm1])

        new_node,new_theta=movement(ndx, rpm2,rpm2,theta[i])
        if (obstacleOut(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [rpm2,rpm2]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([rpm2,rpm2])

        new_node,new_theta=movement(ndx, rpm2,rpm1,theta[i])
        if (obstacleOut(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [rpm2,rpm1]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([rpm2,rpm1])

        new_node,new_theta=movement(ndx, rpm2,0,theta[i])
        if (obstacleOut(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [rpm2,0]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([rpm2,0]) 

        new_node,new_theta=movement(ndx, rpm1,0,theta[i])
        if (obstacleOut(new_node)!=1):
            if checkPoint(new_node, vc_nd)!= True:
                check=0
                for cku in range(0,len(c_nd)):
                    if(new_node == c_nd[cku]):
                        check=1
                        if(cst[cku]>=(cst[i]+distance( ndx , new_node ))):
                            p_nd[cku]=ndx
                            cst[cku] = cst[i]+distance( ndx , new_node )
                            hst[cku] = cst[i] + distance( ndx , new_node ) + distance( goal , new_node )
                            #hst[cku] = distance( goal , new_node )
                            act[cku] = [rpm1,0]
                            theta[cku] = new_theta
                            break
                if check!=1:
                    p_nd.append(ndx)
                    c_nd.append(new_node)
                    theta.append(new_theta)
                    cst.append(cst[i] + distance( ndx , new_node ))
                    hst.append(cst[i] + distance( ndx , new_node ) + distance( goal , new_node ))
                    #hst.append(distance( goal , new_node ))
                    act.append([rpm1,0]) 

        vp_nd.append(p_nd.pop(i))
        vc_nd.append(c_nd.pop(i))
        v_theta.append(theta.pop(i))
        v_hst.append(hst.pop(i))
        v_act.append(act.pop(i))
        v_cst.append(cst.pop(i))

        if(flag==0 and c_nd==[]):         
            sys.exit("Path not found")

        if(goal_achieved(vc_nd[-1])==1):
            flag=1
        #print(vc_nd[-1])
        if(flag!=1):
            i=hst.index(min(hst))
            ndx=c_nd[i][:]


    seq=[v_act[-1]]
    x=vp_nd[-1]
    i=1
    while(x!=ndi):
        if(vc_nd[-i]==x):
            seq.append(v_act[-i])
            x=vp_nd[-i]
        i=i+1     

    seqn=[]
    seqn.append(vc_nd[-1])
    seqn.append(vp_nd[-1])
    x=vp_nd[-1]
    i=1
    while(x!=ndi):
        if(vc_nd[-i]==x):
            seqn.append(vp_nd[-i])
            x=vp_nd[-i]
        i=i+1
    return vc_nd,seqn


# In[8]:


dictItemPoints={"1":"Apple",
                "2":"Mango",
                "3":"Banana",
                "4":"Cherry",
                "5":"Orange",
                "6":"Grape"}


dictItemLoc={"Apple" :[3.5,3],
             "Mango" :[3.5,5],
                "Banana" :[3.5,7],
                "Cherry" :[8.5,7],
                "Orange" :[8.5,5],
                "Grape" :[8.5,3] }
print("Item directory with serial number : \n","1 - Apple \t 1st Aisle\n",
                "2 - Mango \t 1st Aisle\n",
                "3 - Banana \t 1st Aisle\n",
                "4 - Cherry \t 2nd Aisle\n",
                "5 - Orange \t 2nd Aisle\n",
                "6 - Grape \t 2nd Aisle\n")
yi=input("Enter the serial number of item you need seperated with space - ")
listy  = yi.split()
item_needed = []
location_points = []
for i in listy:
    item_needed.append(dictItemPoints[i])
for i in item_needed:
    location_points.append(dictItemLoc[i])
print("Enter initial position of the cart (not in between aisles) ")
xi=float(input("x =  "))
yi=float(input("y =  "))
Bot_pos=[xi,yi]
if (obstacle(Bot_pos)==1):
    sys.exit("Cart position lies either in between aisles or in the storage area")
aisle1=0
aisle2=0
for i in location_points:
    if(i[0]==3.5):
        aisle1 = 1
    elif(i[0]==8.5):
        aisle2 = 1
if aisle1 == 1 and aisle2 == 1:
    endpoint=[[3.5,1],[3.5,9],[8.5,9],[8.5,1]]
elif aisle1 == 1 and aisle2 == 0:
    endpoint=[[3.5,1],[3.5,9]]
elif aisle1 == 0 and aisle2 == 1:
    endpoint=[[8.5,9],[8.5,1]]

xcoor = np.linspace(-1,13,110)
ycoor = np.linspace(-1,11,100)
for x1 in xcoor:
    for y1 in ycoor:
        if obstacleNode([x1, y1]) == True:
            plt.scatter(x1, y1, color = 'k')

ndx = Bot_pos[:]
while endpoint!=[]:
    dist = math.inf
    int_node = [0,0]
    for i in endpoint:
        if dist > distance(ndx,i):
            dist=distance(ndx,i)
            int_node = i[:]
    if(int_node[1]==1):
        fin_node=[int_node[0],9]
    elif(int_node[1]==9):
        fin_node=[int_node[0],1]
    print(ndx,int_node)
    visited_points1,sequence1 = AstarOUT(ndx,int_node,0)

    #for i in visited_points1:
    #    plt.scatter(i[0], i[1],s=2.5,color = 'y')
    for i in sequence1:
        plt.scatter(i[0], i[1],s=2.5,color = 'b')
    
    print(int_node,fin_node)
    visited_points2,sequence2 = Astar(int_node,fin_node,0)

    #for i in visited_points1:
    #    plt.scatter(i[0], i[1],s=2.5,color = 'y')
    for i in sequence2:
        plt.scatter(i[0], i[1],s=2.5,color = 'b')
    
    endpoint.remove(int_node)
    endpoint.remove(fin_node)
    ndx = fin_node[:]

visited_points3,sequence3 = AstarOUT(fin_node,[11.5,0.5],0)

#for i in visited_points1:
#    plt.scatter(i[0], i[1],s=2.5,color = 'y')
for i in sequence3:
    plt.scatter(i[0], i[1],s=2.5,color = 'b')
plt.show()


# In[ ]:




