# -*- coding: utf-8 -*-
"""
Created on Thu Jul 30 14:28:25 2020

@author: valero
"""



import matplotlib.pyplot as plt
        
import numpy as np
import os
import subprocess
import sys
import math
from numpy import loadtxt
from Scripts import processing_yvc as yvc
import cv2
import pandas as pd
from scipy.signal import savgol_filter
from shapely.geometry import Point,Polygon
from geopandas import GeoDataFrame
import shapely.wkt
pahtSQL='Scenario0/'

dataSQL=yvc.getDataSQL(pahtSQL)
fps=29.5


##Geometry
geomet=pd.DataFrame() 
yvc.getGeometry(geomet)

geomet=pd.read_csv('geometry.csv')
poligons=[]
k=0

geomet=geomet.T
geomet = geomet[geomet[0] != 0]
geo=[yvc.Geometry(indx) for indx in geomet.index]

for objgeo in geo:
    objgeo.polygon=shapely.wkt.loads(geomet[0][objgeo.name])




    

col_names=['Vehicle_ID','Frame_ID','Total_Frames','Global_Time','Local_X','Local_Y','Global_X','Global_Y','v_Length','v_Width','v_Class','v_Vel','v_Acc','Lane_ID','Preceeding','Following','Space_Hdwy','Time_Hdwy']

df = pd.DataFrame() 
i=0

vec=[]
for obj in dataSQL:
    if i==0:
        df['Frame_ID']=[k for k in range(obj.getFirstInstant(),obj.getLastInstant()+1)]
        df['Vehicle_ID']=obj.num
        df['Local_X']=yvc.getXCoordinates(obj)
        df['Local_Y']=yvc.getYCoordinates(obj)
        df['v_Class']=obj.getUserType()
        df['v_vel']= ((df['Local_X']-df['Local_X'].shift(-1)).pow(2)+(df['Local_Y']-df['Local_Y'].shift(-1)).pow(2)).pow(0.5).divide(1/fps)
        df['v_Acc']=df['v_vel']-df['v_vel'].shift(-1)
        points=[Point(xy) for xy in zip(df['Local_X'], df['Local_Y'])]
        
        for objgeo in geo:
            objgeo.isLane=[]
            for point in points:
                objgeo.isLane.append(point.within(objgeo.polygon))
            df[objgeo.name]=objgeo.isLane
            
        
        
    else:
        temp = pd.DataFrame() 
        temp['Frame_ID']=[k for k in range(obj.getFirstInstant(),obj.getLastInstant()+1)]
        temp['Vehicle_ID']=obj.num
        temp['Local_X']=yvc.getXCoordinates(obj)
        temp['Local_Y']=yvc.getYCoordinates(obj)
        temp['v_Class']=obj.getUserType()
        temp['v_vel']= ((temp['Local_X']-temp['Local_X'].shift(-1)).pow(2)+(temp['Local_Y']-temp['Local_Y'].shift(-1)).pow(2)).pow(0.5).divide(1/fps)
        temp['v_Acc']=temp['v_vel']-temp['v_vel'].shift(-1)
        points=[Point(xy) for xy in zip(temp['Local_X'], temp['Local_Y'])]
        for objgeo in geo:
            objgeo.isLane=[]
            for point in points:
                objgeo.isLane.append(point.within(objgeo.polygon))
            temp[objgeo.name]=objgeo.isLane
        df=pd.concat([df, temp])
    i+=1
for i in range(len(geo)):
    df.loc[df['Lane '+str(i+1)] == True, 'Lane_ID'] = i+1

df.index = range(df.shape[0])
df['Following']=None
df['Preceding']=None

for temp in range(df['Frame_ID'].min(),df['Frame_ID'].max()+1):
    data_vehicle = df.loc[df['Frame_ID'] == temp]
    k=0
    for index,row in data_vehicle.iterrows():
        if k==0:
            vehicle=row.T
        dist_to_leader=math.inf
        dist_to_follower=-math.inf
        for index1,row1 in data_vehicle.iterrows():
            if index != index1:
                vehicle_f_s=row1.T
                space_headway=((row['Local_X']-row1['Local_X'])**2+(row['Local_Y']-row1['Local_Y'])**2)**0.5
                if row['Local_X']-row1['Local_X'] > 0:
                    
                    if  space_headway < dist_to_leader:
                            dist_to_leader=space_headway
                            df['Following'][index]=row1['Vehicle_ID']
                else:
                    if space_headway > dist_to_follower:
                        dist_to_follower=space_headway
                        df['Preceding'][index]=row1['Vehicle_ID']

                
            
        k+=1
df.to_csv(r'YeltsinNGSIM.csv')
