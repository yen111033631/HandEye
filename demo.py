# System information:
# - Linux Mint 18.1 Cinnamon 64-bit
# - Python 2.7 with OpenCV 3.2.0

import numpy as np
import cv2
from cv2 import aruco
import pickle
import glob
import math
import yaml
from camera import *
import os
from camCalibration import *

class demo:
    def __init__(self,path="data"):
        self.readCalibration(path)
        
        
    def readCalibration(self,path):
        with open(path+"/data/CameraConfig.yaml", 'r') as f:
            data = yaml.load(f,Loader=yaml.Loader)
        if data['ARUCO_size']==4:
            self.ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_250)
        else:
            self.ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.squaresX=data['x']
        self.squaresY=data['y']
        self.squareLength=data['squareL']
        self.markerLength=data['markerL']
        self.ARUCO_size=data['ARUCO_size']
        self.iMtx=data['iMtx']
        self.dist=data['dist']
        self.board = aruco.CharucoBoard_create(
                squaresX=data['x'],
                squaresY=data['y'],
                squareLength=data['squareL'],
                markerLength=data['markerL'],
                dictionary=self.ARUCO_DICT)
        print(data)
    def getSinglePose(self,img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find aruco markers in the query image
        corners, ids, _ = aruco.detectMarkers(
                image=gray,
                dictionary=self.ARUCO_DICT)
            
        # Requiring at least 20 squares
        if len(corners) > 20:

            # Outline the aruco markers found in our query image
            img = aruco.drawDetectedMarkers(
                    image=img, 
                    corners=corners)

            # Get charuco corners and ids from detected aruco markers
            response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    markerCorners=corners,
                    markerIds=ids,
                    image=gray,
                    board=self.board)

            image_size = gray.shape[::-1]
            
            #estimate board pose
            ret3,rvec3,tvec3=aruco.estimatePoseCharucoBoard(charuco_corners,charuco_ids,self.board,self.iMtx,self.dist,np.array([.0,.0,.0]),np.array([.0,.0,.0]))            
            img0 = img
            Euler = AngleAxis2EulerZYX(rvec3);
            # board width and heigh
            tx = (self.board.getChessboardSize()[0]) * (self.board.getSquareLength())/2
            ty = (self.board.getChessboardSize()[1]) * (self.board.getSquareLength())/2
            
            #置中
            TransO = np.array([tx,ty,0.])

            R=cv2.Rodrigues(rvec3)
            # Trans_tvec = R * TransO    C++
            Trans_tvec = R[0].dot(TransO)
            CharucoPose=[0,0,0,0,0,0]
            for i in range(3):
                tvec3[i]+=Trans_tvec[i]
                CharucoPose[i]  = tvec3[i]*1000
                CharucoPose[i+3]= Euler[i]*180/math.pi
            imgNew=aruco.drawAxis(img0,self.iMtx,self.dist,rvec3,tvec3,0.1)
            cv2.imshow('Charuco board Axis', imgNew)
            cv2.waitKey(0)
            
            #x軸
            TransX = np.array([tx,0.,0.])
            #向量2旋轉矩陣
            R=cv2.Rodrigues(rvec3)
            # Trans_tvec = R * TransO    C++
            Trans_tvecX = R[0].dot(TransX)
            CharucoPoseX=[0,0,0,0,0,0]
            tvec4=np.array([0.,0.,0.])
            for i in range(3):
                tvec4[i]=tvec3[i]+Trans_tvecX[i]
                CharucoPoseX[i]  = tvec4[i]*1000
                CharucoPoseX[i+3]= CharucoPose[i+3]

            imgNew2=aruco.drawAxis(imgNew,self.iMtx,self.dist,rvec3,tvec4,0.1)
            cv2.imshow('Charuco board Axis X', imgNew2)
            cv2.waitKey(0)
            
            #Y軸
            TransY = np.array([0.,ty,0.])

            # Trans_tvec = R * TransO    C++
            Trans_tvecY = R[0].dot(TransY)

            CharucoPoseY=[0,0,0,0,0,0]
            tvec5=np.array([0.,0.,0.])
            for i in range(3):
                tvec5[i]=tvec3[i]+Trans_tvecY[i]
                CharucoPoseY[i]  = tvec5[i]*1000
                CharucoPoseY[i+3]= Euler[i]*180/math.pi
                
            imgNew3=aruco.drawAxis(imgNew2,self.iMtx,self.dist,rvec3,tvec5,0.1)

            # Reproportion the image, maxing width or height at 1000
            #proportion = max(imgNew.shape) / 1000.0
            #imgNew = cv2.resize(imgNew, (int(imgNew.shape[1]/proportion), int(imgNew.shape[0]/proportion)))
            cv2.imshow('Charuco board Axis Y', imgNew3)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            output=np.array([CharucoPose,CharucoPoseX,CharucoPoseY])
            #return 1,output
            self.UF=CharucoPose
            return 1,CharucoPose
        else:
            print("Not able to detect a charuco board in image")
            return 0,0
    
    def calUF(self,c,UF=None):
        if UF==None:
            Tco=euler2H(self.UF)
        else:
            Tco = euler2H(UF)
        # --Cal Tbe--
        changeUF(c,UF=0)
        changeTF(c,TF=0)
        pos=getRobotP(c)
        Tbe=euler2H(pos)

        # --ChangeTF(_TFidx)
        # --Get Camera TF--
        # Width, Height, Angle, Roll, Pitch, Yaw = GetTF(_TFidx)
        Tool=getTF(c,TF=2)
        
        # Tx=Width*COS(Angle)
        deg2reg=math.pi/180
        Tx=Tool[0]*math.cos(Tool[2]*deg2reg)
        # Ty=Width*SIN(Angle)
        Ty=Tool[0]*math.sin(Tool[2]*deg2reg)
        # Tec=PO2H(Tx,Ty,Height,Yaw,Pitch,Roll)
        Tec=euler2H([Tx,Ty,Tool[1],Tool[3],Tool[4],Tool[5]])
        
        # --Cal UF--
        axisX=np.array([100,0,0,1])
        axisY=np.array([0,100,0,1])
        Tbc=Tbe.dot(Tec)
        Tbo=Tbc.dot(Tco)
        UF_O=[Tbo[0][3],Tbo[1][3],Tbo[2][3]]
        # UF_X=matrix.mul(Tbo,axisX)
        UF_X=Tbo.dot(axisX)
        # UF_Y=matrix.mul(Tbo,axisY)
        UF_Y=Tbo.dot(axisY)
        
        return UF_O,UF_X,UF_Y
                