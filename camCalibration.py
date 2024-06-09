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

def AngleAxis2EulerZYX(rvec):
    
    R=cv2.Rodrigues(rvec)
    return Mat2Euler(R[0])
    
def readtxt(path):
    if os.path.isfile(path):
        out=np.ones((1, 6))
        f = open(path,'r')
        for line in f.readlines():
            s = line[1:-2].split(', ')
            tmp=[]
            for j in range(len(s)):
                tmp.append(float(s[j]))
            tmp=np.reshape(np.array(tmp),(1,6))
            out=np.append(out,tmp, axis=0)
            #print(line)
        f.close
        return out[1:]
    else:
        print("no such file!")
        return 0
def euler2H(tmp):
    x=tmp[0]
    y=tmp[1]
    z=tmp[2]
    Rx=tmp[3]
    Ry=tmp[4]
    Rz=tmp[5]
    d2r=math.pi/180
    # R_x = np.array([
    #            (1.,       0.,              0.),
    #            (0.,   math.cos(_theta[0]), -math.sin(_theta[0])),
    #            (0.,   math.sin(_theta[0]), math.cos(_theta[0]))])
           
    a=Rz*d2r;
    b=Ry*d2r;
    r=Rx*d2r;

    ca=math.cos(a);
    sa=math.sin(a);

    cb=math.cos(b);
    sb=math.sin(b);

    cr=math.cos(r);
    sr=math.sin(r);
    
    return np.array([ (ca*cb,ca*sb*sr-sa*cr,ca*sb*cr+sa*sr,x),
                      (sa*cb,sa*sb*sr+ca*cr,sa*sb*cr-ca*sr,y),
                      (-sb,cb*sr,cb*cr,z),
                      (0.,0.,0.,1)])
    
def Mat2Euler(_R):
    # sy = math.sqrt(_R.at<double>(0,0) * _R.at<double>(0,0) +  _R.at<double>(1,0) * _R.at<double>(1,0) );
    sy = math.sqrt(_R[0,0] * _R[0,0] +  _R[1,0] * _R[1,0] );
    if (sy < 1e-6): # If
        singular=True
    else:
        singular=False
    # float x, y, z;
    if (not singular):
        # x = atan2(_R.at<double>(2,1) , _R.at<double>(2,2));
        # y = atan2(-_R.at<double>(2,0), sy);
        # z = atan2(_R.at<double>(1,0), _R.at<double>(0,0));

        x = math.atan2(_R[2,1] , _R[2,2]);
        y = math.atan2(-_R[2,0], sy);
        z = math.atan2(_R[1,0], _R[0,0]);
    else:
        # x = atan2(-_R.at<double>(1,2), _R.at<double>(1,1));
        # y = atan2(-_R.at<double>(2,0), sy);
        # z = 0;
        x = math.atan2(-_R[1,2], _R[1,1]);
        y = math.atan2(-_R[2,0], sy);
        z = 0;
    
    return [x, y, z]

def Euler2Mat(_theta):

    # Rx(Roll)
    R_x = np.array([
               (1.,       0.,              0.),
               (0.,   math.cos(_theta[0]), -math.sin(_theta[0])),
               (0.,   math.sin(_theta[0]), math.cos(_theta[0]))])
           
    # Ry(Pitch)
    R_y = np.array([
               ( math.cos(_theta[1]),       0., math.sin(_theta[1])),
               (          0.,           1.,                   0.),
               (-math.sin(_theta[1]),   0.,   math.cos(_theta[1]))])
               
    """
    Mat R_y = (Mat_<double>(3,3) <<
               cos(_theta[1]),    0,      sin(_theta[1]),
            0,                 1,      0,
            -sin(_theta[1]),   0,      cos(_theta[1]));
    # Rz(Yaw)
    """
    R_z = np.array([
               ( math.cos(_theta[2]),   -math.sin(_theta[2]),    0.),
               ( math.sin(_theta[2]),    math.cos(_theta[2]),    0.),
               (          0.,                 0.,                1. )])
    """
    Mat R_z = (Mat_<double>(3,3) <<
               cos(_theta[2]),    -sin(_theta[2]),      0,
            sin(_theta[2]),    cos(_theta[2]),       0,
            0,                 0,                  1);
    # Combined rotation matrix
    Mat R = R_z * R_y * R_x
    """
    R=R_z.dot(R_y).dot(R_x)

    return R


class Charuco:
    def __init__(self,x=9,y=12,squareL=0.02,markerL=0.015,ARUCO_size=6,imgPath="./pic"):
        if ARUCO_size==4:
            self.ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_250)
        else:
            self.ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_250)
        # Create constants to be passed into OpenCV and Aruco methods
        # CHARUCO_BOARD = aruco.CharucoBoard_create(
        #         squaresX=14,
        #         squaresY=10,
        #         squareLength=0.01,
        #         markerLength=0.007,
        #         dictionary=ARUCO_DICT)
        # board = aruco.CharucoBoard_create(9, 12, 0.02, 0.015, aruco_dict)
        self.board = aruco.CharucoBoard_create(
                squaresX=x,
                squaresY=y,
                squareLength=squareL,
                markerLength=markerL,
                dictionary=self.ARUCO_DICT)
        self.squaresX=x
        self.squaresY=y
        self.squareLength=squareL
        self.markerLength=markerL
        self.ARUCO_size=ARUCO_size
        self.imgs=[]
        self.imgPath=imgPath
        self.iMtx=[]

    def readCameraConfig(self,path='data'):
        with open(path+"/CameraConfig.yaml", 'r') as f:
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
    def saveCameraConfig(self,path='data'):
        d={}
        d['x']=self.squaresX
        d['y']=self.squaresY
        d['squareL']=self.squareLength #squareL
        d['markerL']=self.markerLength #markerL
        d['ARUCO_size']=self.ARUCO_size #ARUCO_size
        #if(self.iMtx):
        d['iMtx']=self.iMtx
        d['dist']=self.dist
        with open(path+'/data/CameraConfig.yaml', 'w') as f:
            yaml.dump(d, f)
    def Calibration(self,imgPath=None,show=False):
        if(imgPath==None):
            imgPath=self.imgPath
        else:
            self.imgPath=imgPath
        ARUCO_DICT=self.ARUCO_DICT
        CHARUCO_BOARD=self.board
        # Create the arrays and variables we'll use to store info like corners and IDs from images processed
        corners_all = [] # Corners discovered in all images processed
        ids_all = [] # Aruco ids corresponding to corners discovered
        image_size = None # Determined at runtime


        # This requires a set of images or a video taken with the camera you want to calibrate
        # I'm using a set of images taken with the camera with the naming convention:
        # 'camera-pic-of-charucoboard-<NUMBER>.jpg'
        # All images used should be the same size, which if taken with the same camera shouldn't be a problem
        images = glob.glob(imgPath+'/*_color.jpg')
        # images = glob.glob('./pic3/*_color.jpg')

        # Loop through images glob'ed
        #for iname in range(len(images)):
        for iname in images:
            # Open the image
            img = cv2.imread(iname)
            #img = cv2.imread(imgPath+'/'+str(iname+1)+'_color.jpg')
            #print(imgPath+'/'+str(iname+1)+'_color.jpg')

            # Grayscale the image
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find aruco markers in the query image
            corners, ids, _ = aruco.detectMarkers(
                    image=gray,
                    dictionary=ARUCO_DICT)
                # If a Charuco board was found, let's collect image/corner points
            # Requiring at least 20 squares
            if len(corners) > 10:

            # Outline the aruco markers found in our query image
                img = aruco.drawDetectedMarkers(
                        image=img,
                        corners=corners)

                # Get charuco corners and ids from detected aruco markers
                response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                        markerCorners=corners,
                        markerIds=ids,
                        image=gray,
                        board=CHARUCO_BOARD)


                # Add these corners and ids to our calibration arrays
                corners_all.append(charuco_corners)
                ids_all.append(charuco_ids)
                #self.imgs.append(imgPath+'/'+str(iname+1)+'_color.jpg')#iname
                self.imgs.append(iname)
                # If our image size is unknown, set it now
                if not image_size:
                    image_size = gray.shape[::-1]

                """
                # Draw the Charuco board we've detected to show our calibrator the board was properly detected
                img = aruco.drawDetectedCornersCharuco(
                        image=img,
                        charucoCorners=charuco_corners,
                        charucoIds=charuco_ids)
               

            
                # Reproportion the image, maxing width or height at 1000
                proportion = max(img.shape) / 1000.0
                img = cv2.resize(img, (int(img.shape[1]/proportion), int(img.shape[0]/proportion)))
                # Pause to display each image, waiting for key press
                cv2.imshow('Charuco board', img)
                cv2.waitKey(0)
                """

            else:
                #cv2.imshow('Charuco board', img)
                #cv2.waitKey(0)
                print("Not able to detect a charuco board in image: {}".format(iname))

        # Destroy any open CV windows
        cv2.destroyAllWindows()

        # Make sure at least one image was found
        if len(self.imgs) < 1:
            # Calibration failed because there were no images, warn the user
            print("Calibration was unsuccessful. No images of charucoboards were found. Add images of charucoboards and use or alter the naming conventions used in this file.")
            # Exit for failure
            return 0

        # Make sure we were able to calibrate on at least one charucoboard by checking
        # if we ever determined the image size
        if not image_size:
            # Calibration failed because we didn't see any charucoboards of the PatternSize used
            print("Calibration was unsuccessful. We couldn't detect charucoboards in any of the images supplied. Try changing the patternSize passed into Charucoboard_create(), or try different pictures of charucoboards.")
            # Exit for failure
            return 0

        # Now that we've seen all of our images, perform the camera calibration
        # based on the set of points we've discovered
        calibration, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
                charucoCorners=corners_all,
                charucoIds=ids_all,
                board=CHARUCO_BOARD,
                imageSize=image_size,
                cameraMatrix=None,
                distCoeffs=None)

        # Print matrix and distortion coefficient to the console
        print(cameraMatrix)
        print(distCoeffs)

        # Save values to be used where matrix+dist is required, for instance for posture estimation
        # I save files in a pickle file, but you can use yaml or whatever works for you
        #f = open('calibration.pckl', 'wb')
        #pickle.dump((cameraMatrix, distCoeffs, rvecs, tvecs), f)
        #f.close()

        # Print to console our success
        #print('Calibration successful. Calibration file used: {}'.format('calibration.pckl'))
        print("error:",calibration)
        print("distortion:",distCoeffs)
        self.corners_all=corners_all
        self.ids_all=ids_all
        self.iMtx=cameraMatrix
        self.dist=distCoeffs
        self.saveCameraConfig(imgPath)

        self.getCharucoPose(show)


    def getCharucoPose(self,show=False):
        corners_all=self.corners_all
        ids_all=self.ids_all
        CHARUCO_BOARD=self.board
        images=self.imgs
        #print(images)
        cleanCamP(self.imgPath+"/data")
        for i in range(len(images)):
            #ret2,rvec2,tvec2=aruco.estimatePoseCharucoBoard(allCorners,allIds, board,mtx,dist,rvecs,tvecs)
            ret2,rvec2,tvec2=aruco.estimatePoseCharucoBoard(corners_all[i],ids_all[i], CHARUCO_BOARD,self.iMtx,self.dist,np.array([.0,.0,.0]),np.array([.0,.0,.0]))
            #print("r:",rvec2)
            #print("t:",tvec2)
            img0 = cv2.imread(images[i])
            imgNew=aruco.drawAxis(img0,self.iMtx,self.dist,rvec2,tvec2,0.1)
            # Reproportion the image, maxing width or height at 1000
            proportion = max(imgNew.shape) / 1000.0
            imgNew = cv2.resize(imgNew, (int(imgNew.shape[1]/proportion), int(imgNew.shape[0]/proportion)))
            #cv2.imshow('Charuco board Axis', imgNew)
            #cv2.waitKey(0)
            Euler = AngleAxis2EulerZYX(rvec2);
            tx = (CHARUCO_BOARD.getChessboardSize()[0]) * (CHARUCO_BOARD.getSquareLength())/2
            ty = (CHARUCO_BOARD.getChessboardSize()[1]) * (CHARUCO_BOARD.getSquareLength())/2
            # TransO = (Mat_<double>(3,1) <<tx,ty,0);
            TransO = np.array([tx,ty,0])
            R=cv2.Rodrigues(rvec2)
            Trans_tvec = R[0].dot(TransO)
            CharucoPose=[0,0,0,0,0,0]
            for j in range(3):
                tvec2[j]+=Trans_tvec[j]
            #     tvec[i]=tvec[i]+Trans_tvec.at<double>(i,0)
                CharucoPose[j]  = tvec2[j]*1000
                CharucoPose[j+3]= Euler[j]*180/math.pi
            img01 = cv2.imread(images[i])
            imgNew=aruco.drawAxis(img01,self.iMtx,self.dist,rvec2,tvec2,0.1)
            # Reproportion the image, maxing width or height at 1000
            proportion = max(imgNew.shape) / 1000.0
            imgNew = cv2.resize(imgNew, (int(imgNew.shape[1]/proportion), int(imgNew.shape[0]/proportion)))
            #print(images[i])
            #print("charucoPose:",CharucoPose)
            saveCamP(CharucoPose,self.imgPath+"/data")
            if(show==True):
                cv2.imshow(images[i], imgNew)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
        print("write charuco pose finish with ",len(images),"imgs")

    def HandEyeCalibration_EyeInHand(self,path=None):
        if(path==None):
            path=self.imgPath
        CameraData=readtxt(path+'/data/CameraPose.txt')
        RobotData=readtxt(path+'/data/Robot.txt')
        if(CameraData.shape!=RobotData.shape):
            print("robot & Cam pose dont match")
            return 0

    #     vector<Mat> R_e2b;
        R_e2b=[]   #rotate robot end point to robot base
        T_e2b=[]
        R_o2c=[]
        T_o2c=[]
    #     vector<Mat> T_e2b;
    #     vector<Mat> R_o2c;
    #     vector<Mat> T_o2c;
    #     Mat R_c2e = Mat(3,3,CV_64FC1);
        R_c2e=np.zeros((3,3))
        T_c2e=np.zeros((1,3))
    #     Mat T_c2e = Mat(3,1,CV_64FC1);

        deg2pi=math.pi/180

        for i in range(len(CameraData)):
            
            #Robot
            eulerb= [RobotData[i][3]*deg2pi,RobotData[i][4]*deg2pi,RobotData[i][5]*deg2pi]
            Rb=Euler2Mat(eulerb)
            R_e2b.append(Rb.copy())

    #         tmpTe.at<double>(0,0)=RobotData[i][0];
    #         tmpTe.at<double>(1,0)=RobotData[i][1];
    #         tmpTe.at<double>(2,0)=RobotData[i][2];
            tmpTe=np.array([RobotData[i][0],RobotData[i][1],RobotData[i][2]])

            T_e2b.append(tmpTe)

            #Camera
            eulerc= [CameraData[i][3]*deg2pi,CameraData[i][4]*deg2pi,CameraData[i][5]*deg2pi]
            Rc=Euler2Mat(eulerc)
            R_o2c.append(Rc)

    #         tmpTc.at<double>(0,0)=CameraData[i][0];
    #         tmpTc.at<double>(1,0)=CameraData[i][1];
    #         tmpTc.at<double>(2,0)=CameraData[i][2];
            tmpTc=np.array([CameraData[i][0],CameraData[i][1],CameraData[i][2]])
            T_o2c.append(tmpTc)


        R_c2e,T_c2e=cv2.calibrateHandEye(R_e2b,T_e2b,R_o2c,T_o2c,R_c2e,T_c2e,cv2.CALIB_HAND_EYE_ANDREFF);#input: Tbe,Tco;output:Tec
        print("Rotation matrix tool frame\n",R_c2e)
        print("Trans matrix tool frame\n", T_c2e)
        #Camera TF
        Xec_x=T_c2e[0][0]
        Xec_y=T_c2e[1][0]
        Xec_z=T_c2e[2][0]

        reg2deg=np.array([180/math.pi])

        W = math.sqrt((Xec_x**2)+(Xec_y**2));

        Height = float(Xec_z);
        Angle = math.atan2(Xec_y,Xec_x)*180/math.pi;
        Rec_euler=Mat2Euler(R_c2e)*reg2deg

        CameraTool=[0,0,0,0,0,0]
        CameraTool[0] = W;
        CameraTool[1] = Height;
        CameraTool[2] = Angle;
        CameraTool[3] = float(Rec_euler[0])
        CameraTool[4] = float(Rec_euler[1])
        CameraTool[5] = float(Rec_euler[2])
        #save
        d={}
        d['CameraTool_Width']=CameraTool[0]
        d['CameraTool_Height']=CameraTool[1]
        d['CameraTool_Angle']=CameraTool[2]
        d['CameraTool_Rx']=CameraTool[3]
        d['CameraTool_Ry']=CameraTool[4]
        d['CameraTool_Rz']=CameraTool[5]
        with open(path+'/data/HandEye.yaml', 'w') as f:
            yaml.dump(d, f)
        print("Camera Tool Size:\n","Width(mm): ",CameraTool[0]," Height(mm): ",CameraTool[1]," Angle(degree): ",CameraTool[2])
        print("Camera Tool Orientation:\n","Rx: ",CameraTool[3]," Ry: ",CameraTool[4]," Rz: ",CameraTool[5])
    #     cout<<"Camera Tool Size:\n"<<"Width(mm): "<<CameraTool[0]<<" Height(mm): "<<CameraTool[1]<<" Angle(degree): "<<CameraTool[2]<<endl;
    #     cout<<"Camera Tool Orientation:\n"<<"Rx: "<<CameraTool[3]<<" Ry: "<<CameraTool[4]<<" Rz: "<<CameraTool[5]<<endl;

    def getUF(self,show=False):
        corners_all=self.corners_all
        ids_all=self.ids_all
        CHARUCO_BOARD=self.board
        images=self.imgs
        #print(images)
        cleanCamP(self.imgPath+"/data")
        for i in range(len(images)):
            #ret2,rvec2,tvec2=aruco.estimatePoseCharucoBoard(allCorners,allIds, board,mtx,dist,rvecs,tvecs)
            ret2,rvec2,tvec2=aruco.estimatePoseCharucoBoard(corners_all[i],ids_all[i], CHARUCO_BOARD,self.iMtx,self.dist,np.array([.0,.0,.0]),np.array([.0,.0,.0]))
            #print("r:",rvec2)
            #print("t:",tvec2)
            img0 = cv2.imread(images[i])
            imgNew=aruco.drawAxis(img0,self.iMtx,self.dist,rvec2,tvec2,0.1)
            # Reproportion the image, maxing width or height at 1000
            proportion = max(imgNew.shape) / 1000.0
            imgNew = cv2.resize(imgNew, (int(imgNew.shape[1]/proportion), int(imgNew.shape[0]/proportion)))
            #cv2.imshow('Charuco board Axis', imgNew)
            #cv2.waitKey(0)
            Euler = AngleAxis2EulerZYX(rvec2);
            tx = (CHARUCO_BOARD.getChessboardSize()[0]) * (CHARUCO_BOARD.getSquareLength())/2
            ty = (CHARUCO_BOARD.getChessboardSize()[1]) * (CHARUCO_BOARD.getSquareLength())/2
            # TransO = (Mat_<double>(3,1) <<tx,ty,0);
            TransO = np.array([tx,ty,0])
            R=cv2.Rodrigues(rvec2)
            Trans_tvec = R[0].dot(TransO)
            CharucoPose=[0,0,0,0,0,0]
            for j in range(3):
                tvec2[j]+=Trans_tvec[j]
            #     tvec[i]=tvec[i]+Trans_tvec.at<double>(i,0)
                CharucoPose[j]  = tvec2[j]*1000
                CharucoPose[j+3]= Euler[j]*180/math.pi
            img01 = cv2.imread(images[i])
            imgNew=aruco.drawAxis(img01,self.iMtx,self.dist,rvec2,tvec2,0.1)
            # Reproportion the image, maxing width or height at 1000
            proportion = max(imgNew.shape) / 1000.0
            imgNew = cv2.resize(imgNew, (int(imgNew.shape[1]/proportion), int(imgNew.shape[0]/proportion)))
            #print(images[i])
            #print("charucoPose:",CharucoPose)
            saveCamP(CharucoPose,self.imgPath+"/data")
            if(show==True):
                cv2.imshow(images[i], imgNew)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
        print("write charuco pose finish with ",len(images),"imgs")    