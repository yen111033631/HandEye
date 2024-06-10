import pyrealsense2 as rs
import numpy as np
import cv2
import os
import glob
from Modbus_DW import *

def takePicture(path="picture",waitFrame=60,pixel="high",align=True):
    
    # 要檢查的目錄路徑
    # 檢查目錄是否存在 
    if os.path.isdir(path):
      print("目錄存在。")
    else:
      print("目錄不存在。建一個目錄...")
      os.mkdir(path)
    images = glob.glob(path+'/*.jpg')
    nums=int(len(images)/2)
    print("內有:",nums,"對圖片")
    print("start camera:")
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    colorizer = rs.colorizer()
    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    
    if (pixel=="high"):
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    else :
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        #config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        if (pixel=="high"):
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        else :
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    try:
        i=0
        while True:
        
            i+=1
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            dpt_frame = depth_frame.as_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            #pixel_distance_in_meters = dpt_frame.get_distance(50,50)
            #print("depth:",pixel_distance_in_meters)
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            if(align==False):
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                """
                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape
                #print(color_colormap_dim)

                # If depth and color resolutions are different, resize color image to match depth image for display
                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                    images = np.hstack((resized_color_image, depth_colormap))
                else:
                    images = np.hstack((color_image, depth_colormap))

                # Show images
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                #cv2.waitKey(1)
                key = cv2.waitKey(20) & 0xFF
                if key == 27:
                    #cv2.imwrite(path+'/depth1'+".jpg", depth_colormap)
                    #cv2.imwrite(path+'/color1'+".jpg", color_image)
                    break
                """
            else:
                # Create alignment primitive with color as its target stream:
                align = rs.align(rs.stream.color)
                frameset = align.process(frames)
                
                # Update color and depth frames:
                aligned_depth_frame = frameset.get_depth_frame()
                depth_colormap = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
            if(i>waitFrame):
                cv2.imwrite(path+'/'+str(nums+1).zfill(4)+'_depth'+".jpg", depth_colormap)
                cv2.imwrite(path+'/'+str(nums+1).zfill(4)+'_color'+".jpg", color_image)
                break
    finally:

        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()

def runAndTake(c,path="pic",pos=[437.195, -50.005, 570.37, 177.132, -1.512, -2.359],pixel="high",align=True):
    teachPoint(c,pos)
    pos=getRobotP(c)
    print(pos)
    takePicture(path,pixel=pixel,align=align)
    return pos
    
def makeCamset(c,path="pic",p0=[437.195, -50.005, 570.37, 177.132, -1.512, -2.359],pixel="high",align=True):    
    #runAndTake(c,path,p0)
    path2=path+'/data'
    saveRobotP(runAndTake(c,path,p0,pixel,align),path2)
    p1 = p0[:]
    p1[0]+=10
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[0]-=10
    p1[5]+=15
    p1[1]-=20
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[5]-=30
    p1[1]+=40
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[5]+=15
    p1[1]-=10
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[2]+=10
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[3]+=10
    p1[1]-=30
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[1]-=30
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[3]-=20
    p1[1]+=80
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[1]+=40
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    
    p1 = p0[:]
    p1[5]-=15
    p1[4]+=10
    p1[0]+=80
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1 = p0[:]
    p1[4]+=10
    p1[0]+=50
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[1]+=20
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[2]+=20
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[3]+=10
    p1[1]-=50
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[1]-=30
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[3]-=20
    p1[1]+=80
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[1]+=40
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    

    p1 = p0[:]
    p1[4]-=10
    p1[0]-=50
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[1]+=20
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[2]+=20
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[3]+=10
    p1[1]-=50
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[1]-=30
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[3]-=20
    p1[1]+=80
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)
    p1[1]+=40
    saveRobotP(runAndTake(c,path,p1,pixel,align),path2)

def saveCamP(pos="[]",path="./data"):
    # 檢查目錄是否存在 
    #if not os.path.isdir(path):
        #print("目錄不存在。建一個目錄...")
        #os.mkdir(path)
    #else:
        #print("目錄存在。")
    # 開啟檔案
    fp = open(path+"/CameraPose.txt", "a")
    # 寫入 This is a testing! 到檔案
    #fp.write("This is a testing!")
    #pos=[1,2,3]
    fp.write(str(pos)+"\n")
    # 關閉檔案
    fp.close()
    
def cleanCamP(path="./data"):
    # 檢查目錄是否存在 
    if not os.path.isdir(path):
        print("目錄不存在。建一個目錄...")
        os.mkdir(path)
    else:
        print("目錄存在。")
    # 開啟檔案
    fp = open(path+"/CameraPose.txt", "w")
    # 寫入 This is a testing! 到檔案
    #fp.write("This is a testing!")
    #pos=[1,2,3]
    fp.write("")
    # 關閉檔案
    fp.close()

def CameraStream(pipeline):
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return 1,0,0

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))
        img=color_image.copy()
        # Reproportion the image, maxing width or height at 1000
        proportion = max(img.shape) / 1000.0
        img = cv2.resize(img, (int(img.shape[1]/proportion), int(img.shape[0]/proportion)))
        
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RealSense', images)
        cv2.imshow('RealSense', img)
        #cv2.waitKey(1)
        
        key = cv2.waitKey(20) & 0xFF
        if key == 27:#esc
            #cv2.imwrite('TOFpicture/output1'+".jpg", depth_colormap)
            #cv2.imwrite('TOFpicture/output2'+".jpg", color_image)
            return 0,0,0
        if key == 49:#1
            return 3,color_image,depth_colormap
        return 2,color_image,depth_colormap

def startCamera():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    print("hello")
    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    return pipeline