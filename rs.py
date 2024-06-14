import pandas as pd
from pyModbusTCP.client import ModbusClient
import pyrealsense2 as rs
import numpy as np
import cv2
from datetime import datetime
import os
import time
import statistics
from tqdm import tqdm

class Cam:
    def __init__(self, image_dir="./images"):
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        current_time = datetime.now().strftime("%b%d_H%H_M%M_S%S")
        self.image_dir = f"{image_dir}/{current_time}"
        if not os.path.exists(self.image_dir):
            # 如果資料夾不存在，則創建它
            os.makedirs(self.image_dir)
        
        self.i = 0
        
    def capture_pic(self, image_type=["color"], image_name=None):
        if image_name == None: image_name = f"img_{self.i}"

        try:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            depth_image = np.asanyarray(depth_frame.get_data())
            depth_image = cv2.flip(depth_image, -1)
            color_image = np.asanyarray(color_frame.get_data())
            color_image = cv2.flip(color_image, -1)

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            images = np.hstack((color_image, depth_colormap))

            if "color" in image_type:
                cv2.imwrite(f"{self.image_dir}/{image_name}.jpg",color_image)
            if "depth" in image_type:
                cv2.imwrite(f"{self.image_dir}/{image_name}_depth.jpg",depth_image)
            if "hstack" in image_type:
                cv2.imwrite(f"{self.image_dir}/{image_name}_hstack.jpg",images)        

            self.i += 1
        finally:
            pass
    def close_cam(self):
        self.pipeline.stop()



def connect_robot(ip="192.168.1.1", port=502):
    # 初始化 Modbus 客戶端
    c = ModbusClient(host=ip, port=port, auto_open=True, unit_id=2)
    
    c.open()
    if c.is_open:
        # read 10 registers at address 0, store result in regs list
        print("connect success")
        # reg = c.read_holding_registers(0x1000,2)    
        # print(reg)
        # if(reg[0]==1):
        #     print("DRA is runing now")
        # else:
        #     print("DRA is closed, please run DRA")
    else:
        print("failed to connect")
    
    return c


def read_csv(csv_file_path = 'position.csv'):
    # 使用 pandas 讀取 CSV 檔案    
    return pd.read_csv(csv_file_path, index_col=0)



def get_p_and_j(df, i):
    return df.iloc[i][:3], df.iloc[i][3:9]

# ---------------------------------------------
# write_into_regs
def write_into_regs(x, address=0x1100):
    flattened_list = turn_into_one_list(x)
    i = 0
    while True:
        success = c.write_multiple_registers(address, flattened_list)
        i += 1
        if success:
            break
    return i

def turn_into_one_list(x):
    all_data_DRA = []

    # turn into DRA 
    for data in x:
        for element in data:
            all_data_DRA.append(intL2DRA(element * 10**6))

    # turn into numpy 
    arr_DRA = np.asanyarray(all_data_DRA)
    # reshape into 1-D 
    total = 1
    for size in arr_DRA.shape:
        total *= size
    arr_DRA = np.reshape(arr_DRA, (total))
    # print(total / 2)

    return arr_DRA
# ---------------------------------------------
# read_regs
def read_regs(amount, address=0x1100):
    while True:
        regs = c.read_holding_registers(address, amount*2)
        if regs != None:
            break
    data = []
    for i in range(0, len(regs), 2):
        num = DRA2intL([regs[i], regs[i+1]])
        data.append(num * 10**-6)
    
    return data
# ---------------------------------------------
# transfer
def DRA2intL(n):
    a, b = int(n[0]), int(n[1])
    # print(a, b)
    t = (b << 16) + a
    return t if t < (2**31) else (t - 2**32)

def intL2DRA(i):
    if(i<0):
        return intL2DRA( i + (2**32))
    else:
        return [int(i % (2**16)), int(i // (2**16))] # a, b = i % (2**16), i // (2**16) #(i >> 16)
# --------------------------------------------------------
if __name__ == "__main__":
    # ----------------------------------------------------
    # init connect arm and cam
    c = connect_robot()
    my_cam = Cam(r"\\140.114.141.95\nas\111\111033631_Yen\ARM\capture_images_real")
    address = 0x1100
    
    # ----------------------------------------------------
    # read position csv 
    csv_dir = r"\\140.114.141.95\nas\111\111033631_Yen\ARM\capture_images_sim\arm_cube_shuffle_466\position.csv"
    df = read_csv(csv_dir)

    with open(f'{my_cam.image_dir}/where_csv.txt', 'w') as file:
        file.write(csv_dir)
    # ----------------------------------------------------
    # main loop
    for i in tqdm(range(df.shape[0])):
        # ------------------------------------------------
        # reset memory
        num = write_into_regs([[0]*10], address)
        # ------------------------------------------------
        # get row name 
        df_row_name = df.iloc[i].name
        # ------------------------------------------------
        # get arm position and joints
        p, j = get_p_and_j(df, i)
        # ------------------------------------------------
        # send position and joint data (write regs)
        num = write_into_regs([[1], j, p], address)
        # ------------------------------------------------
        # check arm move done (read regs)
        while True:
            data = read_regs(1)
            if data[0] == 2:
                break
        # ------------------------------------------------
        # take photo 
        my_cam.capture_pic(image_type="color", image_name=df_row_name) # TODO only capture color
        # ------------------------------------------------

    # ----------------------------------------------------
    # write end command into memory 
    write_into_regs([[-1] * 10], address)
    # ----------------------------------------------------
    # close cam
    my_cam.close_cam()
    # ----------------------------------------------------
        
        