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
import keyboard
import threading

def set_image_dir(image_project_dir, csv_name):
    env_listdir = os.listdir(image_project_dir)

    num = 0
    for env_dir in env_listdir:
        if csv_name in env_dir:
            now_num = int(env_dir[-3:]) 
            num = now_num if now_num > num else num
    
    image_dir = f"{image_project_dir}/{csv_name}_{(num + 1):03d}"
    if is_save: os.makedirs(image_dir, exist_ok=True)

    return image_dir


def match_template(image, template):
    image_draw = image.copy()
    
    if len(image.shape) == 3 and image.shape[2] == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
    if len(template.shape) == 3 and template.shape[2] == 3:
        template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    
    
    # 获取模板的尺寸
    w, h = template.shape[::-1]

    # 进行模板匹配
    res = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)

    # 找到匹配数值最高的位置
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

    # 打印出最高匹配值的位置
    # print("最高匹配值位置:", max_loc)
    # print(res[max_loc[1], max_loc[0]])
    max_value = res[max_loc[1], max_loc[0]]

    # 在原图像中绘制矩形框标记匹配位置
    top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)
    
    # 检查图像是否是彩色图像
    if len(image_draw.shape) < 3 or image_draw.shape[2] == 1:
        # print("图像是灰度图，将其转换为彩色图像。")
        image_draw = cv2.cvtColor(image_draw, cv2.COLOR_GRAY2BGR)
    # else:
    #     print("图像是彩色图像。")
    
    
    cv2.rectangle(image_draw, top_left, bottom_right, (0, 255, 0), 2)
    
    return image_draw, max_value


def write_text_on_image(image, max_value, org=(50, 50)):    

    # 定义多行文字
    lines_of_text = [
        f"template matching: {str(max_value)}",
    ]
    # 定义初始位置（左下角）
    org = org
    
    # 定义字体
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    # 定义字体大小
    font_scale = 1
    
    # 定义背景颜色（BGR格式）
    bg_color = (0, 0, 0)  # 黑色
    
    # 定义字体厚度
    thickness = 2
    
    # 定义行间距
    line_spacing = 15
    
    text_color = (255, 255, 255)

    # 在图片上添加多行带背景色的文字
    for i, line in enumerate(lines_of_text):
        # 计算文字大小
        (text_width, text_height), baseline = cv2.getTextSize(line, font, font_scale, thickness)
        
        # 计算每行文字的位置
        x, y = org[0], org[1] + i * (text_height + line_spacing)
        
        # 绘制背景矩形
        cv2.rectangle(image, (x, y - text_height - baseline), (x + text_width, y + baseline), bg_color, -1)

        cv2.putText(image, line, (x, y), font, font_scale, text_color, thickness, cv2.LINE_AA)

    
    return image

class Cam:
    def __init__(self, image_dir="./images", prename_folder=""):
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        current_time = datetime.now().strftime("%b%d_H%H_M%M_S%S")
        self.image_dir = image_dir
        # self.image_dir = f"{image_dir}/{prename_folder}_{current_time}"
        # if not os.path.exists(self.image_dir):
        #     # 如果資料夾不存在，則創建它
        #     os.makedirs(self.image_dir)
        
        self.i = 0
    
    def get_frame(self, image_type=["color"]):

        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        self.depth_image = cv2.flip(depth_image, -1)
        color_image = np.asanyarray(color_frame.get_data())
        self.color_image = cv2.flip(color_image, -1)     
        
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
        self.images = np.hstack((self.color_image, depth_colormap))          

        return self.color_image
    
    def show_image(self):
        while True:
            color_image = self.get_frame()
            
            template = cv2.imread(r"C:\Users\NEAF\code\yen\HandEye\output\tem\tem.bmp", 0)
            template = template[100:400, 950:1250]
            
            color_image, max_value = match_template(color_image, template)
            
            color_image = write_text_on_image(color_image, round(max_value, 3))
            
            
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('y') or key == 27:
                cv2.destroyAllWindows()
                break
    
    def save_image(self, image_type=["color"], image_name=None):
        if "color" in image_type:
            cv2.imwrite(f"{self.image_dir}/{image_name}.bmp", self.color_image)
        if "depth" in image_type:
            cv2.imwrite(f"{self.image_dir}/{image_name}_depth.bmp", self.depth_image)
        if "hstack" in image_type:
            cv2.imwrite(f"{self.image_dir}/{image_name}_hstack.bmp", self.images)        
            

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
                cv2.imwrite(f"{self.image_dir}/{image_name}.bmp",color_image)
            if "depth" in image_type:
                cv2.imwrite(f"{self.image_dir}/{image_name}_depth.bmp",depth_image)
            if "hstack" in image_type:
                cv2.imwrite(f"{self.image_dir}/{image_name}_hstack.bmp",images)        

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
    # some setting
    is_save = False
    # ----------------------------------------------------
    # read position csv 
    csv_dir = r"\\140.114.141.95\nas\111\111033631_Yen\ARM\capture_images_sim\Jul16_H14_M43_S14_010_0100_882_882\position.csv"
    # csv_dir = r"\\140.114.141.95\nas\111\111033631_Yen\ARM\capture_images_sim\cube_points__.csv"
    csv_name = os.path.basename(os.path.dirname(csv_dir))
    df = read_csv(csv_dir)
    # df = pd.read_csv(csv_dir)
    # print(df.shape)
    df = df[703:]
    # ----------------------------------------------------
    # set images dir
    image_project_dir = r"\\140.114.141.95\nas\111\111033631_Yen\ARM\capture_images_real"
    image_dir = set_image_dir(image_project_dir, csv_name)
    print(image_dir)
    
    # ----------------------------------------------------
    # init connect arm and cam
    c = connect_robot()
    # if is_save: my_cam = Cam(r"\\140.114.141.95\nas\111\111033631_Yen\ARM\capture_images_real", csv_name)
    if is_save: my_cam = Cam(image_dir)
    address = 0x1100
    
    # ----------------------------------------------------
    # write where.csv
    if is_save:
        with open(f'{my_cam.image_dir}/where_csv.txt', 'w') as file:
            file.write(csv_dir)
    # ----------------------------------------------------
    previous_cube_position_index = None
    previous_cube_position = None
    # main loop/
    for i in tqdm(range(df.shape[0])):
    # for i in (range(df.shape[0])):
        # ------------------------------------------------
        # reset memory
        num = write_into_regs([[0]*10], address)
        # ------------------------------------------------
        # get row name 
        df_row_name = df.iloc[i].name
        # ------------------------------------------------
        # cube information
        cube_position_index = df["ball_position_index"][i]
        cube_position = df.iloc[i][-4: -1]
        cube_position = [x * 1000 for x in cube_position]

        is_change_cube_position = not(np.all(cube_position == previous_cube_position))
        print()
        print(is_change_cube_position, cube_position)
        print("----")

        if is_change_cube_position:
            write_into_regs([[3]], address)
            time.sleep(0.1)
            previous_cube_position_index = cube_position_index
            previous_cube_position = cube_position
            p = [*cube_position[:2], 100]
            j = [0] * 6
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
            # 這是一個無限循環，用於不斷檢查鍵盤輸入
            my_cam.show_image()
            write_into_regs([[3]], address)
            time.sleep(0.1)

        # ------------------------------------------------
        # get arm position and joints
        p, j = get_p_and_j(df, i)
        # p = df.iloc[i][:3]
        # j = [0] * 6
        # p[-1] = 200
        print(i, p)
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
        my_cam.get_frame()
        
        if is_save: 
            thread = threading.Thread(target=my_cam.save_image, args=(image_type:=["color", "depth", "hstack"], image_name:=df_row_name))
            thread.start()
            # my_cam.save_image(image_type=["color", "depth", "hstack"], image_name=df_row_name)
        # if is_save: my_cam.capture_pic(image_type=["color", "depth", "hstack"], image_name=df_r/ow_name) # TODO only capture color
        
        # write_into_regs([[3]], address)
        # time.sleep(0.1)
        # ------------------------------------------------

    # ----------------------------------------------------
    # write end command into memory 
    write_into_regs([[-1] * 10], address)
    # ----------------------------------------------------
    # close cam
    time.sleep(5)
    if is_save: my_cam.close_cam()
    # ----------------------------------------------------
        
        