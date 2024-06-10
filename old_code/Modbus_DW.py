#!/usr/bin/env python
# coding: utf-8

# In[1]:


import pyModbusTCP


# In[2]:


from pyModbusTCP.client import ModbusClient
import time
import yaml
import os

def closeRobot(c):
    c.open()
    c.write_multiple_registers((0x1001), intL2DRA(4))
    c.close()
    print("stop DRA!")
    
    
def connectRobot(SERVER_HOST = "192.168.1.1",SERVER_PORT = 502):

    #SERVER_HOST = "169.254.194.1"
    # SERVER_HOST = "localhost"
    #SERVER_PORT = 502

    c = ModbusClient()

    # uncomment this line to see debug message
    c.debug(True)

    #c.host("192.168.1.1")
    #c.port(SERVER_PORT)
    c.unit_id(2)
    c.open()
    if c.is_open():
        # read 10 registers at address 0, store result in regs list
        print("connect success")
        reg = c.read_holding_registers(0x1000,1)
        #print(reg)
        if(reg[0]==1):
            print("DRA is runing now")
        else:
            print("DRA is closed, please run DRA")
    else:
        print("failed to connect")
    
    return c

def DRA2int(i):
    if(i>32767):
        return [i-65536]
    else:
        return i
def int2DRA(i):
    if(i<0):
        return [i+65536]
    elif(i<32767):
        return [i]
    else:
        print("out of 32767!")
        return [0]


# In[5]:


def DRA2intL(a1,b1):
    out=bin(b1)
    if(len(out)<18):
        out="0b"+out[2::].zfill(16)
    out2=bin(a1)
    out2=out2[2::]
    if(len(out2)<16):
        out2=out2.zfill(16)
    f=out+out2
    if(f[2]=='1'):
        f=int(f[3::],2)-2147483648
        return f
    f=int(f,2)
    
    return f
#DRA2intL(regs[1],regs[2])


# In[6]:


def intL2DRA(i):
    if(i<0):
        f=i+4294967296
        f=bin(f)
        b=int(f[0:18],2)
        a=int(f[18::],2)
        return [a,b]
    else:
        if(i<65536):
            b=0
            a=i
            return [a,b]
        else:
            f=bin(i)      
            a=int(f[-16::],2)
            b=int(f[0:-16],2)
            return [a,b]

def getRobotP(c):
    c.open()
    #regs = c.read_holding_registers(0x1001,2)
    #print("get p:",regs)
    c.write_multiple_registers(0x1001, int2DRA(1))
    #time.sleep(1)
    for i in range(10):
        print("read try:",i)
        time.sleep(1)
        regs = c.read_holding_registers(0x1001,2)
        try:
            if regs[1]!= None:
                break
        except:
            c.close()
            time.sleep(1)
            c.open()
            #c.write_multiple_registers(0x1001, int2DRA(1))
            #c.write_multiple_registers(0x1003, int2DRA(i))
            pass
    try:
        regs = c.read_holding_registers(0x1100,12)
        pos=[]
        x=int(len(regs)/2)
        for i in range(x):
            a=regs[(2*i)]
            b=regs[(2*i)+1]
            tmp=DRA2intL(a,b)/1000
            #print(a,b,tmp)
            pos.append(tmp)
        c.write_multiple_registers(0x1001, int2DRA(0))
        c.write_multiple_registers(0x1002, int2DRA(0))
        c.close()
    except:
        regs = c.read_holding_registers(0x1100,12)
        #print(regs)
        pos=[]
        x=int(len(regs)/2)
        for i in range(x):
            a=regs[(2*i)]
            b=regs[(2*i)+1]
            tmp=DRA2intL(a,b)/1000
            #print(a,b,tmp)
            pos.append(tmp)
        c.write_multiple_registers(0x1001, int2DRA(0))
        c.write_multiple_registers(0x1002, int2DRA(0))
        c.close()
    return pos
def saveRobotP(pos="[]",path="./data"):
    # 檢查目錄是否存在 
    if os.path.isdir(path):
        print("目錄存在。")
    else:
        print("目錄不存在。建一個目錄...")
        os.mkdir(path)
    # 開啟檔案
    fp = open(path+"/Robot.txt", "a")
    # 寫入 This is a testing! 到檔案
    #fp.write("This is a testing!")
    #pos=[1,2,3]
    fp.write(str(pos)+"\n")
    # 關閉檔案
    fp.close()
    
def float2int(pos):
    out=[]
    for i in range(len(pos)):
        out.append(int(pos[i]*1000))
    return out

def teachPoint(c,pos1,TF=0,UF=0):
#     time.sleep(2)
#     for i in range(len(pos1)):
#         pos[i]=float2int(pos1[i])
    pos=float2int(pos1)
    #print(pos)
    c.open()
    for i in range(5):
        #b=0x1200+(i*2)
        c.write_multiple_registers((0x1200+2*i), intL2DRA(pos[i]))
    c.write_multiple_registers(0x120A, intL2DRA(pos[5]))
    c.write_multiple_registers(0x120C, int2DRA(TF))#TF
    c.write_multiple_registers(0x120D, int2DRA(UF))#UF
    
    
    c.write_multiple_registers(0x1001, int2DRA(2))#tell dra to run
    #time.sleep(1)
    
    for i in range(5):
        print("wait for move",i)
        regs = c.read_holding_registers(0x1001,2)
        time.sleep(1)
        try:
            if regs[1]==2:
                break
        except:
            c.write_multiple_registers(0x1001, int2DRA(2))
            pass
    c.write_multiple_registers(0x1001, int2DRA(0))
    c.write_multiple_registers(0x1002, int2DRA(0))
    time.sleep(0.1)
    c.close


def writeTool(c,tool):
#     time.sleep(2)
#     for i in range(len(pos1)):
#         pos[i]=float2int(pos1[i])
    pos=float2int(tool)
    #print(pos)
    c.open()
    for i in range(5):
        #b=0x1200+(i*2)
        c.write_multiple_registers((0x1300+2*i), intL2DRA(pos[i]))
    c.write_multiple_registers(0x130A, intL2DRA(pos[5]))
    c.write_multiple_registers(0x1001, int2DRA(3))
    time.sleep(1)
    
    c.write_multiple_registers(0x1001, int2DRA(0))
    c.close
    
def readTool(path):
    with open(path+"/data/HandEye.yaml", 'r') as f:
        data = yaml.load(f, Loader=yaml.Loader)
    #print(data)
    CameraTool=[0,0,0,0,0,0]
    CameraTool[0]=data['CameraTool_Width']
    CameraTool[1]=data['CameraTool_Height']
    CameraTool[2]=data['CameraTool_Angle']
    CameraTool[3]=data['CameraTool_Rx']
    CameraTool[4]=data['CameraTool_Ry']
    CameraTool[5]=data['CameraTool_Rz']
    return CameraTool
    
def RWTool(c,path):
    tool=readTool(path)
    print(tool)
    writeTool(c,tool)
    
def changeTF(c,TF=0):

    #print(pos)
    c.open()
    c.write_multiple_registers(0x1310, int2DRA(TF))#TF
    
    c.write_multiple_registers(0x1001, int2DRA(6))#tell dra to run
    time.sleep(0.5)
    
    c.write_multiple_registers(0x1001, int2DRA(0))
    c.close
    
def changeUF(c,UF=0):

    #print(pos)
    c.open()
    c.write_multiple_registers(0x1410, int2DRA(UF))#UF
    
    c.write_multiple_registers(0x1001, int2DRA(7))#tell dra to run
    time.sleep(0.5)
    
    c.write_multiple_registers(0x1001, int2DRA(0))
    c.close 

    
def setUF(c,o,x,y,UF=8):
    TF=0
    """
    if len(o) != 6 or len(y) != 6 or len(y) != 6:
        print("o,x,y point error!")
        return 0
    c.open()
    #o
    pos=float2int(o)
    for i in range(5):
        c.write_multiple_registers((0x1210+2*i), intL2DRA(pos[i]))
    c.write_multiple_registers(0x121A, intL2DRA(pos[5]))
    c.write_multiple_registers(0x121C, int2DRA(TF))#TF
    c.write_multiple_registers(0x121D, int2DRA(0))#UF
    #x
    pos=float2int(x)
    for i in range(5):
        c.write_multiple_registers((0x1220+2*i), intL2DRA(pos[i]))
    c.write_multiple_registers(0x122A, intL2DRA(pos[5]))
    c.write_multiple_registers(0x122C, int2DRA(TF))#TF
    c.write_multiple_registers(0x122D, int2DRA(0))#UF
    #y
    pos=float2int(y)
    for i in range(5):
        c.write_multiple_registers((0x1230+2*i), intL2DRA(pos[i]))
    c.write_multiple_registers(0x123A, intL2DRA(pos[5]))
    c.write_multiple_registers(0x123C, int2DRA(TF))#TF
    c.write_multiple_registers(0x123D, int2DRA(0))#UF
    
    
    c.write_multiple_registers(0x1001, int2DRA(5))#tell dra to run
    time.sleep(1)
    
    c.write_multiple_registers(0x1001, int2DRA(0))
    c.close   
    """
    c.open()
    #o
    pos=float2int(o)
    for i in range(3):
        c.write_multiple_registers((0x1210+2*i), intL2DRA(pos[i]))
    c.write_multiple_registers(0x121C, int2DRA(TF))#TF
    c.write_multiple_registers(0x121D, int2DRA(0))#UF

    #x
    pos=float2int(x)
    for i in range(3):
        c.write_multiple_registers((0x1220+2*i), intL2DRA(pos[i]))
    c.write_multiple_registers(0x122C, int2DRA(TF))#TF
    c.write_multiple_registers(0x122D, int2DRA(0))#UF
    #y
    pos=float2int(y)
    for i in range(3):
        c.write_multiple_registers((0x1230+2*i), intL2DRA(pos[i]))
    c.write_multiple_registers(0x123C, int2DRA(TF))#TF
    c.write_multiple_registers(0x123D, int2DRA(0))#UF

    c.write_multiple_registers(0x121F, int2DRA(UF))#UF
    c.write_multiple_registers(0x1001, int2DRA(5))#tell dra to run
    time.sleep(1)

    c.write_multiple_registers(0x1001, int2DRA(0))
    c.close 
    
def getTF(c,TF=7):
    c.open()    
    c.write_multiple_registers(0x1311, int2DRA(TF))
    c.write_multiple_registers(0x1001, int2DRA(8))
    time.sleep(0.5)
    regs = c.read_holding_registers(0x1300,12)
    
    #print(regs)
    pos=[]
    x=int(len(regs)/2)
    for i in range(x):
        a=regs[(2*i)]
        b=regs[(2*i)+1]
        tmp=DRA2intL(a,b)/1000
        #print(a,b,tmp)
        pos.append(tmp)
    c.write_multiple_registers(0x1001, int2DRA(0))
    c.close
    return pos
    
def graspDemo(c):
    c.open()
    c.write_multiple_registers(0x1001, int2DRA(9))
    time.sleep(1)
    c.write_multiple_registers(0x1001, int2DRA(0))
    c.close   