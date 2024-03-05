# -*- coding: utf-8 -*-
"""
Created on Thu Mar 28 12:07:54 2019

@author: NEMEC
"""

from math import pi, sin, cos, atan2, asin, copysign, degrees, radians
from enum import IntEnum
from typing import List
import socket
import threading
import struct
from time import sleep

class Position:
    """ 
    Cartesian position (x, y, z) in [mm] 
    """
    
    x:float
    y:float
    z:float

    def __init__(self, x:float=0, y:float=0, z:float=0) : 
        self.x = x
        self.y = y
        self.z = z
        
    @staticmethod
    def fromList(raw:list):
        """ Creates position from list of variables. """
        return Position(raw[0], raw[1], raw[2])
        
    def toList(self):
        """ Converts to list """
        return [self.x, self.y, self.z]


    # TODO casting to some nice format


class EulerAngles:
    """ 
    Z-Y-X Euler angles (roll, pitch, yaw) in radians 
    """

    roll:float
    pitch:float
    yaw:float

    def __init__(self, roll:float=0, pitch:float=0, yaw:float=0) :
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def toQuaternion(self):
        """ Converts Euler angles to quaternion """
        
        # code taken from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        cy = cos(self.yaw * 0.5)
        sy = sin(self.yaw * 0.5)
        cp = cos(self.pitch * 0.5)
        sp = sin(self.pitch * 0.5)
        cr = cos(self.roll * 0.5)
        sr = sin(self.roll * 0.5)
        
        w = (cy * cp * cr + sy * sp * sr)
        x = (cy * cp * sr - sy * sp * cr)
        y = (sy * cp * sr + cy * sp * cr)
        z = (sy * cp * cr - cy * sp * sr)
        
        return Quaternion(w, x, y, z)
        
    @staticmethod
    def fromList(raw:List[float]):
        """ Creates Euler angles from three values in array """
        return EulerAngles(list[0], list[1], list[2])

    def toList(self):
        """ Converts to list """
        return [self.roll, self.pitch, self.yaw]

    def __str__(self):
        return "{:.2f}, {:.2f}, {:.2f} deg".format(degrees(self.roll), degrees(self.pitch), degrees(self.yaw))

class Quaternion:
    """ 
    Rotational quaternion (w, x, y, z) or (q1, q2, q3, q4) in ABB RobotStudio 
    """
    
    w : float
    x : float
    y : float
    z : float

    def __init__(self, w:float=1, x:float=0, y:float=0, z:float=0) :
        self.w = w
        self.x = x
        self.y = y
        self.z = z
        
    def toEulerAngles(self):
        """ Creates Euler angles from quaternion """

        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        roll = atan2(sinr_cosp, cosr_cosp)

		# pitch (y-axis rotation)
        sinp = 2.0 * (self.w * self.y - self.z * self.x)
        if abs(sinp) >= 1:
            pitch = copysign(pi / 2, sinp) # use 90 degrees if out of range
        else:
            pitch = asin(sinp)

		# yaw (z-axis rotation)
        siny_cosp = 2.0 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        yaw = atan2(siny_cosp, cosy_cosp)

        return EulerAngles(roll, pitch, yaw)
    
    @staticmethod
    def fromList(raw:List[float]):
        """ Create quaternion from list of values """
        return Quaternion(raw[0], raw[1], raw[2], raw[3])
    
    def toList(self):
        """ converts to list """
        return [self.w, self.x, self.y, self.z]

    def __str__(self):
        return "{:.3f}, {:.3f}, {:.3f}, {:.3f}".format(self.w, self.x, self.y, self.z)


class Conf:
    """ 
    Represents robot configuration (quadrants of joints) (cf1, cf4, cf6, cfx)  
    """

    cf1:float
    cf4:float
    cf6:float
    cfx:float
    
    def __init__(self, cf1:float=0, cf4:float=0, cf6:float=0, cfx:float=0):
        self.cf1 = cf1
        self.cf4 = cf4
        self.cf6 = cf6
        self.cfx = cfx
    
    @staticmethod
    def fromList(raw:List[float]):
        """ creates coniguration from raw data """
        return Conf(raw[0], raw[1], raw[2], raw[3])
        
    def toList(self):
        """ converts to list """
        return [self.cf1, self.cf4, self.cf6, self.cfx]


class Speed:
    """ 
    Represents speed settings of the robot 
    
    Attributes
    ----------
        v_tcp : float
            tool linear speed [mm/s]
        v_ori : float
            joint rotation speed [deg/s]
        v_leax : float
            linear external axis speed [mm/s]
        v_reax : float
            rotating external axis speed [deg/s]
    """
    
    v_tcp:float
    v_ori:float
    v_leax:float
    v_reax:float
    
    def __init__(self, tcp:float=100, ori:float=500, leax:float=5000, reax:float=1000):
        """ Default speed is v100 """
        self.v_tcp = tcp
        self.v_ori = ori
        self.v_leax = leax
        self.v_reax = reax
        
    def __float__(self):
        """ returns TCP speed """
        return self.v_tcp
    
    def toList(self):
        """ converts to list """
        return [self.v_tcp, self.v_ori, self.v_leax, self.v_reax]


class Zone:
    """ 
    Represents zone settings 
        
    Attributes
    ----------
        finep : bool
            true when it is a fine point, false when fly-by point
        pzone_tcp : float
            path zone of the TCP [mm]
        pzone_ori : float
            path zone for orientation [mm]
        pzone_eax : float
            path zone for external axes [mm]
        zone_ori : float
            zone orientation [deg]
        zone_leax : float
            zone for linear external axes [mm]
        zone_reax : float
            zone for rotating external axes [deg]
    """
    
    finep:bool
    pzone_tcp:float
    pzone_ori:float
    pzone_eax:float
    zone_ori:float
    zone_leax:float
    zone_reax:float
    
    def __init__(self, z:float=100):
        """ Simplified constructor. Default z100. Give negative zone for "fine" zone. """
        if z < 0 :
            # fine
            self.finep = True
            self.pzone_tcp = 0
            self.pzone_ori = 0
            self.pzone_eax = 0
            self.zone_ori = 0
            self.zone_leax = 0
            self.zone_reax = 0    
        elif z < 1 :
            # z0 is special
            self.finep = False
            self.pzone_tcp = 0
            self.pzone_ori = 0.3
            self.pzone_eax = 0.3
            self.zone_ori = 0.03
            self.zone_leax = 0.3
            self.zone_reax = 0.03   
        else:
            # simmilar to zX in robotstudio, but not precisely the same
            self.finep = False
            self.pzone_tcp = z
            self.pzone_ori = 1.5 * z
            self.pzone_eax = 1.5 * z
            self.zone_ori = 0.15 * z
            self.zone_leax = 1.5 * z
            self.zone_reax = 0.15 * z
            
    def __float__(self):
        """ returns zX value """
        if self.finep:
            return -1
        else:
            return self.pzone_tcp
        
    @staticmethod
    def fine():
        """ returns fine zone """
        return Zone(-1.0)
    
    def toList(self):
        """ converts to list """
        return [1.0 if self.finep else 0.0, self.pzone_tcp, self.pzone_ori, self.pzone_eax, self.zone_ori, self.zone_leax, self.zone_reax]


class LinearTarget:
    """ 
    Target for linear movement 

    Attributes
    ----------
        trans : Position
            Translation of the target w.r.t. workobject
        rot : Quaternion
            Rotation of the target w.r.t. workobject
        robconf : Conf
            Configuration of the robot
        extax : List[float]
            Position of the external axes [deg or mm]
        speed : Speed
            Speed settings 
        zone:Zone
            Zone settings 
    """

    trans:Position
    rot:Quaternion
    robconf:Conf
    extax:List[float]
    speed:Speed
    zone:Zone
   
    def __init__(self):
        self.trans = Position()
        self.rot = Quaternion()
        self.robconf = Conf()
        self.extax = [0, 0, 0, 0, 0, 0]
        self.speed = Speed()
        self.zone = Zone()
        
        
class JointTarget:
    """ 
    Target for joint movement 
    
    Attributes
    ----------
        joints : List[float]
            Positions of all axes (6x internal, 6x external) [deg or mm]
        speed: Speed
            Speed settings
        zone : Zone
            Zone settings
    """
    
    joints:List[float]
    speed:Speed
    zone:Zone
    
    def __init__(self):
        self.joints = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.speed = Speed()
        self.zone = Zone()

        
class Load:
    """ 
    Payload data 
        
    Attributes
    ----------
        mass : float 
            weight of the load [kg] 
        cog : Position
            position of the Centrum of Gravity w.r.t. tool coordinate system
        aom : Quaternion
            axes of moment
        ix : float
            moment of inertia around x-axis [kg.m2]
        iy : float
            moment of inertia around y-axis [kg.m2]
        iz : float
            moment of inertia around z-axis [kg.m2]
    """

    def __init__(self):
        self.mass = 0.001
        self.cog = Position(0, 0, 0)
        self.aom = Quaternion(1, 0, 0, 0)
        self.ix = 0
        self.iy = 0
        self.iz = 0

    @staticmethod
    def fromList(raw:List[float]):
        """ creates load data from array of 11 floats """
        ld = Load()
        ld.mass = raw[0]
        ld.cog = Position.fromList(raw[1:4])
        ld.aom = Quaternion.fromList(raw[4:8])
        ld.ix = raw[8]
        ld.iy = raw[9]
        ld.iz = raw[10]
        return ld

    def toList(self):
        """ converts load data to array """
        return [self.mass] + self.cog.toList() + self.aom.toList() + [self.ix, self.iy, self.iz]

class Tool:
    """ 
    Tool data

    Attributes
    ----------
        robhold : bool
            True when robot holds the tool
        trans : Position
            Position of the TCP w.r.t. tool0
        rot : Quaternion
            rotation of TCP w.r.t. tool0
        load : Load
            load of the tool itself
    """
    
    robhold:bool
    trans:Position
    rot:Quaternion
    load:Load
    
    def __init__(self):
        self.robhold = True
        self.trans = Position()
        self.rot = Quaternion()
        self.load = Load()
     
    def toList(self):
        """ coverts tool data to list """
        return [1.0 if self.robhold else 0.0] + self.trans.toList() + self.rot.toList() + self.load.toList()
    
    @staticmethod
    def fromList(raw:List[float]):
        """ creates tool data from raw array """
        tool = Tool()
        tool.robhold = raw[0] > 0
        tool.trans = Position.fromList(raw[1:4])
        tool.rot = Quaternion.fromList(raw[4:8])
        tool.load = Load.fromList(raw[8:19])

class Workobject:
    """ 
    Workobject data
    
    Attributes
    ----------
        robhold : bool
            True when robot holds the workobject and the tool is static.
        trans : Position
            position of the object frame w.r.t. global coordinates 
        rot : Quaternion
            rotation of the object frame w.r.t. global coordinates

    """
    
    robhold:bool
    trans:Position
    rot:Quaternion

    def __init__(self):
        self.robhold = False
        self.trans = Position()
        self.rot = Quaternion()
        
    def toList(self):
        """ converts workobject to list """
        return [1.0 if self.robhold else 0.0] + self.trans.toList() + self.rot.toList()


class GripStatusCode(IntEnum):
    """ status code of the gripper """
    Ready = 0x00
    Error = 0x01
    FreeMoveInward = 0x02
    FreeMoveOutward = 0x03
    MoveInward = 0x04
    MoveOutward = 0x05
    ActionCompleted = 0x06
    ForcingInward = 0x07
    ForcingOutward = 0x08
    KeepsObject = 0x09
    Calibrating = 0x0A
    JogOpen = 0x0B
    JogClose = 0x0C
    ChangingChirality = 0x0F
    AgileInward = 0x10
    AgileOutward = 0x11


class GripDirection(IntEnum):
    """ direction of the gripping """
    Outward = 0x00
    Inward = 0x01
    
class MoveStatus(IntEnum):
    """ status of the movement """
    Normal = 0x00
    Paused = 0x01
    Collision = 0x02
 
class RequestCode(IntEnum):
    """ internal: do not use """
    CMD_Ping = 0x00
    CMD_MoveL = 0x01
    CMD_MoveAbsJ = 0x02
    CMD_StopNow = 0x03
    CMD_Wait = 0x04
    CMD_ReadState = 0x05
    CMD_Pause = 0x06
    CMD_Resume = 0x07
    CMD_SetWobj = 0x08
    CMD_SetTool = 0x09
    CMD_GripInit = 0x0A
    CMD_GripMoveTo = 0x0B
    CMD_GripSmart = 0x0C
    CMD_GripStopNow = 0x0D
    CMD_GripBlow = 0x0E
    CMD_GripVacuum = 0x0F
    CMD_GripRead = 0x10
    CMD_MoveJ = 0x11


class ResponseCode(IntEnum):
    """ internal: do not use"""
    RES_OK = 0x00
    RES_State = 0x01
    RES_Overflow = 0x02
    RES_InvalidRequest = 0x03
    RES_NotReachable = 0x04
    RES_GripState = 0x05

class Status:
    """ 
    Status of the robotic arm 

    Attributes
    ----------
        joints : List[float]
            Postion of all axes (internal and external) [mm or deg]
        trans : Position
            Postion of the TCP (Tool Center Point) w.r.t. workobject
        rot : Quaternion
            Rotation of the TCP w.r.t. workobject
        robconf : Conf
            Configuration of the robot axes
        moveState : MoveStatus
            status of the movement
    """
    
    joints:List[float]
    trans:Position
    rot:Quaternion
    robconf:Conf
    moveState:MoveStatus
    pending:int
  
    def __init__(self):
        self.joints = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.trans = Position()
        self.rot = Quaternion()
        self.robconf = Conf()
        self.moveState = MoveStatus.Normal
        self.pending = 0

class GripStatus:
    """
    Status of the Smart gripper
    
    Attributes
    ----------
        code : GripStatusCode
            Status code of the gripper
        pos : float
            position of the gripper in mm
        pressure1 : float
            pressure on vacuum channel 1
        pressure2 : float
            pressure on vacuum channel 2
    """
    
    code:GripStatusCode
    pos:float
    pressure1:float
    pressure2:float
    
    def __init__(self):
        self.code = GripStatusCode.Ready
        self.pos = 0.0
        self.pressure1 = 0.0
        self.pressure2 = 0.0
        

class Arm:
    """ Represents one robotic arm optionally with Smart Gripper """

    ep:socket.socket
    lock:threading.Semaphore
    lastRequestID:int
    lastState:Status
    lastGripState:GripStatus
    host:str
    port:int
    
    START_BYTE = 0x23
    CACHE_SIZE = 256
    HEADER_SIZE = 6

    def __init__(self, host:str, port:int):
        """ Creates new Arm interface but does not connects yet """
        self.host = host
        self.port = port
        self.lock = threading.Semaphore()
        self.lastRequestID = 0
        self.lastState = Status()
        self.lastGripState = GripStatus()
        self.ep = None

    def __del__(self):
        self.Disconnect()
         
    def Connect(self):
        """ Connects to the robotic arm """
        self.Disconnect()
        sleep(0.5)    
        self.ep = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ep.connect((self.host, self.port))
        self.ep.settimeout(1.0)

    def Disconnect(self):
        """ Closes and deletes the connection """
        if self.ep != None:
            self.ep.close()
            del self.ep
        self.ep = None
        
    def parseResponse(self) -> bool:
        """ internal: do not use """
        while True:
            # receive response
            raw = self.ep.recv(Arm.CACHE_SIZE)
                
            if len(raw) < Arm.HEADER_SIZE:
                #print("Invalid header length " + str(len(raw)))
                continue
                
            # unpack header
            header = struct.unpack('<BBHH', raw[:Arm.HEADER_SIZE])
            if header[0] != Arm.START_BYTE:
                #print("invalid start byte")
                continue
            
            if header[2] != self.lastRequestID:
                #print("invalid response ID")
                continue
             
            resCode = header[1]
            argsNum = header[3] // 4
            if len(raw) != Arm.HEADER_SIZE + argsNum * 4:
                 print("invalid message length")
                 return False
             
            # get args
            if argsNum > 0:
                args = struct.unpack('<' + str(argsNum) + 'f', raw[Arm.HEADER_SIZE:])
            else:
                args = []
    
            if resCode == ResponseCode.RES_OK:
                 return True
    
            if resCode == ResponseCode.RES_State:
                 if argsNum == 25:
                     self.lastState.joints = args[0:12]
                     self.lastState.trans = Position.fromList(args[12:15])
                     self.lastState.rot = Quaternion.fromList(args[15:19])
                     self.lastState.robconf = Conf.fromList(args[19:23])
                     self.lastState.moveState = int(args[23])
                     self.lastState.pending = int(args[24])
                     return True
                 else:
                     print("invalid number of arguments")
                     return False
                 
            if resCode == ResponseCode.RES_GripState:
                if argsNum == 4:
                    self.lastGripState.code = GripStatusCode(int(args[0]))
                    self.lastGripState.pos = args[1]
                    self.lastGripState.pressure1 = args[2]
                    self.lastGripState.pressure2 = args[3]
                    return True
                else:
                    return False
                 
            if resCode == ResponseCode.RES_NotReachable:
                    print("point is not reachable")
                    return False
            else:
                 return False # unknown or negative result code   
        #endwhile             

    def sendRequest(self, code:RequestCode, args:List[float]=[]) -> bool:
        """ internal: do not use """
        self.lock.acquire()
        while True:
            try:
                # increment ID of the message
                self.lastRequestID += 1
                if self.lastRequestID > 0xFFFF :
                    self.lastRequestID = 0
                
                # create packet
                argNum = len(args)
                raw = struct.pack('<BBHH' + str(argNum) + 'f', Arm.START_BYTE, code, self.lastRequestID, argNum * 4, *args); 
                
                # send data
                self.ep.sendall(raw)
                     
                # process response
                return self.parseResponse()
            except ConnectionError:
                self.Connect()
            finally:
                self.lock.release()

    def WaitPos(self):
        """ Waits until position is reached. """
        while True:
            self.Read()
            if self.lastState.pending == 0:
                return
            else:
                sleep(0.2)
                     

    def MoveTo(self, target, linear=False, sync=True) -> bool:
        """ Adds new target to the trajectory. Use JointTarget or LinearTarget """
        if type(target) is JointTarget:
            args = 23*[None]
            args[0:12] = target.joints
            args[12:16] = target.speed.toList()
            args[16:23] = target.zone.toList()
            res = self.sendRequest(RequestCode.CMD_MoveAbsJ, args)

        elif type(target) is LinearTarget:
            args = 28*[None]
            args[0:3] = target.trans.toList()
            args[3:7] = target.rot.toList()
            args[7:11] = target.robconf.toList()
            args[11:17] = target.extax
            args[17:21] = target.speed.toList()
            args[21:28] = target.zone.toList()
            if linear:
                res = self.sendRequest(RequestCode.CMD_MoveL, args)
            else:
                res = self.sendRequest(RequestCode.CMD_MoveJ, args)
        else:
            res = False

        if res and sync:
            self.WaitPos()

        return res

    def Stop(self) -> bool:
       """ Stops movement and clears trajectory """
       return self.sendRequest(RequestCode.CMD_StopNow)

    def Pause(self) -> bool:
        """ Pauses the movement """
        res = self.sendRequest(RequestCode.CMD_Pause)
        if res: self.lastState.moveState = MoveStatus.Paused
        return res

    def Resume(self) -> bool :
        """ Resumes the movement """
        res = self.sendRequest(RequestCode.CMD_Resume)
        if res: self.lastState.moveState = MoveStatus.Normal
        return res
    
    def IsPaused(self) -> bool:
        """ Returns True when movement is paused """
        return (self.lastState.moveState == MoveStatus.Paused)

    def ChangeTool(self, tool: Tool) -> bool :
        """ Updates tool data """
        args = tool.toList()
        return self.sendRequest(RequestCode.CMD_SetTool, args)
    
    def ChangeWorkobject(self, wobj: Workobject) -> bool :
        """ Updates workobject """
        args = wobj.toList()
        return self.sendRequest(RequestCode.CMD_SetWobj, args)
    
    def Test(self) -> bool:
        """ Tests connection """
        return self.sendRequest(RequestCode.CMD_Ping)
    
    def Read(self) -> Status:
        """ Reads current state of the robot """
        if self.sendRequest(RequestCode.CMD_ReadState):
            return self.lastState
        else:
            raise Exception("abb.Arm: unable to read state")


class SmartGripper:
    """ Interface for YuMi Smart Gripper """
    
    arm:Arm
    
    def __init__(self, arm:Arm):
        """ Creates new instance of Smart Gripper bound with given robotic arm """
        self.arm = arm
    
    def Calibrate(self) -> bool:
        """ Initializes Smart Gripper and starts its callibration. Run this before any GripX function. """
        return self.arm.sendRequest(RequestCode.CMD_GripInit)
    
    def MoveTo(self, pos:float, noWait:bool=False) -> bool:
        """ Moves gripper to the given position. Optionaly does not wait until done. """
        if pos < 0.0 or pos > 25.0:
            raise ValueError("grip position out of range")
        args = [pos, 1.0 if noWait else 0.0]
        return self.arm.sendRequest(RequestCode.CMD_GripMoveTo, args)
    
    def GripSmart(self, force:float=20, pos:float=0, tol:float=2, direct:GripDirection=GripDirection.Inward, noWait:bool=False) -> bool:
        """ Grips considering the given maximal force, position, tolerance. Optionaly does not wait until done. """
        if pos < 0.0 or pos > 25.0:
            raise ValueError("grip position out of range (0, 25) mm")
        if force < 0.0 or force > 20.0:
            raise ValueError("grip force out of range (0, 20) N")
        if tol < 0.0 or tol > 25.0:
            raise ValueError("grip tolerange out of range (0, 25) mm")
        args = [float(direct), force, pos, tol, 1.0 if noWait else 0.0]
        return self.arm.sendRequest(RequestCode.CMD_GripSmart, args)
    
    def Stop(self) -> bool:
        """ Stops gripper immediately """
        return self.arm.sendRequest(RequestCode.CMD_GripStopNow)
    
    def Blow(self, channel:int, enable:bool) -> bool:
        """ Starts/stops blowing on given channel (1 or 2) """
        args = [1.0 if enable else 0.0, float(channel)]
        return self.arm.sendRequest(RequestCode.CMD_GripBlow, args)
    
    def Vacuum(self, channel:int, enable:bool, pressureMax:float=110) -> bool:
        """ Starts / stops given vacuum channel (1 or 2) with given maximum pressure. """
        if pressureMax < 0 or pressureMax > 110:
            raise ValueError("maximal pressure out of range (0, 110) kPa")
        args = [1.0 if enable else 0.0, float(channel), pressureMax]
        return self.arm.sendRequest(RequestCode.CMD_GripVacuum, args)
    
    def Read(self) -> GripStatus:
        """ Read current state of the smart gripper """
        if self.arm.sendRequest(RequestCode.CMD_GripRead):
            return self.arm.lastGripState
        else:
            raise Exception("abb.SmartGripper: unable to read state")

class YuMi:
    """ ABB YuMi robot """
    
    LeftArm:Arm
    RightArm:Arm
    LeftHand:SmartGripper
    RightHand:SmartGripper
  
    LEFT_ARM_PORT = 10010
    RIGHT_ARM_PORT = 10020
    
    def __init__(self, host:str):
        """ creates interface to YuMi robot at given IP address, but does not connect to it """
        self.LeftArm = Arm(host, YuMi.LEFT_ARM_PORT)
        self.RightArm = Arm(host, YuMi.RIGHT_ARM_PORT)
        self.LeftHand = SmartGripper(self.LeftArm)
        self.RightHand = SmartGripper(self.RightArm)
        
    def __del__(self):
        self.Disconnect()
  
    def Connect(self):
        """ Connects to the controller """
        self.LeftArm.Connect()
        self.RightArm.Connect()
        
    def Disconnect(self):
        """ Disconnects from the controller """
        self.LeftArm.Disconnect()
        self.RightArm.Disconnect()
        
    def Stop(self) -> bool:
        """ Stops movement of the robot """
        return (self.LeftArm.Stop() and self.RightArm.Stop())
            
    def Pause(self) -> bool:
        """ Pauses movement of both arms """
        return (self.LeftArm.Pause() and self.RightArm.Pause())
    
    def Resume(self) -> bool:
        """ Resumes movement of both arms """
        return (self.LeftArm.Resume() and self.RightArm.Resume())
    
    def IsPaused(self) -> bool:
        """ Returns true when both arms are paused """
        return (self.LeftArm.IsPaused() and self.RightArm.IsPaused())
    
    def Test(self) -> bool:
        """ Returns true when both arms respond """
        return (self.LeftArm.Test() and self.RightArm.Test())
        
    def InitGrippers(self) -> bool:
        """ Setups grippers """
        if not(self.LeftHand.Calibrate()) or not(self.RightHand.Calibrate()):
            print("unable to calibrate grippers")
            return False
        # set tool data
        tool = Tool()
        tool.robhold = True
        tool.trans = Position.fromList([0,0,114.2])
        tool.rot = Quaternion.fromList([1,0,0,0])
        tool.load = Load.fromList([0.229,7.9,12.4,48.7,1,0,0,0,0.00021,0.00023,0.00008])
        if not(self.LeftArm.ChangeTool(tool)) or not(self.RightArm.ChangeTool(tool)):
            print("unable to set tooldata")
            return False
        # everything is OK
        return True

    def MoveHome(self) -> bool:
        """ moves to home position """
        left_home = JointTarget()
        left_home.joints = [0,-130,30,0,40,0,135,0,0,0,0,0]
        left_home.speed = Speed(50)
        left_home.zone = Zone.fine()
        
        right_home = JointTarget()
        right_home.joints = [0,-130,30,0,40,0,-135,0,0,0,0,0]
        right_home.speed = Speed(50)
        right_home.zone = Zone.fine()
        
        if not(self.LeftArm.MoveTo(left_home)):
            return False
        
        if not(self.RightArm.MoveTo(right_home)):
            return False
        
        return True
    
    
        
        
