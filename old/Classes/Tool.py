# -*- coding: utf-8 -*-
"""
Created on Tue Feb 27 13:46:36 2018

"""
import serial
import struct
import threading
import time

from data_logger import log_metrics

try:
    from Circle import Circle  
except ImportError:
    from Classes.Circle import Circle


class Tool(Circle, threading.Thread):
    COLOR = [0, 0, 255]  # RGB
    
    def __init__(self, x, y, r, port='COM13', baudrate=115200):  # parent
        threading.Thread.__init__(self)  # super class constructor
        self.daemon = True  # set to True so that this thread stops when the process stops
        Circle.__init__(self, -1, x, y, r)  # super class constructor
        self.img = Circle.CIRCLE_IMG_GREY
        self.portNumber = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None
        self.startTime = 0
        self.connected = False
        self.color = Tool.COLOR
        self.target = None  # the current target circle
        self.logging = False
        self.timingFunc = None
        self.pedalPressCount = 0

    def setColorBlack(self):
        self.img = Circle.CIRCLE_IMG_BLACK

    def setTimingFunc(self, func):
        self.timingFunc = func

    def setTarget(self, mark):
        self.target = mark

    def getTarget(self):
        return self.target

    def enableLogging(self):
        if self.timingFunc is None:
            raise EnvironmentError("Cannot start logging, timing function has not been set!")
        else:
            self.logging = True

    def disableLogging(self):
        self.logging = False

    def getElapsedTime(self):
        """Returns the elapsed time since the first byte was received from the microcontroller
        Note: This value is only accurate if self.isConnected() return True"""
        return time.time() - self.startTime
        
    def connect(self):
        """Opens a connection to the given serial com port"""
        self.ser = serial.Serial(self.portNumber, self.baudrate, parity=serial.PARITY_NONE)
        # read one byte so this thread blocks until the microcontroller starts transmitting data
        # we do this so the startTime variable is set to the exact time we start to receive data
        self.ser.read()
        self.startTime = time.time()
        self.connected = True
        print("Tool: COM Port Opened!")
        
    def disconnect(self):
        if self.ser != None:
            print("Tool: Closing Serial COM Port")
            self.ser.close()
        self.connected = False
        
    def isConnected(self):
        return self.connected

    # Thread
    def run(self): 
        self.connect()

        print("Tool: COM Port Opened!")
        try:
            while self.running:
                count = 0
    
                while count < 6:
                    byte = self.ser.read()  # read just 1 byte
                    if byte == b'\xAA':
                        count += 1
    
                raw_data = self.ser.read(24)
                self.pedalPressCount = struct.unpack("<1B", self.ser.read())[0]
                angle_x, angle_y, angle_z = struct.unpack("<3d", raw_data)
                self.angle_to_coordinates(-angle_z, angle_x, angle_y)
    
                if self.logging:
                    log_metrics(mark=self.target.idx, pedal_presses=self.pedalPressCount,
                                angle_x=angle_x, angle_y=angle_y, angle_z=angle_z,
                                distance_from_target=self.distanceTo(self.target), t=self.timingFunc())
    
                print("p: %d  x: %.04f  y: %.04f  r: %.04f"%(self.pedalPressCount, angle_x, angle_y, -angle_z))
        except:
            print("Exception")
        finally:
            print("Tool: Closing Serial COM Port")
            self.disconnect()
        
    def angle_to_coordinates(self, angle_x, angle_y, angle_z):
        angle_scale = 2
        zoom_scale = 9
        angle_limit_x = 1280.0
        angle_limit_y = 720.0
        radius_limit = 150
        
        angle_x /= angle_scale
        angle_y /= angle_scale
        angle_z /= zoom_scale
        
        temp_angle_x = self.x + angle_x
        temp_angle_y = self.y + angle_y 
        temp_r = self.r + angle_z
        
        if temp_angle_x < 0:
            temp_angle_x = 0
        elif temp_angle_x > angle_limit_x:
            temp_angle_x = angle_limit_x
                    
        if temp_angle_y < 0:
            temp_angle_y = 0
        elif temp_angle_y > angle_limit_y:
            temp_angle_y = angle_limit_y
            
        if temp_r < 1:
            temp_r = 1
        elif temp_r > radius_limit:
            temp_r = radius_limit
                    
        self.x = round(temp_angle_x)
        self.y = round(temp_angle_y)
        self.r = round(temp_r)    
        
    def stop(self):
        self.running = False
        
    def __del__(self):
        self.disconnect()
               
        
        
        
if __name__ == "__main__":
    print("kkk")
    tool1 = Tool(2,2,4)
#    tool1.start()
    