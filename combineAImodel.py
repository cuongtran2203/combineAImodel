"""combineAImodel controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot,Camera,Display,Keyboard
import cv2
import numpy as np
import io
from PIL import Image
import torch
ratio=0.1
velocity=1.0
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
class Control_robot():
    def __init__(self,left_motor,right_motor):
        
        self.left_motor=left_motor
        self.right_motor=right_motor
        self.max_speed=6.28
        self.velocity=3.0
        self.ratio=0.1
  
    def increase_ratio(self):
         self.ratio+=0.1
         return self.ratio
         

    def decrease_ratio(self):
        self.ratio-=0.1
        return self.ratio
     
  
    def goup_velocity(self):
        self.velocity+=self.ratio*self.max_speed
        return self.velocity
        
        
    def decrease_velocity(self):
        self.velocity-=self.ratio*self.max_speed
        return self.velocity
        
        
    def forward_(self):
        self.left_motor.setVelocity(self.velocity)
        self.right_motor.setVelocity(self.velocity)
        print(self.velocity)
        
    def turn_right(self):
        self.left_motor.setVelocity(self.velocity/8)
        self.right_motor.setVelocity(self.velocity)
        print(self.velocity)
        
    def turn_left(self):
        self.left_motor.setVelocity(velocity)
        self.right_motor.setVelocity(velocity/8)
        print(self.velocity)
        
    def back(self):
        self.left_motor.setVelocity(-self.velocity)
        self.right_motor.setVelocity(-self.velocity)
        print(self.velocity)
            
            
def run_robot(robot):
    

    max_speed=6.28
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    left_motor=robot.getDevice('left wheel motor')
    right_motor=robot.getDevice('right wheel motor')
    left_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setPosition(float("inf"))
    right_motor.setVelocity(0.0)
    cam=Camera("camera")
    display=Display("camera")
    display.drawText("Tran Cuong",10,10)
    keyboard=Keyboard()
    keyboard.enable(timestep)
    control=Control_robot(left_motor,right_motor)
    cam.enable(24)
    
    
    prox_sensors=[]
    for ind in range(8):
        sensor_name='ps'+str(ind)
        prox_sensors.append(robot.getDistanceSensor(sensor_name))
        prox_sensors[ind].enable(timestep)
        
        
    
    
    
    
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        # for ind in range(8):
            # print("ind: {},val: {}".format(ind,prox_sensors[ind].getValue()))
            
        key=keyboard.getKey() 
        '''
        Control robot by keyboard
        '''
        if key==keyboard.CONTROL+ord("U"):
            control.increase_ratio()
            print("increase ratio")
        elif key==keyboard.CONTROL+ord("D"):
            control.decrease_ratio()
            print("decrease ratio")
        elif key==keyboard.UP:
            control.forward_()
            print("forward")
        elif key==keyboard.RIGHT:
            control.turn_right()
            print("turn right")
        elif key==keyboard.LEFT:
            control.turn_right()
            print("turn left")
        
        # left_wall=prox_sensors[5].getValue()>80
        # left_conner=prox_sensors[6].getValue()>80
        # front_wall=prox_sensors[7].getValue()>80
        # left_speed=max_speed
        # right_speed=max_speed
        # camdata=cam.getImage()
        # gray = cam.imageGetGray(camdata, cam.getWidth(), 5, 10)
        # cam.saveImage("image.png",100)
        # if front_wall:
        #     print("turn right in place")
        #     left_speed=max_speed
        #     right_speed=-max_speed
        # else:
        #     if left_wall:
        #          print("forward")
        #          left_speed=max_speed
        #          right_speed=max_speed
        #     else :
        #         left_speed=max_speed/8
        #         right_speed=max_speed
        #         if left_conner:
        #             print("drive right")
        #             left_speed=max_speed
        #             right_speed=max_speed/8

        # left_motor.setVelocity(left_speed)
        # right_motor.setVelocity(right_speed)

    
# Enter here exit cleanup code.
if __name__=="__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)
