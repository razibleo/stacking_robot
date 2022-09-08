from __future__ import print_function
from VideoCamera import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2
import numpy as np
import RPi.GPIO as IO

import os
import pygame
import pprint


from time import sleep
from ultrasonicThread import UltrasonicSensor
from ObstacleAvoidanceThread import ObstacleAvoidance

leftMotor = 0
rightMotor = 0
thrust = 0
angle = 0
L3_X_Value = 0
R3_X_Value = 0
R3_Y_Value = 0

thrustFactor = 0.7
angleFactor = 0.3
servo1Pin = 27 #blue
servo2Pin = 17 #yellow
servo3Pin = 10 #green
servo4Pin = 4 #orange

servo1Factor = 0.08# 0.06 - 0.09   # base
servo2Factor = 0.08# 0.03 - 0.08  
servo3Factor = 0.07#0.02- 0.08   
servo4Factor = 0.07# 0.05 - 0.08   # gripper

servo1minFactor = 0.06
servo2minFactor = 0.03
servo3minFactor = 0.02
servo4minFactor = 0.05

servo1maxFactor = 0.09
servo2maxFactor = 0.08
servo3maxFactor = 0.08
servo4maxFactor = 0.08

currentServo1 = servo1Factor#0.16
currentServo2 = servo2Factor#0.1
currentServo3 = servo3Factor#0.14
currentServo4 = servo4Factor#0.14


in1Pin = 18
in2Pin = 23
in3Pin = 24
in4Pin = 25



servoPin = 21
ultrasonic1 = UltrasonicSensor(13,26)


IO.setwarnings(False)
IO.setmode(IO.BCM)

IO.setup(12,IO.IN)
IO.setup(16,IO.IN)

obstacleAvoidance = ObstacleAvoidance(servoPin,in1Pin,in2Pin,in3Pin,in4Pin)
obstacleAvoidance.setLoop(False)


#definig the range of red color
red_lower=np.array([0,49,1],np.uint8)
red_upper=np.array([6,255,207],np.uint8)

#defining the Range of orange color
orange_lower=np.array([8,210,82],np.uint8)
orange_upper=np.array([20,255,208],np.uint8)

#defining the Range of yellow color
yellow_lower=np.array([30,153,72],np.uint8)
yellow_upper=np.array([48,255,207],np.uint8)

blockDetected = False
  

print("[INFO] sampling THREADED frames from `picamera` module...")
vs = PiVideoStream().start()
time.sleep(2.0)


def changeServoPos(servoPinNo,value):
    global currentServo1, currentServo2, currentServo3,currentServo4
    
    currentServoValue = None
    value = round(value, 3)
    
    if servoPinNo == servo1Pin:
        currentServoValue = currentServo1
        currentServo1 = value
    elif servoPinNo == servo2Pin:
        currentServoValue = currentServo2
        currentServo2 = value
    elif servoPinNo == servo3Pin:
        currentServoValue = currentServo3
        currentServo3 = value
    elif servoPinNo == servo4Pin:
        currentServoValue = currentServo4
        currentServo4 = value
    
    while True:
        if value > currentServoValue:
            currentServoValue += 0.0005
            currentServoValue = round(currentServoValue, 4)
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(servoPinNo,currentServoValue))
            print(currentServoValue)
        elif value < currentServoValue:
            currentServoValue -= 0.0005
            currentServoValue = round(currentServoValue, 4)
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(servoPinNo,currentServoValue))
            print(currentServoValue)
        else:
            print("done")
            break
    
    
    

def initDefaultServoPos():
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(servo1Pin,servo1Factor))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(servo2Pin,servo2Factor))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(servo3Pin,servo3Factor))
#     changeServoPos(servo4Pin,servo4Factor)
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(servo4Pin,servo4Factor))


def backward():
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in1Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in2Pin,0.6))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in3Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in4Pin,0.6))
    print("backward")
    
def stop():
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in1Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in2Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in3Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in4Pin,0))
    print("Stop")

def forward():
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in1Pin,0.45))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in2Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in3Pin,0.45))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in4Pin,0))
    print("Forward")
    
def right():
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in1Pin,0.7))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in2Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in3Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in4Pin,0.7))
    print("Right")
    
def left():
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in1Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in2Pin,0.7))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in3Pin,0.7))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in4Pin,0))
    print("Left")    


def slowForward():
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in1Pin,0.35))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in2Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in3Pin,0.35))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in4Pin,0))
    print("slowForward")

def slowRight():
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in1Pin,0.3))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in2Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in3Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in4Pin,0.3))
    print("slowRight")

def slowLeft():
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in1Pin,0))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in2Pin,0.3))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in3Pin,0.3))
    os.system("echo ""{}={}"" > /dev/pi-blaster".format(in4Pin,0))
    print("slowLeft")    



def lineFollower():
    while True:
        print("Following Line")
        left_detect  = IO.input(12)
        right_detect = IO.input(16)
        
        if left_detect == 0 and right_detect == 0:
            forward()
            
        
        if left_detect == 0 and right_detect == 1:
            right()
            
        if left_detect == 1 and right_detect == 0:
            left()
        
        elif left_detect == 1 and right_detect == 1:
            stop()
            break;

def lineBoth():
    print("lineBoth")
    left()
    
    while True:
        left_detect  = IO.input(12)
        right_detect = IO.input(16)
        
        if right_detect == 0 or left_detect == 0:
            lineFollower()
            break
 
 
def lineLeft():
    print("lineLeft")
    left()
    
    while True:
        left_detect  = IO.input(12)
        right_detect = IO.input(16)
        
        if left_detect == 0:
            
            lineFollower()
            break
            
        
def lineRight():
    print("lineRight")
    left()
    
    while True:
        left_detect  = IO.input(12)
        right_detect = IO.input(16)
        
        if right_detect == 1 and left_detect == 0:
            lineRight()
            break
        elif right_detect == 1 and left_detect == 1:
            lineBoth()
            break
    



def grabRedBlock():
    print("Grabbing Red block in progress...")
    backward()
    sleep(0.5)
    stop()
    changeServoPos(servo2Pin,0.03)
    changeServoPos(servo3Pin,0.0489)
    changeServoPos(servo4Pin,0.05)
    forward()
    sleep(2)
    stop()
    changeServoPos(servo4Pin,0.068)
    changeServoPos(servo2Pin,0.08)
    changeServoPos(servo3Pin,0.056)
    
    stop()
    
    
# loop over some frames...this time using the threaded stream
def detectBlock(blockColor):
    global blockDetected, current
    
    if blockColor == Color.RED:
        lower = red_lower
        upper = red_upper
    elif blockColor == Color.ORANGE:
        lower = orange_lower
        upper = orange_upper
    elif blockColor == Color.YELLOW:
        lower = yellow_lower
        upper = yellow_upper

    
    fps = FPS().start()
    frame = vs.read()
    

    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    

  
    #finding the range of red,orange and yellow color in the image
    color=cv2.inRange(hsv, lower, upper)
   
    
    #Morphological transformation, Dilation     
    kernal = np.ones((5 ,5), "uint8")

    color=cv2.dilate(color, kernal)
    res=cv2.bitwise_and(frame, frame, mask = color) 


    #Tracking the  Color
    (_,contours,hierarchy)=cv2.findContours(color,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    
    for pic, contour in enumerate(contours):
            
            area = cv2.contourArea(contour)
         
            if area > 300:
                    
                    if obstacleAvoidance.loop:
                        print("Block Found!")
                        obstacleAvoidance.setLoop(False)
                        blockDetected = True
                        os.system("echo ""{}={}"" > /dev/pi-blaster".format(servoPin,0.07))
                        slowForward()
                        
                    
                    x,y,w,h = cv2.boundingRect(contour)
                    block_X_Center = (x+x+w)/2
                    
                    frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
#                         cv2.putText(frame,"red",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))
                    
                    
                    if block_X_Center > 480:
#                         print("right")
                        slowRight()
                    elif block_X_Center < 290:
#                         print("left")
                        slowLeft()
                    else:
#                         print("forward")
                        slowForward()
                    break
                
                
    
    

    if blockDetected and obstacleAvoidance.getDistance()< 25:
        stop()
        print("Arrived")
#         #pickupblock()
#         obstacleAvoidance.setLoop(True)
#         blockDetected = False
        current = State.ARRIVEDATREDBLOCK
        
                
    # check to see if the frame should be displayed to our screen
    cv2.imshow("Frame", frame)
    #     cv2.imshow("res", res)
    key = cv2.waitKey(1) & 0xFF 
    # update the FPS counter
    fps.update()
    fps.stop()
    #     print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
#     print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    # stop the timer and display FPS information        







class Color:
    RED, ORANGE, YELLOW = range(3)
    
class State:
    MOVETOORANGEBLOCK, ARRIVEDATORANGEBLOCK, MOVETOREDBLOCK, ARRIVEDATREDBLOCK, MOVETOYELLOWBLOCK, ARRIVEDATYELLOWBLOCK, EXIT = range(7) 

current = State.MOVETOREDBLOCK 

class PS4Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""


    class Buttons:
        CROSS, CIRCLE, TRIANGLE, SQUARE, L1, R1, L2, R2, SHARE, OPTIONS, PS, L3, R3, TOUCHPAD = range(14)

    class Axis:
        L3_X, L3_Y, L2, R3_X, R3_Y, R2 = range(6)
        
    class Hat:
        LEFT, RIGHT, DOWN, UP = range(4)    

    controller = None
    axis_data = None
    button_data = None
    hat_data = None

    def init(self):
        """Initialize the joystick components"""

        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def manualMode(self):
        print("manual mode selected")
        stop()      
       
        

        global leftMotor, rightMotor, thrust, angle, thrustFactor, angleFactor, L3_X_Value, R3_X_Value, R3_Y_Value, obstacleAvoidance
        obstacleAvoidance.setLoop(False)

        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)
        
        switchModeButtonPressed = False
        previousForwardThrust = 0
        previousBackwardThrust = 0
        previousLeftMotor = 0
        previousRightMotor = 0
        servoButtonNumber = self.Buttons.CROSS
        hatNumber = None
        count = 0
        

        while True:
            
            if switchModeButtonPressed:
                break
            
            if leftMotor!=0 or rightMotor!=0:            
                self.move(leftMotor, rightMotor)
                print(leftMotor, " ", rightMotor)
            else:
                self.change_servo_angle(servoButtonNumber, hatNumber)
                
                if previousLeftMotor != 0 or previousRightMotor !=0:
                    
                    self.move(0, 0)
                    
            
            previousLeftMotor = leftMotor
            previousRightMotor = rightMotor
#             if count == 25:
#                 
#                 count = 0
#                 self.change_servo_angle(servoButtonNumber, hatNumber)
#             else:
#                 count += 1
            
            
            for event in pygame.event.get():
             

                if event.type == pygame.JOYAXISMOTION and hatNumber == None:
                    self.axis_data[event.axis] = round(event.value, 2)
                    

                    if event.axis == self.Axis.R2:
                        forwardThrust = (self.axis_data[self.Axis.R2] + 1)
                        previousForwardThrust = forwardThrust
                        # print(self.axis_data[self.Axis.R2]+1)
                    else:
                        forwardThrust = previousForwardThrust

                    if event.axis == self.Axis.L2:
                        backwardThrust = (self.axis_data[self.Axis.L2] + 1)
                        previousBackwardThrust = backwardThrust
                        #print(self.axis_data[self.Axis.L2] + 1)
                    else:
                        backwardThrust = previousBackwardThrust

                    thrust = (forwardThrust - backwardThrust) / 2

                    leftMotor = round((thrustFactor * thrust), 2)
                    rightMotor = leftMotor

                    if event.axis == self.Axis.L3_X:
                        L3_X_Value = self.axis_data[self.Axis.L3_X]


                        # print(leftMotor, " ", rightMotor)


                    if (event.axis == self.Axis.R3_X or event.axis == self.Axis.R3_Y): # ensure thrust is zero before even moving R3 for differential controls
                        
                        
                        if event.axis == self.Axis.R3_X:
                            
                            R3_X_Value = self.axis_data[self.Axis.R3_X]
                           
                        

                            leftMotor = round(R3_X_Value * 0.75, 2)
                            rightMotor = round(-1*R3_X_Value * 0.75, 2)


#                             print(leftMotor, rightMotor)
                            

                        if event.axis == self.Axis.R3_Y:
                            R3_Y_Value = self.axis_data[self.Axis.R3_Y]
                            

                    else:
                        
                        #only if the R3 is not being used aka differential steering
                        if thrust != 0:                            
                            leftMotor = round((thrustFactor * thrust) + (angleFactor * L3_X_Value), 2)
                            rightMotor = round((thrustFactor * thrust) - (angleFactor * L3_X_Value), 2)
                            
#                             print(leftMotor, " ", rightMotor)
                        else:
                            pass
#                             print(0, " ", 0)
                        

                elif event.type == pygame.JOYBUTTONDOWN and hatNumber == None:
                    self.button_data[event.button] = True
                    
#                     pprint.pprint(self.button_data)
                    if event.button == self.Buttons.OPTIONS:
                        switchModeButtonPressed = True
                        
                    elif event.button == self.Buttons.CROSS:
                        print("CROSS")
                        servoButtonNumber = self.Buttons.CROSS
                    elif event.button == self.Buttons.SQUARE:
                        print("SQUARE")
                        servoButtonNumber = self.Buttons.SQUARE
                    elif event.button == self.Buttons.CIRCLE:
                        print("CIRCLE")
                        servoButtonNumber = self.Buttons.CIRCLE
                    elif event.button == self.Buttons.TRIANGLE:
                        print("TRIANGLE")
                        servoButtonNumber = self.Buttons.TRIANGLE      
                    

                    # if event.button == self.SQUARE:
                    #     print("SQUARE PRESSED")
                elif event.type == pygame.JOYBUTTONUP and hatNumber == None:
                    self.button_data[event.button] = False
                    
                    
                elif event.type == pygame.JOYHATMOTION and leftMotor == 0 and rightMotor == 0:
                    self.hat_data[event.hat] = event.value
                    
                    if self.hat_data[0][0] == -1:
                        hatNumber = self.Hat.LEFT
                        print("Left Hat")
                    elif self.hat_data[0][0] == 1:
                        hatNumber = self.Hat.RIGHT
                        print("Right Hat")
                    elif self.hat_data[0][1] == -1:
                        hatNumber = self.Hat.DOWN
                        print("Down Hat")
                    elif self.hat_data[0][1] == 1:
                        hatNumber = self.Hat.UP
                        print("Up Hat")
                    else:
                        hatNumber = None
                        print("No Hat")
                        

                # os.system('clear')
                # pprint.pprint(self.button_data)
                # pprint.pprint(self.axis_data)
                # pprint.pprint(self.hat_data)
                
    def autoMode(self):
        
        print("auto mode selected")
        
        global leftMotor, rightMotor, thrust, angle, thrustFactor, angleFactor, L3_X_Value, R3_X_Value, R3_Y_Value, current
        
        obstacleAvoidance.setLoop(True)
        stop()
            
        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)
        
        switchModeButtonPressed = False
            
        current = State.MOVETOREDBLOCK   

        while True:
            
            
            #switching to line follower -----work in progress
#             left_detect  = IO.input(12)
#             right_detect = IO.input(16)
#             
#             print(str(left_detect)+" " + str(right_detect))
#             
#             if left_detect == 1 and right_detect == 1:
#                 lineBoth()
#                 break
#                 
#                       
#             if left_detect == 1 and right_detect == 0:
#                 lineLeft()
#                 break
#                 
#                 
#             elif left_detect == 0 and right_detect == 1:
#                 lineRight()
#                 break
#             
#             elif left_detect == 0 and right_detect == 0:
#                 forward()
     
            if switchModeButtonPressed:
                break
            
        
            if current == State.MOVETOREDBLOCK:
                detectBlock(Color.RED)
            elif current == State.ARRIVEDATREDBLOCK:
                grabRedBlock()
                break
            
 
            
            for event in pygame.event.get():
            
                if event.type == pygame.JOYAXISMOTION:
                    pass
#                     self.axis_data[event.axis] = round(event.value, 2)
                     
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_data[event.button] = True
                    
#                     pprint.pprint(self.button_data)
                    if event.button == self.Buttons.OPTIONS:
                        switchModeButtonPressed = True      
             
                elif event.type == pygame.JOYBUTTONUP:
                    pass
#                     self.button_data[event.button] = False
                             
                elif event.type == pygame.JOYHATMOTION:
                    pass
#                     self.hat_data[event.hat] = event.value
                    
                    
            
                
    def move(self,leftMotor, rightMotor):
        if leftMotor < 0:
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(in2Pin,-1*leftMotor))
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(in1Pin,0))
            
        elif leftMotor > 0:
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(in1Pin,1*leftMotor))
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(in2Pin,0))
        else:
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(in2Pin,0))
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(in1Pin,0))
            
        
        
        if rightMotor < 0:
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(in4Pin,-1*rightMotor))
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(in3Pin,0))
            
        elif rightMotor > 0:
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(in3Pin,1*rightMotor))
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(in4Pin,0))
        else:
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(in3Pin,0))
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(in4Pin,0))



    def change_servo_angle(self,servoButtonNumber, hatNumber):
         global servo1Factor,servo2Factor,servo3Factor,servo4Factor
        
         if hatNumber != None:
                        
                        if servoButtonNumber == self.Buttons.CROSS:
                            
                            minFactor = servo1minFactor
                            maxFactor = servo1maxFactor
                            
                            
                            if hatNumber == self.Hat.LEFT:
                               
                                
                                if servo1Factor < maxFactor:
                                    
                                    
                                    servo1Factor += 0.001
                                    os.system("echo ""{}={}"" > /dev/pi-blaster".format(servo1Pin,servo1Factor))
                                    
                            elif hatNumber == self.Hat.RIGHT:
                                
                                if servo1Factor > minFactor:
                                    
                                    servo1Factor -= 0.001
                                    os.system("echo ""{}={}"" > /dev/pi-blaster".format(servo1Pin,servo1Factor))     
                                    
                            print("servo1Factor ",servo1Factor)
                            currentServo1 = servo1Factor
                            
                        elif servoButtonNumber == self.Buttons.SQUARE:
                            
                            
                            minFactor = servo2minFactor
                            maxFactor = servo2maxFactor
                            
                            
                            if hatNumber == self.Hat.UP:
                                
                                if servo2Factor < maxFactor:
                                    
                                    
                                    servo2Factor += 0.0005
                                    os.system("echo ""{}={}"" > /dev/pi-blaster".format(servo2Pin,servo2Factor))
                                    
                            elif hatNumber == self.Hat.DOWN:
                                
                                if servo2Factor > minFactor:
                                    
                                    servo2Factor -= 0.0005
                                    os.system("echo ""{}={}"" > /dev/pi-blaster".format(servo2Pin,servo2Factor))
                            print("servo2Factor ",servo2Factor)
                            currentServo2 = servo2Factor
                            
                        elif servoButtonNumber == self.Buttons.CIRCLE:
                            minFactor = servo3minFactor
                            maxFactor = servo3maxFactor
                            
                            
                            if hatNumber == self.Hat.UP:
                                
                                if servo3Factor < maxFactor:
                                    
                                    
                                    servo3Factor += 0.001
                                    os.system("echo ""{}={}"" > /dev/pi-blaster".format(servo3Pin,servo3Factor))
                                    
                            elif hatNumber == self.Hat.DOWN:
                                
                                
                                if servo3Factor > minFactor:
                                    
                                    servo3Factor -= 0.001
                                    os.system("echo ""{}={}"" > /dev/pi-blaster".format(servo3Pin,servo3Factor))
                            print("servo3Factor ",servo3Factor)
                            currentServo3 = servo3Factor
                            
                        elif servoButtonNumber == self.Buttons.TRIANGLE:
                            minFactor = servo4minFactor
                            maxFactor = servo4maxFactor
                            
                            
                            if hatNumber == self.Hat.RIGHT:
                               
                                
                                if servo4Factor < maxFactor:
                                    
                                    
                                    servo4Factor += 0.001
                                    os.system("echo ""{}={}"" > /dev/pi-blaster".format(servo4Pin,servo4Factor))                                    
                            elif hatNumber == self.Hat.LEFT:
                                
                                if servo4Factor > minFactor:
                                    
                                    servo4Factor -= 0.001
                                    os.system("echo ""{}={}"" > /dev/pi-blaster".format(servo4Pin,servo4Factor))
                                    
                            print("servo4Factor ",servo4Factor)
                            currentServo4 = servo4Factor
        
    



if __name__ == "__main__":
    
    
    os.system("sudo ./pi-blaster --gpio 4,17,18,27,21,10,23,24,25 ")

    
    initDefaultServoPos()
    
    ps4 = PS4Controller()
    ps4.init()
    
    while True:
        ps4.manualMode()
        ps4.autoMode()