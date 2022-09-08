import threading
from time import sleep
import os
import RPi.GPIO as IO
import time

IO.setwarnings(False)
IO.setmode(IO.BCM)
triggerPin = 13
echoPin = 26

sensorLeft = 0.04
sensorCenter = 0.07
sensorRight = 0.08



class ObstacleAvoidance(object):
   
    def __init__(self, servoPin, in1Pin, in2Pin, in3Pin, in4Pin):
        
        
        self.servoPin = servoPin
        
        self.in1Pin = in1Pin
        self.in2Pin = in2Pin
        self.in3Pin = in3Pin
        self.in4Pin = in4Pin
        
        self.loop = False
        self.reading = 0

        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True                            # Daemonize thread
        thread.start()                                  # Start the execution
    
    def setLoop(self,value):
        self.loop = value
    
    def getDistance(self):
        time.sleep(0.1)
        IO.output(triggerPin, 1)
        time.sleep(0.00001)
        IO.output(triggerPin, 0)
        
        while IO.input(echoPin) == 0:
            pass
        
        start = time.time()
        
        while IO.input(echoPin) == 1:
            pass
        
        stop = time.time()
        
        self.reading = round((stop - start)*17000,1)
        
        print(round((stop - start)*17000,1))
        
        return self.reading
    
    def backward(self):
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in1Pin,0))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in2Pin,0.6))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in3Pin,0))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in4Pin,0.6))
        print("backward")
    
    def stop(self):
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in1Pin,0))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in2Pin,0))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in3Pin,0))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in4Pin,0))
        print("Stop")

    def forward(self):
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in1Pin,0.5))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in2Pin,0))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in3Pin,0.5))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in4Pin,0))
        print("Forward")
    
    def right(self):
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in1Pin,0.8))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in2Pin,0))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in3Pin,0.3))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in4Pin,0))
        print("Right")
    
    def left(self):
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in1Pin,0.3))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in2Pin,0))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in3Pin,0.8))
        os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.in4Pin,0))
        print("Left")
        
    def collisionAvoidance(self):
        while True:

            if not self.loop:
                break
                
            self.backward()
            sleep(1.5)
            self.stop()
            print("STOP SUCCESS")
            
            if not self.loop:
                break            
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.servoPin,sensorLeft))
            sleep(0.75)
            
            leftDistance = self.getDistance()
            
            print("TURN1 SUCCESS")
            
            if not self.loop:
                break
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.servoPin,sensorRight))
            sleep(0.75)
            
            
            rightDistance = self.getDistance()
            print("TURN2 SUCCESS")
            
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.servoPin,sensorCenter))
            sleep(0.75)
            
            print("TURN3 SUCCESS")
            
            if not self.loop:
                break
                        
            if rightDistance > leftDistance:
                print("right > left")
                self.right()
                sleep(0.75)
                break
                 
            else:
                print("left > right")
                self.left()
                sleep(0.75)
                break
            
    def moveServoandCheckDistance(self,value):
        
#         print(self.ultrasonicSensor.reading)
        
        if self.loop:
            
            self.forward()
        
            os.system("echo ""{}={}"" > /dev/pi-blaster".format(self.servoPin,value))
            sleep(0.2)
            
            if self.getDistance() < 35:
                
                self.collisionAvoidance()

    def run(self):
        
        while True:
            
            if self.loop:
    
                self.moveServoandCheckDistance(sensorLeft)
                self.moveServoandCheckDistance(sensorCenter)
                self.moveServoandCheckDistance(sensorRight)
                self.moveServoandCheckDistance(sensorCenter)
