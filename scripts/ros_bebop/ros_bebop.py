#!/usr/bin/python
from bebop import *
#import navdata
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty, Float64, Int64
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
import cvideo
import numpy
import cv2
#from multiprocessing import Process, Queue
import threading
import Queue
import sys
import signal
import time
running = False
def signal_handler(signal,frame):
    global running
    print "Triggered"
    running = False
class ROSBebop:
    def __init__(self, machinename='drone1', ip='192.168.42.1', navdata_port = 43210,port=44444):
        self.thread = None
        self.moveSub = rospy.Subscriber('/'+machinename+'/cmd_vel', Twist, self.moveCallback)
        self.takeoffSub = rospy.Subscriber('/'+machinename+'/takeoff', Empty, self.takeoffCallback)
        self.landSub = rospy.Subscriber('/'+machinename+'/land', Empty, self.landCallback)
        self.panSub = rospy.Subscriber('/'+machinename+'/pan', Int64, self.panCallback)
        self.tiltSub = rospy.Subscriber('/'+machinename+'/tilt', Int64, self.tiltCallback)
        self.emergSub = rospy.Subscriber('/'+machinename+'/emergency', Empty, self.emergCallback)
        self.altitudePub = rospy.Publisher('/'+machinename+'/altitude', Float64, queue_size=10)
        self.batteryPub = rospy.Publisher('/'+machinename+'/battery', Int64, queue_size=10)
        self.imagePub = rospy.Publisher('/'+machinename+'/image_raw', Image, queue_size=10)# but not raw
        #self.imagePub = rospy.Publisher('/'+machinename+'/front/image_raw', Image, queue_size=10)# but not raw
        self.speedPub = rospy.Publisher('/'+machinename+'/speed', Vector3, queue_size=10)
        self.positionPub = rospy.Publisher('/'+machinename+'/position', Vector3, queue_size=10)
        self.anglePub = rospy.Publisher('/'+machinename+'/angle', Vector3, queue_size=10)
        self.gpsPub = rospy.Publisher('/'+machinename+'/positionGPS', Vector3, queue_size=10)
        self.bridge = CvBridge()
        self.drone = Bebop(ip,navdata_port,onlyIFrames=False)
        self.twist = Twist()
        self.tilt = 0
        self.pan = 0
        self.drone.videoCbk = self.videoCallback
        self.drone.videoEnable()
        self.queueOut = None
        self.processor= None
        self.queue = None
        self.machinename = machinename
        self.ip = ip
        self.port = port
        self.is_emergency = False

    def loop(self):
        global running
        r = rospy.Rate(40)
        self.drone.update(cmd=trimCmd())
        self.publish()
        while running:
            if self.is_emergency:
                print "emergency!"
                self.drone.update(cmd=landCmd())
                self.drone.update(cmd=emergencyCmd())
                continue
            r.sleep()
            try:
                if (self.twist.linear.y == 0) and (self.twist.linear.x == 0) and (self.twist.angular.z == 0) and (self.twist.linear.z == 0) :
                    self.drone.update(cmd=movePCMDCmd(False, 0,0,0,0))
                else:
                    self.drone.update(cmd=movePCMDCmd(True,
                                                      self.twist.linear.y*(-50), #roll
                                                      self.twist.linear.x*50,    #pitch
                                                      self.twist.angular.z*(-50),#yaw
                                                      self.twist.linear.z*50))   #up,down
            except :
                continue
            self.publish()
        rospy.signal_shutdown("shutdown")
        if self.queue is not None:
            self.queue.put(None)
            self.thread.join()
    def takeoffCallback(self, msg):
        self.drone.update(cmd=takeoffCmd())
        self.publish()
    def landCallback(self, msg):
        self.drone.update(cmd=landCmd())
        self.publish()
    def moveCallback(self, msg):
        self.twist = msg
    def tiltCallback(self, msg):
        self.tilt = msg.data
        self.drone.update(cmd=moveCameraCmd(tilt=self.tilt, pan=self.pan))
        self.publish()
    def panCallback(self, msg):
        self.pan = msg.data
        self.drone.update(cmd=moveCameraCmd(tilt=self.tilt, pan=self.pan))
        self.publish()
    def emergCallback(self, msg):
        print "emergency called"
        self.is_emergency = True
        self.moveSub.unregister()
        self.takeoffSub.unregister()
        self.landSub.unregister()
        self.panSub.unregister()
        self.tiltSub.unregister()
        self.emergSub.unregister()
    def publish(self):
        self.altitudePub.publish(self.drone.altitude)
        self.batteryPub.publish(self.drone.battery)
        self.positionPub.publish(Vector3(self.drone.position[0],self.drone.position[1],self.drone.position[2]))
        self.speedPub.publish(Vector3(self.drone.speed[0],self.drone.speed[1],self.drone.speed[2]))
        self.anglePub.publish(Vector3(self.drone.roll,self.drone.pitch,self.drone.yaw))
        if self.drone.positionGPS != None:
            self.gpsPub.publish(Vector3(self.drone.positionGPS[0], self.drone.positionGPS[1], self.drone.positionGPS[2]))
    def videoCallback(self, frame, robot=None, debug=False):
        if self.thread is None:
            self.queue = Queue.Queue()
            self.thread = threading.Thread(target=self.imageProcess, name="image publisher", args=(self.queue,))
            self.thread.start()
        self.queue.put(frame)
    def imageProcess(self,queue):
        cvideo.init()
        img = numpy.zeros([360, 640, 3], dtype=numpy.uint8)
        while not rospy.is_shutdown():
            frame = queue.get()
            if frame is None:
                break
            ret = cvideo.frame(img, frame[1], frame[2])
            if not ret:
                continue
            assert ret
            self.imagePub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))


def main(name ,ip, navdata):
    global running
    running = True
    rosbebop = ROSBebop(name,ip,navdata)
    rosbebop.loop()
if __name__ == "__main__":
    rospy.init_node(sys.argv[1]+'_controller', anonymous=True,disable_signals=True)    
    driver_thread = threading.Thread(target=main,args=(sys.argv[1],sys.argv[2],int(sys.argv[3])))
    signal.signal(signal.SIGINT,signal_handler)
    driver_thread.start()
    while driver_thread.isAlive():
        time.sleep(0.01)
    driver_thread.join()

    
