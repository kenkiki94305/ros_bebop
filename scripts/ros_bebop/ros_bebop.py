#!/usr/bin/python
from bebop import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty, Float64, Int64
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import Image, NavSatFix
import cvideo,tf,numpy,cv2
#from multiprocessing import Process, Queue
import threading
import Queue
import sys
import signal
import time

running = False
def signal_handler(signal,frame):
    global running
    print "Quit Triggered"
    running = False
class ROSBebop:
    def __init__(self):
        self.unique_id = rospy.get_param('~unique_id','bebop_1')
        ip = rospy.get_param('~host_address','192.168.11.142')
        navdata_port = rospy.get_param('~port',22222)
        speed_limits = (rospy.get_param('~linear_speed_limit',11.1),rospy.get_param('~angular_speed_limit',60.0))
        self.max_altitude = rospy.get_param('~altitude_limit',5.0)
        self.improc_thread = None
        self.bridge = CvBridge()
        print speed_limits
        self.drone = Bebop(ip,navdata_port,speed_limits,onlyIFrames=False)
        self.twist = Twist()
        self.tilt =0
        self.pan = 0
        self.drone.videoCbk = self.videoCallback
        self.drone.videoEnable()
        self.queue = None
        self.is_emergency = False
        self.initTopics()
        self.hovering = True
    def initTopics(self):
        self.move_sub = rospy.Subscriber('/'+self.unique_id+'/cmd_vel', Twist, self.moveCallback)
        self.takeoff_sub = rospy.Subscriber('/'+self.unique_id+'/takeoff', Empty, self.takeoffCallback)
        self.land_sub = rospy.Subscriber('/'+self.unique_id+'/land', Empty, self.landCallback)
        self.pan_sub = rospy.Subscriber('/'+self.unique_id+'/pan', Int64, self.panCallback)
        self.tilt_sub = rospy.Subscriber('/'+self.unique_id+'/tilt', Int64, self.tiltCallback)
        self.emergency_sub = rospy.Subscriber('/'+self.unique_id+'/emergency', Empty, self.emergCallback)
        
        self.altitude_pub = rospy.Publisher('/'+self.unique_id+'/altitude', Float64, queue_size=10)
        self.battery_pub = rospy.Publisher('/'+self.unique_id+'/battery', Int64, queue_size=10)
        self.image_pub = rospy.Publisher('/'+self.unique_id+'/image_raw', Image, queue_size=10)
        self.speed_pub = rospy.Publisher('/'+self.unique_id+'/speed', Vector3, queue_size=10)
        self.gps_pub = rospy.Publisher('/'+self.unique_id+'/positionGPS', NavSatFix, queue_size=10)
        self.pose_pub = rospy.Publisher('/'+self.unique_id+'/pose',Pose,queue_size=10)

    def fini(self):
        self.move_sub.unregister()
        self.takeoff_sub.unregister()
        self.land_sub.unregister()
        self.pan_sub.unregister()
        self.tilt_sub.unregister()
        self.emergency_sub.unregister()
        rospy.signal_shutdown("shutdown")
    def loop(self):
        global running
        r = rospy.Rate(40)
        self.drone.update(cmd=trimCmd())
        self.setMaxAltitude()
        self.publish_all()
        while running:
            if self.is_emergency:
                print "emergency!"
                self.drone.update(cmd=landCmd())
                self.drone.update(cmd=emergencyCmd())
                continue
            r.sleep()
            try:
                self.drone.update(cmd=movePCMDCmd(not self.hovering,
                                                  self.twist.linear.y*(-50), #roll
                                                  self.twist.linear.x*50,    #pitch
                                                  self.twist.angular.z*(-50),#yaw
                                                  self.twist.linear.z*50))   #up,down
            except :
                continue
            self.publish_all()
        if self.queue is not None:
            self.queue.put(None)
            self.improc_thread.join()
        self.fini()
    def takeoffCallback(self, msg):
        self.drone.update(cmd=takeoffCmd())
        self.publish_all()
    def landCallback(self, msg):
        self.drone.update(cmd=landCmd())
        self.publish_all()
    def setMaxAltitude(self):
        self.drone.update(cmd=maxAltitudeCmd(self.max_altitude))
    def moveCallback(self, msg):
        self.twist = msg
        self.hovering =  self.twist.linear.y == 0 and self.twist.linear.x == 0 and self.twist.angular.z == 0 and self.twist.linear.z == 0 
    def tiltCallback(self, msg):
        self.tilt = msg.data
        self.drone.update(cmd=moveCameraCmd(tilt=self.tilt, pan=self.pan))
        self.publish_all()
    def panCallback(self, msg):
        self.pan = msg.data
        self.drone.update(cmd=moveCameraCmd(tilt=self.tilt, pan=self.pan))
        self.publish_all()
    def emergCallback(self, msg):
        print "emergency called"
        self.is_emergency = True
    def publish_all(self):
        self.publish_altitude()
        self.publish_battery()
        self.publish_pose()
        self.publish_gps()
        self.publish_speed()
    def publish_altitude(self):
        self.altitude_pub.publish(self.drone.altitude)
    def publish_battery(self):
        self.battery_pub.publish(self.drone.battery)
    def publish_pose(self):
        pose = Pose()
        pose.position.x = self.drone.position[0]
        pose.position.y = -self.drone.position[1]
        pose.position.z = -self.drone.position[2]
        quat = tf.transformations.quaternion_from_euler(self.drone.roll,self.drone.pitch,-self.drone.yaw)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.pose_pub.publish(pose)
#        print "roll:"+str(self.drone.roll)# for debug
#        print "pitch:"+str(self.drone.pitch)
#        print "yaw:"+str(-self.drone.yaw)
#        print "---"
        
    def publish_gps(self):
        if self.drone.positionGPS!=None:
           gps_data = NavSatFix()
           gps_data.latitude = self.drone.positionGPS[0]
           gps_data.longitude = self.drone.positionGPS[1]#maybe reversed?
           gps_data.altitude = self.drone.positionGPS[2]
           gps_data.header.frame_id = self.unique_id
           self.gps_pub.publish(gps_data)
    def publish_speed(self):
        self.speed_pub.publish(Vector3(self.drone.speed[0],-self.drone.speed[1],-self.drone.speed[2]))
    def videoCallback(self, frame, robot=None, debug=False):
        if self.improc_thread is None:
            self.queue = Queue.Queue()
            self.improc_thread = threading.Thread(target=self.imageProcess, name="image publisher", args=(self.queue,))
            self.improc_thread.start()
        self.queue.put(frame)
    def imageProcess(self,queue):
        global running
        cvideo.init()
        img = numpy.zeros([360, 640, 3], dtype=numpy.uint8)
        while running:
            frame = queue.get()
            if frame is None:
                break
            ret = cvideo.frame(img, frame[1], frame[2])
            if not ret:
                continue
            assert ret
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))


def main():
    global running
    running = True
    rosbebop = ROSBebop()
    rosbebop.loop()
if __name__ == "__main__":
    rospy.init_node('bebop_driver', anonymous=True,disable_signals=True)    
    driver_thread = threading.Thread(target=main)
    signal.signal(signal.SIGINT,signal_handler)
    driver_thread.start()
    while driver_thread.isAlive():
        time.sleep(0.01)
    driver_thread.join()

    
