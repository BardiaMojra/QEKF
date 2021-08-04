#!/usr/bin/env python

import sys, time
import numpy as np
import glob, os 											# to get image file names

import cv2 as cv

import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

class image_publisher:
    def __init__(self, video_src, video_metadata):
        self.PublisherComp = rospy.Publisher("/input/image_raw/compressed", CompressedImage, queue_size = 10)
        self.PublisherUncomp = rospy.Publisher("/input/image_raw/uncompressed", Image, queue_size = 10)
        rospy.init_node("image_publisher", anonymous=True)
        self.rate = rospy.Rate(2)
        self.video_dir = video_src
        self.current_frame = 0
        
        self.bridge = CvBridge()
        
        # get image file list from directory
        self.images = []
        print(video_src)
        os.chdir(video_src)
        for file in glob.glob("*.png"):
            self.images.append(file)
        self.images.sort()
        
        # get timestamps
        self.timestamps = []
        data = open(video_metadata)
        FirstLine = True 									# for airsim_rec
        for line in data:
            # YYYY-MM-DD HH:MM:SS.sssssssss
            # Convert    HH:MM:SS.sssssssss to seconds and set average difference to the time difference between frames
            # Should make this more precise later
            # ~ self.timestamps.append((float(line[11:13])*60 + float(line[14:16]))*60 + float(line[17:]))
            
            # for airsim_rec
            if FirstLine:
				FirstLine = False
            else:
				self.timestamps.append((float(line[0:13])))
        data.close()
    
    # Can add logic for getting timestamps from the actual video stream here
    # For now having this as a seperate function is not useful
    def get_timestamp(self, frame_idx):
        try:
            print(self.timestamps[frame_idx])
            return self.timestamps[frame_idx]
        except IndexError:
            return -1
            
    def run(self):
        self.rate.sleep()
        while not rospy.is_shutdown():
            self.rate.sleep()
            # ~ self.rate.sleep()
            # ~ self.rate.sleep()
            # ~ self.rate.sleep()
            # ~ self.rate.sleep()
            
            # ~ frame = cv.imread(self.video_dir + "%010d.png" % self.current_frame, flags=cv.IMREAD_COLOR)
            frame = cv.imread(self.video_dir + self.images.pop(0), flags=cv.IMREAD_COLOR)
            cv.imshow('cv_img', frame)
            cv.waitKey(1)
            if frame is None:
                break
            MsgUncomp = Image()
            MsgComp = CompressedImage()
            
            try:
                MsgUncomp = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            except CvBridgeError as e:
                rospy.logwarn(e)
            
            MsgComp.format = "png"
            try:
				MsgComp.data = np.array(cv.imencode(".png", frame)[1]).tostring()
            except CvBridgeError as e:
                rospy.logwarn(e)
            
            # ~ timestamp = self.get_timestamp(self.current_frame)
            # ~ if timestamp >=0:
                # ~ MsgUncomp.header.stamp.nsecs = int((timestamp % 1)*10**9)
                # ~ MsgUncomp.header.stamp.secs = int(timestamp)
                # ~ MsgComp.header.stamp.nsecs = int((timestamp % 1)*10**9)
                # ~ MsgComp.header.stamp.secs = int(timestamp / 1)
            # ~ else:
                # ~ break
            
            try:
                self.PublisherUncomp.publish(MsgUncomp)
                self.PublisherComp.publish(MsgComp)
            except Exception as ex:
                rospy.logwarn(ex)
            self.current_frame += 1
            

def main():
    #source = "/home/developer/Desktop/2011_09_26/2011_09_26_drive_0009_sync/image_00/"
    #source = "/home/developer/Desktop/2011_10_03/2011_10_03_drive_0042_sync/image_00/"
    source = rospy.get_param("video_source")
    mp = image_publisher(source+"data/", source+"timestamps.txt")
    mp.run()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

