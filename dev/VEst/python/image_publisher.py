#!/usr/bin/env python

import sys, time
import numpy as np

import cv2 as cv

import roslib
import rospy
from sensor_msgs.msg import CompressedImage

class image_publisher:
    def __init__(self, source):
        video_src = source + "data/"
        video_metadata = source + "timestamps.txt"

        self.publisher = rospy.Publisher("/input/image_raw/compressed", CompressedImage, queue_size = 3)
        rospy.init_node("image_publisher", anonymous=True)
        self.hertz = int(rospy.get_param("hertz"))
        self.rate = rospy.Rate(self.hertz)
        self.video_dir = video_src
        self.current_frame = 0
        
        # ~ self.kitti_data = False
        # ~ self.missile_data = False
        # ~ self.mp4_data = True
        self.kitti_data = rospy.get_param("/optical_flow_group/image_pub/kitti_data")
        self.missile_data = rospy.get_param("/optical_flow_group/image_pub/missile_data")
        self.mp4_data = rospy.get_param("/optical_flow_group/image_pub/mp4_data")
        self.files = []

        self.timestamps = []
        data = open(video_metadata)
        for line in data:
            # YYYY-MM-DD HH:MM:SS.sssssssss
            # Convert    HH:MM:SS.sssssssss to seconds and set average difference to the time difference between frames
            if self.kitti_data:
                self.timestamps.append((float(line[11:13])*60 + float(line[14:16]))*60 + float(line[17:]))
            elif self.missile_data:
                self.timestamps.append(float(line)/(10**9))
            elif self.mp4_data:
                self.timestamps.append((float(line[11:13])*60 + float(line[14:16]))*60 + float(line[17:]))
        data.close()

        if self.missile_data:
            f = open(source + "files.txt", "r")
            for line in f:
                self.files.append(line.replace("\n", ""))
            f.close()

    # Can add logic for getting timestamps from the actual video stream here
    # For now having this as a seperate function is not useful
    def get_timestamp(self, frame_idx):
        try:
            return self.timestamps[frame_idx]
        except IndexError:
            return -1

    def run(self):
        while not rospy.is_shutdown():
            if self.kitti_data:
                frame = cv.imread(self.video_dir + "%010d.png" % self.current_frame, flags=cv.IMREAD_COLOR)
            
            elif self.missile_data:
                if len(self.files) > self.current_frame:
                    frame = cv.imread(self.video_dir + self.files[self.current_frame], flags=cv.IMREAD_COLOR)
                else:
                    frame = None

            elif self.mp4_data:
                frame = cv.imread(self.video_dir + "frame%04d.png" % self.current_frame, flags=cv.IMREAD_COLOR)


            if frame is None:
                break
            msg = CompressedImage()
            timestamp = self.get_timestamp(self.current_frame)
            if timestamp >=0:
                msg.header.stamp.nsecs = int((timestamp % 1)*10**9)
                msg.header.stamp.secs = int(timestamp / 1)
            else:
                break
            msg.format = "png"
            msg.data = np.array(cv.imencode(".png", frame)[1]).tostring()
            self.publisher.publish(msg)
            self.current_frame += 1
            self.rate.sleep()
            # ~ self.rate.sleep()
            # ~ self.rate.sleep()
            # ~ self.rate.sleep()
            # ~ self.rate.sleep()
            # ~ self.rate.sleep()
            # ~ self.rate.sleep()
            # ~ self.rate.sleep()
            # ~ self.rate.sleep()
            # ~ self.rate.sleep()

def main():
	# get name space from param server 
    source = rospy.get_param("video_source")
    rospy.loginfo('Video Source: %s' %(source))
    print('Video Source: %s' %(source))
    mp = image_publisher(source)
    mp.run()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

