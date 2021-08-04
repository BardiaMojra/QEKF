#!/usr/bin/env python

# things added and changed by Cody
## sub to raw images
## custom messages
## "AT THIS POINT" publishing 


# Python packages
import sys, time
import numpy as np
# ~ from scipy.interpolate import CubicSpline

# OpenCV packages
import cv2 as cv
import video
# ~ from common import anorm2, draw_str

# ROS packages
import rospy
import roslib
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32							# to pack output msg
from vbme_pkg.msg import Pixel2DFlow_msg 				 		# to pack output msg
from vbme_pkg.msg import FlowMatchList_msg 						# ouput msg
# ~ from vbme_pkg.msg import m_mdot
# ~ from std_msgs.msg import MultiArrayDimension as mdim
from cv_bridge import CvBridge, CvBridgeError

lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
# count=10 eps=0.03 was defaut

feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )


class App:
    def __init__(self):
        
        # Optical Flow stuff
        # ~ self.track_len = 10 
        self.track_len = rospy.get_param("track_len")
        self.track_limit = 75
        self.detect_interval = 5 #rospy.get_param("detect_interval")
        self.tracks = []
        self.frame_idx = 0 # maybe use sequence number from CompressedImage message
        #self.K_00 = np.array([[984, 0, 696],[0, 981, 256],[0, 0, 1]])
        # ~ self.K_00 = np.array([[980, 0, 690],[0, 974, 249],[0, 0, 1]]) 
        # ~ self.K_00 = np.array(rospy.get_param("/camera_calibration")).reshape(3,3)
        # ~ self.poly_limit = int(rospy.get_param("/poly_limit"))
        # ~ self.poly_limit = 5
        # ~ self.inner_threshold = 0.2
        # ~ self.outer_threshold = 10
        #self.video_type = video_type
       

        # ROS stuff
        self.bridge = CvBridge()
        self.epsilon = 0.01
        self.image_queue = []
        self.timestamps = []
        self.subscriber = rospy.Subscriber("/input/image_raw/image", Image, self.callbackRaw, queue_size = 3)
        self.subscriber = rospy.Subscriber("/input/image_raw/compressed", CompressedImage, self.callbackCompressed, queue_size = 3)
        #self.subscriber = rospy.Subscriber("/optical_flow_group/image_pub/image_raw/compressed", CompressedImage, self.callback, queue_size = 3)
        # ~ self.publisher = rospy.Publisher("/output/optical_flow_vectors", m_mdot, queue_size = 3)
        self.publisher = rospy.Publisher("optical_flow_short", FlowMatchList_msg, queue_size = 100)
        rospy.init_node("opticFlow", anonymous = True)
        rospy.spin()

    # ~ def optical_vector(self, x, y, t):
        # ~ size = len(t)
        # ~ bx = x
        # ~ by = y

        # ~ uv = np.stack((bx, by, np.ones(len(bx))), axis=0)
        # ~ assert(uv.shape == (3, size))
        # ~ xy = np.matmul(np.linalg.inv(self.K_00), uv)
        # ~ xy_curr = (xy[:,-1]).reshape(3,1)
        # ~ assert(xy.shape == (3, size))
        # ~ bx = xy[0,:]
        # ~ by = xy[1,:]

        # ~ #if False:
        # ~ #    pass
        # ~ #else:
        # ~ if size == 2 or self.poly_limit == 1:
            # ~ xx = bx[-2:]
            # ~ yy = by[-2:]
            # ~ tt = t[-2:]
            
            # ~ A = np.stack((tt, np.ones(2)), axis=1)
            # ~ assert(A.shape == (2,2))
            
            # ~ ax, bx = np.linalg.lstsq(A, xx)[0]
            # ~ ay, by = np.linalg.lstsq(A, yy)[0]
            # ~ mx = ax
            # ~ my = ay
                
        # ~ elif size == 3 or self.poly_limit == 2:
            # ~ xx = bx[-3:]
            # ~ yy = by[-3:]
            # ~ tt = t[-3:]
            
            # ~ A = np.stack((np.power(tt, 2), tt, np.ones(3)), axis=1)
            
            # ~ ax, bx, cx = np.linalg.lstsq(A, xx)[0]
            # ~ ay, by, cy = np.linalg.lstsq(A, yy)[0]
            
            # ~ mx = 2*ax*t[-1] + bx
            # ~ my = 2*ay*t[-1] + by

        # ~ elif size == 4 or self.poly_limit == 3:
            # ~ xx = bx[-4:]
            # ~ yy = by[-4:]
            # ~ tt = t[-4:]
            
            # ~ A = np.stack((np.power(tt, 3), np.power(tt, 2), tt, np.ones(4)), axis=1)
            
            # ~ ax, bx, cx, dx = np.linalg.lstsq(A, xx)[0]
            # ~ ay, by, cy, dy = np.linalg.lstsq(A, yy)[0]
            
            # ~ mx = 3*ax*t[-1]**2 + 2*bx*t[-1] + cx
            # ~ my = 3*ay*t[-1]**2 + 2*by*t[-1] + cy
        
        # ~ elif size == 5 or self.poly_limit == 4:
            # ~ xx = bx[-5:]
            # ~ yy = by[-5:]
            # ~ tt = t[-5:]
            
            # ~ A = np.stack((np.power(tt, 4), np.power(tt, 3), np.power(tt, 2), tt, np.ones(5)), axis=1)
            
            # ~ ax, bx, cx, dx, ex = np.linalg.lstsq(A, xx)[0]
            # ~ ay, by, cy, dy, ey = np.linalg.lstsq(A, yy)[0]
            
            # ~ mx = 4*ax*t[-1]**3 + 3*bx*t[-1]**2 + 2*cx*t[-1] + dx
            # ~ my = 4*ay*t[-1]**3 + 3*by*t[-1]**2 + 2*cy*t[-1] + dy
        
        # ~ else:
            # ~ xx = bx[-6:]
            # ~ yy = by[-6:]
            # ~ tt = t[-6:]
            
            # ~ A = np.stack((np.power(tt, 5), np.power(tt, 4), np.power(tt, 3), np.power(tt, 2), tt, np.ones(6)), axis=1)
            
            # ~ ax, bx, cx, dx, ex, fx = np.linalg.lstsq(A, xx)[0]
            # ~ ay, by, cy, dy, ey, fy = np.linalg.lstsq(A, yy)[0]
            
            # ~ mx = 5*ax*t[-1]**4 + 4*bx*t[-1]**3 + 3*cx*t[-1]**2 + 2*dx*t[-1] + ex
            # ~ my = 5*ay*t[-1]**4 + 4*by*t[-1]**3 + 3*cy*t[-1]**2 + 2*dy*t[-1] + ey
        
        # ~ #else:
        # ~ #    xy_prev = (xy[:,-2]).reshape(3,1)
        # ~ #    xy_delta = np.divide((xy_curr - xy_prev),(t[-1]-t[-2]))
        # ~ #    mx = xy_delta[0,0]
        # ~ #    my = xy_delta[1,0]
        
        # ~ xy_vel = np.array([[mx],[my],[0]])
        
        
        # ~ return (xy_vel, xy_curr)

    def callbackCompressed(self, ros_data):
        # Get image data into opencv format
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        
        self.HandleImage(ros_data, image_np)
        
    def callbackRaw(self, ros_data):
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, "passthrough")
        self.HandleImage(ros_data, cv_image)
        
    def HandleImage(self, ros_data, image_np):
        # Add most recent image and timestamp to the end of queues
        # queue structure might not be necessary if there is an easier way to make sure 
        # frames are processed in the correct order.
        self.image_queue.append(image_np)

        # combine second and nanosecond times into a single float
        time = float(ros_data.header.stamp.secs) + float(ros_data.header.stamp.nsecs) / (10**9)
        self.timestamps.append(time)
        
        frame = None
        
        if len(self.image_queue) > 0:
            #frame = self.image_queue[self.frame_idx]
            frame = self.image_queue.pop(0)
        if frame is None:
            return
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        vis = frame.copy()

        if len(self.tracks) > 0:
            img0, img1 = self.prev_gray, frame_gray
            # reshape(-1, 1, 2) makes a matrix of the structure

            # [[x1, y1],
            #  [x2, y2],
            #  [x3, y3],
            #  [x4, y4],
            #  ...
            #  [xn, yn]]

            # where n is derived by the numpy library (hence the -1)
            # p0 is the most recently added point for each track
            p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)

            # p1 is the new location of the tracked features with _st being a vector of unsigned chars that represent 
            # if the new location was successfully found, and _err is a vector of floats representing the error of each output vector
            p1, _st, _err = cv.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)

            # p0r = p0 reversed, does the backwards optical flow to filter out inconsistant features
            p0r, _st, _err = cv.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)

            # takes the largest difference in x or y between the forward and backward optical flow
            d = abs(p0-p0r).reshape(-1, 2).max(-1)

            # turns the vector of differences into a vector of booleans based on the condition d < 1
            good = d < 1
            
            new_tracks = []
            for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                tr.append((x, y))
                if len(tr) > self.track_len:
                    del tr[0]
                new_tracks.append(tr)
                cv.circle(vis, (x, y), 2, (0, 255, 0), -1)
            self.tracks = new_tracks

            ########################################################################################################################################################
            #
            #
            #     AT THIS POINT, SELF.TRACKS HAS THE LIST OF FEATURE POINTS THAT VELEST WILL USE.
            # ~ print()
            # ~ print(len(self.tracks))
            # ~ for i in self.tracks:
				# ~ print(len(i))
				# ~ print (i)
            # ~ self.publisher.publish(self.tracks)
            MsgOut = FlowMatchList_msg()
            MsgOut.header = ros_data.header
            # ~ MsgOut.resize(len(self.tracks))
            for PixFlow in self.tracks:
				PixArray = Pixel2DFlow_msg()
				for Pixs in PixFlow:
					 Pix = Point32()
					 Pix.x = Pixs[0]
					 Pix.y = Pixs[1]
					 PixArray.FlowArray.append(Pix)
				MsgOut.Points.append(PixArray)
            # ~ print(MsgOut)
            self.publisher.publish(MsgOut)
            #
            #
            #
            #
            #
            #
            #
            #
            #########################################################################################################################################################



            # This draws the trails "tracks" behind the feature points based on the past feature points
            # Seems like finding the line that is the least squares solution to the past n points in the trail
            # might be a good way to approximate the velocity of the feature point...
            # Alternatively, fitting a curve and finding the gradient at the most recent point could be more useful...

            # let dt = difference in time between frames
            # fit a curve (cubic spline has some assumptions that makes this annoying) to x(t) and y(t) based on the x and y values in the track
            # find dx/dt and dy/dt based on these curves
            # calculate the gradient vector at the (x,y) point of interest
            # Use the L2 norm to get a speed and 
            # Normalize the vector to get a direction
            # Draw an arrow to the video that points in the direction and whose length is proportional to the speed
            
        if self.frame_idx % self.detect_interval == 0:
            mask = np.zeros_like(frame_gray)
            mask[:] = 255
            for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                # looks in a 5 pixel radius around the last feature point added to each track for the next feature point
                # Increasing this radius might be necessary for faster moving objects... 
                # Alternatively, making the detect interval more frequent could have a similar effect...
                cv.circle(mask, (x, y), 5, 0, -1)

            # mask provides a region of interest that says where to look for the feature points
            p = cv.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    # When the number of tracks gets too large, lk_track_ros.py cannot
                    # keep up with the 30 Hz publishing rate of the camera, so we limit the number of tracks
                    # by favoring tracks with the longest lengths over newer tracks.
                    # If we wanted to favor pixels near the boundaries of the image we would do so here.
                    if len(self.tracks) > self.track_limit:
                        continue
                    self.tracks.append([(x, y)])


        self.frame_idx += 1
        self.prev_gray = frame_gray
        
        


def main():
    node = App()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

