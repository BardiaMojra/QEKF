#!/usr/bin/env python

# Python packages
import sys, os, time
import random
import numpy as np
from scipy.interpolate import CubicSpline

# OpenCV packages
import cv2 as cv
from cv2 import optflow
# import video works well if it is install properly
try:
	import video                           
except ImportError as IE1:
	try:
		# import include copy of video if it is not installed properly
		from helpers.video import *
	except ImportError as IE2:
		print(IE1)
		print(IE2)
		exit(1)
try:
	from common import anorm2, draw_str                        
except ImportError as IE1:
	try:
		from helpers.common import anorm2, draw_str
	except ImportError as IE2:
		print(IE1)
		print(IE2)
		exit(1)

# ROS packages
import rospy
import roslib
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from std_msgs.msg import MultiArrayDimension as mdim
from geometry_msgs.msg import Point32

from vbme_msgs.msg import m_mdot
from vbme_msgs.msg import Pixel2DFlow_msg
from vbme_msgs.msg import OpticalFlow_msg

lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )


class App:
    def __init__(self):
        
        # Optical Flow stuff
        self.track_len = 1000 #rospy.get_param("track_len")
        self.track_limit = 75
        self.detect_interval = 5 #rospy.get_param("detect_interval")
        self.tracks = []
        self.frame_idx = 0 # maybe use sequence number from CompressedImage message
        self.K_00 = np.array(rospy.get_param("camera_calibration")).reshape(3,3)
        self.poly_limit = int(rospy.get_param("poly_limit"))

        # options, should be put on param server maybe
        self.showImages = False
        self.save_images = True

        # setup a folder for output images
        self.output_path = rospy.get_param("/output_path")
        new_dir = os.path.join(self.output_path, "feature_images")        
        if not os.path.exists(new_dir) and self.save_images:
            os.mkdir(new_dir)
        
        # These are for defining a region away from the center of the image
        # not currently in use.
        self.inner_threshold = 0.2
        self.outer_threshold = 10

        # ROS stuff
        self.epsilon = 0.01
        self.image_queue = []
        self.timestamps = []
        self.subscriber = rospy.Subscriber("/input/image_raw/compressed", CompressedImage, self.callback, queue_size = 3)
        self.publisher = rospy.Publisher("/vbme/optical_flow", OpticalFlow_msg, queue_size = 3)
        
        rospy.init_node("opticFlow", anonymous = True)
        rospy.spin()


    def optical_vector(self, x, y, t):
        size = len(t)
        bx = x
        by = y

        uv = np.stack((bx, by, np.ones(len(bx))), axis=0)
        assert(uv.shape == (3, size))
        xy = np.matmul(np.linalg.inv(self.K_00), uv)
        xy_curr = (xy[:,-1]).reshape(3,1)
        assert(xy.shape == (3, size))
        bx = xy[0,:]
        by = xy[1,:]

        if size == 2 or self.poly_limit == 1:
            xx = bx[-2:]
            yy = by[-2:]
            tt = t[-2:]
            
            A = np.stack((tt, np.ones(2)), axis=1)
            assert(A.shape == (2,2))
            
            ax, bx = np.linalg.lstsq(A, xx)[0]
            ay, by = np.linalg.lstsq(A, yy)[0]
            mx = ax
            my = ay
                
        elif size == 3 or self.poly_limit == 2:
            xx = bx[-3:]
            yy = by[-3:]
            tt = t[-3:]
            
            A = np.stack((np.power(tt, 2), tt, np.ones(3)), axis=1)
            
            ax, bx, cx = np.linalg.lstsq(A, xx)[0]
            ay, by, cy = np.linalg.lstsq(A, yy)[0]
            
            mx = 2*ax*t[-1] + bx
            my = 2*ay*t[-1] + by

        elif size == 4 or self.poly_limit == 3:
            xx = bx[-4:]
            yy = by[-4:]
            tt = t[-4:]
            
            A = np.stack((np.power(tt, 3), np.power(tt, 2), tt, np.ones(4)), axis=1)
            
            ax, bx, cx, dx = np.linalg.lstsq(A, xx)[0]
            ay, by, cy, dy = np.linalg.lstsq(A, yy)[0]
            
            mx = 3*ax*t[-1]**2 + 2*bx*t[-1] + cx
            my = 3*ay*t[-1]**2 + 2*by*t[-1] + cy
        
        elif size == 5 or self.poly_limit == 4:
            xx = bx[-5:]
            yy = by[-5:]
            tt = t[-5:]
            
            A = np.stack((np.power(tt, 4), np.power(tt, 3), np.power(tt, 2), tt, np.ones(5)), axis=1)
            
            ax, bx, cx, dx, ex = np.linalg.lstsq(A, xx)[0]
            ay, by, cy, dy, ey = np.linalg.lstsq(A, yy)[0]
            
            mx = 4*ax*t[-1]**3 + 3*bx*t[-1]**2 + 2*cx*t[-1] + dx
            my = 4*ay*t[-1]**3 + 3*by*t[-1]**2 + 2*cy*t[-1] + dy
        
        else:
            xx = bx[-6:]
            yy = by[-6:]
            tt = t[-6:]
            
            A = np.stack((np.power(tt, 5), np.power(tt, 4), np.power(tt, 3), np.power(tt, 2), tt, np.ones(6)), axis=1)
            
            ax, bx, cx, dx, ex, fx = np.linalg.lstsq(A, xx)[0]
            ay, by, cy, dy, ey, fy = np.linalg.lstsq(A, yy)[0]
            
            mx = 5*ax*t[-1]**4 + 4*bx*t[-1]**3 + 3*cx*t[-1]**2 + 2*dx*t[-1] + ex
            my = 5*ay*t[-1]**4 + 4*by*t[-1]**3 + 3*cy*t[-1]**2 + 2*dy*t[-1] + ey
        
        xy_vel = np.array([[mx],[my],[0]])
        
        
        return (xy_vel, xy_curr)

    def callback(self, ros_data):
        # Get image data into opencv format
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        
        # Add most recent image and timestamp to the end of queues
        # queue structure might not be necessary if there is an easier way to make sure 
        # frames are processed in the correct order.
        self.image_queue.append(image_np)

        # combine second and nanosecond times into a single float
        time = float(ros_data.header.stamp.secs) + float(ros_data.header.stamp.nsecs) / (10**9)
        self.timestamps.append(time)
        
        frame = None
        
        if len(self.image_queue) > 0:
            frame = self.image_queue.pop(0)
        if frame is None:
            cv.destroyAllWindows()
            return
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # add gauss smooth 
        # ~ frame_gray = cv.GaussianBlur(frame_gray,(5,5),0)
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
            good = d < 1.0
                
            new_tracks = []
            for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                tr.append((x, y))
                if len(tr) > self.track_len:
                    del tr[0]
                new_tracks.append(tr)
                cv.circle(vis, (x, y), 4, (0, 0, 180), -1)
            self.tracks = new_tracks

            # Get subpixel accuracy for the feature points that passed the forward-backward consistency check
            # This is done before using the camera calibration matrix
            #current_points = np.array([ x[-1] for x in self.tracks ])
            #criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.1)
            #current_points = cv.cornerSubPix(frame_gray, current_points, (5,5), (-1,-1), criteria)
            
            #for idx, point in enumerate(current_points):
            #    self.tracks[idx][-1] = point

            m = []
            m_dot = []
            
            for tr in self.tracks:
                size = len(tr)
                if size<2:
                    continue
                x = np.array([i[0] for i in tr])
                y = np.array([i[1] for i in tr])
                t = self.timestamps[self.frame_idx-size+1:self.frame_idx+1]
                
                t = np.array(t).reshape(-1,)
                xy_vel, xy_curr = self.optical_vector(x, y, t)
                
                radius = np.linalg.norm(xy_curr[:2], 2)
                
                if (radius > self.inner_threshold and radius < self.outer_threshold) or True:
                
                    try:
                        m = np.append(m, xy_curr, axis=1)
                    except:
                        m = xy_curr  

                    try:
                        m_dot = np.append(m_dot, xy_vel, axis=1)
                    except:
                        m_dot = xy_vel
                
                #uv_curr = np.array([[x[-1]],[y[-1]],[1]])
                #uv_prev = np.array([[x[-2]],[y[-2]],[1]])
                #uv_vel = np.divide((uv_curr - uv_prev), (t[-1]-t[-2]))
                
                #uv_curr = np.matmul(self.K_00, xy_curr)
                #uv_vel = np.matmul(self.K_00, xy_vel)
                
                #pos_x = np.int32(uv_curr[0,0])
                #pos_y = np.int32(uv_curr[1,0])
                #pos = (pos_x, pos_y)
                
                #vel_x = np.int32(uv_curr[0,0] + uv_vel[0,0])
                #vel_y = np.int32(uv_curr[1,0] + uv_vel[1,0])
                #vel = (vel_x, vel_y)
                
                #pos = (int(x[-1]), int(y[-1]))
                #vel = (int(uv_vel[0,0]+pos[0]), int(uv_vel[1,0]+pos[1]))
                
                #speed = np.linalg.norm(xy_vel, 2)
                #speed *= 1000
                #cv.arrowedLine(vis, pos, vel, (0, 255, 0))
                #draw_str(vis, pos, '%.f' % speed)

            draw_str(vis, (20, 20), 'feature points: %d' % len(self.tracks))

            # Choose random subset of 6 points BEFORE sending points to QuEst and VelEst
            #if len(m) >= 6:
            #    subset = sample(range(m.shape[1]),6)
            #    new_m = np.stack( [ m[:,i] for i in subset ], axis = 1)
            #    new_m_dot = np.stack( [ m_dot[:,i] for i in subset ], axis = 1)
            #    m = new_m
            #    m_dot = new_m_dot
            
            # This ensures a message is always sent (although a single point means VelEst just skips this frame)
            if len(m) < 1:
                m = np.array([[0],[0],[0]])
                m_dot = np.array([[0],[0],[0]])

            subset = []
            # Choose a "random" subset based on trail length
            if m.shape[1] >= 6:
				UseRandom = False
				if UseRandom:
					size_list = [] 					      # start of random choose
					for tr in self.tracks:
						size = len(tr)
						if size < 2:
							continue
						size_list.append(size)
					
					for i in range(6):
						max_size = 0
						ties = []
					
						for idx, size in enumerate(size_list):
							if size >= max_size and idx not in subset:
								if size == max_size:
									ties.append(idx)
								else:
									ties = [idx]
									max_size = size
						
						assert(len(ties) != 0)
						subset.append(random.choice(ties))
					subset = sorted(subset) 					# end of random choose
				else:
					subset = xrange(6) 								# disable random, choose first 6
					new_m = np.stack( [ m[:,i] for i in subset ], axis = 1)
					new_m_dot = np.stack( [ m_dot[:,i] for i in subset ], axis = 1)
					m = new_m
					m_dot = new_m_dot
            
            # Here I can send the 6 longest trails to the quest trail filter at which point QuEst and VelEst will be using the same points
            
            optical_flow_msg = OpticalFlow_msg()
            optical_flow_msg.header = ros_data.header
            
            # If there's no tracks with at least 2 points, subset will be an empty list
            if len(subset) == 0:
		PixArray = Pixel2DFlow_msg()
                Pix = Point32()
                Pix.x = 0
                Pix.y = 0
                PixArray.FlowArray.append(Pix)
                optical_flow_msg.Points.append(PixArray)
            else:
                idx = 0
                for PixFlow in self.tracks:
                    # tracks with less than 2 points aren't in the list that idx indexes
                    if len(PixFlow) < 2:
                        continue
                    if idx not in subset:
                        idx += 1
                        continue
                    idx += 1

		    PixArray = Pixel2DFlow_msg()
		    for Pixs in PixFlow:
		        Pix = Point32()
		        Pix.x = Pixs[0]
		        Pix.y = Pixs[1]
		        PixArray.FlowArray.append(Pix)
		    optical_flow_msg.Points.append(PixArray)
            
            print('m: %s \n' %(m))
            print('m_dot: %s \n' %(m_dot))
            # If there's no tracks we don't send a message
            if len(m) > 0:

                # publish results from current pair of frames
                new_msg = m_mdot()
                new_msg.header = ros_data.header                
                # M matrix metadata
                dimensions = []
                dimensions.append(mdim())
                dimensions.append(mdim())

                new_msg.m.layout.dim = dimensions
                new_msg.m.layout.dim[0].label = "rows"
                new_msg.m.layout.dim[0].size = m.shape[0]
                new_msg.m.layout.dim[0].stride = m.shape[0] * m.shape[1]
                new_msg.m.layout.dim[1].label = "cols"
                new_msg.m.layout.dim[1].size = m.shape[1]
                new_msg.m.layout.dim[1].stride = m.shape[1]
                # make matrix 1D
                new_msg.m.data = m.reshape(-1,)

                # M_dot matrix metadata
                dimensions_2 = []
                dimensions_2.append(mdim())
                dimensions_2.append(mdim())

                new_msg.m_dot.layout.dim = dimensions_2
                new_msg.m_dot.layout.dim[0].label = "rows"
                new_msg.m_dot.layout.dim[0].size = m_dot.shape[0]
                new_msg.m_dot.layout.dim[0].stride = m_dot.shape[0] * m_dot.shape[1]
                new_msg.m_dot.layout.dim[1].label = "cols"
                new_msg.m_dot.layout.dim[1].size = m_dot.shape[1]
                new_msg.m_dot.layout.dim[1].stride = m_dot.shape[1]
                # make matrix 1D
                new_msg.m_dot.data = m_dot.reshape(-1,)


                optical_flow_msg.m_mdot = new_msg
                self.publisher.publish(optical_flow_msg)


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
                #criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.1)
                #p = cv.cornerSubPix(frame_gray, p, (5,5), (-1,-1), criteria)
                
                for x, y in np.float32(p).reshape(-1, 2):
                    # When the number of tracks gets too large, lk_track_ros.py cannot
                    # keep up with the 30 Hz publishing rate of the camera, so we limit the number of tracks
                    # by favoring tracks with the longest lengths over newer tracks.
                    # If we wanted to favor pixels near the boundaries of the image we would do so here.
                    if len(self.tracks) > self.track_limit:
                        continue
                    self.tracks.append([(x, y)])

        # Save feature point images, need to put an output_path on the param server that every node can access
        if self.save_images:        
            cv.imwrite(self.output_path + "feature_images/feat_image%04d.png" % self.frame_idx, vis)


        self.frame_idx += 1
        self.prev_gray = frame_gray
        
        if self.showImages:
            cv.namedWindow('lk_track', cv.WINDOW_NORMAL)
            cv.resizeWindow('lk_track', 1920, 1080)
            cv.imshow('lk_track', vis)
        
            ch = cv.waitKey(1)
            if ch == 27:
                cv.destroyAllWindows()
        


def main():
    node = App()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

