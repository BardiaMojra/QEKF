#!/usr/bin/env python
import rospy
# ~ from geometry_msgs.msg import Transform
# ~ from geometry_msgs.msg import TransformStamped
from vbme_msgs.msg import VbmeData_msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import os, os.path
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import math 													# acos()

class TransformPlotter:
	def __init__(self):  										# constructor
		# params
		self.UseRansac = rospy.get_param('UseRansac', False)
		self.NormalizeOutput = rospy.get_param('NormalizeOutput', False)
		self.PlotType = rospy.get_param('Plot/Type', 2)		# 1 RPY/Eular, 2 Quaternion, 3 AngleAxis
		
		# variables 
		self.rate = rospy.Rate(30)
		self.NewMsg = False 									# flag for new data
		self.FirstCallback = True
		self.ArrayTransX = [] 									# array of translation x estimate
		self.ArrayTransY = [] 									# array of translation y estimate
		self.ArrayTransZ = [] 									# array of translation z estimate
		
		# orientation.
		# if euler angles are used, W is irgnored 
		self.ArrayOrientX = [] 									
		self.ArrayOrientY = [] 									
		self.ArrayOrientZ = [] 									
		self.ArrayOrientW = [] 									
		
		self.fig = plt.figure()
		# ~ self.fig = plt.figure(True)
		# ~ self.fig = plt.figure(constrained_layout=True)
		# ~ gs = gridspec.GridSpec(2, 1, figure=self.fig)
		gs = gridspec.GridSpec(2, 1)
		# ~ gs = self.fig.add_gridspec(3, 3)
		# ~ gs = gridspec.GridSpec(2, 2, width_ratios=[1, 2], height_ratios=[4, 1])
		# ~ gs = gridSpec(3, 3)
		
		# ~ self.ax = self.fig.add_subplot(1,1,1)
		self.ax = self.fig.add_subplot(gs[0, :])
		self.ax.grid(True, which='both')
		self.ax.axhline(y=0, color='k') 						# emphasize horizontal 0
		self.ax.set_title('QuEst Translation. Using RANSAC: %s. Normalize: %s' % (self.UseRansac, self.NormalizeOutput))
		self.ax.set_xlabel('Samples')
		self.ax.set_ylabel('Position')
		
		# https://stackoverflow.com/questions/4700614/how-to-put-the-legend-out-of-the-plot
		# Shrink current axis's height by 15% on the bottom
		box = self.ax.get_position()
		self.ax.set_position([box.x0, box.y0 + box.height * 0.15, box.width, box.height * 0.9])
		
		# ~ self.ax2 = self.fig.add_subplot(2,1,2)
		self.ax2 = self.fig.add_subplot(gs[1, :])
		self.ax2.grid(True, which='both')
		self.ax2.axhline(y=0, color='k') 						# emphasize horizontal 0
		self.ax2.set_title('QuEst Orientation. Plot Type: %s' % (self.PlotType))
		self.ax2.set_xlabel('Samples')
		self.ax2.set_ylabel('Value')
		
		box = self.ax2.get_position()
		self.ax2.set_position([box.x0, box.y0 + box.height * 0.15, box.width, box.height * 0.9])
		
		
		# output publishers
		# ~ pub = rospy.Publisher('chatter', String, queue_size=10)
		
		# output clients
		
		# input services
		
		# input subcribers
		rospy.Subscriber("QuestData", VbmeData_msg, self.callback)
		rospy.sleep(0.2)
		
		# ~ self.ReadGroundTruth() 							# truth plotting is incomplete 
		self.Loop()

	# ~ def __del__(self): 									# deconstructor
		# ~ pass
		
	def ReadGroundTruth(self):
		Truth_Path = "/home/developer/Desktop/2011_09_26/2011_09_26_drive_0002_sync/oxts/data/"
		Timestamps_Path = "/home/developer/Desktop/2011_09_26/2011_09_26_drive_0002_sync/"
		
		# get the number of files for ground truth
		self.NumFile =  len([name for name in os.listdir(Truth_Path) if os.path.isfile(os.path.join(Truth_Path, name))])
		self.v_true_list = np.zeros((3,self.NumFile)) 				# velocity
		self.w_true_list = np.zeros((3,self.NumFile)) 				# rotational velocity
		self.pos_true_list = np.zeros((3,self.NumFile)) 			# position
		self.ori_true_list = np.zeros((3,self.NumFile)) 			# orientation (yaw)
		
		self.FirstHeading = 0.0 									# hold vehicle frame zero heading offset
		
		# ~ for x in range(self.NumFile-1):
		for x in range(self.NumFile):
			# ~ print 'NumFile: %s' % x
				
			# ~ file_idx = "%010d.txt" % (x+2)
			file_idx = "%010d.txt" % (x)
			f = open(Truth_Path+file_idx, "r")
			data = []
			for line in f:
				data.append(list(map(float, line.split(" "))))
			v_true = np.array([-1*float(data[0][9]), -1*float(data[0][10]), float(data[0][8])])
			w_true = np.array([-1*float(data[0][21]), -1*float(data[0][22]), float(data[0][20])])
			# ~ v_true = np.divide(v_true, np.linalg.norm(v_true, 2))
			pos_true = np.array([-1*float(data[0][9]), -1*float(data[0][10]), float(data[0][8])])
			ori_true = float(data[0][5])
			# ~ print("V: " + str(v_true))
			# ~ print("W: " + str(w_true))
			
			if x == 0:
				# ~ print 'First truth sample!'
				self.FirstHeading = float(data[0][5])
				# ~ print "Heading %s" % self.FirstHeading
			# ~ print 'Heading: %s' % ori_true
			# ~ print 'Heading relative to starting postition: %s' % (ori_true - self.FirstHeading)
			ori_rel_true = float(ori_true - self.FirstHeading)
			
			f.close()
			self.v_true_list[:,x] = v_true
			self.w_true_list[:,x] = w_true
			# ~ self.pos_true_list[:,x] = v_true
			self.ori_true_list[:,x] = ori_rel_true
		
		# ~ print self.v_true_list
		
	def Loop(self):
		while not rospy.is_shutdown():
			# ~ rospy.loginfo("X Data %s" % self.ArrayTransTR)
			
			if self.NewMsg == True: 							# we cannot plot old/empty data
				# ~ print("Plotting")
				# ~ print("X array: " + str(self.ArrayTransX))
				self.NewMsg = False
				
				# start plotting. There seems to have been a change?
				# labeling is not reset any more. Labeling only need
				# be done the first loop
				if self.FirstCallback: 
					# Tanslation X estimate
					self.ax.plot(self.ArrayTransX, 'r--', label='Translation X Estimate')
					# Tanslation X ground truth
					# ~ self.ax.plot(self.v_true_list[0,:], 'r-', label='Translation X Truth')
					# ~ self.ax.plot(self.v_true_list[0,:], 'r-', label='Velocity X Truth')
					
					# Tanslation Y estimate
					self.ax.plot(self.ArrayTransY, 'g--', label='Translation Y Estimate')
					# Tanslation Y ground truth
					# ~ self.ax.plot(self.v_true_list[1,:], 'g-', label='Translation Y Truth')
					# ~ self.ax.plot(self.v_true_list[1,:], 'g-', label='Velocity Y Truth')
					
					# Tanslation Z estimate
					self.ax.plot(self.ArrayTransZ, 'b--', label='Translation Z Estimate')
					# Tanslation Z ground truth
					# ~ self.ax.plot(self.v_true_list[2,:], 'b-', label='Translation Z Truth')
					# ~ self.ax.plot(self.v_true_list[2,:], 'b-', label='Velocity Z Truth')
				else:
					self.ax.plot(self.ArrayTransX, 'r--')
					self.ax.plot(self.ArrayTransY, 'g--')
					self.ax.plot(self.ArrayTransZ, 'b--')
				
                # Put a legend below current axis
                # ~ self.ax.legend()
				self.ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.12), fancybox=True, shadow=True, ncol=5)
				
				if self.FirstCallback:
					self.FirstCallback = False
					if self.PlotType == 1:
						self.ax2.plot(self.ArrayOrientX, 'r--', label='Roll Estimate')
						self.ax2.plot(self.ArrayOrientY, 'g--', label='Pitch Estimate')
						self.ax2.plot(self.ArrayOrientZ, 'b--', label='Yaw Estimate')
					elif self.PlotType == 2:
						self.ax2.plot(self.ArrayOrientX, 'r--', label='Quaternion X Estimate')
						self.ax2.plot(self.ArrayOrientY, 'g--', label='Quaternion Y Estimate')
						self.ax2.plot(self.ArrayOrientZ, 'b--', label='Quaternion Z Estimate')
						self.ax2.plot(self.ArrayOrientW, 'y--', label='Quaternion W Estimate')
					elif self.PlotType == 3:
						self.ax2.plot(self.ArrayOrientX, 'r--', label='AngleAxis X Estimate')
						self.ax2.plot(self.ArrayOrientY, 'g--', label='AngleAxis Y Estimate')
						self.ax2.plot(self.ArrayOrientZ, 'b--', label='AngleAxis Z Estimate')
						self.ax2.plot(self.ArrayOrientW, 'y--', label='AngleAxis Angle Estimate')
				else:
					if self.PlotType == 1:
						self.ax2.plot(self.ArrayOrientX, 'r--')
						self.ax2.plot(self.ArrayOrientY, 'g--')
						self.ax2.plot(self.ArrayOrientZ, 'b--')
					elif self.PlotType == 2:
						self.ax2.plot(self.ArrayOrientX, 'r--')
						self.ax2.plot(self.ArrayOrientY, 'g--')
						self.ax2.plot(self.ArrayOrientZ, 'b--')
						self.ax2.plot(self.ArrayOrientW, 'y--')
					elif self.PlotType == 3:
						self.ax2.plot(self.ArrayOrientX, 'r--')
						self.ax2.plot(self.ArrayOrientY, 'g--')
						self.ax2.plot(self.ArrayOrientZ, 'b--')
						self.ax2.plot(self.ArrayOrientW, 'y--')
					
				# ~ self.ax2.legend()
				self.ax2.legend(loc='upper center', bbox_to_anchor=(0.5, -0.12), fancybox=True, shadow=True, ncol=5)
				
				
				self.fig.canvas.draw()				
				plt.pause(0.01)
				self.fig.canvas.flush_events()
				# ~ plt.clf()
				
			
			self.rate.sleep()
		
	def callback(self, data):
		# ~ if data.transform.translation.z < 0.0:
			# ~ data.transform.translation.x = -1.0 * data.transform.translation.x
			# ~ data.transform.translation.y = -1.0 * data.transform.translation.y
			# ~ data.transform.translation.z = -1.0 * data.transform.translation.z
			
			
		
		# ~ rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
		self.ArrayTransX.append(data.transform.translation.x)
		self.ArrayTransY.append(data.transform.translation.y)
		self.ArrayTransZ.append(data.transform.translation.z)
		
		if self.PlotType == 1: 									# plot roll pitch yaw
			orientation_list = [data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
			(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
			self.ArrayOrientX.append(roll)
			self.ArrayOrientY.append(pitch)
			self.ArrayOrientZ.append(yaw)
		elif self.PlotType == 2: 								# plot quaternion
			self.ArrayOrientX.append(data.transform.rotation.x)
			self.ArrayOrientY.append(data.transform.rotation.y)
			self.ArrayOrientZ.append(data.transform.rotation.z)
			self.ArrayOrientW.append(data.transform.rotation.w)
		elif self.PlotType == 3: 								# plot angle axis
			# x = qx / sqrt(1-qw*qw)
			self.ArrayOrientX.append( data.transform.rotation.x / (pow( 1.0-pow(data.transform.rotation.w, 2.0), 0.5)) )
			# ~ y = qy / sqrt(1-qw*qw)
			self.ArrayOrientY.append( data.transform.rotation.y / (pow( 1.0-pow(data.transform.rotation.w, 2.0), 0.5)) )
			# ~ z = qz / sqrt(1-qw*qw)
			self.ArrayOrientZ.append( data.transform.rotation.z / (pow( 1.0-pow(data.transform.rotation.w, 2.0), 0.5)) )
			# ~ angle = 2 * acos(qw), use W array to hold
			self.ArrayOrientW.append( 2.0 * math.acos(data.transform.rotation.w) )
			
		self.NewMsg = True
	
if __name__ == '__main__':
	rospy.init_node('TransformPlotter', anonymous=True)
	
	TP = TransformPlotter()
	
	rospy.spin()

