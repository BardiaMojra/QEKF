#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import csv
import numpy as np

from std_msgs.msg import Header
from vbme_msgs.msg import m_mdot
from geometry_msgs.msg import Point32
from vbme_msgs.msg import Pixel2DFlow_msg
from std_msgs.msg import MultiArrayDimension as mdim
from vbme_msgs.msg import OpticalFlow_msg

class CsvPublisher:
	def __init__(self):  								# constructor
		rospy.sleep(2.0) 								# give time for param server
		# params
		# ~ default_param = rospy.get_param('default_param', 'default_value')
		# ~ self.K_00 = np.array(rospy.get_param("camera_calibration")).reshape(3,3)
		Hertz = rospy.get_param("/optical_flow_group/hertz")
		self.K_00 = np.array(rospy.get_param("/optical_flow_group/camera_calibration")).reshape(3,3)
		print("Csv Input Calibraton: \n%s" %(self.K_00))
		
		# variables 
		self.rate = rospy.Rate(Hertz) # 10hz
		self.ReadFromSingleFile = False
		self.PublishFlow = True
		
		# output publishers
		self.pub = rospy.Publisher('/vbme/optical_flow', OpticalFlow_msg, queue_size=1000)
		
		# output clients
		
		# input services
		
		# input subcribers
		# ~ rospy.Subscriber("chatter", String, self.callback)
		
		rospy.sleep(0.1) 								# give time for publisher registration

	# ~ def __del__(self): 										# deconstructor
		# ~ pass
	
	def ReadInFiles(self):
		# ~ DirFiles = '/home/codyl/Documents/vbme_results/Y2020M08D31_matlab/'
		# ~ UCoorFile = 'u_cord.csv'
		# ~ VCoorFile = 'v_cord.csv'
		# ~ UFlowFile = 'u_cord.csv'
		# ~ VFlowFile = 'v_cord.csv'
		# ~ DirFiles = '/home/codyl/Downloads/X-trans/'
		# ~ DirFiles = '/home/codyl/Downloads/Z-trans/'
		# ~ DirFiles = '/home/codyl/Downloads/X-trans + large Z rotation/'
		# ~ DirFiles = '/home/codyl/Downloads/Z-trans + large Z rotation/'
		# ~ DirFiles = '/home/codyl/Downloads/hand_extract_x_trans/'
		# ~ DirFiles = '/home/codyl/Downloads/Ppt tracked x-trans/'
		# ~ DirFiles = '/home/codyl/Downloads/tracked x trans/'
		# ~ DirFiles = '/home/codyl/Downloads/surf_matched/'
		# ~ DirFiles = '/home/codyl/Downloads/traccked photo xtrans/'
		# ~ DirFiles = '/home/codyl/Downloads/Off1/'
		# ~ DirFiles = '/home/codyl/Downloads/Off1_matlab_opticalflow/'
		# ~ DirFiles = '/home/codyl/Downloads/Off1_matlab_test2/'
		
		DirFiles = '/home/codyl/Downloads/v_cord_subpix_iphone_mouse_zoom/'
		# ~ DirFiles = '/home/codyl/Downloads/iphone_mouse_zoom_2/'
		
		# ~ DirFiles = '/home/codyl/Downloads/OfficialData - missile moving left to right on white background - not semented/'
		# ~ DirFiles = '/home/codyl/Downloads/OfficialData - missile moving left to right on white background - not semented - 2/'
		
		# ~ DirFiles = '/home/codyl/Downloads/Off3/'
		self.U = []
		self.V = []
		self.Udot =[]
		self.Vdot = []
		
		
		rospy.loginfo('Finished reading points from ' + DirFiles )
		if self.ReadFromSingleFile:
			FpFile = 'features_raw.csv'
			
			with open( DirFiles + FpFile, 'rb') as csvfile:
				reader = csv.reader(csvfile, delimiter=',', quotechar='|')
				for row in reader:
					# ~ print(row)
					# ~ print(', '.join(row))
					# row[0] is seq number
					print('Reading seq %d' %( float(row[0]) ))
					self.U.append([ row[1], row[5],  row[9], row[13], row[17], row[21] ])
					self.V.append([ row[2], row[6], row[10], row[14], row[18], row[22] ])
					# ~ print(row[1])
					# ~ self.U.append(row[1])
					# ~ print(self.U)
		else:
			# ~ UCoorFile = 'u_cord_pix.csv'
			# ~ VCoorFile = 'v_cord_pix.csv'
			# ~ UFlowFile = 'u_flow_pix.csv'
			# ~ VFlowFile = 'v_flow_pix.csv'
			# ~ UCoorFile = 'u_cord_pix_noise.csv'
			# ~ VCoorFile = 'v_cord_pix_noise.csv'
			# ~ UFlowFile = 'u_flow_pix_noise.csv'
			# ~ VFlowFile = 'v_flow_pix_noise.csv'
			UCoorFile = 'u_cord_subpix.csv'
			VCoorFile = 'v_cord_subpix.csv'
			UFlowFile = 'u_flow_subpix.csv'
			VFlowFile = 'v_flow_subpix.csv'
			# ~ UCoorFile = 'u_cord_subpix_noise.csv'
			# ~ VCoorFile = 'v_cord_subpix_noise.csv'
			# ~ UFlowFile = 'u_flow_subpix_noise.csv'
			# ~ VFlowFile = 'v_flow_subpix_noise.csv'
			# ~ UCoorFile = 'urootsurf.csv'
			# ~ VCoorFile = 'vrootsurf.csv'
			# ~ UFlowFile = 'uflowsurf.csv'
			# ~ VFlowFile = 'vflowsurf.csv'
			
			with open( DirFiles + UCoorFile, 'rb') as csvfile:
				reader = csv.reader(csvfile, delimiter=',', quotechar='|')
				for row in reader:
					# ~ print(row)
					# ~ print(', '.join(row))
					self.U.append(row)
			with open( DirFiles + VCoorFile, 'rb') as csvfile:
				reader = csv.reader(csvfile, delimiter=',', quotechar='|')
				for row in reader:
					self.V.append(row)
			with open( DirFiles + UFlowFile, 'rb') as csvfile:
				reader = csv.reader(csvfile, delimiter=',', quotechar='|')
				for row in reader:
					self.Udot.append(row)
			with open( DirFiles + VFlowFile, 'rb') as csvfile:
				reader = csv.reader(csvfile, delimiter=',', quotechar='|')
				for row in reader:
					self.Vdot.append(row)
		
	def Loop(self):
		self.rate.sleep()
		# ~ seq = 0
		# keep build off the same message, more points, longer trails
		msg = OpticalFlow_msg() 				
		# ~ TimeStart = rospy.get_rostime()
		
		# reserve space for the number of points (in 0 flow step)
		# ~ for i in xrange(len(self.U[0])):
		for i in xrange(6):
			msg.Points.append(Pixel2DFlow_msg())
		
		for seq in xrange(len(self.U)):
			if not rospy.is_shutdown():
				# ~ pass
				self.rate.sleep()
				# ~ self.rate.sleep()
				# ~ self.rate.sleep()
				# ~ self.rate.sleep()
			else:
				break
			
			if seq > 0: 										# need 2 sets of feature points to calc optical flow
				# ~ rospy.loginfo('%% Publishing sequence %d ' %(seq))
				print('%% Publishing sequence %d ' %(seq))
				HeaderSeq = Header()
				
				HeaderSeq.seq = seq
				HeaderSeq.stamp = rospy.get_rostime()
				# ~ HeaderSeq.stamp = TimeStart + rospy.Duration(float(seq) * 0.1)
				
				msg.header = HeaderSeq
				
				# populate MatchList.Points
				# ~ for i in xrange(len(self.U[seq])): 				# loop over the number of points
				for i in xrange(6): 								# loop over publish only 6 points
					# ~ print("Seq %d. Point Index %d \n" %(seq, i))
					pt = Point32()
					pt.x = float(self.U[seq][i])
					pt.y = float(self.V[seq][i])
					# ~ Flow.FlowArray.append(pt)
					msg.Points[i].FlowArray.append(pt)
				
				
				if self.PublishFlow:
					# create m and m_dot, format copied from lk_track_ros.py
					# ~ m = np.array([[0],[0],[0]])
					# ~ m_dot = np.array([[0],[0],[0]])
					m = []
					m_dot = []
					
					# ~ uv = np.stack((bx, by, np.ones(len(bx))), axis=0)
					# ~ assert(uv.shape == (3, size))
					# ~ xy = np.matmul(np.linalg.inv(self.K_00), uv)
					# ~ xy_curr = (xy[:,-1]).reshape(3,1)
					# ~ assert(xy.shape == (3, size))
					# ~ bx = xy[0,:]
					# ~ by = xy[1,:]
					
					# ~ for i in xrange(len(self.U[seq])): 				# loop over the number of points
					for i in xrange(6): 								# loop over publish only 6 points
						# ~ xy_curr = np.array([ float(self.U[seq][i]) ,   float(self.V[seq][i]) ,   1.0]).T
						xy_curr = np.array([ [float(self.U[seq][i])] ,   [float(self.V[seq][i])] ,   [1.0]])
						try:
							m = np.append(m, xy_curr, axis=1)
						except:
							m = xy_curr  
					
					# calibrate
					# ~ rospy.loginfo('m: %s shape: %s - \n%s' %(type(m), m.shape, m))
					# ~ rospy.loginfo('self.K_00: %s shape: %s - \n%s' %(type(self.K_00), self.K_00.shape, self.K_00)) 
					# ~ assert(m.shape == (3, len(self.U[seq])))
					assert(m.shape == (3, 6))
					assert(self.K_00.shape == (3, 3))
					# ~ m = np.matmul(np.linalg.inv(self.K_00), m.T)
					mk = np.matmul( np.linalg.inv(self.K_00) , m)
					# ~ mk = np.matmul( self.K_00 , m)
					# ~ m = np.linalg.inv(self.K_00) * m.T
					# ~ m = m.dot(self.K_00)
					# ~ rospy.loginfo('k inv: \n%s \n' %(np.linalg.inv(self.K_00)))
					# ~ rospy.loginfo('m: \n%s \nmk: \n%s - \n' %(m, mk))
					
					# ~ for i in xrange(len(self.U[seq])): 				# loop over the number of points
					for i in xrange(6): 								# loop over publish only 6 points
						# ~ xy_vel = np.array([ float(self.Udot[seq][i]) , float(self.Vdot[seq][i]) , 0.0]).T
						xy_vel = np.array([ [float(self.Udot[seq][i])] , [float(self.Vdot[seq][i])] , [0.0]])
						try:
							m_dot = np.append(m_dot, xy_vel, axis=1)
						except:
							m_dot = xy_vel
					
					# ~ rospy.loginfo('m_dot: %s shape: %s - \n%s' %(type(m_dot), m_dot.shape, m_dot))
					# ~ assert(m_dot.shape == (3,  len(self.Udot[seq])))
					assert(m_dot.shape == (3,  6))
					
					m_dotk = np.matmul( np.linalg.inv(self.K_00) , m_dot)
					# ~ m_dotk = np.matmul( self.K_00 , m_dot)
					# ~ rospy.loginfo('m_dot: \n%s \nm_dotk: \n%s - \n' %(m_dot, m_dotk))
					
					# populate m_mdot(), format copied from lk_track_ros.py
					new_msg = m_mdot()
					new_msg.header = HeaderSeq 
					# M matrix metadata
					dimensions = []
					dimensions.append(mdim())
					dimensions.append(mdim())

					# ~ rospy.loginfo('seq+1: %s len(self.U[0]): %s stride: %s ' %(seq+1, len(self.U[0]), ( float(seq+1) * float(len(self.U[0])) ) ))
					
					new_msg.m.layout.dim = dimensions
					new_msg.m.layout.dim[0].label = "rows"
					new_msg.m.layout.dim[0].size = mk.shape[0]
					new_msg.m.layout.dim[0].stride = mk.shape[0] * mk.shape[1]
					new_msg.m.layout.dim[1].label = "cols"
					new_msg.m.layout.dim[1].size = mk.shape[1]
					# ~ new_msg.m.layout.dim[1].size = len(self.U[0])
					new_msg.m.layout.dim[1].stride = mk.shape[1]
					# ~ new_msg.m.layout.dim[1].stride = len(self.U[0])
					# make matrix 1D
					# ~ new_msg.m.data = m.reshape(-1,)
					new_msg.m.data = mk.reshape(-1,)

					# M_dot matrix metadata
					dimensions_2 = []
					dimensions_2.append(mdim())
					dimensions_2.append(mdim())

					new_msg.m_dot.layout.dim = dimensions_2
					new_msg.m_dot.layout.dim[0].label = "rows"
					new_msg.m_dot.layout.dim[0].size = m_dotk.shape[0]
					new_msg.m_dot.layout.dim[0].stride = m_dotk.shape[0] * m_dotk.shape[1]
					new_msg.m_dot.layout.dim[1].label = "cols"
					new_msg.m_dot.layout.dim[1].size = m_dotk.shape[1]
					# ~ new_msg.m_dot.layout.dim[1].size = len(self.U[0])
					new_msg.m_dot.layout.dim[1].stride = m_dotk.shape[1]
					# ~ new_msg.m_dot.layout.dim[1].stride = len(self.U[0])
					# make matrix 1D
					new_msg.m_dot.data = m_dotk.reshape(-1,)

					msg.m_mdot = new_msg
				
				self.pub.publish(msg)
		
	
if __name__ == '__main__':
	rospy.init_node('CsvPublisher', anonymous=True)
	
	Obj = CsvPublisher()
	Obj.ReadInFiles()
	rospy.sleep(1.0)
	Obj.Loop()

