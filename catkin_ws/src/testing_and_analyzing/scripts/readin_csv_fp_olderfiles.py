#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import csv
import numpy as np

from std_msgs.msg import Header
# ~ from vbme_msgs.msg import m_mdot
# ~ from geometry_msgs.msg import Point32
# ~ from vbme_msgs.msg import Pixel2DFlow_msg
# ~ from std_msgs.msg import MultiArrayDimension as mdim
# ~ from vbme_msgs.msg import OpticalFlow_msg

from vbme_pkg.msg import KeyPoint_msg
from vbme_pkg.msg import MatchList_msg


class CsvPublisher:
	def __init__(self):  								# constructor
		rospy.sleep(1.0) 								# give time for param server
		# params
		# ~ default_param = rospy.get_param('default_param', 'default_value')
		# ~ self.K_00 = np.array(rospy.get_param("camera_calibration")).reshape(3,3)
		# ~ self.K_00 = np.array(rospy.get_param("/optical_flow_group/camera_calibration")).reshape(3,3)
		
		# variables 
		self.rate = rospy.Rate(2) # 10hz
		self.ReadFromSingleFile = True
		self.FirstMsg = True
		
		# output publishers
		self.pub = rospy.Publisher('/vbme/Matches', MatchList_msg, queue_size=100)
		
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
		DirFiles = '/home/codyl/Downloads/Off1/'
		
		self.U = []
		self.V = []
		self.Udot =[]
		self.Vdot = []
		
		if self.ReadFromSingleFile:
			FpFile = 'features_raw.csv'
			
			with open( DirFiles + FpFile, 'rb') as csvfile:
				reader = csv.reader(csvfile, delimiter=',', quotechar='|')
				for row in reader:
					# ~ print(row)
					# ~ print(', '.join(row))
					# row[0] is seq number
					self.U.append([ row[1], row[3], row[5], row[7], row[9], row[11], row[13], row[15], row[17], row[19], row[21], row[23] ])
					self.V.append([ row[2], row[4], row[6], row[8], row[10], row[12], row[14], row[16], row[18], row[20], row[22], row[24] ])
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
		msg = MatchList_msg() 				
		TimeStart = rospy.get_rostime()
		
		# reserve space for the number of points (in 0 flow step)
		# ~ for i in xrange(len(self.U[0])):
		for i in xrange(6):
			pt = KeyPoint_msg()
			pt.X = float(self.U[0][i])
			pt.Y = float(self.V[0][i])
			
			msg.KeyPtList1.append(pt) 						 	# populate first list, this one is static
			msg.KeyPtList2.append(KeyPoint_msg())
		
		# populate first list
		# ~ for i in xrange(6):
			
		
		
		for seq in xrange(len(self.U)):
			if not rospy.is_shutdown():
				self.rate.sleep()
			else:
				break
			
			if seq > 0: 										# need 2 sets of feature points to calc optical flow
				rospy.loginfo('Publishing sequence %d ' %(seq))
				# ~ HeaderSeq = Header()
				
				msg.Header.seq = seq
				# ~ HeaderSeq.stamp = rospy.get_rostime()
				msg.Header.stamp = TimeStart + rospy.Duration(float(seq) * 0.1)
				
				# ~ msg.header = HeaderSeq
				
				# populate MatchList.Points
				# ~ for i in xrange(len(self.U[seq])): 				# loop over the number of points
				for i in xrange(6): 								# loop over publish only 6 points
					# ~ print("Seq %d. Point Index %d \n" %(seq, i))
					pt = KeyPoint_msg()
					pt.X = float(self.U[seq][i])
					pt.Y = float(self.V[seq][i])
					# ~ Flow.FlowArray.append(pt)
					msg.KeyPtList2[i] = pt
				
				self.pub.publish(msg)
		
	
if __name__ == '__main__':
	rospy.init_node('CsvPublisher', anonymous=True)
	
	Obj = CsvPublisher()
	Obj.ReadInFiles()
	rospy.sleep(1.0)
	Obj.Loop()

