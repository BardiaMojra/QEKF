#!/usr/bin/env python
import rospy
from vbme_msgs.msg import VbmeData_msg
import numpy as np

class Heading:
	def __init__(self):  									# constructor
		# params
		# ~ default_param = rospy.get_param('default_param', 'default_value')
		self.CamYawStart = rospy.get_param('~/CamYawStart', 0.0)
		self.CamPitchStart = rospy.get_param('~/CamPitchStart', 0.0)
		
		# method 0=velocity only, 1=translation difference only, 2=average of vel and trans
		self.Method = rospy.get_param('~/Method', 1)
		
		# variables 
		self.MsgLast = VbmeData_msg() 						# to get translation difference
		
		# output publishers
		self.pub = rospy.Publisher('/vbme/HeadingData', VbmeData_msg, queue_size=10)
		
		# output clients
		
		# input services
		
		# input subcribers
		rospy.Subscriber("/vbme/EKF", VbmeData_msg, self.callback)

	# ~ def __del__(self): 										# deconstructor
		# ~ pass

	
		
	def callback(self, msg):
		# ~ print("EDN VX %f, VY %f VZ %f " %(msg.Velocity.linear.x , msg.Velocity.linear.y , msg.Velocity.linear.z))
		# ~ print("Trans D VX %f, VY %f VZ %f " %( msg.transform.translation.x - self.MsgLast.transform.translation.x ,  msg.transform.translation.y - self.MsgLast.transform.translation.y ,  msg.transform.translation.z - self.MsgLast.transform.translation.z))
		
		# starting vars
		CalcVelCoor = False 									# calculate from velocity 
		CalcTransCoor = False 									# calculate from translation
		YawRelVel = 0.0 										# velocity based relative yaw
		PitchRelVel = 0.0
		YawRelTrans = 0.0										# translation based relative yaw
		PitchRelVelTrans = 0.0
		YawAbs = 0.0
		PitchAbs = 0.0
		
		if self.Method == 0:
			CalcVelCoor = True
		elif self.Method == 1:
			CalcTransCoor = True
		elif self.Method == 2:
			CalcVelCoor = True
			CalcTransCoor = True
		# read cam0 and apply before calling cart2sph
		
		if CalcVelCoor:											# calculate from velocity 
			YawRelVel, PitchRelVel, RadRelVel = self.cart2sph(msg.Velocity.linear.x , msg.Velocity.linear.y , msg.Velocity.linear.z)
			# ~ print("Relative Velocity Pitch %f, Yaw %f " %(PitchRelVel, YawRelVel))
			
		if CalcTransCoor: 										# calculate from translation
			DX = msg.transform.translation.x - self.MsgLast.transform.translation.x
			DY = msg.transform.translation.y - self.MsgLast.transform.translation.y
			DZ = msg.transform.translation.z - self.MsgLast.transform.translation.z
			YawRelTrans, PitchRelVelTrans, RadRelTrans = self.cart2sph(DX , DY , DZ)
			# ~ print("Relative Translation Difference Pitch %f, Yaw %f " %(PitchRelVelTrans, YawRelTrans))
			
		if self.Method == 0:
			# this assumes camstart never moves (no chaining)
			YawAbs = YawRelVel + self.CamYawStart
			PitchAbs = PitchRelVel + self.CamPitchStart
			
			msg.PitchCam = PitchRelVel
			msg.HeadingCam = YawRelVel
		
		elif self.Method == 1:
			YawAbs = YawRelTrans + self.CamYawStart
			PitchAbs = PitchRelVelTrans + self.CamPitchStart
			msg.PitchCam = PitchRelVelTrans
			msg.HeadingCam = YawRelTrans
			
		elif self.Method == 2:
			Yavg = (YawRelVel + YawRelTrans)/2.0
			Pavg = (PitchRelVel + PitchRelVelTrans)/2.0
			
			YawAbs = Yavg + self.CamYawStart
			PitchAbs = Pavg + self.CamPitchStart
			
			msg.PitchCam = Pavg
			msg.HeadingCam = Yavg
			
		if YawAbs > 360.0:
			YawAbs = YawAbs - 360.0
		elif YawAbs < -181:
			YawAbs = YawAbs + 360.0
		if PitchAbs > 181.0:
			PitchAbs = PitchAbs - 360.0
		elif PitchAbs < -181.0:
			PitchAbs = PitchAbs + 360.0
		
		msg.PitchAbsolute = PitchAbs
		msg.HeadingAbsolute = YawAbs
		
		self.pub.publish(msg)
		
	# https://stackoverflow.com/questions/30084174/efficient-matlab-cart2sph-and-sph2cart-functions-in-python
	def cart2sph(self, x,y,z):
		# NOTES 		X Y Z
		# CAMERA 		E D N
		# SPH expect 	E N U
		# OUTPUT  		N E D
		
		r = np.sqrt(x**2 + y**2 + z**2)
		
		# ~ azimuth = np.arctan2(y,x) 								# orginal sphere
		# ~ elevation = np.arctan2(z,np.sqrt(x**2 + y**2))
		# ~ azimuth = np.arctan2(z,x) * (180.0 / np.pi) 			# modified coord system sphere
		# ~ elevation = np.arctan2(y,np.sqrt(x**2 + z**2)) * (180 / np.pi)
		
		
		azimuth = np.arctan2( y , z ) * (180.0 / np.pi) 			# modified coord system sphere
		# ~ azimuth = np.arctan2( -1.0*y , z ) * (180.0 / np.pi) 	# modified coord system sphere
		# ~ elevation = np.arctan2( x , z ) * (180.0 / np.pi)
		
		elevation = np.arcsin( x / r ) * (180.0 / np.pi) 			#  asin(l/||dT||) frome https://onlinemschool.com/math/library/analytic_geometry/plane_line/
		
		# measure clockwise from north not. anti clockwise from east
		# ~ azimuth = 90 - azimuth
		
		return azimuth, elevation, r

	# ~ def sph2cart(self, azimuth,elevation,r):
		# ~ x = r * np.cos(elevation) * np.cos(azimuth)
		# ~ y = r * np.cos(elevation) * np.sin(azimuth)
		# ~ z = r * np.sin(elevation)
		# ~ return x, y, z
		

	
if __name__ == '__main__':
	rospy.init_node('CalculateHeading', anonymous=True)
	
	Obj = Heading()
	
	rospy.spin()

