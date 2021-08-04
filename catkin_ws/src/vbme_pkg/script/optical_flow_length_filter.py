#!/usr/bin/env python
import rospy
from vbme_pkg.msg import FlowMatchList_msg
from vbme_pkg.msg import Pixel2DFlow_msg
from geometry_msgs.msg import Point32

class OpticalFlowFilter:
	def __init__(self):  						# constructor
		# params
		self.PrintToScreen = rospy.get_param('PrintToScreen', True)
		self.TrailLimit = rospy.get_param('track_len', 1000)
		self.DesPtMin = rospy.get_param('DesiredMinimumTrailLength', 4)
		
		# variables 
		# ~ rate = rospy.Rate(10) # 10hz
		
		# output publishers
		self.pub = rospy.Publisher('optical_flow_filtered', FlowMatchList_msg, queue_size=100)
		
		# output clients
		
		# input services
		
		# input subcribers
		rospy.Subscriber("optical_flow_short", FlowMatchList_msg, self.callback)

	# ~ def __del__(self): 										# deconstructor
		# ~ pass
	
	def callback(self, msg):
		# ~ rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)
		if len(msg.Points) == 0:
			self.pub.publish(msg)
			return
		
		Output = FlowMatchList_msg()
		AvgLen = 0 												# used to get average of trail lengths
		NumDes = 0 												# number of trails with desired or great length
		if self.PrintToScreen:
			rospy.loginfo("Point num: %s" %(len(msg.Points)))
		for i in xrange(len(msg.Points)):
			if self.PrintToScreen:
				rospy.loginfo("  Trail length: %s" %(len(msg.Points[i].FlowArray)))
		
		# find average
		AvgLen = AvgLen / len(msg.Points)
		
		if (len(msg.Points) <= 5): # or (Shortest >= self.DesPtMin): 
			# keep all the points but shorten all the trails to the shortest
			## if there are 5 or less points we have to have them all
			## if the shortest is fairly long than min desired length we can keep them all 
			KeptLength = len(msg.Points[-1].FlowArray)
			for i in xrange(len(msg.Points)): 					# loop through all trails
				DiffLen = len(msg.Points[i].FlowArray) - KeptLength # this is the amount to be cutoff each trail
				Trail_i = Pixel2DFlow_msg()
				for j in xrange(KeptLength): 					# only loop the length of the shortest
					Pt_j = msg.Points[i].FlowArray[j + DiffLen] # everything before DiffLen is cut off
					Trail_i.FlowArray.append(Pt_j)
				Output.Points.append(Trail_i)
		else:
		# ~ elif (len(msg.Points) > 5) and (AvgLen < self.DesPtMin):
			# if we have enough points but most the lengths are short
			## get the length of the 5 longest trail and use that length
			## for as many points as possible, drop the rest
			try:
				KeptLength = len(msg.Points[4].FlowArray)
				i = 0
				while (i < len(msg.Points)) and ( len(msg.Points[i].FlowArray) >= KeptLength ):
					# while we have point trails and the current trail is at least as long as the kept length threshold
					DiffLen = len(msg.Points[i].FlowArray) - KeptLength # this is the amount to be cutoff each trail
					Trail_i = Pixel2DFlow_msg()
					for j in xrange(KeptLength): 					# only loop the length of the shortest
						Pt_j = msg.Points[i].FlowArray[j + DiffLen] # everything before DiffLen is cut off
						Trail_i.FlowArray.append(Pt_j)
					Output.Points.append(Trail_i)
					# ~ print("KPt: %s" %(i))
					i += 1
					
			except Exception as ex:
				rospy.logerr("Optical flow filter error: %s" %(ex))
				rospy.logerr("Desired TrailLen: %s" %(len(msg.Points[4].FlowArray)))
				
		
		Output.header = msg.header 
		self.pub.publish(Output)
		if self.PrintToScreen:
			rospy.loginfo("Output Point num: %s" %(len(Output.Points)))
			for i in xrange(len(Output.Points)):
				rospy.loginfo("  Output Trail length: %s" %(len(Output.Points[i].FlowArray)))
				
if __name__ == '__main__':
	rospy.init_node('OpticalFlowFilter', anonymous=True)
	
	OFF = OpticalFlowFilter();
	
	rospy.spin()

