#!/usr/bin/env python

import rospy
import roslib
import message_filters

from std_msgs.msg import Float64
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped 
from kalman_filter.msg import ekf_transform
from optical_flow.msg import velocity

from vbme_msgs.msg import *

import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise

# x is the state and z is the state measurement
# F is the state transition function
# H is the observation function (given a state at time t it will produce the measurement predicted at time t)
# R is the measurement noise covariance matrix
# Q is the process noise covariance matrix
# P 

class Kalman:
    def __init__(self):
        rospy.init_node("kalman", anonymous=True)
        # Constants
        # TODO
        # Change to actual time delta
        # ~ time_period = 2 # inital value?
        self.use_pkf = True					# use pkf update step
        self.sigma = [1, 1, 1, 1, 1, 1] 	# These are place holders until more rigorious testing can be done 
        self.pkf_gamma_x = np.ones((6,1)) # this can be used to convert the EKF to a Partial KF. 0=EKF, 1=SKF, 0<gamma<1 = PKF
        self.pkf_gamma_x *= 0.9
        self.pkf_gamma_p = np.ones((6,1)) # this can be used to convert the EKF to a Partial KF. 0=EKF, 1=SKF, 0<gamma<1 = PKF
        self.pkf_gamma_p *= 0.9
        self.my_filter = ExtendedKalmanFilter(dim_x=6, dim_z=6)

        # Initialize state
        #self.my_filter.x = np.array([[1],[2],[3],[4],[5],[6]])
        self.my_filter.x = np.zeros((6,1))
        # setup F matrix 
        # ~ F_mat = np.identity(6)
        # ~ F_mat[0:3,3:6] = self.time_period * np.identity(3)
        # ~ F_mat[0:3,3:6] = time_period * np.identity(3)
        # ~ self.my_filter.F = F_mat
        
        self.my_filter.H = np.identity(6)
        rospy.loginfo('Kalman P: %s' %(self.my_filter.P))
        self.my_filter.P *= 1000
        # ~ self.my_filter.P *= 10
        # ~ self.my_filter.P *= .0001
        rospy.loginfo('Kalman P: %s' %(self.my_filter.P))
        
        # Setup R matrix
        # R should be the covariance of the measurement noise
        
        
        #R_mat_raw = np.array([[ 2.09914031e-02,  1.14536566e-02, -5.24526764e-02,  1.93012517e-02, 3.22973913e-03, -1.05248747e-01],
        #                      [ 1.14536566e-02,  1.20715206e-02, -7.06546918e-02, -1.90931901e-03, 9.92022909e-04,  2.06819955e-02],
        #                      [-5.24526764e-02, -7.06546918e-02,  4.88389625e-01,  5.21457206e-02, -1.44696919e-03, -3.68198622e-01],
        #                      [ 1.93012517e-02, -1.90931901e-03,  5.21457206e-02,  9.86815153e-02, 4.19130308e-02, -5.96124582e-01],
        #                      [ 3.22973913e-03,  9.92022909e-04, -1.44696919e-03,  4.19130308e-02, 3.10561197e-02, -2.57669106e-01],
        #                      [-1.05248747e-01,  2.06819955e-02, -3.68198622e-01, -5.96124582e-01, -2.57669106e-01,  3.66140175e+00]])
        
	R_mat_raw = np.array([[ 5.94262740e-01, -2.10863240e-01, -5.19096805e+01, -1.25446220e-01,  2.77192855e-02,  1.14681593e+00],
		              [-2.10863240e-01,  2.58727665e-01,  4.24021616e+01, -4.38689896e-02, -1.77888886e-02, -9.56680550e-01],
			      [-5.19096805e+01,  4.24021616e+01,  9.28639923e+03,  5.34567710e+00, -4.33255602e+00, -2.21100725e+02],
			      [-1.25446220e-01, -4.38689896e-02,  5.34567710e+00,  2.47655381e-01, -2.85957253e-02,  2.02231825e-01],
			      [ 2.77192855e-02, -1.77888886e-02, -4.33255602e+00, -2.85957253e-02,  3.85694786e-02, -1.08890944e-01],
			      [ 1.14681593e+00, -9.56680550e-01, -2.21100725e+02,  2.02231825e-01, -1.08890944e-01,  1.53426900e+01]])
        
        R_mat_raw[3,3] = R_mat_raw[3,3]*50
        R_mat_raw[4,4] = R_mat_raw[4,4]*50
        R_mat_raw[5,5] = R_mat_raw[5,5]*50
        
        R_mat = np.identity(6)
        for i in range(6):
            R_mat[i,i] = R_mat_raw[i,i]
        
        self.my_filter.R = R_mat
       
        # setup Q matrix
        # Q should be the covariance of the process noise
        self.N = np.zeros((6,6))
        # ~ self.N[0,0] = 0.001
        # ~ self.N[1,1] = 0.001
        # ~ self.N[2,2] = 0.001
        # ~ self.N[3,3] = 0.009
        # ~ self.N[4,4] = 0.009
        # ~ self.N[5,5] = 0.009
        # ~ self.N[0,0] = 0.01
        # ~ self.N[1,1] = 0.01
        # ~ self.N[2,2] = 0.01
        # ~ self.N[3,3] = 0.09
        # ~ self.N[4,4] = 0.09
        # ~ self.N[5,5] = 0.09
        self.N[0,0] = 100.0
        self.N[1,1] = 100.0
        self.N[2,2] = 100.0
        self.N[3,3] = 10.0
        self.N[4,4] = 10.0
        self.N[5,5] = 10.0
        # ~ N[0,0] = 0
        # ~ N[1,1] = 0
        # ~ N[2,2] = 0
        # ~ N[3,3] = 0.1
        # ~ N[4,4] = 0.0001
        # ~ N[5,5] = 0.5
        
        #N[3,3] = N[0,0]**2
        #N[4,4] = N[1,1]**2
        #N[5,5] = N[2,2]**2
        
        
        # ~ Q_mat = np.matmul(np.matmul(F_mat, self.N), F_mat.T)
        # ~ self.my_filter.Q = Q_mat
        
        #self.quest_sub = message_filters.Subscriber("/vbme/Transform", TransformStamped)
        #self.velest_sub = message_filters.Subscriber("/output/real_velocity", velocity)
        self.sub = rospy.Subscriber("/vbme/VestData", VbmeData_msg, self.callback, queue_size = 10)
        #self.pub = rospy.Publisher("/vbme/EKF", ekf_transform, queue_size=3)
        self.pub = rospy.Publisher("/vbme/EKF", VbmeData_msg, queue_size=10)

        
        #self.ts = message_filters.TimeSynchronizer([self.quest_sub, self.velest_sub], 20)
        #self.ts.registerCallback(self.callback)
        rospy.sleep(0.05)
        self.LastTime = rospy.get_rostime() # default start time
        
        rospy.loginfo("Kalman Linear Ready" )
        rospy.spin()


    def callback(self, ros_data):
        rospy.loginfo("Kalman Seq: %f " %(ros_data.header.seq))
        # time stamps should match, although seq numbers might not match
        # assert(quest.header.stamp == velest.header.stamp)
        

        time_period_ros = ros_data.header.stamp - self.LastTime
        time_period = time_period_ros.to_sec() 
        
        observation = np.array([[0],[0],[0],[0],[0],[0]], dtype= np.float64)
        observation[0,0] = ros_data.transform.translation.x
        observation[1,0] = ros_data.transform.translation.y
        observation[2,0] = ros_data.transform.translation.z
    
        observation[3,0] = ros_data.Velocity.linear.x
        observation[4,0] = ros_data.Velocity.linear.y
        observation[5,0] = ros_data.Velocity.linear.z

        # setup F matrix 
        F_mat = np.identity(6)
        # ~ F_mat[0:3,3:6] = self.time_period * np.identity(3)
        F_mat[0:3,3:6] = time_period * np.identity(3)
        self.my_filter.F = F_mat
        Q_mat = np.matmul(np.matmul(F_mat, self.N), F_mat.T)
        self.my_filter.Q = Q_mat

        # ~ print("My filter observation: " + str(observation[:,0]))
        # ~ print("My filter X start: " + str(self.my_filter.x))
        
        # Do EKF filterimg
        self.my_filter.predict()
        if self.use_pkf:
            # ~ print("My filter X m: " + str(self.my_filter.x))
            x_minus = self.my_filter.x
            p_minus = self.my_filter.P
        for i in observation[:,0]:
            # NaN results from QuEst or VelEst will break the state, so we use the prediction instead
            if np.isnan(i):
                self.my_filter_x = self.my_filter.x_post
                break
        else:
            func_jacobian = lambda x: np.identity(6)
            func_measurement = lambda x: x
            self.my_filter.update(observation, func_jacobian, func_measurement)
        
        if self.use_pkf:
            # ~ print("My filter X p: " + str(self.my_filter.x))
            x_plus = self.my_filter.x  	
            p_plus = self.my_filter.P
            [x_pplus, p_pplus] = self.pkf_update(x_minus, x_plus, p_minus, p_plus)
            self.my_filter.x[:,0] = x_pplus
            self.my_filter.p = p_pplus
            # ~ print("My filter X pp: " + str(self.my_filter.x))
        # Publish result
        result = self.my_filter.x
        
        new_msg = VbmeData_msg()
        
        # Sequence number will be wrong, but timestamp will be correct
        new_msg.header = ros_data.header
        new_msg.MatchList = ros_data.MatchList
        new_msg.Depths = ros_data.Depths
        new_msg.X1 = ros_data.X1
        new_msg.X2 = ros_data.X2
        new_msg.Rotation = ros_data.Rotation
        new_msg.transform = ros_data.transform
        new_msg.Velocity = ros_data.Velocity


        new_msg.transform.translation.x = result[0,0]
        new_msg.transform.translation.y = result[1,0]
        new_msg.transform.translation.z = result[2,0]
        
        new_msg.Velocity.linear.x = result[3,0]
        new_msg.Velocity.linear.y = result[4,0]
        new_msg.Velocity.linear.z = result[5,0]
        
        self.pub.publish(new_msg)
        self.LastTime = ros_data.header.stamp
        
    def pkf_update (self, x_minus, x_plus, p_minus, p_plus):
		x_output = np.zeros(x_minus.size)
		# ~ print("x minus: " + str(x_minus))
		# ~ print("x plus: " + str(x_plus))
		# ~ print("p minus: " + str(p_minus))
		# ~ print("p plus: " + str(p_plus))
		for i in xrange(x_output.size):
			# ~ print("OUTPUT1: " + str(self.pkf_gamma_x[i]))
			# ~ print("OUTPUT2: " + str(x_minus[i]))
			# ~ print("OUTPUT3: " + str((1.0-self.pkf_gamma_x[i])))
			# ~ print("OUTPUT4: " + str(x_plus[i,0]))
			# ~ print("OUTPUT: " + str(self.pkf_gamma_x[i]*x_minus[i] + (1.0-self.pkf_gamma_x[i])*x_plus[i,0]))
			x_output[i] = self.pkf_gamma_x[i]*x_minus[i] + (1.0-self.pkf_gamma_x[i])*x_plus[i,0]
		
		[pr, pc] = p_minus.shape 							# rows and cols
		p_output = np.zeros((pr, pc))
		for i in xrange(pr):
			for j in xrange(pc):
				p_output[i,j] = self.pkf_gamma_p[i]*self.pkf_gamma_p[j]*p_minus[i,j] + (1.0-self.pkf_gamma_p[i]*self.pkf_gamma_p[j])*p_plus[i,j]
				
		return [x_output, p_output]

def main():

    k = Kalman()

if __name__ == "__main__":
    main()
