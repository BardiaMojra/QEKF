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

import math

# x is the state and z is the state measurement
# F is the state transition function
# H is the observation function (given a state at time t it will produce the measurement predicted at time t)
# R is the measurement noise covariance matrix
# Q is the process noise covariance matrix
# P 

class Kalman:
    def __init__(self):
        # Constants
        self.time_period = 0.1 
        # ~ self.prev_omega = np.array([1,1,1]).reshape(3,)
        # ~ self.prev_omega = np.array([0,0,0]).reshape(3,) 	# this will cause a divide by zero error 
        # ~ self.prev_omega = np.array([sys.float_info.min, sys.float_info.min, sys.float_info.min]).reshape(3,) 
        self.prev_omega = np.array([1.0E-64, 1.0E-64, 1.0E-64]).reshape(3,) 
        self.prev_quat = np.array([0,0,0,1]).reshape(4,)
        
        self.use_pkf = True									# use pkf update step
        self.pkf_gamma_x = np.ones((7,1)) # this can be used to convert the EKF to a Partial KF. 0=EKF, 1=SKF, 0<gamma<1 = PKF
        self.pkf_gamma_x *= 0.9
        self.pkf_gamma_p = np.ones((7,1)) # this can be used to convert the EKF to a Partial KF. 0=EKF, 1=SKF, 0<gamma<1 = PKF
        self.pkf_gamma_p *= 0.9
        self.my_filter = ExtendedKalmanFilter(dim_x=7, dim_z=7)

        # Initialize state
        self.my_filter.x = np.zeros((7,1))
        self.my_filter.x[3] = 1.0 								# quat w
        # setup F matrix 
        F_mat = np.identity(7)
        #F_mat[0:3,4:7] = self.time_period * np.identity(3)
        self.my_filter.F = F_mat
        
        self.my_filter.H = np.identity(7)
        # ~ print("P before: ")
        # ~ print(self.my_filter.P)
        # ~ self.my_filter.P *= 1000 
        # ~ self.my_filter.P = np.zeros((7,7)) 					# changed by Cody
        # ~ self.my_filter.P_prior = np.zeros((7,7)) 		
        # ~ print("P after: ")
        # ~ print(self.my_filter.P)
        
        # Setup R matrix
        # R should be the covariance of the measurement noise
        
        # measured from David. From Kitti dataset?
        # ~ R_mat_raw = np.array([[ 3.77705992e-02, -1.76643042e-02,  8.07051877e-03,  7.44030964e-03, -2.33498754e-04,  9.20007703e-04,  4.78161804e-04],
                              # ~ [-1.76643042e-02,  2.92599905e-02, -2.20114817e-02, -1.21479866e-02,  1.60031030e-04, -1.86763252e-03, -4.72050550e-04],
                              # ~ [ 8.07051877e-03, -2.20114817e-02,  1.17050587e-01,  4.26926491e-03, -3.12529353e-04,  4.05137218e-03,  1.37673572e-03],
                              # ~ [ 7.44030964e-03, -1.21479866e-02,  4.26926491e-03,  3.06703190e-02, -2.05238748e-05,  1.17548082e-03, -9.61173256e-04],
                              # ~ [-2.33498754e-04,  1.60031030e-04, -3.12529353e-04, -2.05238748e-05,  2.45308854e-04,  7.20462044e-05, -2.69492077e-04],
                              # ~ [ 9.20007703e-04, -1.86763252e-03,  4.05137218e-03,  1.17548082e-03,  7.20462044e-05,  2.39534072e-03,  8.86890015e-05],
                              # ~ [ 4.78161804e-04, -4.72050550e-04,  1.37673572e-03, -9.61173256e-04, -2.69492077e-04,  8.86890015e-05,  1.61419177e-03]])
        R_mat_raw = np.zeros((7,7))
        # ~ R_mat_raw[0,0] = 3.77705992e-02
        # ~ R_mat_raw[1,1] = 2.92599905e-02
        # ~ R_mat_raw[2,2] = 1.17050587e-01
        # ~ R_mat_raw[3,3] = 3.06703190e-02
        # ~ R_mat_raw[4,4] = 2.45308854e-04
        # ~ R_mat_raw[5,5] = 2.39534072e-03
        # ~ R_mat_raw[6,6] = 1.61419177e-03
        R_mat_raw[0,0] = 0.1
        R_mat_raw[1,1] = 0.1
        R_mat_raw[2,2] = 0.1
        R_mat_raw[3,3] = 0.1
        R_mat_raw[4,4] = 0.1
        R_mat_raw[5,5] = 0.1
        R_mat_raw[6,6] = 0.1
        # ~ R_mat_raw[0,0] = 100.0
        # ~ R_mat_raw[1,1] = 100.0
        # ~ R_mat_raw[2,2] = 100.0
        # ~ R_mat_raw[3,3] = 100.0
        # ~ R_mat_raw[4,4] = 100.0
        # ~ R_mat_raw[5,5] = 100.0
        # ~ R_mat_raw[6,6] = 100.0
		
		
        R_mat = np.identity(7)
        for i in range(7):
            R_mat[i,i] = R_mat_raw[i,i]
        
        self.my_filter.R = R_mat_raw
       
        # setup Q matrix
        # Q should be the covariance of the process noise
        N = np.zeros((7,7))
        # ~ N[0,0] = 0.1
        # ~ N[1,1] = 0.1
        # ~ N[2,2] = 0.1
        # ~ N[3,3] = 0.1
        # ~ N[4,4] = 0.2
        # ~ N[5,5] = 0.2
        # ~ N[6,6] = 0.2
        # ~ N[0,0] = 0.001
        # ~ N[1,1] = 0.001
        # ~ N[2,2] = 0.001
        # ~ N[3,3] = 0.001
        # ~ N[4,4] = 0.001
        # ~ N[5,5] = 0.001
        # ~ N[6,6] = 0.001
        N[0,0] = 0.01
        N[1,1] = 0.01
        N[2,2] = 0.01
        N[3,3] = 0.01
        N[4,4] = 0.1
        N[5,5] = 0.1
        N[6,6] = 0.1
        
        Q_mat = np.matmul(np.matmul(F_mat, N), F_mat.T)
        
        self.my_filter.Q = Q_mat
        self.fake_Q = True
        
        # added by Cody
        self.fake_Q_mat = Q_mat
        
        #self.quest_sub = message_filters.Subscriber("/vbme/Transform", TransformStamped)
        #self.velest_sub = message_filters.Subscriber("/output/real_velocity", velocity)
        self.sub = rospy.Subscriber("/vbme/VestData", VbmeData_msg, self.callback, queue_size = 10)
        #self.pub = rospy.Publisher("/vbme/EKF", ekf_transform, queue_size=3)
        self.pub = rospy.Publisher("/vbme/EKF", VbmeData_msg, queue_size=10)

        rospy.init_node("kalman", anonymous=True)
        
        #self.ts = message_filters.TimeSynchronizer([self.quest_sub, self.velest_sub], 20)
        #self.ts.registerCallback(self.callback)
        rospy.spin()

    def linearized_state_transition_matrix(self, observation):
        # ~ rospy.loginfo("observation \n%s \n" %( observation ))
        # I'm not sure if q should be [x, y, z, w] or [w, x, y, z]. Currently it's the first option.
        # ~ q = observation[0:4,0].reshape(4,)
        q = self.prev_quat.reshape(4,)
        #w = observation[4:7,0].reshape(3,)
        w = self.prev_omega.reshape(3,)
        if np.linalg.norm(w, 2) == 0.0: 							# exactly zero will cause a divide by zero error later
			rospy.logwarn("Angular velocity (w) norm is zero, subsituting for %f" %(1.0E-64))
			w = np.array([1.0E-64, 1.0E-64, 1.0E-64]).reshape(3,) 

        F_mat = np.identity(7)

        Q_tran = np.zeros((4,4))
        Q_sigma = np.zeros((4,3))

        # Setup the linearized version of the current state transition matrix
        # ~ A = np.cos(np.linalg.norm(w, 2)*self.time_period/2)
        A = np.cos( (np.linalg.norm(w, 2))*(self.time_period/2) )
        B = (2/np.linalg.norm(w, 2)) * np.sin(np.linalg.norm(w, 2)*self.time_period/2)

        sigma_skew = 0.5 * np.array([[   0, -w[0], -w[1], -w[2]],
                                     [w[0],     0, -w[2],  w[1]],
                                     [w[1],  w[2],     0, -w[0]],
                                     [w[2], -w[1],  w[0],     0]])

        sigma_skew_partial = [0,0,0]
        sigma_skew_partial[0] = 0.5 * np.array([[   0, -w[0],     0,     0],
                                                [w[0],     0,     0,     0],
                                                [   0,     0,     0, -w[0]],
                                                [   0,     0,  w[0],     0]])
        
        sigma_skew_partial[1] = 0.5 * np.array([[   0,     0, -w[1],     0],
                                                [   0,     0,     0,  w[1]],
                                                [w[1],     0,     0,     0],
                                                [   0, -w[1],     0,     0]])
        
        sigma_skew_partial[2] = 0.5 * np.array([[   0,     0,     0, -w[2]],
                                                [   0,     0, -w[2],     0],
                                                [   0,  w[2],     0,     0],
                                                [w[2],     0,     0,     0]])
        
        # ~ rospy.loginfo("A start \n%s \n" %( A ))
        Q_tran = A*np.identity(4) + B*sigma_skew
        for i in range(3):
            A = (-w[i]*self.time_period/(2*np.linalg.norm(w,2))) * np.sin(np.linalg.norm(w, 2)*self.time_period/2)
            # ~ rospy.loginfo("A %d \n%s \n" %( i, A ))
            # ~ rospy.loginfo("w %d \n%s \n" %( i, w ))
            # ~ rospy.loginfo("w[i] %d \n%f \n" %( i, w[i] ))
            # ~ rospy.loginfo("W norm %d \n%f \n" %( i, np.linalg.norm(w, 2) ))
            # ~ rospy.loginfo("A1 %d \n%f \n" %( i, -w[i]*self.time_period/(2*np.linalg.norm(w,2))) )           
            # ~ rospy.loginfo("A2 %d \n%f \n" %( i, np.sin(np.linalg.norm(w, 2)*self.time_period/2 )))
            
            B = ( (w[i]*self.time_period/(np.linalg.norm(w,2)**2)) * np.cos(np.linalg.norm(w, 2)*self.time_period/2) ) - ( 2*w[i]/(np.linalg.norm(w,2)**3) * np.sin(np.linalg.norm(w,2)*self.time_period/2) )
            
            C = (2/np.linalg.norm(w,2)) * np.sin(np.linalg.norm(w, 2)*self.time_period/2)
            
            Q_sigma[:,i] = np.matmul(A*np.identity(4) + B*sigma_skew + C*sigma_skew_partial[i], q)


        # ~ rospy.loginfo("A \n%s \n" %( A ))
        # Change the current state transition matrix
        F_mat[0:4,0:4] = Q_tran
        F_mat[0:4,4:7] = Q_sigma
        
        # ~ rospy.loginfo("observation2 \n%s \ntime_period \n%s \nq \n%s \nw \n%s \nA \n%s \nA1 \n%s \nA2 \n%s \nB \n%s \nQ_tran \n%s \nQ_sigma \n%s \nF_mat \n%s \n \n" %(observation, self.time_period, q, w, A, np.linalg.norm(w, 2), self.time_period/2, B, Q_tran, Q_sigma, F_mat))
        
        return F_mat





    def callback(self, ros_data):
        
        # time stamps should match, although seq numbers might not match
        # assert(quest.header.stamp == velest.header.stamp)


        observation = np.array([[0],[0],[0],[0],[0],[0],[0]], dtype= np.float64)
        observation[0,0] = ros_data.transform.rotation.x
        observation[1,0] = ros_data.transform.rotation.y
        observation[2,0] = ros_data.transform.rotation.z
        observation[3,0] = ros_data.transform.rotation.w
    
        observation[4,0] = ros_data.Velocity.angular.x
        observation[5,0] = ros_data.Velocity.angular.y
        observation[6,0] = ros_data.Velocity.angular.z

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
            func_jacobian = lambda x: np.identity(7)
            func_measurement = lambda x: x
            
            F_mat = self.linearized_state_transition_matrix(observation)
            self.my_filter.F = F_mat 
            if self.fake_Q:
                # ~ N = np.zeros((7,7))
                # ~ N[0,0] = 0.1
                # ~ N[1,1] = 0.1
                # ~ N[2,2] = 0.1
                # ~ N[3,3] = 0.1
                # ~ N[4,4] = 0.2
                # ~ N[5,5] = 0.2
                # ~ N[6,6] = 0.2
                # ~ Q_mat = np.matmul(np.matmul(F_mat, N), F_mat.T)
                Q_mat = self.fake_Q_mat # added by Cody
                self.my_filter.Q = Q_mat
                self.fake_Q = False
            # ~ print("my_filter: %s" %(self.my_filter))
            # ~ rospy.loginfo("observation \n%s \nfunc_jacobian \n%s \nfunc_measurement \n%s\n" %(observation, func_jacobian, func_measurement ))
            # ~ rospy.loginfo("filter \n%s \n" %(self.my_filter ))
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
        
        # re normalize quat
        mag = math.sqrt( result[0,0]**2 + result[1,0]**2 + result[2,0]**2 + result[3,0]**2 )
        result[0,0] = result[0,0] / mag
        result[1,0] = result[1,0] / mag
        result[2,0] = result[2,0] / mag
        result[3,0] = result[3,0] / mag
        
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

        # Overwrite values with current results
        new_msg.transform.rotation.x = result[0,0]
        new_msg.transform.rotation.y = result[1,0]
        new_msg.transform.rotation.z = result[2,0]
        new_msg.transform.rotation.w = result[3,0]
    
        new_msg.Velocity.angular.x = result[4,0]
        new_msg.Velocity.angular.y = result[5,0]
        new_msg.Velocity.angular.z = result[6,0]

        self.prev_omega = result[4:7,0]
        self.prev_quat = result[0:4,0]
        delta_omega = result[4:7,0] - observation[4:7,0]
        #assert(np.linalg.norm(delta_omega, 2) > 0.0001)
        self.pub.publish(new_msg)
        
        
        # ~ rospy.loginfo("save w \n%s \n" %(self.prev_omega ))
        
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
