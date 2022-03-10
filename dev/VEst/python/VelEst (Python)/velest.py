#!/usr/bin/env python

import time

# helper functions
from helpers.Find_A_Matrix import *
from helpers.Find_B_Matrix import *
from helpers.Find_C_Matrix import *
from helpers.Find_AngularVelocity import *
from helpers.Find_LinearVelocity import *
from helpers.PoCo import *
from helpers.Residu import *

# ROS packages
import rospy
import roslib

# has two 64 bit float matricies
# one represents m
# one represents m_dot

from vbme_msgs.msg import m_mdot
from optical_flow.msg import velocity
from std_msgs.msg import Float64
from std_msgs.msg import Header
from std_msgs.msg import MultiArrayDimension as mdim

from vbme_msgs.msg import *

def angularError((w_est, w_true)):
    return np.linalg.norm(w_true-w_est)

def linearError((v_est, v_true)):
    return (1/np.pi)*np.arccos(np.dot(v_est, v_true)/(np.linalg.norm(v_est)*np.linalg.norm(v_true)))

class App:
    def __init__(self):
		# params
        # ~ self.NormalizeMode =  rospy.get_param('~NormalizeMode', 0)
        self.NormalizeMode = rospy.get_param("/optical_flow_group/velocity_estimator/NormalizeMode")
        self.using_SVD = rospy.get_param("/optical_flow_group/velocity_estimator/using_SVD")
        self.using_BigC = rospy.get_param("/optical_flow_group/velocity_estimator/using_BigC")
        self.NegateSol = rospy.get_param("/optical_flow_group/velocity_estimator/NegateSol")
        rospy.loginfo('NormalizeMode %s ' %(self.NormalizeMode))
        # These might not be necessary
        self.num_trials = int(rospy.get_param("num_trials")) 
        
        self.idx = 0
        # ~ self.using_SVD = False       
        
        # all logging move to new node
        #self.output_path = "/home/developer/Desktop/"
        #self.truth_path = rospy.get_param("truth_source")
        
        # Clear output file. Output moved to new node in testing pkg
        #rospy.loginfo("Clearing result files in %s" % (self.output_path))
        # ~ f = open("/home/developer/Desktop/velest_output.txt", "w")
        #f = open(self.output_path + "velest_output.txt", "w")
        #f.close()
        # ~ f = open("/home/developer/Desktop/error_analysis.txt", "w")
        #f = open(self.output_path + "error_analysis.txt", "w")
        #f.close()
            
        #f = open(self.output_path + "C_best.txt", "w")
        #f.close()

        #f = open(self.output_path + "V_best.txt", "w")
        #f.close()
		
		# pub sub
        self.subscriber = rospy.Subscriber("/vbme/QuestData", VbmeData_msg, self.callback, queue_size=100)
        self.publisher = rospy.Publisher("/vbme/VestData", VbmeData_msg, queue_size=100)
        self.publisher_c = rospy.Publisher("/vbme/C_matrix", MatrixStamped_msg, queue_size=10)
        self.publisher_v = rospy.Publisher("/vbme/V_matrix", MatrixStamped_msg, queue_size=10)
        rospy.init_node("VelEst", anonymous=True)
        
        rospy.loginfo('Vest solvinging. Using SVD %s' %(self.using_SVD))
        rospy.sleep(0.05)
        rospy.spin()

    def callback(self, ros_data):
        # This is where we unpackage m and m_dot from the ros_data
        m_mdot_msg = ros_data.MatchList.m_mdot
        rotation_matrix = ros_data.Rotation
        x1_matrix = []
        x2_matrix = []
        

        if len(rotation_matrix) == 9:
            rotation_matrix = np.array(rotation_matrix).reshape((3,3))
        else:
            rotation_matrix = np.array([0])
        
        for point in ros_data.X1:
            point_array = np.array([point.x, point.y, point.z]).reshape((3,1))
            try:
                x1_matrix = np.append(x1_matrix, point_array, axis=1)
            except IndexError:
                x1_matrix = point_array

        for point in ros_data.X2:
            point_array = np.array([point.x, point.y, point.z]).reshape((3,1))
            try:
                x2_matrix = np.append(x2_matrix, point_array, axis=1)
            except IndexError:
                x2_matrix = point_array
        
        if len(x1_matrix) < 1 or len(x2_matrix) < 1:
            x1_matrix = np.array([0])
            x2_matrix = np.array([0])

        can_check = True
        if x1_matrix.shape != (3,5) or x2_matrix.shape != (3,5):
            can_check = False

        #x1 should be the same as m
        #x2 is the extra argument to PoCo

        r = m_mdot_msg.m.layout.dim[0].size
        c = m_mdot_msg.m.layout.dim[1].size
        assert(r*c == m_mdot_msg.m.layout.dim[0].stride)
        m = np.array(m_mdot_msg.m.data).reshape(r,c)
           
        r = m_mdot_msg.m_dot.layout.dim[0].size
        c = m_mdot_msg.m_dot.layout.dim[1].size
        assert(r*c == m_mdot_msg.m_dot.layout.dim[0].stride)
        m_dot = np.array(m_mdot_msg.m_dot.data).reshape(r,c)
        
        if can_check:
            m_temp = np.array([row for row in m.T], ndmin=2)
       
            for idx, row in enumerate(m_temp):
                m_temp[idx] = np.divide(row, sum(np.absolute(row)))

            m_temp = m_temp.T

            error = m_temp[:,:-1] - x1_matrix
            if not (np.absolute(error).sum() < 0.0001):
				rospy.logwarn("Feature Points are not in sync")
            # ~ assert(np.absolute(error).sum() < 0.0001) 		# check feature point sync of quest and vest


        for trial in range(self.num_trials):    
            v, w, t, C, V, depths = PoCo(m, m_dot, rotation_matrix, x2_matrix, self.using_SVD, self.using_BigC)
			# ~ rospy.loginfo("Depths: \n%s" %(depths))
            if v is None or w is None:
                v = np.array([np.nan, np.nan, np.nan])
                w = np.array([np.nan, np.nan, np.nan])
                t = np.array([np.nan, np.nan, np.nan])
                C = np.array([[np.nan, np.nan], [np.nan, np.nan]], ndmin=2)
                V = np.array([[np.nan, np.nan], [np.nan, np.nan]], ndmin=2)
            
            v = v.reshape(3,)
            

            # Create output message for EKF or other node
            ## pass through the variables we are not changing 
            Output = VbmeData_msg()
            Output.header = ros_data.header
            Output.MatchList = ros_data.MatchList
            # ~ Output.Depths = ros_data.Depths
            Output.Depths =[]
            
            Output.X1 = ros_data.X1
            Output.X2 = ros_data.X2
            Output.Rotation = ros_data.Rotation
            
            # rescale based on mode
            scale = 1.0;
            if self.NormalizeMode == 0: 					# by vest depth
                if self.using_BigC:
					scale = depths[0][0]
					Output.transform.translation.x = t[0]  / scale
					Output.transform.translation.y = t[1]  / scale
					Output.transform.translation.z = t[2]  / scale
					Output.transform.rotation = ros_data.transform.rotation # divide?
					for i in xrange(5):
						Output.DepthsVest.append(depths[0][i] / scale)
						Output.DepthsVest.append(depths[1][i] / scale)
                else: 
					scale = depths[0]
					Output.transform = ros_data.transform # translation and quat
					for i in xrange(len(depths)):
						Output.DepthsVest.append(depths[i] / scale)
                Output.Velocity.linear.x = v[0] / scale
                Output.Velocity.linear.y = v[1] / scale
                Output.Velocity.linear.z = v[2] / scale
                
                Output.Velocity.angular.x = w[0] / scale
                Output.Velocity.angular.y = w[1] / scale
                Output.Velocity.angular.z = w[2] / scale
                
            elif self.NormalizeMode == 1: 					# normalize to magnitude 1.0
                if self.using_BigC:
					scale = depths[0][0]
					mag = np.sqrt( (t[0]**2 + t[1]**2 + t[2]**2) )
					Output.transform.translation.x = t[0]  / mag
					Output.transform.translation.y = t[1]  / mag
					Output.transform.translation.z = t[2]  / mag
					Output.transform.rotation = ros_data.transform.rotation
					for i in xrange(5):
						Output.DepthsVest.append(depths[0][i] / scale)
						Output.DepthsVest.append(depths[1][i] / scale)
                else: 
					scale = depths[0]
					for i in xrange(len(depths)):
						Output.DepthsVest.append(depths[i] / scale)
                mag = np.sqrt( (v[0]**2 + v[1]**2 + v[2]**2) )
                Output.Velocity.linear.x = v[0] / mag
                Output.Velocity.linear.y = v[1] / mag
                Output.Velocity.linear.z = v[2] / mag
				
            elif self.NormalizeMode == 2: 					# normalize to quest and vest depths
				# scale all vest terms by v1q/(u1q*v1v)
				# this assumes quest divides its solution by u1q already
                # ~ u1q = ros_data.Depths[0]
                v1q = ros_data.Depths[1]
                if self.using_BigC:
                    # ~ v1v = depths[0][0]
                    # ~ scale = v1q/(u1q*v1v)
                    # ~ scale = v1q / v1v
                    scale = depths[0][0]
                    Output.transform.translation.x = t[0] / scale
                    Output.transform.translation.y = t[1] / scale
                    Output.transform.translation.z = t[2] / scale
                    for i in xrange(5):
						Output.DepthsVest.append(depths[0][i] / scale)
						Output.DepthsVest.append(depths[1][i] / scale)
                else: 
					Output.transform.translation = ros_data.transform.translation
					v1v = depths[0]
					# ~ scale = v1q/(u1q*v1v)
					scale = v1q / v1v
					Output.transform.translation = ros_data.transform.translation
					for i in xrange(len(depths)):
						Output.DepthsVest.append(depths[i] / scale)
                Output.Velocity.linear.x = v[0] / scale
                Output.Velocity.linear.y = v[1] / scale
                Output.Velocity.linear.z = v[2] / scale
                
                Output.Velocity.angular.x = w[0] 
                Output.Velocity.angular.y = w[1] 
                Output.Velocity.angular.z = w[2] 
                
                Output.transform.rotation = ros_data.transform.rotation
            
            if self.NegateSol:
                Output.Velocity.linear.x = - Output.Velocity.linear.x
                Output.Velocity.linear.y = - Output.Velocity.linear.y
                Output.Velocity.linear.z = - Output.Velocity.linear.z
                Output.Velocity.angular.x = - Output.Velocity.angular.x
                Output.Velocity.angular.y = - Output.Velocity.angular.y
                Output.Velocity.angular.z = - Output.Velocity.angular.z
                if self.using_BigC:
                    Output.transform.translation.x = - Output.transform.translation.x
                    Output.transform.translation.y = - Output.transform.translation.y
                    Output.transform.translation.z = - Output.transform.translation.z
                    Output.transform.rotation.x = - Output.transform.rotation.x
                    Output.transform.rotation.y = - Output.transform.rotation.y
                    Output.transform.rotation.z = - Output.transform.rotation.z
                    # do not negate rotation.w
            
            self.publisher.publish(Output)

            # Publish the C matrix
            C_msg = MatrixStamped_msg()
            C_msg.header = ros_data.header

            dimensions = []
            dimensions.append(mdim())
            dimensions.append(mdim())
            C_msg.MatrixStamped.layout.dim = dimensions
            
            C_msg.MatrixStamped.layout.dim[0].label = "rows"
            C_msg.MatrixStamped.layout.dim[0].size = C.shape[0]
            C_msg.MatrixStamped.layout.dim[0].stride = C.shape[0] * C.shape[1]
            
            C_msg.MatrixStamped.layout.dim[1].label = "cows"
            C_msg.MatrixStamped.layout.dim[1].size = C.shape[1]
            C_msg.MatrixStamped.layout.dim[1].stride = C.shape[1]

            C_msg.MatrixStamped.data = C.reshape(-1,)

            self.publisher_c.publish(C_msg)

            # Publish the SVD result, it can be edited later
            # ~ if self.using_SVD:
            if False:
				if (V.any() != None):
					V_msg = MatrixStamped_msg()
					V_msg.header = ros_data.header

					dimensions = []
					dimensions.append(mdim())
					dimensions.append(mdim())
					V_msg.MatrixStamped.layout.dim = dimensions
				
					V_msg.MatrixStamped.layout.dim[0].label = "rows"
					V_msg.MatrixStamped.layout.dim[0].size = V.shape[0]
					V_msg.MatrixStamped.layout.dim[0].stride = V.shape[0] * V.shape[1]
				
					V_msg.MatrixStamped.layout.dim[1].label = "cows"
					V_msg.MatrixStamped.layout.dim[1].size = V.shape[1]
					V_msg.MatrixStamped.layout.dim[1].stride = V.shape[1]

					V_msg.MatrixStamped.data = V.reshape(-1,)

					self.publisher_v.publish(V_msg)
				else:
					rospy.logerr('SVD matrix invalid cannot publish')
        
        self.idx += 1


def main():
    A = App()


if __name__ == "__main__":
    main()


