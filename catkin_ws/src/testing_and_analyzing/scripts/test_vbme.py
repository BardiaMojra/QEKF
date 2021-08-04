#!/usr/bin/env python

import numpy as np

import roslib
import rospy

from vbme_msgs.msg import *

class testing_node():
    def __init__(self):
        self.kitti = False
        self.missile = False
        self.output_path = rospy.get_param("/output_path")
        rospy.loginfo("Output path: %s",  self.output_path)
        #self.output_path = "/home/developer/Desktop/just_another_mkdir/"

        # Clearing output files
        f = open(self.output_path+"features_raw.csv", "w")
        f.close()
        f = open(self.output_path+"features_calib.csv", "w")
        f.close()
        f = open(self.output_path+"quest.csv", "w")
        f.close()
        f = open(self.output_path+"quest_depths.csv", "w")
        f.close()
        f = open(self.output_path+"vest_depths.csv", "w")
        f.close()
        f = open(self.output_path+"vest.csv", "w")
        f.close()
        f = open(self.output_path+"ekf_linear.csv", "w")
        f.close()
        f = open(self.output_path+"ekf_angular.csv", "w")
        f.close()
        f = open(self.output_path+"c_matrix.csv", "w")
        f.close()
        f = open(self.output_path+"SVD_results.csv", "w")
        f.close()
        f = open(self.output_path+"covariance_data.csv", "w")
        f.close()
        f = open(self.output_path+"heading_data.csv", "w")
        f.close()
        f = open(self.output_path+"time.csv", "w")
        f.close()

        rospy.Subscriber("/vbme/QuestData", VbmeData_msg, self.call_quest, queue_size=100)
        rospy.Subscriber("/vbme/VestData", VbmeData_msg, self.call_vest, queue_size=100)
        rospy.Subscriber("/vbme/EKF", VbmeData_msg, self.call_ekf, queue_size=100)
        rospy.Subscriber("/vbme/C_matrix", MatrixStamped_msg, self.call_C_matrix, queue_size=100)
        rospy.Subscriber("/vbme/V_matrix", MatrixStamped_msg, self.call_V_matrix, queue_size=100)
        rospy.Subscriber("/vbme/HeadingData", VbmeData_msg, self.call_Heading, queue_size=100)
        rospy.init_node("testing_node", anonymous=True)
        rospy.spin()

    def call_quest(self, ros_data):
        t_vec = ros_data.transform.translation
        q_vec = ros_data.transform.rotation
        idx = int(ros_data.header.seq)

        out_file = open(self.output_path+"quest.csv", "a")
        out_file.write("%d %f %f %f " % (idx, t_vec.x, t_vec.y, t_vec.z))
        out_file.write("%f %f %f %f\n" % (q_vec.x, q_vec.y, q_vec.z, q_vec.w))
        out_file.close()
        
        FeatRaw = ros_data.MatchList.Points
        X1 = ros_data.X1
        X2 = ros_data.X2
        
        # depths 
        Depths = ros_data.Depths
        
        # ~ rospy.loginfo("FeatRaw ")
        # ~ rospy.loginfo("%s " %(FeatRaw))
        # ~ rospy.loginfo("X1 %s " %(X1))
        # ~ rospy.loginfo("X2 %s " %(X2))
        
        out_file2 = open(self.output_path+"features_raw.csv", "a")
        out_file2.write("%d " % (idx))
        for i in xrange(len(FeatRaw)):
			out_file2.write("%f %f %f %f " % (FeatRaw[i].FlowArray[-1].x, FeatRaw[i].FlowArray[-1].y, FeatRaw[i].FlowArray[0].x, FeatRaw[i].FlowArray[0].y))
			# ~ print("Newest %f %f oldest %f %f " % (FeatRaw[i].FlowArray[-1].x, FeatRaw[i].FlowArray[-1].y, FeatRaw[i].FlowArray[0].x, FeatRaw[i].FlowArray[0].y))
        out_file2.write(" \n" )
        out_file2.close()
		
        out_file3 = open(self.output_path+"features_calib.csv", "a")
        out_file3.write("%d " % (idx))
        for i in xrange(len(X1)):
			out_file3.write("%f %f %f %f " % (X1[i].x, X1[i].y, X2[i].x, X2[i].y))
			# ~ print("X1 %f %f X2 %f %f " % (X1[i].x, X1[i].y, X2[i].x, X2[i].y))
        out_file3.write(" \n" )
        out_file3.close()
		
		# depths
        out_file4 = open(self.output_path+"quest_depths.csv", "a")
        out_file4.write("%d " % (idx))
        print("length %d " % (len(Depths)))
        print("%s " % ( str(ros_data.Depths) ))
        print("%d " % (idx))
        for i in xrange(len(Depths)):
			out_file4.write("%f " % (Depths[i]))
			print("%f " % (Depths[i]))
        out_file4.write(" \n" )
        print(" \n" )
        out_file4.close()
        
        

    def call_vest(self, ros_data):
        v_vec = ros_data.Velocity.linear
        w_vec = ros_data.Velocity.angular
        t_vec = ros_data.transform.translation
        Depths = ros_data.DepthsVest
        idx = int(ros_data.header.seq)

        out_file = open(self.output_path+"vest.csv", "a")
        out_file.write("%d %f %f %f " % (idx, v_vec.x, v_vec.y, v_vec.z))
        out_file.write("%f %f %f\n" % (w_vec.x, w_vec.y, w_vec.z))
        out_file.close()

        out_file = open(self.output_path+"covariance_data.csv", "a")
        out_file.write("%d %f %f %f " % (idx, t_vec.x, t_vec.y, t_vec.z))
        out_file.write("%f %f %f\n" % (v_vec.x, v_vec.y, v_vec.z))
        out_file.close()

        out_file = open(self.output_path+"vest_depths.csv", "a")
        out_file.write("%d " % (idx))
        # ~ print("length %d " % (len(Depths)))
        # ~ print("%s " % ( str(ros_data.Depths) ))
        # ~ print("%d " % (idx))
        for i in xrange(len(Depths)):
			out_file.write("%f " % (Depths[i]))
			# ~ print("%f " % (Depths[i]))
        out_file.write(" \n" )
        # ~ print(" \n" )
        out_file.close()

    def call_ekf(self, ros_data):
        EndTime = rospy.get_rostime()
        StartTime = ros_data.header.stamp
        
        t_vec = ros_data.transform.translation
        q_vec = ros_data.transform.rotation
        v_vec = ros_data.Velocity.linear
        w_vec = ros_data.Velocity.angular
        idx = int(ros_data.header.seq)
        

        out_file = open(self.output_path+"ekf_linear.csv", "a")
        out_file.write("%d %f %f %f " % (idx, t_vec.x, t_vec.y, t_vec.z))
        out_file.write("%f %f %f\n" % (v_vec.x, v_vec.y, v_vec.z))
        out_file.close()

        out_file = open(self.output_path+"ekf_angular.csv", "a")
        out_file.write("%d %f %f %f %f " % (idx, q_vec.x, q_vec.y, q_vec.z, q_vec.w))
        out_file.write("%f %f %f\n" % (w_vec.x, w_vec.y, w_vec.z))
        out_file.close()
        
        out_file = open(self.output_path+"time.csv", "a")
        out_file.write("%d %f %f %f   \n" % (idx, StartTime.to_sec(), EndTime.to_sec(), (EndTime.to_sec())-(StartTime.to_sec())))
        out_file.close()
        

    def call_C_matrix(self, ros_data):
        out_file = open(self.output_path + "c_matrix.csv", "a")

        idx = int(ros_data.header.seq)
        rows = ros_data.MatrixStamped.layout.dim[0].size
        cols = ros_data.MatrixStamped.layout.dim[1].size
        assert(rows*cols == ros_data.MatrixStamped.layout.dim[0].stride)

        C_matrix = np.array(ros_data.MatrixStamped.data).reshape(rows, cols)
       
        if np.isnan(C_matrix).any():
            out_file.write("% not enough feature points\n\n")
        
        else:
            out_file.write("C(:,:,%d) = [ " % (idx))
            for row in C_matrix:
                for value in row:
                    out_file.write("%f " % (value))
                if (row == C_matrix[-1,:]).all():
                    out_file.write("];")
                out_file.write("\n")
            out_file.write("\n")
        out_file.close()

    def call_V_matrix(self, ros_data):
        out_file = open(self.output_path + "SVD_results.csv", "a")

        idx = int(ros_data.header.seq)
        rows = ros_data.MatrixStamped.layout.dim[0].size
        cols = ros_data.MatrixStamped.layout.dim[1].size
        assert(rows*cols == ros_data.MatrixStamped.layout.dim[0].stride)

        V_matrix = np.array(ros_data.MatrixStamped.data).reshape(rows, cols)
       
        if np.isnan(V_matrix).any():
            out_file.write("% not enough feature points\n\n")
        
        else:
            chosen = V_matrix[:,-1]
            t = chosen[0:3]
            v = chosen[3:6]
            du = chosen[6::3]
            dv = chosen[7::3]
            ddu = chosen[8::3]
            assert(len(du) == 5 and len(dv) == 5 and len(ddu)==5)
            out_file.write("t(:,%d) = [%f %f %f];\n" % (idx, t[0], t[1], t[2]))
            out_file.write("v(:,%d) = [%f %f %f];\n" % (idx, v[0], v[1], v[2]))
            out_file.write("u_depth(:,%d) = [%f %f %f %f %f];\n" % (idx, du[0], du[1], du[2], du[3], du[4]))
            out_file.write("v_depth(:,%d) = [%f %f %f %f %f];\n" % (idx, dv[0], dv[1], dv[2], dv[3], dv[4]))
            out_file.write("u_dot(:,%d) = [%f %f %f %f %f];\n" % (idx, ddu[0], ddu[1], ddu[2], ddu[3], ddu[4]))
            out_file.write("\n")

        out_file.close()
        
    def call_Heading(self, ros_data):
        idx = int(ros_data.header.seq)

        out_file = open(self.output_path+"heading_data.csv", "a")
        out_file.write("%d %f %f %f %f \n" % (idx, ros_data.PitchCam, ros_data.PitchAbsolute, ros_data.HeadingCam, ros_data.HeadingAbsolute))
        out_file.close()

def main():
    App = testing_node()


if __name__ == "__main__":
    main()
