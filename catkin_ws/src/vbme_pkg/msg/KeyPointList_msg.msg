# packaging of opencv keypoint object https://docs.opencv.org/3.4/d2/d29/classcv_1_1KeyPoint.html
# this is meant to build a list of keypoints and use there contructor

std_msgs/Header 			Header
vbme_pkg/KeyPoint_msg[] 	KeyPointList 		# details in sub msg file
string 						KeyType 			# sift / surf / orb
sensor_msgs/Image 			Descriptors 		# Descripters can be encoded as a image

