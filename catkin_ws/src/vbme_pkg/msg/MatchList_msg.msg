# packaging of opencv vector DMatch object https://docs.opencv.org/3.4/d4/de0/classcv_1_1DMatch.html
# this is meant to build a list of DMatch and use there contructor

std_msgs/Header 			Header
# vbme_pkg/Match_msg[] 		MatchList 		# details in sub msg file. no longer used
float32[] 					MatchDistance 	# distance taken from DMatch, and sorted with the keypoints below
string 						KeyType 		# sift / surf / orb
vbme_pkg/KeyPoint_msg[] 	KeyPtList1 		# first keypoint list, details in sub file
vbme_pkg/KeyPoint_msg[] 	KeyPtList2 		# second keypoint list. Both these lists should be sorted to where KeyPtList1[0] matches KeyPtList2[0], KeyPtList1[1] matches KeyPtList2[1], etc
