# This msg is designed for QuEst, Vest, SBA, and vbme EKF to share data in any order

Header 								header
vbme_msgs/OpticalFlow_msg 			MatchList 			# raw feature points
float32[]  							Depths 				# QuEst point depths
float32[] 							DepthsVest 			# depths from vest or combined big C matrix
geometry_msgs/Transform		 		TransformCam0		# Transform for cam0 from t_0, the older camera positon. 
geometry_msgs/Point32[] 			X1 					# QuEst Chosen 5 points from image 1
geometry_msgs/Point32[] 			X2 					# QuEst Chosen 5 points from image 2
float32[] 							Rotation 			# rotation matrix 3x3 [r1 , r2 , r3]
geometry_msgs/Transform		 		transform 			# QuEst output. Transform from begining to end camera

geometry_msgs/Twist 				Velocity 			# Vest output
geometry_msgs/Point32[] 			Points3D 			# SBA output
	
float32 							PitchCam 			# NED pitch, -90=down, 0=level, 90=up. Relative to first camera from
float32 							HeadingCam 			# heading. -90=west, 0=north, 90=east. Relative to first camera from	
float32 							PitchAbsolute 		# NED pitch, -90=down, 0=level, 90=up. Absolute from level, if given camera starting position via param
float32 							HeadingAbsolute 	# heading. -90=west, 0=north, 90=east. Absolute, if given camera starting position via param
