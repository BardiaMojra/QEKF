#include <chrono> // time module 
#include <random>  
#include <math.h>   												// for nan

#include "vbme_pkg/CVXGEN/solver.h" 								// for quadratic programming solver

#include "ros/ros.h"
#include <tf/tf.h> 													// for quaternion rotation transformations
//~ #include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
//~ #include "vbme_pkg/FlowMatchList_msg.h"
#include "vbme_msgs/OpticalFlow_msg.h"
#include "vbme_msgs/VbmeData_msg.h"

#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/calib3d.hpp> 
#include <opencv2/features2d.hpp> 
#include <opencv2/xfeatures2d.hpp> 

#include <Eigen/Dense>  											// sudo apt install libeigen3-dev
#include <Eigen/Eigenvalues> 	
#include <Eigen/QR> 
#include <Eigen/SVD>

#include <armadillo>  												// https://solarianprogrammer.com/2017/03/24/getting-started-armadillo-cpp-linear-algebra-windows-mac-linux/


// custom headers
//~ #include <vbme_pkg/PackFeatDescptMatch.h>

// for QP
Vars vars;
Params params;
Workspace work;
Settings settings;
int SolveMode; 														// populated from param server
int DepthMode; 														// populated from param server

// For plotting and testing depths
// no longer part of testing, depths are now published
Eigen::MatrixXd Csave;
Eigen::MatrixXd Csave2;
Eigen::MatrixXd Nsave;
Eigen::MatrixXd Nsave2;
double DepthScale = 1.0; 											// for scaling (normalizing) by depth, and chaining by depth

// Use global veriables to pull data out of static methods
// // depths are now a core variable
Eigen::MatrixXd Depthsave; 	 										
Eigen::MatrixXd Depthsave2;
Eigen::Matrix3Xd X1_select;
Eigen::Matrix3Xd X2_select;

int Ccount = 0; 													// counts callbacks

// ROS PARAMS
bool PrintToScreen;
bool UseRansacTrans; 												// rosparam, QuEst translation with or without Ransac
// QuEst params, pulled from param server
double distance_threshold;// = 1e-6; 
double p;// = 0.99; 												// desired probability of choosing at least one sample free from outliers 
int maxDataTrials;// = 100; 										// max number to select non-degenerate data set 
int maxIter;// = 1000; 												// max number of iterations 

ros::Time TimeBegin; 												// measure time at begining of call back, subtract at publishing

class QuEst
{
public: // public vars
	
private: // private varsS
	ros::NodeHandle * n;
	//~ bool PrintToScreen; 										// loaded from rosparam, flag to print
	//~ bool DrawImages; 											// loaded from rosparam, flag to display windows
	int TrackLength; 												// loaded from rosparam, max length of optical flow point trails
	bool ActivateChaining; 											// loaded from rosparam, use chaining when input array shrinks
	bool LoopSlow; 													// loaded from rosparam, flag to use sleep in loops to view images
	float LoopSleep; 												// rosparam, time to sleep in a loop secs
	bool UseRansac; 												// rosparam, QuEst with or without Ransac
	bool CalcQuat; 													// rosparam, calculate quaternion or skip. used for more complicated pipleline
	bool CalcTrans; 												// rosparam, calculate translation or skip. used for more complicated pipleline
	int NormalizeOutput; 											// rosparam, QuEst with or without output normilized 
	int PostFilter; 												// rosparam, QuEst with or without output filtering such as median filter
	std::vector<geometry_msgs::Transform> FilterWindow; 			// window/kernal to hold data for filter
	vbme_msgs::OpticalFlow_msg MsgLast; 							// holds old msg for chaining events
	std::vector<geometry_msgs::TransformStamped> Outputs_u; 		// vector of prior outputs. Hold onto for chaining events
	std::vector<double> CA; 										// array to hold calibration matrix for param server
	Eigen::Matrix3d CalibrationMatrix;
	// homogeneous transform matrix to hold data lost via chaining when optical flow loses points
	std::vector<Eigen::Matrix4d> HChain; 										
	// index to get transform from t_0 to the position chosen for cam0 (M, X1). Final output is for cam1(N, X2)
	int Cam0Index;
	geometry_msgs::Quaternion SavedQuat; 							// used with CallbackPipeline to save quat for CalcQuat=false. saves quat from input msg
	
	geometry_msgs::TransformStamped quest_msg; 						// primary output variable
	ros::Subscriber SubPipelineMatch; 								// input type is same as output, used for more complicated pipleline
	ros::Subscriber SubMatches; 									// only activated if we draw images, unused otherwise
	ros::Subscriber SubMatch;
	ros::Publisher PubTransform;
	
	// original QuEst methods
	void QuEst_RANSAC(const Eigen::Matrix3Xd& m, const Eigen::Matrix3Xd& n, geometry_msgs::TransformStamped & ReturnOutput); 
	void GetKeyPointsAndDescriptors(const cv::Mat& img_1_input, const cv::Mat& img_2_input, 
		std::vector<cv::KeyPoint>& keypoints_1_raw, std::vector<cv::KeyPoint>& keypoints_2_raw, 
		cv::Mat& descriptors_1, cv::Mat& descriptors_2); 
	void MatchFeaturePoints(const cv::Mat& descriptors_1, const cv::Mat& descriptors_2, 
						std::vector<cv::DMatch>& matches_select); 

	static bool ransac(void (*QuEst_fit)(const Eigen::MatrixXd&, const std::vector<int>&, Eigen::VectorXd&), 
				const Eigen::MatrixXd& data, const std::vector<int>& ind, Eigen::VectorXd& test_model, 
			void (*QuEst_distance)(const Eigen::MatrixXd&, const Eigen::VectorXd&, const double, Eigen::VectorXd&, std::vector<int>&), 
				const Eigen::MatrixXd& data_1, const Eigen::VectorXd& ind_1, const double distance_threshold, Eigen::VectorXd& select_model, std::vector<int>& select_inliers, 
			bool (*QuEst_degenerate)(const Eigen::MatrixXd&, const std::vector<int>&),  
				const Eigen::MatrixXd& data_2, const std::vector<int>& ind_2, 
			const int minimumSizeSamplesToFit, 
			Eigen::VectorXd& best_model, 
			std::vector<int>& best_inliers); 

	static void QuEst_fit(const Eigen::MatrixXd& allData, const std::vector<int>& useIndices, Eigen::VectorXd& test_model); 
	static void QuEst_fit(const Eigen::Matrix3Xd & x1, Eigen::Matrix3Xd & x2, const std::vector<int>& useIndices, Eigen::VectorXd& test_model);
	static void QuEst_distance(const Eigen::MatrixXd& data, const Eigen::VectorXd& test_model, const double distance_threshold, 
		Eigen::VectorXd& select_model, std::vector<int>& select_inliers); 

	static bool QuEst_degenerate(const Eigen::MatrixXd& data, const std::vector<int>& ind); 

	static void Q2R(Eigen::MatrixXd& R, const Eigen::Matrix4Xd& Q); 					// quaternion to rotation matrix 
	static void Q2R_3by3(Eigen::Matrix3d& R_Q2R, const Eigen::Vector4d& Q); 
	static void R2Q_3by3(const Eigen::Matrix3d& R_R2Q,  Eigen::Vector4d& Q);

	static void CoefsVer_3_1_1(Eigen::MatrixXd& Cf, const Eigen::Matrix3Xd& m1, const Eigen::Matrix3Xd& m2); 
	static void QuEst_5Pt_Ver5_2(Eigen::Matrix4Xd& Q ,const Eigen::Matrix3Xd& m, const Eigen::Matrix3Xd& n); 
	static void FindDepths(const Eigen::MatrixXd& allData, const Eigen::Vector4d& Q);
	static void FindDepths(const Eigen::Matrix3Xd& m, const Eigen::Matrix3Xd& n, const Eigen::Vector4d& Q);
	static void FindTrans(Eigen::Vector3d& T, const Eigen::MatrixXd& allData, const Eigen::Vector4d& Q);
	static void FindTrans(Eigen::Vector3d& T, const Eigen::Matrix3Xd& m, const Eigen::Matrix3Xd& n, const Eigen::Vector4d& Q);
	static void QuEst_Ver1_1(Eigen::Matrix4Xd& Q, const Eigen::Matrix3Xd& m, const Eigen::Matrix3Xd& n);  
	void QuEst_once_pub(Eigen::Matrix3Xd & m1, Eigen::Matrix3Xd & m2, geometry_msgs::TransformStamped & HeaderandReturnOutput);
	static void QuatResidue(Eigen::RowVectorXd& residu, const Eigen::Matrix3Xd& m1, const Eigen::Matrix3Xd& m2, const Eigen::Matrix4Xd& qSol); 
	
	static void coefsNum(Eigen::MatrixXd& coefsN, const Eigen::VectorXd& mx1, const Eigen::VectorXd& mx2, 
		const Eigen::VectorXd& my1, const Eigen::VectorXd& my2, 
		const Eigen::VectorXd& nx2, const Eigen::VectorXd& ny2, 
		const Eigen::VectorXd& r2, const Eigen::VectorXd& s1, const Eigen::VectorXd& s2); 
	static void coefsDen(Eigen::MatrixXd& coefsD, const Eigen::VectorXd& mx2, const Eigen::VectorXd& my2, 
		const Eigen::VectorXd& nx1, const Eigen::VectorXd& nx2, 
		const Eigen::VectorXd& ny1, const Eigen::VectorXd& ny2, 
		const Eigen::VectorXd& r1, const Eigen::VectorXd& r2, const Eigen::VectorXd& s2); 
	static void coefsNumDen(Eigen::MatrixXd& coefsND, const Eigen::VectorXd& a1, const Eigen::VectorXd& a2, const Eigen::VectorXd& a3, 
		const Eigen::VectorXd& a4, const Eigen::VectorXd& a5, const Eigen::VectorXd& a6, const Eigen::VectorXd& a7, 
		const Eigen::VectorXd& a8, const Eigen::VectorXd& a9, const Eigen::VectorXd& a10, 
		const Eigen::VectorXd& b1, const Eigen::VectorXd& b2, const Eigen::VectorXd& b3, const Eigen::VectorXd& b4, 
		const Eigen::VectorXd& b5, const Eigen::VectorXd& b6, const Eigen::VectorXd& b7, const Eigen::VectorXd& b8, 
		const Eigen::VectorXd& b9, const Eigen::VectorXd& b10); 
		
	
	//~ static void SolveQPData(const Eigen::MatrixXd C, Params params, Eigen::MatrixXd & X);
	static void SolveQPData(const Eigen::MatrixXd C, Eigen::MatrixXd & X);
	static void QuatFindNoRotationQ(Eigen::RowVectorXd& residu, const Eigen::Matrix3Xd& m1, const Eigen::Matrix3Xd& m2, const Eigen::Matrix4Xd& qSol);
	static void QuatResidueReprojection(Eigen::RowVectorXd& residu, const Eigen::Matrix3Xd& m1, const Eigen::Matrix3Xd& m2, const Eigen::Matrix4Xd& qSol);
	
public: // public methods
	QuEst(ros::NodeHandle nh) // constructor
	{
		n = &nh;
		std::string InputTopicOptical;
		std::string InputTopicPipeline;
		std::string OutputTopic;
		GetParams(InputTopicOptical, InputTopicPipeline, OutputTopic);
		//~ PubTransform = nh.advertise<geometry_msgs::TransformStamped>("Transform", 100);
		//~ PubTransform = nh.advertise<vbme_msgs::VbmeData_msg>("QuestData", 100);
		PubTransform = nh.advertise<vbme_msgs::VbmeData_msg>(OutputTopic, 100);
		
		ros::Duration(0.1).sleep(); 								// sleep to let ros clock init
		
		//~ SubMatch = nh.subscribe("optical_flow_short", 100, &QuEst::CallbackMatches, this);
		//~ SubPipelineMatch = nh.subscribe("quest_pipeline_input", 1000, &QuEst::CallbackPipeline, this);
		//~ SubMatch = nh.subscribe("optical_flow", 1000, &QuEst::Callback, this);
		SubPipelineMatch = nh.subscribe(InputTopicPipeline, 1000, &QuEst::CallbackPipeline, this);
		SubMatch = nh.subscribe(InputTopicOptical, 1000, &QuEst::Callback, this);
		
		// initialize Homogeneous transform matrix for chaining, this will just be a eye matrix (no inputs)
		Cam0Index = 1;
		ConstuctFirstHChain(); 		
		
		// starting point is 0,0,0. Calculate transform from this
		geometry_msgs::TransformStamped OriginT;
		OriginT.transform.rotation.w =1.0;
		Outputs_u.insert(Outputs_u.begin(), OriginT);
		
		// QP
		set_defaults();  // Set basic algorithm parameters.
		setup_indexing();
		if (PrintToScreen)
		{
			settings.verbose = 1;
			settings.verbose_refinement = 1;
			settings.debug = 1;
			//~ settings.max_iters = 200;
			//~ settings.better_start = 1;
			//~ settings.eps = 0.00001;
			//~ settings.resid_tol = 0.0001;
		}
		else
		{
			settings.verbose = 0;
			settings.verbose_refinement = 0;
			settings.debug = 0;
			//~ settings.max_iters = 200;
			//~ settings.better_start = 1;
			//~ settings.eps = 0.00001;
			//~ settings.resid_tol = 0.0001;
		}
		
		
		ROS_INFO("QuEst Ready");
	}
	
private: // private methods
	void GetParams(std::string & InputTopicOptical, std::string & InputTopicPipeline, std::string & OutputTopic)
	{ 
		ros::Duration(0.2).sleep();  							// some params are from different launch files
		n->getParam("InputTopicOptical", InputTopicOptical);
		ROS_INFO("InputTopicOptical %s", InputTopicOptical.c_str());
		
		n->getParam("InputTopicPipeline", InputTopicPipeline);
		ROS_INFO("InputTopicPipeline %s", InputTopicPipeline.c_str());
		
		n->getParam("OutputTopic", OutputTopic);
		ROS_INFO("OutputTopic %s", OutputTopic.c_str());
		
		n->param<bool>("PrintToScreen", PrintToScreen, true);
		//~ n->param<bool>("DrawImages", DrawImages, true);
		n->param<bool>("LoopSlow", LoopSlow, true);
		n->param<float>("LoopSleep", LoopSleep, 1.0);
		n->param<bool>("UseRansac", UseRansac, false);
		n->param<bool>("UseRansacTrans", UseRansacTrans, true);
		n->param<int>("NormalizeOutput", NormalizeOutput, 0);
		n->param<int>("track_len", TrackLength, 10);
		n->param<bool>("ActivateChaining", ActivateChaining, true);
		n->param<int>("SolveMode", SolveMode, 0);
		n->param<int>("DepthMode", DepthMode, 0);
		n->param<int>("PostFilter", PostFilter, 0);
		n->param<bool>("CalcQuat", CalcQuat, true); 			// calculate quaternion or skip. used for more complicated pipleline
		n->param<bool>("CalcTrans", CalcTrans, true); 			// calculate translation or skip. used for more complicated pipleline
		ROS_INFO("Quest - CalcQuat %d, CalcTrans %d \n", CalcQuat, CalcTrans);
		
		
		// QuEst variables
		//~ n->param<std::vector<double>>("Quest/CalibrationMatrix", CalibrationArray);
		if (ros::param::has("/optical_flow_group/camera_calibration")) 
		{
			ros::param::get("/optical_flow_group/camera_calibration", CA);
			//~ n->param<std::vector<double>>("/optical_flow_group/camera_calibration", CA);
		}
		else
		{
			ROS_ERROR("Cannot find calibration matrix on param server!");
		}
		//~ n->param<std::vector<double>>("/optical_flow_group/camera_calibration", CA);
		n->param<double>("Ransac/DistanceThreshold", distance_threshold, 1e-6);
		n->param<double>("Ransac/DesiredProbability", p, 0.99);
		n->param<int>("Ransac/MaxDataTrials", maxDataTrials, 100);
		n->param<int>("Ransac/MaxIter", maxIter, 1000);
		
		if (CA.size() != 9)
		{
			ROS_FATAL("Calibration matrix is wrong size. Size %zd ", CA.size());
		}
		CalibrationMatrix.resize(3,3);
		int i = 0;
		for (int r = 0; r < 3; r++)
		{
			for (int c = 0; c < 3; c++)
			{
				printf("%f ", CA[i]);
				CalibrationMatrix(r, c) = CA[i];
				++i;
			}
		}
		printf("\n ");
		std::cout << "Calibration Matrix: " << std::endl << CalibrationMatrix << std::endl << std::endl;
		
	}
	
	// CallbackPipeline subscribes to the same msg type as the publishers.
	// This allows putting quest in more complicated pipelines such as 
	// quest( CalcQuat ) -> angular ekf -> quest ( CalcTrans )
	void CallbackPipeline(const vbme_msgs::VbmeData_msg::ConstPtr& msg)
	{
		SavedQuat = msg->transform.rotation;
		vbme_msgs::OpticalFlow_msg OpticInput;  			// simulates normal input for other callback
		
	
	}
	void Callback(const vbme_msgs::OpticalFlow_msg::ConstPtr& msg)
	{
		// get time, subtract at publishing
		TimeBegin = ros::Time::now();
		//~ printf("%% Num Points %zd \n", msg->Points.size());
		//~ ROS_INFO("%% Trail length: %zd \n", msg->Points[0].FlowArray.size());
		printf("%% Trail length: %zd \n", msg->Points[0].FlowArray.size());
		//~ std::cout << "msg: " << std::endl << *msg << std::endl << std::endl;
		//~ for (auto i = msg->Points.begin(); i < msg->Points.end(); ++i)
		//~ {
			//~ printf("Trail length: %zd \n", i->FlowArray.size());
			//~ std::cout << "it: " << std::endl << *i << std::endl << std::endl;
		//~ }
		
		if (msg->Points.size() < 5)
		{
			ROS_WARN("Flow array does not include enough points. Points in array %zd", msg->Points.size());
			vbme_msgs::VbmeData_msg Output;
			Output.header = msg->header;
			Output.MatchList.m_mdot = msg->m_mdot;
			Output.MatchList.Points = msg->Points;
			Output.transform.translation.x = NAN;
			Output.transform.translation.y = NAN;
			Output.transform.translation.z = NAN;
			Output.transform.rotation.x = NAN;
			Output.transform.rotation.y = NAN;
			Output.transform.rotation.z = NAN;
			Output.transform.rotation.w = NAN;
			Output.TransformCam0.translation.x = NAN;
			Output.TransformCam0.translation.y = NAN;
			Output.TransformCam0.translation.z = NAN;
			Output.TransformCam0.rotation.x = NAN;
			Output.TransformCam0.rotation.y = NAN;
			Output.TransformCam0.rotation.z = NAN;
			Output.TransformCam0.rotation.w = NAN;
			PublishMsg(Output);
			return;
		}
		
		//~ ROS_INFO("Length of point trail %lu", msg->Points[0].FlowArray.size());
		//~ std::cout<< "Front point: " << msg->Points.front().FlowArray.front() << std::endl;
		//~ std::cout<< "Back point: " << msg->Points.front().FlowArray.back() << std::endl;
		
		// This code is very resistant to stopping with ctl-c
		if (!ros::ok()) 									
		{
			return;
		}
		
		//~ Eigen::Matrix3d K; 
		Eigen::Matrix3d K = CalibrationMatrix; 
		//~ K << 	9.842439e+02, 	0.0, 			6.90e+02, 
				//~ 0.0,			9.808141e+02, 	2.331966e+02, 
				//~ 0.0,			0.0,	 		1.0;  			// kitti data set
		//~ Eigen::Matrix3d K = CalibrationMatrix; 
		//~ K << 	529.957482, 		0.000000, 		313.842267, 
				//~ 0.000000, 			530.251806, 	245.472186, 
				//~ 0.000000, 			0.000000, 		1.000000;  		// UTARI Asus Xtion
		//~ K << 	1, 	0, 		400, 
				//~ 0, 		1, 	300, 
				//~ 0, 		0, 		1; 							// synthetic data
		if (PrintToScreen)
		{
			std::cout << "Calibration Matrix: " << std::endl << K << std::endl << std::endl;
		}
		
		//~ Eigen::Matrix3d K1; 
		//~ K1 << 527,0,322,0,532,257,0,0,1; 
		// K1 << 1350,0,999,0,1358,525,0,0,1; 
		//~ Eigen::Matrix3d K2; 
		//~ K2 << 517,0,306,0,521,263,0,0,1; 
		// K2 << 1413,0,919,0,1422,533,0,0,1; 
		
		// convert to Eigen variables
		Eigen::Matrix3Xd M1(3,msg->Points.size()); 
		for(int i=0;i < msg->Points.size();i++)
		{
			//~ M1(0,i) = msg->Points[i].FlowArray.back().x; 
			//~ M1(1,i) = msg->Points[i].FlowArray.back().y; 
			M1(0,i) = msg->Points[i].FlowArray.back().x; 
			M1(1,i) = msg->Points[i].FlowArray.back().y; 
			//~ std::cout<< msg->Points[i].FlowArray.front().x << std::endl;
			M1(2,i) = 1; 
			if ( (i < 5) && PrintToScreen ) 					// print first input points
			{
				ROS_INFO("M1 front point %f, %f\n", M1(0,i), M1(1,i));
			}
		}
		Eigen::Matrix3Xd M2(3,msg->Points.size());
		for(int i=0;i < msg->Points.size();i++)
		{
			//~ M2(0,i) = msg->Points[i].FlowArray.front().x; 
			//~ M2(1,i) = msg->Points[i].FlowArray.front().y; 
			M2(0,i) = msg->Points[i].FlowArray.front().x; 
			M2(1,i) = msg->Points[i].FlowArray.front().y; 
			M2(2,i) = 1; 
			if ( (i < 5) && PrintToScreen ) 					// print first input points
			{
				ROS_INFO("M2 back point %f, %f\n", M2(0,i), M2(1,i));
			}
		}
		
		// feature points in image coordinate ---------- 
		//~ Eigen::Matrix3Xd m1 = K1.inverse() * M1; 
		Eigen::Matrix3Xd m1 = K.inverse() * M1; 
		// cout<<"m1: "<<endl<<m1<<endl; 
		//~ Eigen::Matrix3Xd m2 = K2.inverse() * M2; 
		Eigen::Matrix3Xd m2 = K.inverse() * M2; 
		// cout<<"m2: "<<endl<<m2<<endl<<endl; 
		
		geometry_msgs::TransformStamped OutputCurr;
		OutputCurr.header = msg->header; 						// copy time stamp
		
		//~ ros::Time Now = ros::Time::now();
		//~ ros::Duration ComputeTime = Now - TimeBegin;
		//~ std::cout <<  "Quest compute time CallbackMatches 1:     \t\t " << ComputeTime.toSec() << std::endl;
		if (CalcQuat) 											// calculate quaternion if desired
		{
			if (UseRansac)
			{
				QuEst::QuEst_RANSAC(m1,m2, OutputCurr); 
			}
			else
			{
				//~ QuEst::QuEst(m1,m2); 
				QuEst::QuEst_once_pub(m1,m2, OutputCurr); 
			}
		}
		else 													// copy quat from input and calc translation
		{
			// normalize feature points for publishing
			Eigen::RowVectorXd x1n(1,5); 
			for(int i=0; i<5; i++){
				x1n(0,i) = abs(m1(0,i)) + abs(m1(1,i)) + abs(m1(2,i)); 
			} 
			Eigen::Matrix3Xd x1_norm(3,5); 
			for(int i=0; i<5; i++){
				x1_norm(0,i) = m1(0,i)/x1n(0,i); 
				x1_norm(1,i) = m1(1,i)/x1n(0,i); 
				x1_norm(2,i) = m1(2,i)/x1n(0,i); 
			} 
			Eigen::RowVectorXd x2n(1,5); 
			for(int i=0; i<5; i++){
				x2n(0,i) = abs(m2(0,i)) + abs(m2(1,i)) + abs(m2(2,i)); 
			} 
			Eigen::Matrix3Xd x2_norm(3,5); 
			for(int i=0; i<5; i++){
				x2_norm(0,i) = m2(0,i)/x2n(0,i); 
				x2_norm(1,i) = m2(1,i)/x2n(0,i); 
				x2_norm(2,i) = m2(2,i)/x2n(0,i); 
			} 
			
			X1_select = x1_norm; 								// save to global for publishing
			X2_select = x2_norm; 								// these will be moved to the publishing variable later
			OutputCurr.transform.rotation = SavedQuat; 			// republish input quat
		
			Eigen::Vector3d T; 
			Eigen::Vector4d Q_select; 
			Q_select(0) = SavedQuat.w; 							// use input quat to calc trans
			Q_select(1) = SavedQuat.x;
			Q_select(2) = SavedQuat.y;
			Q_select(3) = SavedQuat.z;
			FindTrans(T, m1.block(0,0,3, 5),m2.block(0,0,3, 5) ,Q_select); 
			//~ FindTrans(T, x1_norm, x2_norm ,Q_select); 
			if (DepthMode == 1)
			{
				// find the depths using the alternative c matrix
				FindDepths(m1.block(0,0,3, 5),m2.block(0,0,3, 5),Q_select); // test of alternate depth equation
				//~ FindDepths(x1_norm, x2_norm,Q_select); // test of alternate depth equation
			}
			if (T.cols() == 1) 										// without RANSAC will have a single column
			{
				OutputCurr.transform.translation.x = T(0); 
				OutputCurr.transform.translation.y = T(1); 
				OutputCurr.transform.translation.z = T(2); 
			}
		}
		//~ std::cout << "Finished calling QuEst. Output: " << std::endl << OutputCurr << std::endl;
		//~ Now = ros::Time::now();
		//~ ComputeTime = Now - TimeBegin;
		//~ std::cout <<  "Quest compute time CallbackMatches 2:     \t\t " << ComputeTime.toSec() << std::endl;
		
		// Look into chaining
		// if the length of the trail has not increased by 1, then a chaining event needs to occur
		if (ActivateChaining)
		{
			// log new data, check if vector is not too long
			if (Outputs_u.size() < 2*TrackLength)
			{
				// add latest output to the front
				Outputs_u.insert(Outputs_u.begin(), OutputCurr);
			}
			else
			{
				// remove output from the end
				Outputs_u.pop_back();
				// add latest output to the front
				Outputs_u.insert(Outputs_u.begin(), OutputCurr);
			}
			// detect if LastMsg is blank this should be the first msg. (might as well double check NewMsg)
			// we dont need to do this if PostFilter=true, ActivateChaining=false
			printf("Points: %zd, Last Point size %zd. \n", msg->Points.size(), MsgLast.Points.size());
			if ( (msg->Points.size() > 0) && (MsgLast.Points.size() > 0) )
			{
				// detect if chaining is needed
				std::cout << "Flow size: " << msg->Points[0].FlowArray.size() << ", Last Flow Size: " << MsgLast.Points[0].FlowArray.size() << std::endl;
				if ( int(msg->Points[0].FlowArray.size()) != ( int(MsgLast.Points[0].FlowArray.size()) +1) )
				{
					vbme_msgs::OpticalFlow_msg MsgNew;
					MsgNew.header = msg->header;
					MsgNew.Points = msg->Points;
					ChainingEvent(MsgLast, MsgNew, Outputs_u);
				}
				else
				{
					// Cam0 (M, the old camera view) is now 1 more time index back
					Cam0Index++;
				}
			}
			
			MsgLast = *msg;
		}
		
		// ///////////// Prepare final output message
		if (PrintToScreen)
		{
			ROS_INFO("Preparing final output to publish");
		}
		vbme_msgs::VbmeData_msg OutputFinal;
		OutputFinal.header = OutputCurr.header;
		//~ OutputFinal.TransformCam0 = OutputCurr.transform; 		// direct transfrom between cameras. can still be normalized
		OutputFinal.transform = OutputCurr.transform;			// transform that chaining can be applied to
		// depths
		for (int i = 0; i < 10; i++)
		{
			if (DepthMode == 0)
			{
				// this will pull the depths from the either the 
				// the "primary c matrix" or quadratic programing
				OutputFinal.Depths.push_back(Depthsave(i));
			}
			else if(DepthMode == 1)
			{
				// this will pull the depths from 
				// "alternative c matrix" and SVD
				OutputFinal.Depths.push_back(Depthsave2(i));
			}
		}
		
		for (int i = 0; i < (int)X1_select.cols(); i++)
		{
			geometry_msgs::Point32 Pt1i, Pt2i;
			Pt1i.x =X1_select(0, i);
			Pt1i.y =X1_select(1, i);
			Pt1i.z =X1_select(2, i);
			OutputFinal.X1.push_back(Pt1i);
			Pt2i.x =X2_select(0, i);
			Pt2i.y =X2_select(1, i);
			Pt2i.z =X2_select(2, i);
			OutputFinal.X2.push_back(Pt2i);
		}
		
		// // // // // convert calibrated m1 and m2 to ros msg
		//~ std::vector<vbme_pkg::Pixel2DFlow_msg> PixFlow; 			// the optial flow of several points
		//~ vbme_pkg::Pixel2DFlow_msg PixFlow; 							// the optial flow of several points
		//~ std::vector<geometry_msgs::Point32> FlowArray1; 			// the optical flow of a single point
		//~ std::vector<geometry_msgs::Point32> FlowArray2; 			// the optical flow of a single point
		//~ vbme_pkg::Pixel2DFlow_msg FlowArray1; 			// the optical flow of a single point
		//~ vbme_pkg::Pixel2DFlow_msg FlowArray2; 			// the optical flow of a single point
		
		//~ // transfer first point. we are now outputting everything
		//~ for (int i = 0; i < (int)m1.cols(); i++)
		//~ {
			//~ geometry_msgs::Point32 Point;
			//~ Point.x = m1(0, i);
			//~ Point.y = m1(1, i);
			//~ FlowArray1.FlowArray.push_back(Point);
		//~ }
		//~ OutputFinal.MatchList.Points.push_back(FlowArray1);
		//~ // transfer end point. we are now outputting everything
		//~ for (int i = 0; i < (int)m2.cols(); i++)
		//~ {
			//~ geometry_msgs::Point32 Point;
			//~ Point.x = m2(0, i);
			//~ Point.y = m2(1, i);
			//~ FlowArray2.FlowArray.push_back(Point);
		//~ }
		//~ OutputFinal.MatchList.Points.push_back(FlowArray2);
		
		// transfer all points. raw feature points
		OutputFinal.MatchList = *msg;
		
		// // // // // 
		// ///////////// 
		
		PublishMsg(OutputFinal);
		Ccount++; 
		// /////////////////////////////////////////////////////////////
		// output formated for matlab. verification testing
		//~ std::cout << "Count: " << Ccount << std::endl << std::endl;
		//~ std::cout << "M1(:,:," << Ccount << ") = [" << m1.block<3, 5>(0,0) << "];" << std::endl << std::endl;
		//~ std::cout << "M2(:,:," << Ccount << ") = [" << m2.block<3, 5>(0,0) << "];" << std::endl << std::endl;
		//~ std::cout << "C1(:,:," << Ccount << ") = [" << Csave << "];" << std::endl << std::endl;
		//~ std::cout << "C2(:,:," << Ccount << ") = [" << Csave2 << "];" << std::endl << std::endl;
		//~ if (SolveMode == 0)
		//~ {
			//~ // N1 isnt populated if SolveMode == 1 (quadratic programming)
			//~ std::cout << "N1(:,:," << Ccount << ") = [" << Nsave << "];" << std::endl << std::endl;
		//~ }
		//~ std::cout << "N2(:,:," << Ccount << ") = [" << Nsave2 << "];" << std::endl << std::endl;
		
		// for printing out depths for testing and verification
		//~ std::cout<<"Depth1: "<<std::endl<<Depthsave<<std::endl<<std::endl; 
		//~ std::cout<<"Depth1(:," << Ccount << ") = ["; 
		//~ if (Depthsave.cols() > 0)
		//~ {
			//~ for(int i=0;i<10;i++)
			//~ {
				//~ printf("%f ", Depthsave(i));
			//~ }
		//~ }
		//~ std::cout << "];" << std::endl << std::endl;
		// std::cout<<"Depth2: "<<std::endl<<Depthsave2<<std::endl<<std::endl; 
		
		//~ std::cout<<"Depth2(:," << Ccount << ") = ["; 
		//~ if (Depthsave2.cols() > 0)
		//~ {
			//~ for(int i=0;i<10;i++)
			//~ {
				//~ printf("%f ", Depthsave2(i));
			//~ }
		//~ }
		//~ std::cout << "];" << std::endl << std::endl;
		
		// trans and quaternion as seperate arrays
		//~ std::cout << "T(:,:," << Ccount << ") = [" << OutputCurr.transform.translation.x << " ";
		//~ std::cout << OutputCurr.transform.translation.y << " ";
		//~ std::cout << OutputCurr.transform.translation.z << "];" << std::endl << std::endl;
		//~ std::cout << "Q(:,:," << Ccount << ") = [" << OutputCurr.transform.rotation.w << " ";
		//~ std::cout << OutputCurr.transform.rotation.x << " ";
		//~ std::cout << OutputCurr.transform.rotation.y << " ";
		//~ std::cout << OutputCurr.transform.rotation.z << "];" << std::endl << std::endl;
		
		// transform
		//~ if (OutputCurr.transform.translation.z < 0.0)
		//~ {
			//~ OutputCurr.transform.translation.x = -1.0 * OutputCurr.transform.translation.x;
			//~ OutputCurr.transform.translation.y = -1.0 * OutputCurr.transform.translation.y;
			//~ OutputCurr.transform.translation.z = -1.0 * OutputCurr.transform.translation.z;
		//~ }
		//~ std::cout<<"T(:," << Ccount << ") = [";  				// matlab
		
		//~ printf("%f ", OutputCurr.transform.translation.x); 		// matlab
		//~ printf("%f ", OutputCurr.transform.translation.y);
		//~ printf("%f ", OutputCurr.transform.translation.z);
		//~ printf("%f ", OutputCurr.transform.rotation.w);
		//~ printf("%f ", OutputCurr.transform.rotation.x);
		//~ printf("%f ", OutputCurr.transform.rotation.y);
		//~ printf("%f ", OutputCurr.transform.rotation.z);
		
		//~ printf("%f, ", OutputCurr.transform.translation.x); 		// csv
		//~ printf("%f, ", OutputCurr.transform.translation.y);
		//~ printf("%f, ", OutputCurr.transform.translation.z);
		//~ printf("%f, ", OutputCurr.transform.rotation.w);
		//~ printf("%f, ", OutputCurr.transform.rotation.x);
		//~ printf("%f, ", OutputCurr.transform.rotation.y);
		//~ printf("%f, ", OutputCurr.transform.rotation.z);
		
			
		//~ std::cout << "];" << std::endl << std::endl; 			// matlab
		//~ std::cout << "\n" << std::endl << std::endl; 				// csv
		
		// /////////////////////////////////////////////////////////////
		
		// draw point matching, best 20 points ---------- 
		//~ std::vector<DMatch> matches_select_draw; 
		//~ for(int i=0;i<numPts;i++){
			//~ matches_select_draw.push_back(matches_select[i]); 
		//~ } 

		//~ Mat img_matches; 
		//~ drawMatches(img_1_input,keypoints_1_raw,
					//~ img_2_input,keypoints_2_raw,
					//~ matches_select_draw,
					//~ img_matches,
					//~ Scalar::all(-1),Scalar::all(-1),
					//~ std::vector<char>(),
					//~ DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
					//~ ); 
		//~ namedWindow("img_matches", WINDOW_NORMAL); 
		//~ imshow("img_matches",img_matches); 
		//~ printf("Matches drawn: %zd \n", matches_select_draw.size());
		//~ waitKey(1);
	} // end CallbackMatches
	
	void ChainingEvent(const vbme_msgs::OpticalFlow_msg MsgOld, const vbme_msgs::OpticalFlow_msg MsgNew, const std::vector<geometry_msgs::TransformStamped> OutputsPast)
	{
		// goals. 
		// 1 find the missing number of feature point time samples.
		// 2 calculate a Homogenious transform matrix over the set of now missing feature points
		// 3 multiple the new HTM againt the prior one.
		// 4 save output (final HTM) as class member
		// side task, find the HTM at the first missing point, multiple by corresponding HTM and save 
		
		// compare how far back in time the old msg and the new msg go
		int LengthDiff = MsgLast.Points[0].FlowArray.size() - MsgNew.Points[0].FlowArray.size();
		
		// Find indexes that we need to calculate a transform for 
		//~ int IndexTransformStart = int(MsgLast.Points[0].FlowArray.size()) - (LengthDiff + 2);
		//~ int IndexTransformStart = int(MsgLast.Points[0].FlowArray.size()) - (LengthDiff + 1);
		int IndexTransformStart = int(MsgLast.Points[0].FlowArray.size()) - (LengthDiff);
		//~ int IndexTransformEnd = int(MsgNew.Points[0].FlowArray.size()) - 3 ; // new message has 1 additional time sample, and index from 0
		//~ int IndexTransformEnd = int(MsgNew.Points[0].FlowArray.size()) - 2 ; // new message has 1 additional time sample, and index from 0
		int IndexTransformEnd = int(MsgNew.Points[0].FlowArray.size()) - 1 ; // new message has 1 additional time sample, and index from 0
		// calculate homogeneous transform matrix over this span
		Eigen::Matrix4d H; 
		//~ std::cout << "OutputsPast: " << std::endl << OutputsPast[0] << std::endl;
		//~ std::cout << "OutputsPast[IndexTransformStart] transform: " << std::endl << OutputsPast[IndexTransformStart].transform << std::endl;
		//~ std::cout << "OutputsPast[IndexTransformStart] header: " << std::endl << OutputsPast[IndexTransformStart].header << std::endl;
		//~ std::cout << "OutputsPast[IndexTransformStart]: " << std::endl << OutputsPast[IndexTransformStart] << std::endl;
		//~ std::cout << "OutputsPast[IndexTransformEnd]: " << OutputsPast[IndexTransformEnd] << std::endl;
		//~ CalculateTransformMatrix(OutputsPast[IndexTransformStart].transform, OutputsPast[IndexTransformEnd].transform, H);
		CalculateTransformMatrix(OutputsPast[IndexTransformStart].transform, OutputsPast[IndexTransformEnd].transform, H);
		//~ std::cout << "H New: " << std::endl << H << std::endl;
		// combine with current HTM. HChain(new) = HChain(current)*H
		//~ std::cout << "H Chain Old: " << std::endl << HChain.front() << std::endl;
		HChain.insert(HChain.begin(), (HChain.front())*H);
		//~ std::cout << "H Chain New: " << std::endl << HChain.front() << std::endl;
		
		if (HChain.size() > 2*TrackLength) 							// if we have too many HChains
		{
			HChain.pop_back(); 										// delete the oldest
		}
		
		
		// update class so we can publish transform of Cam0
		Cam0Index = IndexTransformStart;
		
		if (PrintToScreen)
		{
			printf("ChainingEvent \n");
			printf("LengthGap: %d \n", LengthDiff);
			printf("IndexTransformStart: %d \n", IndexTransformStart);
			printf("IndexTranormEnd: %d \n", IndexTransformEnd);
			std::cout << "OutputStart: " << std::endl << OutputsPast[IndexTransformStart] << std::endl;
			std::cout << "OutputEnd: " << std::endl << OutputsPast[IndexTransformEnd] << std::endl;
			
			printf("MsgLast Time length: %zd \n", MsgLast.Points[0].FlowArray.size());
			printf("MsgNew Time length: %zd \n", MsgNew.Points[0].FlowArray.size());
			printf("OutputPastLength: %zd \n", OutputsPast.size());
			std::cout << "H: " << std::endl << H << std::endl;
			std::cout << "HChain: " << std::endl << HChain.front() << std::endl;
			for (int i = 0; i < OutputsPast.size(); i++)
			{
				std::cout << std::endl << OutputsPast[i] << std::endl;
			}
		}
	}
	
	void ConstuctFirstHChain ()
	{
		Eigen::Vector4d q(4,1); 
		q(0,0) = 1;  											// w, x, y, z
		q(1,0) = 0; 
		q(2,0) = 0; 
		q(3,0) = 0; 
		ConstuctFirstHChain(q);
	}
	
	void ConstuctFirstHChain (geometry_msgs::TransformStamped Q)
	{
		Eigen::Vector4d q(4,1); 
		q(0,0) = Q.transform.rotation.w; 
		q(1,0) = Q.transform.rotation.x; 
		q(2,0) = Q.transform.rotation.y; 
		q(3,0) = Q.transform.rotation.z; 
		ConstuctFirstHChain(q);
	}
	
	void ConstuctFirstHChain (Eigen::Vector4d q)
	{
		// build the initial Homogeneous Transform Matrix H for chaining
		// start with build rotation matrix from quat 
		Eigen::Matrix3d R; 
		Q2R_3by3(R,q); 
		
		// next build the translation vector
		Eigen::Matrix<double, 3, 1> T;
		Eigen::Matrix<double, 1, 4> fill;
		T.fill(0);
		fill << 0,0,0,1;
		
		// finally the complete homogenious transform matrix [R, t; 0 0 0 1]
		Eigen::Matrix4d H;
		H << R, T, fill;
		//~ std::cout << "HChain0: " << std::endl << HChain << std::endl;
		if (HChain.size() < 1)
		{
			HChain.push_back(H);
		}
		else
		{
			HChain.insert(HChain.begin(), H);
		}
	}
	
	//~ void CalculateTransformMatrix( const geometry_msgs::TransformStamped OldPoint, const geometry_msgs::TransformStamped NewPoint, Eigen::Matrix4d & Hreturn)
	void CalculateTransformMatrix( const geometry_msgs::Transform OldPoint, const geometry_msgs::Transform NewPoint, Eigen::Matrix4d & Hreturn)
	{	
		// build a homogenious transform matrix for the old point
		// start with build rotation matrix from quat 
		Eigen::Vector4d q(4,1); 
		q(0,0) = OldPoint.rotation.w; 
		q(1,0) = OldPoint.rotation.x; 
		q(2,0) = OldPoint.rotation.y; 
		q(3,0) = OldPoint.rotation.z; 
		Eigen::Matrix3d R; 
		Q2R_3by3(R,q); 
		// next build the translation vector
		// next build the translation vector
		Eigen::Matrix<double, 3, 1> T;
		Eigen::Matrix<double, 1, 4> fill;
		T << OldPoint.translation.x, OldPoint.translation.y, OldPoint.translation.z;
		fill << 0,0,0,1;
		// finally the complete homogenious transform matrix [R, t; 0 0 0 1]
		Eigen::Matrix4d Hold; 
		Hold << R, T, fill;
		
		// next build the homogenious transform matrix for the first point
		q(0,0) = NewPoint.rotation.w; 
		q(1,0) = NewPoint.rotation.x; 
		q(2,0) = NewPoint.rotation.y; 
		q(3,0) = NewPoint.rotation.z; 
		Q2R_3by3(R,q); 
		T << NewPoint.translation.x, NewPoint.translation.y, NewPoint.translation.z;
		Eigen::Matrix4d Hnew; 
		Hnew << R, T, fill;
		
		Eigen::Matrix4d HoldInv = Hold.inverse();
		//~ std::cout << "HoldInv: " << std::endl << HoldInv << std::endl;
		//~ std::cout << "Hnew: " << std::endl << Hnew << std::endl;
		
		// finally multiply the two for the result and return it. 
		//~ Hreturn = Hnew*(Hold);
		Hreturn = Hnew*(HoldInv);
	}
	
	void PublishMsg(vbme_msgs::VbmeData_msg & Output)
	{
		
		if (ActivateChaining)
		{
			// output transform
			Eigen::Vector4d Q(4); 								// quaternion
			Eigen::Matrix3d R;  								// rotation matrix	
			Eigen::Matrix<double, 3, 1> T; 						// translation vector
			Eigen::Matrix<double, 1, 4> fill; 					// filler for HTM
			T.fill(0);
			fill << 0,0,0,1;
			Q(0) = Output.transform.rotation.w; 
			Q(1) = Output.transform.rotation.x; 
			Q(2) = Output.transform.rotation.y; 
			Q(3) = Output.transform.rotation.z;
			Q2R_3by3(R, Q);
			T << Output.transform.translation.x, Output.transform.translation.y, Output.transform.translation.z;
			//~ std::cout << " Current output: " << Output.transform << std::endl;
			Eigen::Matrix4d Hnew; 
			Hnew << R, T, fill;
			//~ std::cout << " Current output transform: " << Hnew << std::endl;
			Hnew = (HChain.front()) * Hnew;
			//~ std::cout << " Prior chain transform: " << HChain.front() << std::endl;
			//~ std::cout << " New output transform: " << Hnew << std::endl;
			
			Output.transform.translation.x = Hnew(0, 3);
			Output.transform.translation.y = Hnew(1, 3);
			Output.transform.translation.z = Hnew(2, 3);
			// convert back to quat
			//~ R2Q_3by3(R, Q);
			R2Q_3by3(Hnew.block<3,3>(0,0), Q);
			Output.transform.rotation.w = Q(0);
			Output.transform.rotation.x = Q(1);
			Output.transform.rotation.y = Q(2);
			Output.transform.rotation.z = Q(3);
			//~ std::cout << " New output: " << Output.transform << std::endl;
			
			// cam0 transform
			//~ std::cout << " Cam0 index: " << Cam0Index << std::endl;
			//~ std::cout << " Outputs_u size: " << Outputs_u.size() << std::endl;
			//~ std::cout << " Cam0 orig: " << Outputs_u[Cam0Index].transform << std::endl;
			if (Cam0Index >= Outputs_u.size()) 						// first loop
			{
				Cam0Index = 0; 										// there is only a single index on first loop
				Q(0) = 1;
				Q(1) = 0; 
				Q(2) = 0; 
				Q(3) = 0;
				T << 0, 0, 0;
			}
			else
			{
				Q(0) = Outputs_u[Cam0Index].transform.rotation.w;
				Q(1) = Outputs_u[Cam0Index].transform.rotation.x; 
				Q(2) = Outputs_u[Cam0Index].transform.rotation.y; 
				Q(3) = Outputs_u[Cam0Index].transform.rotation.z;
				T << Outputs_u[Cam0Index].transform.translation.x, Outputs_u[Cam0Index].transform.translation.y, Outputs_u[Cam0Index].transform.translation.z;
			}
			//~ std::cout << " Cam0 Q: " << Q << std::endl;
			//~ R.setZero();
			Q2R_3by3(R, Q);
			
			//~ Hnew.setZero() ; 
			//~ std::cout << " Cam0 T: " << T << std::endl;
			//~ std::cout << " Cam0 R: " << R << std::endl;
			//~ std::cout << " Cam0 fill: " << fill << std::endl;
			
			Eigen::Matrix4d Hnew2;  
			// for some reason all of these options in any combination is turning the matrix into gibberish
			//~ Hnew2.setZero();
			//~ Hnew2 << R, T, fill;
			//~ Hnew2.block<3,3>(0,0) = R;
			//~ Hnew2.block<3,1>(0,2) = T;
			//~ Hnew2.block<1,4>(3,0) = fill;
			
			// rotation
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					Hnew2(i, j) = R(i,j);
				}
			}
			// translation
			for (int i = 0; i < 3; i++)
			{
					Hnew2(i, 3) = T(i);
			}
			// bottom fill
			for (int i = 0; i < 4; i++)
			{
					Hnew2(3, i) = fill(i);
			}
			
			//~ std::cout << " Cam0 new output transform: " << Hnew2 << std::endl << std::endl;
			
			//~ std::cout << " Cam0 HChain: " << HChain.front() << std::endl << std::endl;
			Hnew2 = (HChain.front()) * Hnew2;
			//~ std::cout << " Cam0 new output transform: " << Hnew2 << std::endl;
			Output.TransformCam0.translation.x = Hnew2(0, 3);
			Output.TransformCam0.translation.y = Hnew2(1, 3);
			Output.TransformCam0.translation.z = Hnew2(2, 3);
			// convert back to quat
			//~ R2Q_3by3(R, Q); 
			R2Q_3by3(Hnew2.block<3,3>(0,0), Q);
			Output.TransformCam0.rotation.w = Q(0);
			Output.TransformCam0.rotation.x = Q(1);
			Output.TransformCam0.rotation.y = Q(2);
			Output.TransformCam0.rotation.z = Q(3);
			//~ std::cout << " Cam0 Output: " << Output.TransformCam0 << std::endl;
		}
		else // chaining == False
		{
			Output.TransformCam0.rotation.w = 1.0;
		}
		
		// force translation z to be positive 
		//~ if (Output.transform.translation.z < 0.0)
		//~ {
			//~ Output.transform.translation.x = -1.0 * Output.transform.translation.x;
			//~ Output.transform.translation.y = -1.0 * Output.transform.translation.y; 
			//~ Output.transform.translation.z = -1.0 * Output.transform.translation.z; 
		//~ }
		//~ if (Output.TransformCam0.translation.z < 0.0)
		//~ {
			//~ Output.TransformCam0.translation.x = -1.0 * Output.TransformCam0.translation.x;
			//~ Output.TransformCam0.translation.y = -1.0 * Output.TransformCam0.translation.y; 
			//~ Output.TransformCam0.translation.z = -1.0 * Output.TransformCam0.translation.z; 
		//~ }
		//~ if (Output.transform.rotation.y < 0.0)
		//~ {
			//~ Output.transform.rotation.y = -1.0 * Output.transform.rotation.y;
		//~ }
		
		if (NormalizeOutput == 1) 								// ros param, normalize to a magnatude of 1.0
		{
			// original magnitude 
			double MagOri1 = sqrt(pow(Output.transform.translation.x, 2.0) + pow(Output.transform.translation.y, 2.0) + pow(Output.transform.translation.z, 2.0)); 
			if (MagOri1 > 0.0)
			{
				Output.transform.translation.x = Output.transform.translation.x / MagOri1;
				Output.transform.translation.y = Output.transform.translation.y / MagOri1;
				Output.transform.translation.z = Output.transform.translation.z / MagOri1;
			}
			double MagOri2 = sqrt(pow(Output.TransformCam0.translation.x, 2.0) + pow(Output.TransformCam0.translation.y, 2.0) + pow(Output.TransformCam0.translation.z, 2.0)); 
			if (MagOri2 > 0.0)
			{
				Output.TransformCam0.translation.x = Output.transform.translation.x / MagOri2;
				Output.TransformCam0.translation.y = Output.transform.translation.y / MagOri2;
				Output.TransformCam0.translation.z = Output.transform.translation.z / MagOri2;
			}
			// scale depths
			for (auto i = 0; i < int(Output.Depths.size()); i++)
			{
				Output.Depths[i] = Output.Depths[i] / Output.Depths[0];
			}
		}
		else if (NormalizeOutput == 2) 							// normalize to depth from function findTrans (DepthSave[0]). very first depth t(0)
		{
			// Ccount == 0 will mean first loop. This will set the starting condition for DepthScale
			// This will need to be changed for chaining by depth
			if (Ccount == 0)
			{
				DepthScale = double(Depthsave(0));
			}
			Output.transform.translation.x = Output.transform.translation.x / DepthScale;
			Output.transform.translation.y = Output.transform.translation.y / DepthScale;
			Output.transform.translation.z = Output.transform.translation.z / DepthScale;
			for (auto i = 0; i < int(Output.Depths.size()); i++)
			{
				Output.Depths[i] = Output.Depths[i] / DepthScale;
			}
		}
		else if (NormalizeOutput == 3) 							// normalize to depth from function FindDepths (DepthSave2[0]). very first depth t(0)
		{
			// Ccount == 0 will mean first loop. This will set the starting condition for DepthScale
			// This will need to be changed for chaining by depth
			if (Ccount == 0)
			{
				DepthScale = double(Depthsave2(0));
			}
			Output.transform.translation.x = Output.transform.translation.x / DepthScale;
			Output.transform.translation.y = Output.transform.translation.y / DepthScale;
			Output.transform.translation.z = Output.transform.translation.z / DepthScale;
			for (auto i = 0; i < int(Output.Depths.size()); i++)
			{
				Output.Depths[i] = Output.Depths[i] / DepthScale;
				//~ Output.Depths[i] = Output.Depths[i] / Depthsave2(0);
			}
		}
		else if (NormalizeOutput == 4) 							// normalize to depth from function FindDepths (DepthSave2[0])
		{
			Output.transform.translation.x = Output.transform.translation.x / Depthsave(0);
			Output.transform.translation.y = Output.transform.translation.y / Depthsave(0);
			Output.transform.translation.z = Output.transform.translation.z / Depthsave(0);
			for (auto i = 0; i < int(Output.Depths.size()); i++)
			{
				Output.Depths[i] = Output.Depths[i] / Depthsave(0);
			}
		}
		else if (NormalizeOutput == 5) 							// normalize to depth from function FindDepths (DepthSave2[0])
		{
			Output.transform.translation.x = Output.transform.translation.x / Depthsave2(0);
			Output.transform.translation.y = Output.transform.translation.y / Depthsave2(0);
			Output.transform.translation.z = Output.transform.translation.z / Depthsave2(0);
			for (auto i = 0; i < int(Output.Depths.size()); i++)
			{
				Output.Depths[i] = Output.Depths[i] / Depthsave2(0);
			}
		}
		
		if (PostFilter == 1)
		{
			FilterWindow.insert(FilterWindow.begin(), Output.transform);
			if (FilterWindow.size() < 3)
			{
				// not enough to run filter
				ROS_INFO("Not enough data for filter window...");
				return; 
			}
			if (FilterWindow.size() > 3)
			{
				// throw away un needed values
				FilterWindow.pop_back();
			}
			//~ for (auto it = FilterWindow.begin(); it < FilterWindow.end(); it++)
			//~ {
				//~ std::cout << "Filter window : "  << std::endl << *it << std::endl;
			//~ }
			
			std::vector<double> Tx;
			std::vector<double> Ty;
			std::vector<double> Tz;
			std::vector<double> Qx;
			std::vector<double> Qy;
			std::vector<double> Qz;
			std::vector<double> Qw;
			for (int i = 0; i < 3; i++)
			{
				Tx.push_back(FilterWindow[i].translation.x);
				Ty.push_back(FilterWindow[i].translation.y);
				Tz.push_back(FilterWindow[i].translation.z);
				Qx.push_back(FilterWindow[i].rotation.x);
				Qy.push_back(FilterWindow[i].rotation.y);
				Qz.push_back(FilterWindow[i].rotation.z);
				Qw.push_back(FilterWindow[i].rotation.w);
			}
			//~ ROS_INFO("Tz vec: %f %f %f\n", Tz[0], Tz[1], Tz[2]);
			sort(Tx.begin(), Tx.end()); 
			sort(Ty.begin(), Ty.end()); 
			sort(Tz.begin(), Tz.end()); 
			sort(Qx.begin(), Qx.end()); 
			sort(Qy.begin(), Qy.end()); 
			sort(Qz.begin(), Qz.end()); 
			sort(Qw.begin(), Qw.end()); 
			//~ ROS_INFO("Tz vec: %f %f %f\n", Tz[0], Tz[1], Tz[2]);
			
			Output.transform.translation.x = Tx[1];
			Output.transform.translation.y = Ty[1];
			Output.transform.translation.z = Tz[1];
			// re normalize quat
			float mag = sqrt(pow(Qx[1], 2.0) + pow(Qy[1], 2.0) + pow(Qz[1], 2.0) + pow(Qw[1], 2.0));
			Output.transform.rotation.x = Qx[1]/mag;
			Output.transform.rotation.y = Qy[1]/mag;
			Output.transform.rotation.z = Qz[1]/mag;
			Output.transform.rotation.w = Qw[1]/mag;
			
		}
		
		// prepare rotation in 3x3 matrix form for publishing
		Eigen::Matrix3d Rpub; 
	    Eigen::Vector4d q(4,1); 
	    q[0] = Output.transform.rotation.w;
	    q[1] = Output.transform.rotation.x;
	    q[2] = Output.transform.rotation.y;
	    q[3] = Output.transform.rotation.z;
		Q2R_3by3(Rpub, q);
		
		for (int i = 0; i < (int)Rpub.rows(); i++)
		{
			for (int j = 0; j < (int)Rpub.cols(); j++)
			{
				Output.Rotation.push_back((float)Rpub(i, j));
			}
		}
		
		if (PrintToScreen)
		{
			ROS_INFO("QuEst publishing transform");
			std::cout << "Output final transform: " << std::endl << Output.transform << std::endl;
		}
		PubTransform.publish(Output);
		//~ if (PrintToScreen)
		//~ {
			//~ std::cout << "Time begin: " << TimeBegin << std::endl;
			ros::Time Now = ros::Time::now();
			ros::Duration ComputeTime = Now - TimeBegin;
			std::cout <<  "Quest compute time PublishMsg Final:       \t\t " << ComputeTime.toSec() << std::endl << std::endl;
		//~ }
	}
	
	
};




// contain two parts: recover quaternion first, then get translation ---------- 
// m: (3 by 10) 
// n: (3 by 10) 
void QuEst::QuEst_RANSAC(const Eigen::Matrix3Xd& x1, const Eigen::Matrix3Xd& x2, geometry_msgs::TransformStamped & ReturnOutput)
{

	int numPts = x1.cols();
	// normalize x1 and x2 ---------- 
	Eigen::RowVectorXd x1n(1,numPts); 
	for(int i=0; i<numPts; i++){
		x1n(0,i) = abs(x1(0,i)) + abs(x1(1,i)) + abs(x1(2,i)); 
	} 
	Eigen::Matrix3Xd x1_norm(3,numPts); 
	for(int i=0; i<numPts; i++){
		x1_norm(0,i) = x1(0,i)/x1n(0,i); 
		x1_norm(1,i) = x1(1,i)/x1n(0,i); 
		x1_norm(2,i) = x1(2,i)/x1n(0,i); 
	} 
	//~ std::cout<<"x1_norm: "<<std::endl<<x1_norm<<std::endl<<std::endl; 
	Eigen::RowVectorXd x2n(1,numPts); 
	for(int i=0; i<numPts; i++){
		x2n(0,i) = abs(x2(0,i)) + abs(x2(1,i)) + abs(x2(2,i)); 
	} 
	Eigen::Matrix3Xd x2_norm(3,numPts); 
	for(int i=0; i<numPts; i++){
		x2_norm(0,i) = x2(0,i)/x2n(0,i); 
		x2_norm(1,i) = x2(1,i)/x2n(0,i); 
		x2_norm(2,i) = x2(2,i)/x2n(0,i); 
	} 
	//~ std::cout<<"x2_norm: "<<std::endl<<x2_norm<<std::endl<<std::endl; 
	// end of normalization
	
	// Or do not normalize 
	//~ Eigen::Matrix3Xd x1_norm(3,numPts); 
	//~ Eigen::Matrix3Xd x2_norm(3,numPts);
	//~ x1_norm = x1;
	//~ x2_norm = x2; 
	// end of not normalize
	
	// RANSAC written by Yujie --------------------
	// RANSAC written by Yujie --------------------
	// RANSAC written by Yujie --------------------

	// formulate data ---------- 
	Eigen::MatrixXd data(6,numPts); 
	for(int i=0; i<numPts; i++){
		data(0,i) = x1_norm(0,i); 
		data(1,i) = x1_norm(1,i); 
		data(2,i) = x1_norm(2,i); 
		data(3,i) = x2_norm(0,i); 
		data(4,i) = x2_norm(1,i); 
		data(5,i) = x2_norm(2,i); 
		// data(0,i) = x1(0,i); 
		// data(0,i) = x1(0,i); 
		// data(1,i) = x1(1,i); 
		// data(2,i) = x1(2,i); 
		// data(3,i) = x2(0,i); 
		// data(4,i) = x2(1,i); 
		// data(5,i) = x2(2,i); 
	}
	//~ std::cout<<"data: "<<std::endl<<data<<std::endl<<std::endl; 

	// minimum samples for fittingfn 
	const int minimumSizeSamplesToFit = 6; 

	// distance threshold between data and model 
	//~ const double distance_threshold = 1e-6; 

	// output model and inliers 
	Eigen::VectorXd test_model(7); 
	Eigen::VectorXd select_model(7), best_model(7); 
	std::vector<int> select_inliers, best_inliers; 
	
	// random indicies in range (0, minimumSizeSamplesToFit-1) 
	std::vector<int> ind; 
	for(int i=0; i<minimumSizeSamplesToFit; i++){
		ind.push_back(i); 
	}
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count(); 
	std::shuffle(ind.begin(), ind.end(), std::default_random_engine(seed)); 
// cout<<"ind: "; 
// for(int i=0; i<ind.size(); i++){
// 	cout<<ind.at(i)<<' '; 
// }
// cout<<endl<<endl; 
	
// high_resolution_clock::time_point t1 = high_resolution_clock::now(); // start time 

	// run RANSAC by Yujie ---------- 
	bool res = QuEst::ransac(&QuEst_fit, data, ind, test_model, 
		&QuEst_distance, data, test_model, distance_threshold, select_model, select_inliers, 
		&QuEst_degenerate, data, ind, 
		minimumSizeSamplesToFit, 
		best_model, 
		best_inliers); 
	//~ std::cout<<"ransac res: "<<std::endl<<res<<std::endl<<std::endl;
	if (res == false)
	{
		ReturnOutput.transform.translation.x = 0; 
		ReturnOutput.transform.translation.y = 0; 
		ReturnOutput.transform.translation.z = 0; 
		ReturnOutput.transform.rotation.w = 0; 
		ReturnOutput.transform.rotation.x = 0; 
		ReturnOutput.transform.rotation.y = 0; 
		ReturnOutput.transform.rotation.z = 0; 
	}
	// high_resolution_clock::time_point t2 = high_resolution_clock::now(); // end time 
	// auto duration = duration_cast<microseconds>(t2-t1).count(); // running time for RANSAC 
	// cout<<"RANSAC takes: "<<duration<<" microseconds"<<endl<<endl; 
	
	if (PrintToScreen)
	{
		std::cout<<"best_model: "<<std::endl<<best_model<<std::endl<<std::endl; 
	}
	

	double model_trans_x_sat; 		if(best_model(0)>5)	model_trans_x_sat=5; 		else if(best_model(0)<-5) model_trans_x_sat = -5; 		else model_trans_x_sat = best_model(0);  
	double model_trans_y_sat; 		if(best_model(1)>5)	model_trans_y_sat=5; 		else if(best_model(1)<-5) model_trans_y_sat = -5; 		else model_trans_y_sat = best_model(1);  
	double model_trans_z_sat; 		if(best_model(2)>5)	model_trans_z_sat=5; 		else if(best_model(2)<-5) model_trans_z_sat = -5; 		else model_trans_z_sat = best_model(2); 
	double model_rotation_w_sat; 	if(best_model(3)>1)	model_rotation_w_sat=1; 	else if(best_model(3)<-1) model_rotation_w_sat = -1; 	else model_rotation_w_sat = best_model(3);  
	double model_rotation_x_sat; 	if(best_model(4)>1)	model_rotation_x_sat=1; 	else if(best_model(4)<-1) model_rotation_x_sat = -1; 	else model_rotation_x_sat = best_model(4);  
	double model_rotation_y_sat; 	if(best_model(5)>1)	model_rotation_y_sat=1; 	else if(best_model(5)<-1) model_rotation_y_sat = -1; 	else model_rotation_y_sat = best_model(5);  
	double model_rotation_z_sat; 	if(best_model(6)>1)	model_rotation_z_sat=1; 	else if(best_model(6)<-1) model_rotation_z_sat = -1; 	else model_rotation_z_sat = best_model(6);  
	
	//~ quest_msg.header = ReturnOutput.header;
	//~ quest_msg.transform.translation.x = model_trans_x_sat; 
	//~ quest_msg.transform.translation.y = model_trans_y_sat; 
	//~ quest_msg.transform.translation.z = model_trans_z_sat; 
	//~ quest_msg.transform.rotation.w = model_rotation_w_sat; 
	//~ quest_msg.transform.rotation.x = model_rotation_x_sat; 
	//~ quest_msg.transform.rotation.y = model_rotation_y_sat; 
	//~ quest_msg.transform.rotation.z = model_rotation_z_sat; 
	ReturnOutput.transform.translation.x = model_trans_x_sat; 
	ReturnOutput.transform.translation.y = model_trans_y_sat; 
	ReturnOutput.transform.translation.z = model_trans_z_sat; 
	ReturnOutput.transform.rotation.w = model_rotation_w_sat; 
	ReturnOutput.transform.rotation.x = model_rotation_x_sat; 
	ReturnOutput.transform.rotation.y = model_rotation_y_sat; 
	ReturnOutput.transform.rotation.z = model_rotation_z_sat; 
	
	if (PrintToScreen)
	{
		std::cout<<"ReturnOutput: "<<std::endl<<ReturnOutput<<std::endl<<std::endl; 
	}
	
// 	// MRPT version of RANSAC, deprecated --------------------
// 	// MRPT version of RANSAC, deprecated --------------------
// 	// MRPT version of RANSAC, deprecated --------------------
// 	// stack x1_norm and x2_norm together ---------- 
// 	CMatrixDouble data(6, numPts); 
// 	for(int i=0; i<numPts; i++){
// 		data(0,i) = x1_norm(0,i); 
// 		data(1,i) = x1_norm(1,i); 
// 		data(2,i) = x1_norm(2,i); 
// 		data(3,i) = x2_norm(0,i); 
// 		data(4,i) = x2_norm(1,i); 
// 		data(5,i) = x2_norm(2,i); 
// 	} 

// 	// run RANSAC ---------- 
// 	CMatrixDouble best_model; 
// 	std::vector<size_t> best_inliers; 
// 	const double DIST_THRESHOLD = 0.0001;  // 1e-4
// 	const unsigned int minimumSizeSamplesToFit = 20; 
// 	const double prob_good_sample = 0.8; 
// 	const size_t maxIter = 20; 

// 	math::RANSAC myransac; 
// 	myransac.execute(data, 
// 		QuEst_fit, 
// 		QuEst_distance,
// 		QuEst_degenerate, 
// 		DIST_THRESHOLD,
// 		minimumSizeSamplesToFit, 
// 		best_inliers, 
// 		best_model, 
// 		prob_good_sample,
// 		maxIter); 

// // cout<<"best_model: "<<endl<<best_model<<endl<<endl; 
	//~ ReturnOutput = quest_msg;
} 

bool QuEst::ransac(void (*QuEst_fit)(const Eigen::MatrixXd&, const std::vector<int>&, Eigen::VectorXd&), 
			const Eigen::MatrixXd& data, const std::vector<int>& ind, Eigen::VectorXd& test_model, 
		void (*QuEst_distance)(const Eigen::MatrixXd&, const Eigen::VectorXd&, const double, Eigen::VectorXd&, std::vector<int>&), 
			const Eigen::MatrixXd& data_1, const Eigen::VectorXd& ind_1, const double distance_threshold, Eigen::VectorXd& select_model, std::vector<int>& select_inliers, 
		bool (*QuEst_degenerate)(const Eigen::MatrixXd&, const std::vector<int>&),  
			const Eigen::MatrixXd& data_2, const std::vector<int>& ind_2, 
		const int minimumSizeSamplesToFit, 
		Eigen::VectorXd& best_model, 
		std::vector<int>& best_inliers)
{	

	const int Npts = data.cols(); // number of points in data 
	if (Npts == 0)
	{
		ROS_WARN("Npts in QuEst::ransac is 0!");
		//~ ros::Duration(0.1).sleep(); 
		ros::Duration(0.05).sleep();  							// why is there a sleep here?
		return false; 
	}

	//~ const double p = 0.99; // desired probability of choosing at least one sample free from outliers 

	//~ const int maxDataTrials = 100; // max number to select non-degenerate data set 

	//~ const int maxIter = 1000; // max number of iterations 
	
	int trialcount = 0; 

	int bestscore = 0; 

	double N = 1; // dummy initialisation for number of trials 
	while ( (N > trialcount) && (ros::ok()) ){

		bool degenerate = true; 
		int count = 1; 

		while ( (degenerate) && (ros::ok()) ){
			// test that these points are not a degenerate configuration 
			degenerate = QuEst_degenerate(data_2, ind_2); 
			
			if(!degenerate){
				// fit model to this random selection of data points
				std::vector<int> ind_degen; 
				;
				for(int i=0; i<Npts; i++){
					ind_degen.push_back(i); 
				}
				unsigned seed_degen = std::chrono::system_clock::now().time_since_epoch().count(); 
				std::shuffle(ind_degen.begin(), ind_degen.end(), std::default_random_engine(seed_degen)); 
				
				std::vector<int> ind_degen_select; 
				for(int i=0; i<minimumSizeSamplesToFit; i++){
					//~ std::cout << ind_degen[i] << std::endl;
					ind_degen_select.push_back(ind_degen[i]);
				} 
// cout<<"index: "; 
// for(int i=0; i<ind_degen_select.size(); i++){
// 	cout<<ind_degen_select.at(i)<<" "; 
// } 
// cout<<endl<<endl; 

// high_resolution_clock::time_point t1 = high_resolution_clock::now(); 
				(*QuEst_fit)(data, ind_degen_select, test_model); // cout<<"test_model: "<<endl<<test_model<<endl<<endl; 
// high_resolution_clock::time_point t2 = high_resolution_clock::now(); 
// auto duration = duration_cast<microseconds>(t2-t1).count(); 
// cout<<"QuEst takes:  "<<duration<<" microseconds"<<endl<<endl; 
			}

			if(++count > maxDataTrials){
				// safeguard against being stuck in this loop forever 
				if (PrintToScreen)
				{
					std::cout<<"Unable to select a nondegenerate data set"<<std::endl<<std::endl; 
				}
				
				break; 
			}
		}
		// once we are out here, we should have some kind of model
		// evaluate distances between points and model 
		// returning the indices of elements that are inliers 
		(*QuEst_distance)(data, test_model, distance_threshold, select_model, select_inliers); 

		// find the number of inliers to this model 
		int ninliers = select_inliers.size(); // cout<<"ninliers: "<<ninliers<<endl<<endl; 

		if(ninliers > bestscore){

			bestscore = ninliers; 
			best_inliers = select_inliers; 
			best_model = select_model; 

			// update estimate of N, the number of trials,   
			// to ensure we pick with probability p, a data set with no outliers 
			double fracinliers = ninliers / static_cast<double>(Npts); 
			double pNoOutliers = 1 - pow(fracinliers, static_cast<double>(minimumSizeSamplesToFit)); 

			// avoid division by -Inf
			pNoOutliers = std::max(std::numeric_limits<double>::epsilon(), pNoOutliers); 
			// avoid division by 0 
			pNoOutliers = std::min(1.0-std::numeric_limits<double>::epsilon(), pNoOutliers); 

			// update N 
			N = log(1-p) / log(pNoOutliers); 
			//~ std::cout<<"N: "<<N<<std::endl << "trialcount: "<< trialcount << std::endl<<std::endl; 

		} 
		trialcount = trialcount + 1; 
		//~ std::cout<<"trialcount: "<<trialcount<<std::endl<<std::endl; 

		if(trialcount > maxIter){
			if (PrintToScreen)
			{
				std::cout<<"RANSAC reached the maximum number of trials. "<<std::endl<<std::endl; 
			}
			
			break; 
		}
	} 
	if (PrintToScreen)
	{
		std::cout<<"total trialcount: "<<trialcount<<std::endl<<std::endl; 
	}
	if (UseRansacTrans == false)
	{
		if (PrintToScreen)
		{
			std::cout<<"Using non-ransac translation with ransac quaternion "<<std::endl<<std::endl; 
			std::cout<<"data:  "<<std::endl<<data<<std::endl<<std::endl; 
			std::cout<<"model:  "<<std::endl<<best_model<<std::endl<<std::endl; 
		}
		// we want to recapture the translation not using ransac from a
		// quat that was calculated using ransac
		Eigen::Vector4d Q_select; 
		Q_select(0,0) = best_model(3);  							// quat from ransac
		Q_select(1,0) = best_model(4); 
		Q_select(2,0) = best_model(5); 
		Q_select(3,0) = best_model(6);
		Eigen::Vector3d T;  										// new translation
		FindTrans(T, data, Q_select);
		if (DepthMode == 1)
		{
			// find the depths using the alternative c matrix
			//~ FindDepths(T,x1_5pts,x2_5pts,Q_select);  				// test of alternate depth equation
			FindDepths(data,Q_select);  				// test of alternate depth equation
		}
		
		//~ std::cout<<"Depths:  "<<std::endl<<Depthsave<<std::endl<<std::endl; 
		best_model(0) = T(0);
		best_model(1) = T(1);
		best_model(2) = T(2);
		//~ std::cout<<"model:  "<<std::endl<<best_model<<std::endl<<std::endl; 
	}
	
	if(best_model.rows() > 0){
		return true; 
	} 
	else{
		return false; 
	} 

} 

bool QuEst::QuEst_degenerate(const Eigen::MatrixXd& data, const std::vector<int>& ind){
	return false; 
}

void QuEst::QuEst_fit(const Eigen::MatrixXd& allData, const std::vector<int>& useIndices, Eigen::VectorXd& test_model)
{

	int numPts = useIndices.size(); 

	// take allData out, into x1 and x2 ---------- 
	Eigen::Matrix3Xd x1(3,numPts); 
	for(int i=0; i<numPts; i++){
		x1(0,i) = allData(0,useIndices[i]); 
		x1(1,i) = allData(1,useIndices[i]); 
		x1(2,i) = allData(2,useIndices[i]); 
	} 
	Eigen::Matrix3Xd x2(3,numPts); 
	for(int i=0; i<numPts; i++){
		x2(0,i) = allData(3,useIndices[i]); 
		x2(1,i) = allData(4,useIndices[i]); 
		x2(2,i) = allData(5,useIndices[i]); 
	} 
// cout<<"x1: "<<endl<<x1<<endl<<endl; 
// cout<<"x2: "<<endl<<x2<<endl<<endl; 
	QuEst_fit(x1, x2, useIndices, test_model);
}

void QuEst::QuEst_fit(const Eigen::Matrix3Xd & x1, Eigen::Matrix3Xd & x2, const std::vector<int>& useIndices, Eigen::VectorXd& test_model)
{
	// take first 5 points of x1 and x2, feeding into QuEst ---------- 
	Eigen::Matrix3Xd x1_5pts(3,5); 
	for(int i=0; i<5; i++){
		x1_5pts(0,i) = x1(0,i); 
		x1_5pts(1,i) = x1(1,i); 
		x1_5pts(2,i) = x1(2,i); 
	} 
	Eigen::Matrix3Xd x2_5pts(3,5); 
	for(int i=0; i<5; i++){
		x2_5pts(0,i) = x2(0,i); 
		x2_5pts(1,i) = x2(1,i); 
		x2_5pts(2,i) = x2(2,i); 
	} 
	X1_select = x1_5pts; 								// save to global for publishing
	X2_select = x2_5pts;
	
	//~ std::cout <<  "Debug x1_5 cols: " << x1_5pts.cols() << std::endl;
	//~ std::cout <<  "Debug x2_5 cols: " << x2_5pts.cols() << std::endl;
	
	//~ ros::Time Now = ros::Time::now();
	//~ ros::Duration ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time QuEst_fit 1.1.1: \t\t " << ComputeTime.toSec() << std::endl;
	
	// run QuEst algorithm, get 35 candidates ---------- 
	Eigen::Matrix4Xd Q(4,35); 
	// Matrix3Xd T(3,35); 
	QuEst_Ver1_1(Q,x1_5pts,x2_5pts); 
	//~ std::cout<<"Debug Q transpose: "<<std::endl<<Q.transpose()<<std::endl<<std::endl; 
	
	//~ Now = ros::Time::now();
	//~ ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time QuEst_fit 1.1.2: \t\t " << ComputeTime.toSec() << std::endl;
	
	// score function, pick the best estimated pose solution ----------
	Eigen::RowVectorXd res = Eigen::RowVectorXd::Zero(35); 
	//~ QuatResidue(res,x1,x2,Q);   
	QuatFindNoRotationQ(res,x1,x2,Q);  								// q-select, q_select
	//~ QuatResidueReprojection(res,x1,x2,Q);
	//~ std::cout<<"Debug QuEst_fit res: "<<std::endl<<res<<std::endl<<std::endl; 
	
	//~ Now = ros::Time::now();
	//~ ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time QuEst_fit 1.1.3: \t\t " << ComputeTime.toSec() << std::endl;
	
	int mIdx = 0; 
	for(int i=1; i<35; i++){
		//~ printf("res(0,i) < res(0,mIdx) \n");
		//~ printf("%6f < %6f : %d \n", res(0,i), res(0,mIdx), (res(0,i) < res(0,mIdx))); 
		if(res(0,i) < res(0,mIdx))
			mIdx = i; 
			//~ printf("New index: %d \n", mIdx);
	} 
	//~ std::cout<<"mIdx: "<<std::endl<<mIdx<<std::endl<<std::endl; 
	//~ std::cout<<"Quaternion: "<<std::endl<<Q(0,mIdx)<<"   "<<Q(1,mIdx)<<"   "<<Q(2,mIdx)<<"   "<<Q(3,mIdx)<<std::endl<<std::endl; 
	
	Eigen::Vector4d Q_select; 
	Q_select(0,0) = Q(0,mIdx); 
	Q_select(1,0) = Q(1,mIdx); 
	Q_select(2,0) = Q(2,mIdx); 
	Q_select(3,0) = Q(3,mIdx); 
	
	Eigen::Vector3d T; 
	
	FindTrans(T,x1_5pts,x2_5pts,Q_select); 
	if (DepthMode == 1)
	{
		// find the depths using the alternative c matrix
		FindDepths(x1_5pts,x2_5pts,Q_select);  					// test of alternate depth equation
	}
	//~ std::cout<<"T matrix: "<< T << std::endl<<std::endl; 
	//~ std::cout<<"Translation: "<<std::endl<<T(0,0)<<"   "<<T(1,0)<<"   "<<T(2,0)<<std::endl<<std::endl; 
	
	//~ double T_norm = sqrt(T(0,0)*T(0,0) + T(1,0)*T(1,0) + T(2,0)*T(2,0)); 
	
	//~ Now = ros::Time::now();
	//~ ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time QuEst_fit 1.1.4: \t\t " << ComputeTime.toSec() << std::endl;
	
	//~ std::cout<<"Translation: "<<T<<std::endl<<std::endl;
	//~ std::cout<<"Quaternion: "<<Q_select<<std::endl<<std::endl;
	//~ std::cout<<"test_model: "<<test_model<<std::endl<<std::endl;
	if (test_model.cols() == 0 or test_model.rows() == 0)			// without RANSAC this is empty
	{
		test_model.resize(7,1);
	}
	// test_model(0) = T(0,0) / T_norm; 
	// test_model(1) = T(1,0) / T_norm; 
	// test_model(2) = T(2,0) / T_norm; 
	if (T.cols() == 1) 												// without RANSAC will have a single column
	{
		test_model(0) = T(0); 
		test_model(1) = T(1); 
		test_model(2) = T(2); 
	}
	else 															// RANSAC will have multiple columns
	{
		test_model(0) = T(0,0); 
		test_model(1) = T(1,0); 
		test_model(2) = T(2,0); 
	}
	
	if (Q_select.cols() == 1)										// without RANSAC will have a single column
	{
		test_model(3) = Q_select(0); 
		test_model(4) = Q_select(1); 
		test_model(5) = Q_select(2); 
		test_model(6) = Q_select(3); 
	}
	else
	{
		test_model(3) = Q_select(0,0); 
		test_model(4) = Q_select(1,0); 
		test_model(5) = Q_select(2,0); 
		test_model(6) = Q_select(3,0); 
	}
	//~ std::cout<<"test_model: "<<test_model<<std::endl<<std::endl;
	//~ Now = ros::Time::now();
	//~ ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time QuEst_fit 1.1.5: \t\t " << ComputeTime.toSec() << std::endl;

} 

void QuEst::QuEst_distance(const Eigen::MatrixXd& data, const Eigen::VectorXd& test_model, const double distance_threshold, 
	Eigen::VectorXd& select_model, std::vector<int>& select_inliers){

	int numPts = data.cols(); 

	// extract all feature points 
	Eigen::Matrix3Xd x1(3,numPts); 
	for(int i=0; i<numPts; i++){
		x1(0,i) = data(0,i); 
		x1(1,i) = data(1,i); 
		x1(2,i) = data(2,i); 
	} 
	Eigen::Matrix3Xd x2(3,numPts); 
	for(int i=0; i<numPts; i++){
		x2(0,i) = data(3,i); 
		x2(1,i) = data(4,i); 
		x2(2,i) = data(5,i); 
	} 
// cout<<"in dist function, x1: "<<endl<<x1<<endl; 
// cout<<"in dist function, x2: "<<endl<<x2<<endl<<endl; 
	
	// rotation matrix 
	Eigen::Vector4d q(4,1); 
	q(0,0) = test_model(3,0); 
	q(1,0) = test_model(4,0); 
	q(2,0) = test_model(5,0); 
	q(3,0) = test_model(6,0); 
	Eigen::Matrix3d R; 
	Q2R_3by3(R,q); 
// cout<<"R: "<<endl<<R<<endl<<endl; 

	// skew matrix 
	double t_norm = sqrt(test_model(0,0)*test_model(0,0)+test_model(1,0)*test_model(1,0)+test_model(2,0)*test_model(2,0)); 

	Eigen::Vector3d t(3,1); 
	t(0,0) = test_model(0,0) / t_norm; 
	t(1,0) = test_model(1,0) / t_norm; 
	t(2,0) = test_model(2,0) / t_norm; 

	Eigen::Matrix3d Tx; 
	Tx(0,0) = 0; 
	Tx(0,1) = -t(2,0); 
	Tx(0,2) = t(1,0); 
	Tx(1,0) = t(2,0); 
	Tx(1,1) = 0; 
	Tx(1,2) = -t(0,0); 
	Tx(2,0) = -t(1,0); 
	Tx(2,1) = t(0,0); 
	Tx(2,2) = 0; 
// cout<<"Tx: "<<endl<<Tx<<endl<<endl; 

	// fundamental matrix ---------- 
	Eigen::Matrix3d F(3,3); 
	//~ F = Tx * R; // Cody and Dr Gans
	F = R * Tx; 
// cout<<"F: "<<endl<<F<<endl<<endl; 

	Eigen::RowVectorXd x2tFx1 =Eigen:: RowVectorXd::Zero(numPts); 
// cout<<x2tFx1<<endl; 
	for(int i=0; i<numPts; i++){
		x2tFx1(0,i)=(x2(0,i)*F(0,0)+x2(1,i)*F(1,0)+x2(2,i)*F(2,0))*x1(0,i)+(x2(0,i)*F(0,1)+x2(1,i)*F(1,1)+x2(2,i)*F(2,1))*x1(1,i)+(x2(0,i)*F(0,2)+x2(1,i)*F(1,2)+x2(2,i)*F(2,2))*x1(2,i); 
	} 
// cout<<"x2tFx1: "<<endl<<x2tFx1<<endl<<endl; 
	
	// evaluate distance ---------- 
	Eigen::Matrix3Xd Fx1(3,numPts); 
	Fx1 = F * x1; 
// cout<<"Fx1: "<<endl<<Fx1<<endl<<endl; 	

	Eigen::Matrix3Xd Ftx2(3,numPts); 
	Ftx2 = F.transpose() * x2; 
// cout<<"Ftx2: "<<endl<<Ftx2<<endl<<endl; 

	select_inliers.clear(); 

	for(int i=0; i<numPts; i++){
		double d = x2tFx1(0,i)*x2tFx1(0,i)/(Fx1(0,i)*Fx1(0,i)+Fx1(1,i)*Fx1(1,i)+Ftx2(0,i)*Ftx2(0,i)+Ftx2(1,i)*Ftx2(1,i)); 
// cout<<"d: "<<d<<endl; 

		if(abs(d)<distance_threshold) 
			select_inliers.push_back(i); 
	} 
// cout<<endl; 
// cout<<"select_inliers: "<<endl<<select_inliers.size()<<endl<<endl; 

	select_model = test_model; 

} // end void QuEst::QuEst_distance

// contain two parts: recover quaternion and recover translation 
// void QuEst::QuEst_Ver1_1(Matrix4Xd& Q, Matrix3Xd& T, const Matrix3Xd& m, const Matrix3Xd& n){
void QuEst::QuEst_Ver1_1(Eigen::Matrix4Xd& Q, const Eigen::Matrix3Xd& m, const Eigen::Matrix3Xd& n){

	QuEst::QuEst_5Pt_Ver5_2(Q,m,n); 
	// QuEst::FindTrans(T,m,n,Q); 

}

void QuEst::QuEst_once_pub(Eigen::Matrix3Xd & m1, Eigen::Matrix3Xd & m2, geometry_msgs::TransformStamped & HeaderandReturnOutput)
{
	// HeaderandReturnOutput has two purposes
	// the header is passed to this function to be published
	// transform is returned to be used by the calling function 
	
	//~ std::vector<int> UseIndex(6, 0); 
	std::vector<int> UseIndex;
	for (int i = 0; i < 5; i++)
	{
		UseIndex.push_back(i);									// use the first indices 0-4
	}
	
	//~ ros::Time Now = ros::Time::now();
	//~ ros::Duration ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time QuEst_once_pub 1.1:   \t\t " << ComputeTime.toSec() << std::endl;
	
	// new Y2020 M06 D15. QuEst_fit is apperantly limited to only recieving
	// 6 pts. Even if UseIndex is limited to 0-4 the other points are used
	// for QuEst_Resdue
	int Limit;
	//~ if ( (int(m1.cols()) < 6) && (int(m2.cols()) < 6) )
	if ( (int(m1.cols()) > 5) )
	{
		Limit = 6;
	}
	else
	{
		Limit = 5;
	}
	
	// normalize data, data is normalized elsewhere?
	Eigen::RowVectorXd x1n(1,Limit); 
	for(int i=0; i<Limit; i++){
		x1n(0,i) = abs(m1(0,i)) + abs(m1(1,i)) + abs(m1(2,i)); 
	} 
	Eigen::Matrix3Xd x1_norm(3,Limit); 
	for(int i=0; i<Limit; i++){
		x1_norm(0,i) = m1(0,i)/x1n(0,i); 
		x1_norm(1,i) = m1(1,i)/x1n(0,i); 
		x1_norm(2,i) = m1(2,i)/x1n(0,i); 
	} 
	//~ std::cout<<"Debug QuEst_once_pub x1_norm: "<<std::endl<<x1_norm<<std::endl<<std::endl; 
	Eigen::RowVectorXd x2n(1,Limit); 
	for(int i=0; i<Limit; i++){
		x2n(0,i) = abs(m2(0,i)) + abs(m2(1,i)) + abs(m2(2,i)); 
	} 
	Eigen::Matrix3Xd x2_norm(3,Limit); 
	for(int i=0; i<Limit; i++){
		x2_norm(0,i) = m2(0,i)/x2n(0,i); 
		x2_norm(1,i) = m2(1,i)/x2n(0,i); 
		x2_norm(2,i) = m2(2,i)/x2n(0,i); 
	} 
	//~ std::cout<<"Debug QuEst_once_pub x2_norm: "<<std::endl<<x2_norm<<std::endl<<std::endl; 
	// end of normalize
	
	// or do not normalize
	//~ Eigen::Matrix3Xd x1_norm(3,Limit); 
	//~ Eigen::Matrix3Xd x2_norm(3,Limit); 
	//~ x1_norm = m1;
	//~ x2_norm = m2;
	// end of not normalize
	
	
	//~ Eigen::Matrix3Xd m1_Limit(3, Limit);
	//~ Eigen::Matrix3Xd m2_Limit(3, Limit);
	//~ for (int i = 0; i < Limit; i++)
	//~ {
		//~ m1_Limit(0, i) = m1(0, i);
		//~ m1_Limit(1, i) = m1(2, i);
		//~ m1_Limit(2, i) = m1(2, i);
		//~ m2_Limit(0, i) = m2(0, i);
		//~ m2_Limit(1, i) = m2(2, i);
		//~ m2_Limit(2, i) = m2(2, i);
	//~ }
	//~ std::cout <<  "Debug Limit:   \t\t " << Limit << std::endl;
	//~ std::cout <<  "Debug x1_norm cols:   \t\t " << x1_norm.cols() << std::endl;
	//~ std::cout <<  "Debug x2_norm cols:   \t\t " << x2_norm.cols() << std::endl;
	
	//~ Eigen::Matrix4Xd Output;
	Eigen::VectorXd Output;
	//~ std::cout<<"m1: "<<  m1 << std::endl<< "m2: " << m2 <<std::endl<<std::endl;
	//~ QuEst::QuEst_fit(m1, m2, UseIndex, Output);
	//~ QuEst::QuEst_fit(m1_Limit, m2_Limit, UseIndex, Output);
	QuEst::QuEst_fit(x1_norm, x2_norm, UseIndex, Output);
	// noQuEst::QuEst_5Pt_Ver5_2(Q,m,n);
	//~ std::cout << "The matrix Output is of size " << Output.rows() << "x" << Output.cols() << ". contents: " << Output << std::endl;
	//~ // no distance
	
	//~ Now = ros::Time::now();
	//~ ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time QuEst_once_pub 1.2:   \t\t " << ComputeTime.toSec() << std::endl;
	
	//~ quest_msg.translation.x = model_trans_x_sat; 
	//~ quest_msg.translation.y = model_trans_y_sat; 
	//~ quest_msg.translation.z = model_trans_z_sat; 
	//~ quest_msg.rotation.w = model_rotation_w_sat; 
	//~ quest_msg.rotation.x = model_rotation_x_sat; 
	//~ quest_msg.rotation.y = model_rotation_y_sat; 
	//~ quest_msg.rotation.z = model_rotation_z_sat;
	geometry_msgs::TransformStamped Q;
	//~ Q.translation.x = Output(0,0); 
	//~ Q.translation.y = Output(1,0); 
	//~ Q.translation.z = Output(2,0); 
	//~ Q.rotation.w = Output(3,0); 
	//~ Q.rotation.x = Output(4,0); 
	//~ Q.rotation.y = Output(5,0); 
	//~ Q.rotation.z = Output(6,0); 
	Q.header = HeaderandReturnOutput.header;
	Q.transform.translation.x = Output(0); 
	Q.transform.translation.y = Output(1); 
	Q.transform.translation.z = Output(2); 
	Q.transform.rotation.w = Output(3); 
	Q.transform.rotation.x = Output(4); 
	Q.transform.rotation.y = Output(5); 
	Q.transform.rotation.z = Output(6); 
	// is this right?
	
	HeaderandReturnOutput = Q;
}

void QuEst::FindDepths(const Eigen::MatrixXd& allData, const Eigen::Vector4d& Q)
{
	//~ int numPts = allData.cols(); 
	int numPts = 5; 

	// take allData out, into x1 and x2 ---------- 
	Eigen::Matrix3Xd x1(3,numPts); 
	for(int i=0; i<numPts; i++){
		x1(0,i) = allData(0,i); 
		x1(1,i) = allData(1,i); 
		x1(2,i) = allData(2,i); 
	} 
	Eigen::Matrix3Xd x2(3,numPts); 
	for(int i=0; i<numPts; i++){
		x2(0,i) = allData(3,i); 
		x2(1,i) = allData(4,i); 
		x2(2,i) = allData(5,i); 
	} 
	// cout<<"x1: "<<endl<<x1<<endl<<endl; 
	// cout<<"x2: "<<endl<<x2<<endl<<endl; 
	FindDepths(x1, x2, Q);
}

// CODY recover depth attempt new Y2020 M04 D27
// QuEst::FindTrans can also find the depths but the values are not expected.
// This fuction can be used to test against it
//~ void QuEst::FindDepths(Eigen::Vector3d& T, const Eigen::Matrix3Xd& m, const Eigen::Matrix3Xd& n, const Eigen::Vector4d& Q)
void QuEst::FindDepths(const Eigen::Matrix3Xd& m, const Eigen::Matrix3Xd& n, const Eigen::Vector4d& Q)
{
	if (PrintToScreen)
	{
		std::cout<<"Find Depths method: "<<std::endl; 
		std::cout<<"Q: "<<std::endl<<Q<<std::endl<<std::endl; 
		std::cout<<"m: "<<std::endl<<m<<std::endl<<std::endl; 
		std::cout<<"n: "<<std::endl<<n<<std::endl<<std::endl;
	}
	int numCols = Q.cols(); 

	Eigen::MatrixXd R = Eigen::MatrixXd::Zero(9, numCols); 
	QuEst::Q2R(R,Q); // convert quaternion into rotation matrix 
	if (PrintToScreen)
	{
		std::cout<<"R transpose: "<<std::endl<<R.transpose()<<std::endl<<std::endl; 
	}
	
	int numPts = m.cols(); 
	/////// TEST old 3 points 
	//~ numPts = 3;
	int numInp = R.cols(); 

	//~ std::cout<<"numInp: "<<std::endl<<numInp<<std::endl<<std::endl; 
	for(int k=0; k<numInp; k++)
	{
		Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3*(numPts-1), 2*(numPts)); 
		for(int i=1; i<numPts; i++){
			
			// R * m(0)
			C((i-1)*3,  0) = R(0,k)*m(0,0) + R(1,k)*m(1,0) + R(2,k)*m(2,0); 
			C((i-1)*3+1,0) = R(3,k)*m(0,0) + R(4,k)*m(1,0) + R(5,k)*m(2,0); 
			C((i-1)*3+2,0) = R(6,k)*m(0,0) + R(7,k)*m(1,0) + R(8,k)*m(2,0); 
			
			// - n(0)
			C((i-1)*3,  1) = -n(0,0); 
			C((i-1)*3+1,1) = -n(1,0); 
			C((i-1)*3+2,1) = -n(2,0); 
			
			// R * m(i) 
			C((i-1)*3,  (i-1)*2+2) = -1.0*R(0,k)*m(0,i) - R(1,k)*m(1,i) - R(2,k)*m(2,i); 
			C((i-1)*3+1,(i-1)*2+2) = -1.0*R(3,k)*m(0,i) - R(4,k)*m(1,i) - R(5,k)*m(2,i); 
			C((i-1)*3+2,(i-1)*2+2) = -1.0*R(6,k)*m(0,i) - R(7,k)*m(1,i) - R(8,k)*m(2,i); 
			
			// - n(i)
			C((i-1)*3,  (i-1)*2+3) = n(0,i); 
			C((i-1)*3+1,(i-1)*2+3) = n(1,i); 
			C((i-1)*3+2,(i-1)*2+3) = n(2,i);
			
			//~ std::cout<<"C: "<<std::endl<<C<<std::endl<<std::endl;
		} 
		if (PrintToScreen)
		{
			std::cout<<"C: "<<std::endl<<C<<std::endl<<std::endl;
		}
		
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(C, Eigen::ComputeThinV); 
		Eigen::MatrixXd N = svd.matrixV(); 
		if (PrintToScreen)
		{
			std::cout<<"N: "<<std::endl<<N<<std::endl<<std::endl; 
		}
		
		// adjust the sign 
		int numPos=0, numNeg=0; 
		for(int i=0;i<2*numPts;i++){
			if(N(i,2*numPts-1)>0) numPos++; 
			if(N(i,2*numPts-1)<0) numNeg++; 
		}
		//~ std::cout<<"N: "<<std::endl<<N<<std::endl<<std::endl;
		//~ std::cout<<"N row: "<<std::endl<<N.rows()<<std::endl<<std::endl;
		//~ std::cout<<"N col: "<<std::endl<<N.cols()<<std::endl<<std::endl;
		//~ std::cout<<""<<std::endl<<N.block<10,1>(3,12).transpose()<<std::endl<<std::endl;
		
		// for testing and printing 
		//~ Eigen::MatrixXd Nsub = N.block<10,1>(0,9);
		//~ Eigen::MatrixXd Nsub = N.block<6,1>(0,5);
		Eigen::MatrixXd Nsub;;
		
		
		//~ std::cout<<"numPos: "<<numPos<<"   "<<"numNeg: "<<numNeg<<std::endl<<std::endl; 
		if(numPos<numNeg){
			Nsub = -1.0 * (N.block<10,1>(0,9));
			 //~ Nsub = (-N).block<6,1>(0,5);
		}
		else{
			Nsub = N.block<10,1>(0,9);
			//~ Nsub = N.block<6,1>(0,5);
			
		} 
		Nsave2 = N;
		Depthsave2 = Nsub;
		Csave2 = C;
		//~ std::cout<<"Depths2: "<<std::endl<<Depthsave2<<std::endl<<std::endl;
		//~ Depthsave2 = Depthsave2 / Depthsave2(0); 			// normalize moved to PublishMsg()
		//~ std::cout<<"Depths2: "<<std::endl<<Depthsave2<<std::endl<<std::endl;
		//~ std::cout<<"T: "<<std::endl<<T<<std::endl<<std::endl; 
		//~ std::cout<<"T transpose: "<<std::endl<<T.transpose()<<std::endl<<std::endl; 
		if (PrintToScreen)
		{
			//~ std::cout<<"Nsub: "<<std::endl<<Nsub<<std::endl<<std::endl;
			std::cout<<"Nsub transpose: "<<std::endl<<Nsub.transpose()<<std::endl<<std::endl;
		}
	}
}

void QuEst::FindTrans(Eigen::Vector3d& T, const Eigen::MatrixXd& allData, const Eigen::Vector4d& Q)
{
	//~ int numPts = allData.cols(); 
	int numPts = 5; 

	// take allData out, into x1 and x2 ---------- 
	Eigen::Matrix3Xd x1(3,numPts); 
	for(int i=0; i<numPts; i++){
		x1(0,i) = allData(0,i); 
		x1(1,i) = allData(1,i); 
		x1(2,i) = allData(2,i); 
	} 
	Eigen::Matrix3Xd x2(3,numPts); 
	for(int i=0; i<numPts; i++){
		x2(0,i) = allData(3,i); 
		x2(1,i) = allData(4,i); 
		x2(2,i) = allData(5,i); 
	} 
	// cout<<"x1: "<<endl<<x1<<endl<<endl; 
	// cout<<"x2: "<<endl<<x2<<endl<<endl; 
	FindTrans(T, x1, x2, Q);
}

// recover translation, 3 by 35 ----------
void QuEst::FindTrans(Eigen::Vector3d& T, const Eigen::Matrix3Xd& m, const Eigen::Matrix3Xd& n, const Eigen::Vector4d& Q)
{
	if (PrintToScreen)
	{
		std::cout<<"Q: "<<std::endl<<Q<<std::endl<<std::endl; 
		std::cout<<"m: "<<std::endl<<m<<std::endl<<std::endl; 
		std::cout<<"n: "<<std::endl<<n<<std::endl<<std::endl;
	}
	 
	int numCols = Q.cols(); 

	Eigen::MatrixXd R = Eigen::MatrixXd::Zero(9, numCols); 
	QuEst::Q2R(R,Q); // convert quaternion into rotation matrix 
	if (PrintToScreen)
	{
		std::cout<<"R transpose: "<<std::endl<<R.transpose()<<std::endl<<std::endl; 
	}
	
	int numPts = m.cols(); 
	int numInp = R.cols(); 

	//~ std::cout<<"numInp: "<<std::endl<<numInp<<std::endl<<std::endl; 
	for(int k=0; k<numInp; k++)
	{
		Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3*numPts, 2*numPts+3); 

		for(int i=1; i<=numPts; i++){
			C((i-1)*3,  0) = 1; 
			C((i-1)*3+1,1) = 1; 
			C((i-1)*3+2,2) = 1; 

			C((i-1)*3,  (i-1)*2+3) = R(0,k)*m(0,i-1) + R(1,k)*m(1,i-1) + R(2,k)*m(2,i-1); 
			C((i-1)*3+1,(i-1)*2+3) = R(3,k)*m(0,i-1) + R(4,k)*m(1,i-1) + R(5,k)*m(2,i-1); 
			C((i-1)*3+2,(i-1)*2+3) = R(6,k)*m(0,i-1) + R(7,k)*m(1,i-1) + R(8,k)*m(2,i-1); 

			C((i-1)*3,  (i-1)*2+4) = -n(0,i-1); 
			C((i-1)*3+1,(i-1)*2+4) = -n(1,i-1); 
			C((i-1)*3+2,(i-1)*2+4) = -n(2,i-1); 
			
			// //////////////////////
			// Testing dividing by bottom left in each group, C((i-1)*3+2,(i-1)*2+3)
			//~ C((i-1)*3,  (i-1)*2+3) = C((i-1)*3,  (i-1)*2+3)  / C((i-1)*3+2,(i-1)*2+3);
			//~ C((i-1)*3+1,(i-1)*2+3) = C((i-1)*3+1,(i-1)*2+3)  / C((i-1)*3+2,(i-1)*2+3);
			//~ C((i-1)*3+2,(i-1)*2+3) = C((i-1)*3+2,(i-1)*2+3)  / C((i-1)*3+2,(i-1)*2+3);

			//~ C((i-1)*3,  (i-1)*2+4) = C((i-1)*3,  (i-1)*2+4)  / C((i-1)*3+2,(i-1)*2+3);
			//~ C((i-1)*3+1,(i-1)*2+4) = C((i-1)*3+1,(i-1)*2+4)  / C((i-1)*3+2,(i-1)*2+3);
			//~ C((i-1)*3+2,(i-1)*2+4) = C((i-1)*3+2,(i-1)*2+4)  / C((i-1)*3+2,(i-1)*2+3);
			//~ std::cout<<"C: "<<std::endl<<C<<std::endl<<std::endl;
			// //////////////////////
		} 
		if (PrintToScreen)
		{
			std::cout<<"C: "<<std::endl<<C<<std::endl<<std::endl;
		}
		Csave = C;
		
		// use SVD or Quadratic Programming to find singular std::vectors 
		//~ ROS_INFO("SolveMode: %d", SolveMode);
		if (SolveMode == 0)
		{
			Eigen::MatrixXd N;
			// BDCSVD<MatrixXd> svd(C, ComputeFullV); 
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(C, Eigen::ComputeThinV); 
			N = svd.matrixV(); 
		
			if (PrintToScreen)
			{
				std::cout<<"N: "<<std::endl<<N<<std::endl<<std::endl; 
			}


	// 		// start armadillo 	----------------------------------------

	// 		arma::Mat<double> C_arma(3*numPts, 2*numPts+3); 
	// 		for(int i=0;i<3*numPts;i++){
	// 			for(int j=0;j<2*numPts+3;j++){
	// 				C_arma(i,j) = C(i,j); 
	// 			}
	// 		}		
	// // cout<<"C_arma: "<<endl<<C_arma<<endl<<endl; 

	// 		arma::mat U_arma; 
	// 		arma::vec s_arma; 
	// 		arma::mat N_arma; 
	// 		arma::svd_econ(U_arma,s_arma,N_arma,C_arma); 

	// 		MatrixXd N(13,13); 
	// 		for(int i=0; i<13; i++){
	// 			for(int j=0; j<13; j++){
	// 				N(i,j) = N_arma(i,j); 
	// 			} 
	// 		} 		

	// 		// end armadillo 	----------------------------------------

			// adjust the sign 
			int numPos=0, numNeg=0; 
			//~ for(int i=0;i<2*numPts;i++){
			for(int i=3;i<2*numPts+3;i++){
				//~ if(N(i+3,2*numPts+2)>0) numPos++; 
				//~ if(N(i+3,2*numPts+2)<0) numNeg++; 
				if(N(i,2*numPts+2)>0) numPos++; 
				if(N(i,2*numPts+2)<0) numNeg++; 
			}
			//~ std::cout<<"N: "<<std::endl<<N<<std::endl<<std::endl;
			//~ std::cout<<"N row: "<<std::endl<<N.rows()<<std::endl<<std::endl;
			//~ std::cout<<"N col: "<<std::endl<<N.cols()<<std::endl<<std::endl;
			//~ std::cout<<""<<std::endl<<N.block<10,1>(3,12).transpose()<<std::endl<<std::endl;
			Nsave = N; 								
			
			
			// for testing and printing 
			//~ Eigen::MatrixXd Nsub = N.block<10,1>(1,12)
			//~ std::cout<<"N: "<<std::endl<<N<<std::endl<<std::endl;
			
			//~ std::cout<<"numPos: "<<numPos<<"   "<<"numNeg: "<<numNeg<<std::endl<<std::endl; 
			//~ std::cout<<"T transpose: "<<std::endl<<T.transpose()<<std::endl<<std::endl; 
			if(numPos<numNeg){
				T(0,k) = -N(0,2*numPts+2); 
				T(1,k) = -N(1,2*numPts+2); 
				T(2,k) = -N(2,2*numPts+2); 
				Depthsave = -N.block<10,1>(3,12); 						// for printing out depths
			}
			else{
				T(0,k) = N(0,2*numPts+2); 
				T(1,k) = N(1,2*numPts+2); 
				T(2,k) = N(2,2*numPts+2); 
				Depthsave = N.block<10,1>(3,12); 						// for printing out depths
			} 
			//~ std::cout<<"T: "<<std::endl<<T<<std::endl<<std::endl; 
			//~ std::cout<<"T transpose: "<<std::endl<<T.transpose()<<std::endl<<std::endl; 
			
			
			//~ std::cout<<"Depths1: "<<std::endl<<Nsave<<std::endl<<std::endl;
			//~ Depthsave = Depthsave / Depthsave(0); 					// normalize moved to publish msg
			//~ std::cout<<"Depths1: "<<std::endl<<Nsave<<std::endl<<std::endl;
		}
		else if(SolveMode == 1)
		{
			Eigen::MatrixXd X;
			
			
			//~ std::cout << "C = " << std::endl << C << std::endl;
			//~ SolveQPData(C, params, X);  						// pass in global var params and C matrix
			SolveQPData(C, X);  						// pass in global var params and C matrix
			//~ std::cout << "X = " << std::endl << X << std::endl;
			T(0) = X(0); 
			T(1) = X(1); 
			T(2) = X(2);
			Depthsave = X.block(0,0,10,1);
			
			//~ std::cout << "T = " << std::endl << T << std::endl;
			//~ std::cout << "Depthsave = " << std::endl << Depthsave << std::endl;
		}
		else
		{
			ROS_FATAL("Unknown requested solver! SolveMode %d\n", SolveMode);
		}
	}
} // end void QuEst::FindTrans

void QuEst::QuatResidue(Eigen::RowVectorXd& residu, const Eigen::Matrix3Xd& m1, const Eigen::Matrix3Xd& m2, const Eigen::Matrix4Xd& qSol)
{

	int numPts = m1.cols(); 

	int numEq = numPts*(numPts-1)*(numPts-2)/6; 
	
	Eigen::MatrixXd C0(numEq,35); 
	CoefsVer_3_1_1(C0,m1,m2); // coefficient matrix such that C * x = c 
	//~ std::cout<<"Debug QuatResidue C0: "<<std::endl<<C0<<std::endl<<std::endl; 

	//~ ros::Time Now = ros::Time::now();
	//~ ros::Duration ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time QuatResidue 1.1.2.7:\t\t" << ComputeTime.toSec() << std::endl;

	Eigen::MatrixXd xVec(35,35); 
	for(int i=0; i<35; i++){
		xVec(0,i) = qSol(0,i) * qSol(0,i) * qSol(0,i) * qSol(0,i); 
		xVec(1,i) = qSol(0,i) * qSol(0,i) * qSol(0,i) * qSol(1,i); 
		xVec(2,i) = qSol(0,i) * qSol(0,i) * qSol(1,i) * qSol(1,i); 
		xVec(3,i) = qSol(0,i) * qSol(1,i) * qSol(1,i) * qSol(1,i); 
		xVec(4,i) = qSol(1,i) * qSol(1,i) * qSol(1,i) * qSol(1,i); 
		xVec(5,i) = qSol(0,i) * qSol(0,i) * qSol(0,i) * qSol(2,i); 
		xVec(6,i) = qSol(0,i) * qSol(0,i) * qSol(1,i) * qSol(2,i); 
		xVec(7,i) = qSol(0,i) * qSol(1,i) * qSol(1,i) * qSol(2,i); 
		xVec(8,i) = qSol(1,i) * qSol(1,i) * qSol(1,i) * qSol(2,i); 
		xVec(9,i) = qSol(0,i) * qSol(0,i) * qSol(2,i) * qSol(2,i); 
		xVec(10,i) = qSol(0,i) * qSol(1,i) * qSol(2,i) * qSol(2,i); 
		xVec(11,i) = qSol(1,i) * qSol(1,i) * qSol(2,i) * qSol(2,i); 
		xVec(12,i) = qSol(0,i) * qSol(2,i) * qSol(2,i) * qSol(2,i); 
		xVec(13,i) = qSol(1,i) * qSol(2,i) * qSol(2,i) * qSol(2,i); 
		xVec(14,i) = qSol(2,i) * qSol(2,i) * qSol(2,i) * qSol(2,i); 
		xVec(15,i) = qSol(0,i) * qSol(0,i) * qSol(0,i) * qSol(3,i); 
		xVec(16,i) = qSol(0,i) * qSol(0,i) * qSol(1,i) * qSol(3,i); 
		xVec(17,i) = qSol(0,i) * qSol(1,i) * qSol(1,i) * qSol(3,i); 
		xVec(18,i) = qSol(1,i) * qSol(1,i) * qSol(1,i) * qSol(3,i); 
		xVec(19,i) = qSol(0,i) * qSol(0,i) * qSol(2,i) * qSol(3,i); 
		xVec(20,i) = qSol(0,i) * qSol(1,i) * qSol(2,i) * qSol(3,i); 
		xVec(21,i) = qSol(1,i) * qSol(1,i) * qSol(2,i) * qSol(3,i); 
		xVec(22,i) = qSol(0,i) * qSol(2,i) * qSol(2,i) * qSol(3,i); 
		xVec(23,i) = qSol(1,i) * qSol(2,i) * qSol(2,i) * qSol(3,i); 
		xVec(24,i) = qSol(2,i) * qSol(2,i) * qSol(2,i) * qSol(3,i); 
		xVec(25,i) = qSol(0,i) * qSol(0,i) * qSol(3,i) * qSol(3,i); 
		xVec(26,i) = qSol(0,i) * qSol(1,i) * qSol(3,i) * qSol(3,i); 
		xVec(27,i) = qSol(1,i) * qSol(1,i) * qSol(3,i) * qSol(3,i); 
		xVec(28,i) = qSol(0,i) * qSol(2,i) * qSol(3,i) * qSol(3,i); 
		xVec(29,i) = qSol(1,i) * qSol(2,i) * qSol(3,i) * qSol(3,i); 
		xVec(30,i) = qSol(2,i) * qSol(2,i) * qSol(3,i) * qSol(3,i); 
		xVec(31,i) = qSol(0,i) * qSol(3,i) * qSol(3,i) * qSol(3,i); 
		xVec(32,i) = qSol(1,i) * qSol(3,i) * qSol(3,i) * qSol(3,i); 
		xVec(33,i) = qSol(2,i) * qSol(3,i) * qSol(3,i) * qSol(3,i); 
		xVec(34,i) = qSol(3,i) * qSol(3,i) * qSol(3,i) * qSol(3,i); 
	} 
	//~ std::cout<<"Debug QuatResidue xVec: "<<std::endl<<xVec<<std::endl<<std::endl; 
	
	//~ Now = ros::Time::now();
	//~ ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time QuatResidue 1.1.2.8:\t\t" << ComputeTime.toSec() << std::endl;
	
	Eigen::MatrixXd residuMat(numEq,35); 
	residuMat = C0 * xVec; 
	//~ std::cout<<"Debug QuatResidue residuMat: "<<std::endl<<residuMat<<std::endl<<std::endl; 

	for(int i=0; i<35; i++){
		for(int j=0; j<numEq; j++){
			residu(0,i) = residu(0,i) + abs(residuMat(j,i)); 
		}
	} 
	//~ std::cout<<"Debug QuatResidue residu: "<<std::endl<<residu<<std::endl<<std::endl; 
	
	//~ Now = ros::Time::now();
	//~ ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time QuatResidue 1.1.2.9:\t\t" << ComputeTime.toSec() << std::endl;

} // end void QuEst::QuatResidue


void QuEst::QuatFindNoRotationQ(Eigen::RowVectorXd& residu, const Eigen::Matrix3Xd& m1, const Eigen::Matrix3Xd& m2, const Eigen::Matrix4Xd& qSol)
{
	// simple function to find the rotation closest to zero - quat [1 0 0 0]
	//~ std::cout << "Qs: " << std::endl << qSol << std::endl << std::endl;
	for (int i = 0; i < 35; i++)
	{
		//~ std::cout << "Q[i]: " << std::endl << qSol(0,i) << " " << qSol(1,i) << " " << qSol(2,i) << " " << qSol(3,i) << std::endl;
		residu(i) = pow(1.0 - qSol(0,i), 2.0) + pow(qSol(1,i), 2.0) + pow(qSol(2,i), 2.0) + pow(qSol(3,i), 2.0);
	}
	//~ std::cout << "res: " << std::endl << residu << std::endl << std::endl;
}

void QuEst::QuatResidueReprojection(Eigen::RowVectorXd& residu, const Eigen::Matrix3Xd& m1, const Eigen::Matrix3Xd& m2, const Eigen::Matrix4Xd& qSol)
{
	// function to find qSol with the least reprojection error
	std::cout << "qSol.cols(): " << std::endl << qSol.cols() << std::endl << std::endl;
	std::cout << "Qs: " << std::endl << qSol << std::endl << std::endl;
	for (int i = 0; i < qSol.cols(); i++)
	{
		// find translation for this q
		Eigen::Vector3d T; 
		QuEst::FindTrans(T,m1,m2,qSol.block(0,i, 3, 1)); 
		
		Eigen::Matrix3d R; 
		Q2R_3by3(R,qSol.block(0,i, 3, 1)); 							// convert quaternion into rotation matrix 
		
		Eigen::Vector3d Tnorm; 										// make normalization of translation
		double Norm = sqrt( pow(T(0), 2.0) + pow(T(1), 2.0) + pow(T(2), 2.0) );
		Tnorm(0) = T(0) / Norm; 
		Tnorm(1) = T(1) / Norm; 
		Tnorm(2) = T(2) / Norm; 
		Eigen::Matrix3d Tx = Eigen::MatrixXd::Zero(3, 3); 			// make skew matrix (Tnorm)
		Tx(0,1) = -1.0 * Tnorm(2);
		Tx(0,2) =  Tnorm(1);
		Tx(1,2) = -1.0 * Tnorm(0);
		
		Tx(1,0) = Tnorm(2);
		Tx(2,0) = -1.0 * Tnorm(1);
		Tx(2,1) = Tnorm(0);
		
		Eigen::Matrix3d F; 											// make fundamental matrix
		F = Tx * R;
		
		Eigen::Matrix3d e3x = Eigen::MatrixXd::Zero(3, 3); 			// make ex3 skew matrix (0 0 1)
		e3x(0,1) = -1.0;
		e3x(1,0) = 1.0;
		
		Eigen::VectorXd reproj = Eigen::VectorXd::Zero(m1.cols());
		//~ Eigen::VectorXd Matrix3d = Eigen::Matrix3d::Zero(m1.cols());
		for (int n = 0 ; n < int(m1.cols()) ; n++)
		{
			Eigen::Matrix3d x1 = m1.block(0,n, 3,1);
			Eigen::Matrix3d x2 = m2.block(0,n, 3,1);
			Eigen::MatrixXd A = x2.transpose()*F*x2;
			Eigen::MatrixXd AA = A*A;
			Eigen::MatrixXd B = e3x*F*x1;
			double Bnorm = sqrt( pow(B(0),2.0) + pow(B(0),2.0) + pow(B(0),2.0) );
			Eigen::MatrixXd C = x2*F*e3x;
			double Cnorm = sqrt( pow(C(0),2.0) + pow(C(0),2.0) + pow(C(0),2.0) );
			
			
			//~ Eigen::Vector3d ReprojTemp = Eigen::Vector3d::Zero();
			//~ ReprojTemp = ( (AA) / (pow(Bnorm, 2.0)) ) + ( (AA) / (pow(Cnorm, 2.0)) );
		}
		
	}
	
		//~ for j=1:length(pose.Q)
			//~ R = Q2R(pose.Q(:,j));
			//~ Tx = Skew(pose.T(:,j)/norm(pose.T(:,j)));  
			//~ F = Tx * R;
			//~ e3x=Skew([0 0 1]);
			//~ for n=1:length(x2)
				   //~ reproj(n) = ((x2(:,n)'*F*x1(:,n))^2)/(norm(e3x*F*x1(:,n))^2) + ((x2(:,n)'*F*x1(:,n))^2)/(norm(x2(:,n)'*F*e3x')^2);
				   
				//~ reproj(n) = ((x2(:,n)'*F*x1(:,n))^2)/(norm(e3x*F*x1(:,n))^2)+...
					//~ ((x2(:,n)'*F*x1(:,n))^2)/(norm(x2(:,n)'*F*e3x')^2);
                        
                        
			//~ end  
			//~ scorer(j)=norm(reproj);
		//~ end
}

// recover quaternion, 4 by 35 ---------
void QuEst::QuEst_5Pt_Ver5_2(Eigen::Matrix4Xd& Q ,const Eigen::Matrix3Xd& m, const Eigen::Matrix3Xd& n)
{

	//~ std::cout<<"QuEst_5Pt_Ver5_2 start -------------------------"<<std::endl<<std::endl; 

	int numPts = m.cols(); 
	//~ std::cout<<"Debug numPts "<< numPts << std::endl<<std::endl; 

	Eigen::Matrix4Xi Idx(4,35); 
	Idx << 1,2,5,11,21,3,6,12,22,8,14,24,17,27,31,4,7,13,23,9,15,25,18,28,32,10,16,26,19,29,33,20,30,34,35, 
		2,5,11,21,36,6,12,22,37,14,24,39,27,42,46,7,13,23,38,15,25,40,28,43,47,16,26,41,29,44,48,30,45,49,50, 
		3,6,12,22,37,8,14,24,39,17,27,42,31,46,51,9,15,25,40,18,28,43,32,47,52,19,29,44,33,48,53,34,49,54,55, 
		4,7,13,23,38,9,15,25,40,18,28,43,32,47,52,10,16,26,41,19,29,44,33,48,53,20,30,45,34,49,54,35,50,55,56; 

	Eigen::RowVectorXi idx_w(35); 
	idx_w << 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35; 

	Eigen::RowVectorXi idx_w0(21); 
	idx_w0 << 36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56; 

	Eigen::MatrixX4i Idx1(20,4); 
	Idx1 << 1,2,3,4, 
		2,5,6,7,
		3,6,8,9,
		4,7,9,10,
		5,11,12,13,
		6,12,14,15,
		7,13,15,16,
		8,14,17,18,
		9,15,18,19,
		10,16,19,20,
		11,21,22,23,
		12,22,24,25,
		13,23,25,26,
		14,24,27,28,
		15,25,28,29,
		16,26,29,30,
		17,27,31,32,
		18,28,32,33,
		19,29,33,34,
		20,30,34,35; 

	Eigen::MatrixX4i Idx2(15,4); 
	Idx2 <<	21,1,2,3, 
			22,2,4,5, 
			23,3,5,6, 
			24,4,7,8, 
			25,5,8,9, 
			26,6,9,10, 
			27,7,11,12, 
			28,8,12,13, 
			29,9,13,14, 
			30,10,14,15, 
			31,11,16,17, 
			32,12,17,18, 
			33,13,18,19, 
			34,14,19,20, 
			35,15,20,21; 

	Eigen::MatrixXd Bx = Eigen::MatrixXd::Zero(35,35); 

	int numEq = numPts*(numPts-1)*(numPts-2)/6; 

	Eigen::MatrixXd Cf(numEq,35); // coefficient matrix such that Cf * V = 0 
	CoefsVer_3_1_1(Cf,m,n); 
// cout<<"Cf: "<<endl<<Cf<<endl<<endl<<endl; 

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4*numEq, 56); // coefficient matrix such that A * X = 0 
	for(int i=0; i<numEq; i++){
		for(int j=0; j<35; j++){
			int idx_0 = Idx(0,j); 
			A(i,idx_0-1) = Cf(i,j); 
			
			int idx_1 = Idx(1,j); 
			A(i+numEq,idx_1-1) = Cf(i,j); 
			
			int idx_2 = Idx(2,j); 
			A(i+numEq*2,idx_2-1) = Cf(i,j); 
			
			int idx_3 = Idx(3,j); 
			A(i+numEq*3,idx_3-1) = Cf(i,j); 
		}
	} 
// cout<<"A: "<<endl<<A<<endl<<endl; 

	Eigen::MatrixXd A1(4*numEq,35); // split A into A1 and A2, A1 contains term w 
	for(int i=0; i<40; i++){
		for(int j=0; j<35; j++){
			A1(i,j) = A(i,j); 
		}
	} 
// cout<<"A1: "<<endl<<A1<<endl<<endl; 

	Eigen::MatrixXd A2(4*numEq,21); // A2 doesn't contains term w 
	for(int i=0; i<40; i++){
		for(int j=0; j<21; j++){
			A2(i,j) = A(i,j+35); 
		}
	} 
// cout<<"A2: "<<endl<<A2<<endl<<endl; 

	// MatrixXd A2_pseudo = A2.completeOrthogonalDecomposition().pseudoInverse(); 
	// MatrixXd Bbar = -A2_pseudo*A1; 
	Eigen::MatrixXd Bbar = -A2.completeOrthogonalDecomposition().solve(A1); 
// cout<<"Bbar: "<<endl<<Bbar<<endl<<endl; 

	for(int i=0; i<20; i++){
		Bx(Idx1(i,0)-1,Idx1(i,1)-1) = 1; 
	} 
	for(int i=0; i<15; i++){
		for(int j=0; j<35; j++){
			Bx(20+i,j) = Bbar(i,j); 
		}
	} 
// cout<<"Bx: "<<endl<<Bx<<endl<<endl; 

// 	// Eigen version of eigensolver, deprecated ----------
// 	EigenSolver<MatrixXd> es; 
// 	es.compute(Bx, true); 
// 	MatrixXcd Ve = es.eigenvectors(); 
// // cout<<"Ve: "<<endl<<Ve<<endl<<endl; 

// start armadillo ------------------------------------------------------------ 
	arma::Mat<double> Bx_arma(35,35); 
	for(int i=0;i<35;i++){
		for(int j=0;j<35;j++){
			Bx_arma(i,j) = Bx(i,j); 
		}
	}
// cout<<"Bx in arma: "<<endl<<Bx_arma<<endl<<endl; 
	
	arma::cx_vec eigval; 
	arma::cx_mat Ve_arma; 
	arma::eig_gen(eigval, Ve_arma, Bx_arma, "balance");  					// causing build errors?
//~ // cout<<"Ve_arma in arma: "<<endl<<Ve_arma<<endl<<endl; 
//~ // cout<<"e_values in arma: "<<endl<<eigval<<endl<<endl; 

	arma::mat V_arma; 
	V_arma = real(Ve_arma); 
// cout<<"V_arma in arma: "<<endl<<V_arma<<endl<<endl; 

// end armadillo  ------------------------------------------------------------ 

	Eigen::MatrixXd V(35,35); 
	for(int i=0; i<35; i++){
		for(int j=0; j<35; j++){
			V(i,j) = V_arma(i,j); 
		} 
	} 
// cout<<"V in eigen: "<<endl<<V<<endl<<endl; 

	// correct sign of each column, the first element is always positive 
	Eigen::MatrixXd V_1(35,35); 
	for(int i=0; i<35; i++){ // i represents column 
		for(int j=0; j<35; j++){ // j represent row 

			if(V(0,i)<0)
				V_1(j,i) = V(j,i) * (-1); 
			else 
				V_1(j,i) = V(j,i) * (1); 				

		} 
	} 
// cout<<"V_1: "<<endl<<V_1<<endl<<endl; 

	// recover quaternion elements 
	Eigen::RowVectorXd w(35); 
	for(int i=0; i<35; i++){
		w(0,i) = sqrt(sqrt(V_1(0,i))); 
	} 
// cout<<"w: "<<endl<<w<<endl<<endl; 

	Eigen::RowVectorXd w3(35); 
	for(int i=0; i<35; i++){
		w3(0,i) = w(0,i)*w(0,i)*w(0,i); 
	} 
// cout<<"w3: "<<endl<<w3<<endl<<endl; 

	Eigen::Matrix4Xd Q_0(4,35); 
	for(int i=0; i<35; i++){
		Q_0(0,i) = w(0,i); 
		Q_0(1,i) = V_1(1,i) / w3(0,i); 
		Q_0(2,i) = V_1(2,i) / w3(0,i); 
		Q_0(3,i) = V_1(3,i) / w3(0,i); 
	} 
// cout<<"Q_0: "<<endl<<Q_0<<endl<<endl; 

	Eigen::RowVectorXd QNrm(1,35); 
	for(int i=0; i<35; i++){
		QNrm(0,i) = sqrt(Q_0(0,i)*Q_0(0,i)+Q_0(1,i)*Q_0(1,i)+Q_0(2,i)*Q_0(2,i)+Q_0(3,i)*Q_0(3,i)); 
	}
// cout<<"QNrm: "<<endl<<QNrm<<endl<<endl; 

	// normalize each column 
	for(int i=0; i<35; i++){
		Q(0,i) = Q_0(0,i) / QNrm(0,i); 
		Q(1,i) = Q_0(1,i) / QNrm(0,i); 
		Q(2,i) = Q_0(2,i) / QNrm(0,i); 
		Q(3,i) = Q_0(3,i) / QNrm(0,i); 
	} 
// cout<<"Q: "<<endl<<Q<<endl<<endl; 

	//~ std::cout<<"QuEst_5Pt_Ver5_2 end -----------------------------------"<<std::endl<<std::endl; 

} // end void QuEst::QuEst_5Pt_Ver5_2

// convert quaternion into rotation matrix ---------- 
void QuEst::Q2R(Eigen::MatrixXd& R_Q2R, const Eigen::Matrix4Xd& Q)
{

	int numInp_Q2R = Q.cols(); 

	for(int i=0; i<numInp_Q2R; i++){

		Eigen::Vector4d q; 
		q(0,0) = Q(0,i); 
		q(1,0) = Q(1,i); 
		q(2,0) = Q(2,i); 
		q(3,0) = Q(3,i); 

		R_Q2R(0,i) = 1 - 2*q(2,0)*q(2,0) - 2*q(3,0)*q(3,0); 
		R_Q2R(1,i) = 2*q(1,0)*q(2,0) - 2*q(3,0)*q(0,0); 
		R_Q2R(2,i) = 2*q(1,0)*q(3,0) + 2*q(0,0)*q(2,0); 
		R_Q2R(3,i) = 2*q(1,0)*q(2,0) + 2*q(3,0)*q(0,0); 
		R_Q2R(4,i) = 1 - 2*q(1,0)*q(1,0) - 2*q(3,0)*q(3,0); 
		R_Q2R(5,i) = 2*q(2,0)*q(3,0) - 2*q(1,0)*q(0,0); 
		R_Q2R(6,i) = 2*q(1,0)*q(3,0) - 2*q(0,0)*q(2,0); 
		R_Q2R(7,i) = 2*q(2,0)*q(3,0) + 2*q(1,0)*q(0,0); 
		R_Q2R(8,i) = 1 - 2*q(1,0)*q(1,0) - 2*q(2,0)*q(2,0); 

	}
// cout<<"R_Q2R: "<<endl<<R_Q2R.transpose()<<endl<<endl;
} // end void QuEst::Q2R

void QuEst::Q2R_3by3(Eigen::Matrix3d& R_Q2R, const Eigen::Vector4d& Q)
{
		R_Q2R(0,0) = 1 - 2*Q(2,0)*Q(2,0) - 2*Q(3,0)*Q(3,0); 
		R_Q2R(0,1) = 2*Q(1,0)*Q(2,0) - 2*Q(3,0)*Q(0,0); 
		R_Q2R(0,2) = 2*Q(1,0)*Q(3,0) + 2*Q(0,0)*Q(2,0); 
		R_Q2R(1,0) = 2*Q(1,0)*Q(2,0) + 2*Q(3,0)*Q(0,0); 
		R_Q2R(1,1) = 1 - 2*Q(1,0)*Q(1,0) - 2*Q(3,0)*Q(3,0); 
		R_Q2R(1,2) = 2*Q(2,0)*Q(3,0) - 2*Q(1,0)*Q(0,0); 
		R_Q2R(2,0) = 2*Q(1,0)*Q(3,0) - 2*Q(0,0)*Q(2,0); 
		R_Q2R(2,1) = 2*Q(2,0)*Q(3,0) + 2*Q(1,0)*Q(0,0); 
		R_Q2R(2,2) = 1 - 2*Q(1,0)*Q(1,0) - 2*Q(2,0)*Q(2,0); 
}

void QuEst::R2Q_3by3(const Eigen::Matrix3d& R,  Eigen::Vector4d& Q)
{
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	// max is used to avoid round off errors close to 0
	Q[0] = sqrt( std::max( 0.0, 1 + R(0,0) + R(1,1) + R(2,2) ) ) / 2.0; // W
	Q[1] = sqrt( std::max( 0.0, 1 + R(0,0) - R(1,1) - R(2,2) ) ) / 2.0; // X
	Q[2] = sqrt( std::max( 0.0, 1 - R(0,0) + R(1,1) - R(2,2) ) ) / 2.0;	// Y
	Q[3] = sqrt( std::max( 0.0, 1 - R(0,0) - R(1,1) + R(2,2) ) ) / 2.0; // Z
	
	Q[0] = std::copysign( Q[0], R(2,1) - R(1,2) ); 					// X
	Q[1] = std::copysign( Q[1], R(0,2) - R(2,0) ); 					// Y
	Q[2] = std::copysign( Q[2], R(1,0) - R(0,1) ); 					// Z
	
	if (Q[0] < 0.0) 									// make sure W is positive
	{
		Q[0] = -Q[0];  
		Q[1] = -Q[1]; 
		Q[2] = -Q[2]; 
		Q[3] = -Q[3];
	}
}

// m1:(3 by n), m2:(3 by n), n is number of points 
void QuEst::CoefsVer_3_1_1(Eigen::MatrixXd& Cf, const Eigen::Matrix3Xd& m1, const Eigen::Matrix3Xd& m2)
{

	int numPts = m1.cols(); 

	int numCols = numPts*(numPts-1)/2 - 1; 

	//~ std::cout<< "Debug CoefsVer_3_1_1 numPts " << numPts << ". numCols " << numCols << std::endl << std::endl; 

	Eigen::Matrix2Xi idxBin1(2,numCols); 
	int counter = 0; 
	for(int i=1; i<=numPts-2; i++){
		for(int j=i+1; j<=numPts; j++){
			counter = counter + 1; 
			idxBin1(0,counter-1) = i; 
			idxBin1(1,counter-1) = j; 
		}
	} 
// cout<<idxBin1<<endl; 

	//~ ros::Time Now = ros::Time::now();
	//~ ros::Duration ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time CoefsVer_3_1_1 1.1.2.1:\t\t" << ComputeTime.toSec() << std::endl;

	Eigen::VectorXd mx1(numCols,1); 
	Eigen::VectorXd my1(numCols,1); 
	Eigen::VectorXd s1(numCols,1); 
	Eigen::VectorXd nx1(numCols,1); 
	Eigen::VectorXd ny1(numCols,1); 
	Eigen::VectorXd r1(numCols,1); 
	Eigen::VectorXd mx2(numCols,1); 
	Eigen::VectorXd my2(numCols,1); 
	Eigen::VectorXd s2(numCols,1); 
	Eigen::VectorXd nx2(numCols,1); 
	Eigen::VectorXd ny2(numCols,1); 
	Eigen::VectorXd r2(numCols,1); 
	for(int i=0; i<numCols; i++){
		int index_1 = idxBin1(0,i); 
		mx1(i,0) = m1(0,index_1-1); 
		my1(i,0) = m1(1,index_1-1); 
		s1(i,0) = m1(2,index_1-1); 
		nx1(i,0) = m2(0,index_1-1); 
		ny1(i,0) = m2(1,index_1-1); 
		r1(i,0) = m2(2,index_1-1); 

		int index_2 = idxBin1(1,i); 
		mx2(i,0) = m1(0,index_2-1); 
		my2(i,0) = m1(1,index_2-1); 
		s2(i,0) = m1(2,index_2-1); 
		nx2(i,0) = m2(0,index_2-1); 
		ny2(i,0) = m2(1,index_2-1); 
		r2(i,0) = m2(2,index_2-1); 
	} 
// cout<<mx1<<endl<<my1<<endl<<s1<<endl<<nx1<<endl<<ny1<<endl<<r1<<endl<<endl; 
// cout<<mx2<<endl<<my2<<endl<<s2<<endl<<nx2<<endl<<ny2<<endl<<r2<<endl<<endl; 

	//~ Now = ros::Time::now();
	//~ ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time CoefsVer_3_1_1 1.1.2.2:\t\t" << ComputeTime.toSec() << std::endl;

	Eigen::MatrixXd coefsN(numCols, 10); 
	coefsNum(coefsN,mx1,mx2,my1,my2,nx2,ny2,r2,s1,s2); 	
// cout<<coefsN<<endl; 

	Eigen::MatrixXd coefsD(numCols, 10); 
	coefsDen(coefsD,mx2,my2,nx1,nx2,ny1,ny2,r1,r2,s2); 	
// cout<<coefsD<<endl; 
	
	//~ Now = ros::Time::now();
	//~ ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time CoefsVer_3_1_1 1.1.2.3:\t\t" << ComputeTime.toSec() << std::endl;
	
	int numEq = numPts*(numPts-1)*(numPts-2)/6; 
	//~ std::cout<< "Debug CoefsVer_3_1_1 numEq " << numEq << std::endl << std::endl; 
	
	Eigen::Matrix2Xi idxBin2(2,numEq); 
	int counter_bin2_1 = 0; 
	int counter_bin2_2 = 0; 
	for(int i=numPts-1; i>=2; i--){
		for(int j=1+counter_bin2_2; j<=i-1+counter_bin2_2; j++){
			for(int k=j+1; k<=i+counter_bin2_2; k++){
				counter_bin2_1 = counter_bin2_1 + 1; 
				idxBin2(0,counter_bin2_1-1) = j; 
				idxBin2(1,counter_bin2_1-1) = k; 
			}
		}
		counter_bin2_2 = i + counter_bin2_2; 
	} 
// cout<<idxBin2<<endl; 
	
	//~ Now = ros::Time::now();
	//~ ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time CoefsVer_3_1_1 1.1.2.4:\t\t" << ComputeTime.toSec() << std::endl;
	
	int numEqDouble = 2 * numEq; 
	
	Eigen::VectorXd a1(numEqDouble,1); 
	Eigen::VectorXd a2(numEqDouble,1); 
	Eigen::VectorXd a3(numEqDouble,1); 
	Eigen::VectorXd a4(numEqDouble,1); 
	Eigen::VectorXd a5(numEqDouble,1); 
	Eigen::VectorXd a6(numEqDouble,1); 
	Eigen::VectorXd a7(numEqDouble,1); 
	Eigen::VectorXd a8(numEqDouble,1); 
	Eigen::VectorXd a9(numEqDouble,1); 
	Eigen::VectorXd a10(numEqDouble,1); 
	Eigen::VectorXd b1(numEqDouble,1); 
	Eigen::VectorXd b2(numEqDouble,1); 
	Eigen::VectorXd b3(numEqDouble,1); 
	Eigen::VectorXd b4(numEqDouble,1); 
	Eigen::VectorXd b5(numEqDouble,1); 
	Eigen::VectorXd b6(numEqDouble,1); 
	Eigen::VectorXd b7(numEqDouble,1); 
	Eigen::VectorXd b8(numEqDouble,1); 
	Eigen::VectorXd b9(numEqDouble,1); 
	Eigen::VectorXd b10(numEqDouble,1); 
	for(int i=0; i<numEq; i++){ 

		int index_1 = idxBin2(0,i); 
		a1(i,0) = coefsN(index_1-1,0); 
		a1(i+numEq,0) = coefsD(index_1-1,0);
		a2(i,0) = coefsN(index_1-1,1); 
		a2(i+numEq,0) = coefsD(index_1-1,1); 
		a3(i,0) = coefsN(index_1-1,2); 
		a3(i+numEq,0) = coefsD(index_1-1,2); 
		a4(i,0) = coefsN(index_1-1,3); 
		a4(i+numEq,0) = coefsD(index_1-1,3); 
		a5(i,0) = coefsN(index_1-1,4); 
		a5(i+numEq,0) = coefsD(index_1-1,4); 
		a6(i,0) = coefsN(index_1-1,5); 
		a6(i+numEq,0) = coefsD(index_1-1,5); 
		a7(i,0) = coefsN(index_1-1,6); 
		a7(i+numEq,0) = coefsD(index_1-1,6); 
		a8(i,0) = coefsN(index_1-1,7); 
		a8(i+numEq,0) = coefsD(index_1-1,7); 
		a9(i,0) = coefsN(index_1-1,8); 
		a9(i+numEq,0) = coefsD(index_1-1,8); 
		a10(i,0) = coefsN(index_1-1,9); 
		a10(i+numEq,0) = coefsD(index_1-1,9); 

		int index_2 = idxBin2(1,i); 
		b1(i,0) = coefsD(index_2-1,0); 
		b1(i+numEq,0) = coefsN(index_2-1,0); 
		b2(i,0) = coefsD(index_2-1,1); 
		b2(i+numEq,0) = coefsN(index_2-1,1); 
		b3(i,0) = coefsD(index_2-1,2); 
		b3(i+numEq,0) = coefsN(index_2-1,2); 
		b4(i,0) = coefsD(index_2-1,3); 
		b4(i+numEq,0) = coefsN(index_2-1,3); 
		b5(i,0) = coefsD(index_2-1,4); 
		b5(i+numEq,0) = coefsN(index_2-1,4); 
		b6(i,0) = coefsD(index_2-1,5); 
		b6(i+numEq,0) = coefsN(index_2-1,5); 
		b7(i,0) = coefsD(index_2-1,6); 
		b7(i+numEq,0) = coefsN(index_2-1,6); 
		b8(i,0) = coefsD(index_2-1,7); 
		b8(i+numEq,0) = coefsN(index_2-1,7); 
		b9(i,0) = coefsD(index_2-1,8); 
		b9(i+numEq,0) = coefsN(index_2-1,8); 
		b10(i,0) = coefsD(index_2-1,9); 
		b10(i+numEq,0) = coefsN(index_2-1,9); 
	} 
// cout<<a1<<endl<<a2<<endl<<a3<<endl<<a4<<endl<<a5<<endl<<a6<<endl<<a7<<endl<<a8<<endl<<a9<<endl<<a10<<endl<<endl; 
// cout<<b1<<endl<<b2<<endl<<b3<<endl<<b4<<endl<<b5<<endl<<b6<<endl<<b7<<endl<<b8<<endl<<b9<<endl<<b10<<endl<<endl; 
	
	//~ Now = ros::Time::now();
	//~ ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time CoefsVer_3_1_1 1.1.2.5:\t\t" << ComputeTime.toSec() << std::endl;
	
	Eigen::MatrixXd coefsND(numEqDouble, 35); 
	//~ std::cout<<"Debug CoefsVer_3_1_1 coefsND Size: "<< coefsND.rows() << " x " << coefsND.cols() <<std::endl<<std::endl; 
	coefsNumDen(coefsND,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,b1,b2,b3,b4,b5,b6,b7,b8,b9,b10); 
	//~ std::cout<<"coefsND: "<<std::endl<<coefsND<<std::endl<<std::endl; 

	for(int i=0; i<numEq; i++){
		Cf(i,0) = coefsND(i,0) - coefsND(i+numEq,0); 		
		Cf(i,1) = coefsND(i,1) - coefsND(i+numEq,1); 		
		Cf(i,2) = coefsND(i,2) - coefsND(i+numEq,2); 		
		Cf(i,3) = coefsND(i,3) - coefsND(i+numEq,3); 		
		Cf(i,4) = coefsND(i,4) - coefsND(i+numEq,4); 		
		Cf(i,5) = coefsND(i,5) - coefsND(i+numEq,5); 		
		Cf(i,6) = coefsND(i,6) - coefsND(i+numEq,6); 		
		Cf(i,7) = coefsND(i,7) - coefsND(i+numEq,7); 		
		Cf(i,8) = coefsND(i,8) - coefsND(i+numEq,8); 		
		Cf(i,9) = coefsND(i,9) - coefsND(i+numEq,9); 		
		Cf(i,10) = coefsND(i,10) - coefsND(i+numEq,10); 		
		Cf(i,11) = coefsND(i,11) - coefsND(i+numEq,11); 		
		Cf(i,12) = coefsND(i,12) - coefsND(i+numEq,12); 		
		Cf(i,13) = coefsND(i,13) - coefsND(i+numEq,13); 		
		Cf(i,14) = coefsND(i,14) - coefsND(i+numEq,14); 		
		Cf(i,15) = coefsND(i,15) - coefsND(i+numEq,15); 		
		Cf(i,16) = coefsND(i,16) - coefsND(i+numEq,16); 		
		Cf(i,17) = coefsND(i,17) - coefsND(i+numEq,17); 		
		Cf(i,18) = coefsND(i,18) - coefsND(i+numEq,18); 		
		Cf(i,19) = coefsND(i,19) - coefsND(i+numEq,19); 		
		Cf(i,20) = coefsND(i,20) - coefsND(i+numEq,20); 		
		Cf(i,21) = coefsND(i,21) - coefsND(i+numEq,21); 		
		Cf(i,22) = coefsND(i,22) - coefsND(i+numEq,22); 		
		Cf(i,23) = coefsND(i,23) - coefsND(i+numEq,23); 		
		Cf(i,24) = coefsND(i,24) - coefsND(i+numEq,24); 		
		Cf(i,25) = coefsND(i,25) - coefsND(i+numEq,25); 		
		Cf(i,26) = coefsND(i,26) - coefsND(i+numEq,26); 		
		Cf(i,27) = coefsND(i,27) - coefsND(i+numEq,27); 		
		Cf(i,28) = coefsND(i,28) - coefsND(i+numEq,28); 		
		Cf(i,29) = coefsND(i,29) - coefsND(i+numEq,29); 		
		Cf(i,30) = coefsND(i,30) - coefsND(i+numEq,30); 		
		Cf(i,31) = coefsND(i,31) - coefsND(i+numEq,31); 		
		Cf(i,32) = coefsND(i,32) - coefsND(i+numEq,32); 		
		Cf(i,33) = coefsND(i,33) - coefsND(i+numEq,33); 		
		Cf(i,34) = coefsND(i,34) - coefsND(i+numEq,34); 		
	} 
	//~ std::cout<<"Cf: "<<std::endl<<Cf<<std::endl<<std::endl; 
	//~ std::cout<<"Debug CoefsVer_3_1_1 Cf Size: "<< Cf.rows() << " x " << Cf.cols() <<std::endl<<std::endl;

	//~ Now = ros::Time::now();
	//~ ComputeTime = Now - TimeBegin;
	//~ std::cout <<  "Quest compute time CoefsVer_3_1_1 1.1.2.6:\t\t" << ComputeTime.toSec() << std::endl;

}

void QuEst::coefsNum(Eigen::MatrixXd& coefsN, const Eigen::VectorXd& mx1, const Eigen::VectorXd& mx2, 
	const Eigen::VectorXd& my1, const Eigen::VectorXd& my2, 
	const Eigen::VectorXd& nx2, const Eigen::VectorXd& ny2, 
	const Eigen::VectorXd& r2, const Eigen::VectorXd& s1, const Eigen::VectorXd& s2)
{

	int numPts = mx1.rows(); 

	Eigen::VectorXd t2(numPts,1); 
	Eigen::VectorXd t3(numPts,1); 
	Eigen::VectorXd t4(numPts,1); 
	Eigen::VectorXd t5(numPts,1); 
	Eigen::VectorXd t6(numPts,1); 
	Eigen::VectorXd t7(numPts,1); 
	Eigen::VectorXd t8(numPts,1); 
	Eigen::VectorXd t9(numPts,1); 
	Eigen::VectorXd t10(numPts,1); 
	Eigen::VectorXd t11(numPts,1); 
	Eigen::VectorXd t12(numPts,1); 
	Eigen::VectorXd t13(numPts,1); 
	for(int i=0; i<numPts; i++){
		t2(i,0) = mx1(i,0)*my2(i,0)*r2(i,0); 
		t3(i,0) = mx2(i,0)*ny2(i,0)*s1(i,0); 
		t4(i,0) = my1(i,0)*nx2(i,0)*s2(i,0); 
		t5(i,0) = mx1(i,0)*nx2(i,0)*s2(i,0)*2.0; 
		t6(i,0) = my1(i,0)*ny2(i,0)*s2(i,0)*2.0; 
		t7(i,0) = mx1(i,0)*my2(i,0)*nx2(i,0)*2.0; 
		t8(i,0) = my2(i,0)*r2(i,0)*s1(i,0)*2.0; 
		t9(i,0) = mx2(i,0)*my1(i,0)*r2(i,0); 
		t10(i,0) = mx1(i,0)*ny2(i,0)*s2(i,0); 
		t11(i,0) = mx2(i,0)*my1(i,0)*ny2(i,0)*2.0; 
		t12(i,0) = mx2(i,0)*r2(i,0)*s1(i,0)*2.0; 
		t13(i,0) = my2(i,0)*nx2(i,0)*s1(i,0); 
// cout<<t2<<endl<<t3<<endl<<t4<<endl<<t5<<endl<<t6<<endl<<t7<<endl<<t8<<endl<<t9<<endl<<t10<<endl<<t11<<endl<<t12<<endl<<t13<<endl<<endl; 
	
		coefsN(i,0) = t2(i,0)+t3(i,0)+t4(i,0)-mx2(i,0)*my1(i,0)*r2(i,0)-mx1(i,0)*ny2(i,0)*s2(i,0)-my2(i,0)*nx2(i,0)*s1(i,0); 
		coefsN(i,1) = t11(i,0)+t12(i,0)-mx1(i,0)*my2(i,0)*ny2(i,0)*2.0-mx1(i,0)*r2(i,0)*s2(i,0)*2.0; 
		coefsN(i,2) = t7(i,0)+t8(i,0)-mx2(i,0)*my1(i,0)*nx2(i,0)*2.0-my1(i,0)*r2(i,0)*s2(i,0)*2.0; 
		coefsN(i,3) = t5(i,0)+t6(i,0)-mx2(i,0)*nx2(i,0)*s1(i,0)*2.0-my2(i,0)*ny2(i,0)*s1(i,0)*2.0; 
		coefsN(i,4) = -t2(i,0)-t3(i,0)+t4(i,0)+t9(i,0)+t10(i,0)-my2(i,0)*nx2(i,0)*s1(i,0); 
		coefsN(i,5) = -t5(i,0)+t6(i,0)+mx2(i,0)*nx2(i,0)*s1(i,0)*2.0-my2(i,0)*ny2(i,0)*s1(i,0)*2.0; 
		coefsN(i,6) = t7(i,0)-t8(i,0)-mx2(i,0)*my1(i,0)*nx2(i,0)*2.0+my1(i,0)*r2(i,0)*s2(i,0)*2.0; 
		coefsN(i,7) = -t2(i,0)+t3(i,0)-t4(i,0)+t9(i,0)-t10(i,0)+t13(i,0); 
		coefsN(i,8) = -t11(i,0)+t12(i,0)+mx1(i,0)*my2(i,0)*ny2(i,0)*2.0-mx1(i,0)*r2(i,0)*s2(i,0)*2.0; 
		coefsN(i,9) = t2(i,0)-t3(i,0)-t4(i,0)-t9(i,0)+t10(i,0)+t13(i,0); 
	} 
// cout<<coefsN<<endl<<endl; 

}

void QuEst::coefsDen(Eigen::MatrixXd& coefsD, const Eigen::VectorXd& mx2, const Eigen::VectorXd& my2, 
	const Eigen::VectorXd& nx1, const Eigen::VectorXd& nx2, 
	const Eigen::VectorXd& ny1, const Eigen::VectorXd& ny2, 
	const Eigen::VectorXd& r1, const Eigen::VectorXd& r2, const Eigen::VectorXd& s2)
{

	int numPts = mx2.rows(); 

	Eigen::VectorXd t2_D(numPts,1); 
	Eigen::VectorXd t3_D(numPts,1); 
	Eigen::VectorXd t4_D(numPts,1); 
	Eigen::VectorXd t5_D(numPts,1); 
	Eigen::VectorXd t6_D(numPts,1); 
	Eigen::VectorXd t7_D(numPts,1); 
	Eigen::VectorXd t8_D(numPts,1); 
	Eigen::VectorXd t9_D(numPts,1); 
	Eigen::VectorXd t10_D(numPts,1); 
	Eigen::VectorXd t11_D(numPts,1); 
	Eigen::VectorXd t12_D(numPts,1); 
	Eigen::VectorXd t13_D(numPts,1); 
	for(int i=0; i<numPts; i++){
		t2_D(i,0) = mx2(i,0)*ny1(i,0)*r2(i,0); 
		t3_D(i,0) = my2(i,0)*nx2(i,0)*r1(i,0); 
		t4_D(i,0) = nx1(i,0)*ny2(i,0)*s2(i,0); 
		t5_D(i,0) = mx2(i,0)*nx2(i,0)*r1(i,0)*2.0; 
		t6_D(i,0) = my2(i,0)*ny2(i,0)*r1(i,0)*2.0; 
		t7_D(i,0) = mx2(i,0)*nx2(i,0)*ny1(i,0)*2.0; 
		t8_D(i,0) = ny1(i,0)*r2(i,0)*s2(i,0)*2.0; 
		t9_D(i,0) = my2(i,0)*nx1(i,0)*r2(i,0); 
		t10_D(i,0) = nx2(i,0)*ny1(i,0)*s2(i,0); 
		t11_D(i,0) = my2(i,0)*nx1(i,0)*ny2(i,0)*2.0; 
		t12_D(i,0) = nx1(i,0)*r2(i,0)*s2(i,0)*2.0; 
		t13_D(i,0) = mx2(i,0)*ny2(i,0)*r1(i,0); 
// cout<<t2_D<<endl<<t3_D<<endl<<t4_D<<endl<<t5_D<<endl<<t6_D<<endl<<t7_D<<endl<<t8_D<<endl<<t9_D<<endl<<t10_D<<endl<<t11_D<<endl<<t12_D<<endl<<t13_D<<endl<<endl; 

		coefsD(i,0) = t2_D(i,0)+t3_D(i,0)+t4_D(i,0)-mx2(i,0)*ny2(i,0)*r1(i,0)-my2(i,0)*nx1(i,0)*r2(i,0)-nx2(i,0)*ny1(i,0)*s2(i,0); 
		coefsD(i,1) = t11_D(i,0)+t12_D(i,0)-my2(i,0)*nx2(i,0)*ny1(i,0)*2.0-nx2(i,0)*r1(i,0)*s2(i,0)*2.0; 
		coefsD(i,2) = t7_D(i,0)+t8_D(i,0)-mx2(i,0)*nx1(i,0)*ny2(i,0)*2.0-ny2(i,0)*r1(i,0)*s2(i,0)*2.0; 
		coefsD(i,3) = t5_D(i,0)+t6_D(i,0)-mx2(i,0)*nx1(i,0)*r2(i,0)*2.0-my2(i,0)*ny1(i,0)*r2(i,0)*2.0; 
		coefsD(i,4) = t2_D(i,0)-t3_D(i,0)-t4_D(i,0)+t9_D(i,0)+t10_D(i,0)-mx2(i,0)*ny2(i,0)*r1(i,0); 
		coefsD(i,5) = t5_D(i,0)-t6_D(i,0)-mx2(i,0)*nx1(i,0)*r2(i,0)*2.0+my2(i,0)*ny1(i,0)*r2(i,0)*2.0; 
		coefsD(i,6) = -t7_D(i,0)+t8_D(i,0)+mx2(i,0)*nx1(i,0)*ny2(i,0)*2.0-ny2(i,0)*r1(i,0)*s2(i,0)*2.0; 
		coefsD(i,7) = -t2_D(i,0)+t3_D(i,0)-t4_D(i,0)-t9_D(i,0)+t10_D(i,0)+t13_D(i,0); 
		coefsD(i,8) = t11_D(i,0)-t12_D(i,0)-my2(i,0)*nx2(i,0)*ny1(i,0)*2.0+nx2(i,0)*r1(i,0)*s2(i,0)*2.0; 
		coefsD(i,9) = -t2_D(i,0)-t3_D(i,0)+t4_D(i,0)+t9_D(i,0)-t10_D(i,0)+t13_D(i,0); 
	} 	
// cout<<coefsD<<endl<<endl; 

}

void QuEst::coefsNumDen(Eigen::MatrixXd& coefsND, const Eigen::VectorXd& a1, const Eigen::VectorXd& a2, const Eigen::VectorXd& a3, 
	const Eigen::VectorXd& a4, const Eigen::VectorXd& a5, const Eigen::VectorXd& a6, const Eigen::VectorXd& a7, 
	const Eigen::VectorXd& a8, const Eigen::VectorXd& a9, const Eigen::VectorXd& a10, 
	const Eigen::VectorXd& b1, const Eigen::VectorXd& b2, const Eigen::VectorXd& b3, const Eigen::VectorXd& b4, 
	const Eigen::VectorXd& b5, const Eigen::VectorXd& b6, const Eigen::VectorXd& b7, const Eigen::VectorXd& b8, 
	const Eigen::VectorXd& b9, const Eigen::VectorXd& b10)
{

	int numPts = a1.rows(); 

	for(int i=0; i<numPts; i++)
	{

		coefsND(i,0) = a1(i,0)*b1(i,0); 
		coefsND(i,1) = a1(i,0)*b2(i,0)+a2(i,0)*b1(i,0); 
		coefsND(i,2) = a2(i,0)*b2(i,0)+a1(i,0)*b5(i,0)+a5(i,0)*b1(i,0); 
		coefsND(i,3) = a2(i,0)*b5(i,0)+a5(i,0)*b2(i,0); 
		coefsND(i,4) = a5(i,0)*b5(i,0); 
		coefsND(i,5) = a1(i,0)*b3(i,0)+a3(i,0)*b1(i,0); 
		coefsND(i,6) = a2(i,0)*b3(i,0)+a3(i,0)*b2(i,0)+a1(i,0)*b6(i,0)+a6(i,0)*b1(i,0); 
		coefsND(i,7) = a2(i,0)*b6(i,0)+a3(i,0)*b5(i,0)+a5(i,0)*b3(i,0)+a6(i,0)*b2(i,0); 
		coefsND(i,8) = a5(i,0)*b6(i,0)+a6(i,0)*b5(i,0); 
		coefsND(i,9) = a3(i,0)*b3(i,0)+a1(i,0)*b8(i,0)+a8(i,0)*b1(i,0); 
		coefsND(i,10) = a3(i,0)*b6(i,0)+a6(i,0)*b3(i,0)+a2(i,0)*b8(i,0)+a8(i,0)*b2(i,0); 
		coefsND(i,11) = a6(i,0)*b6(i,0)+a5(i,0)*b8(i,0)+a8(i,0)*b5(i,0); 
		coefsND(i,12) = a3(i,0)*b8(i,0)+a8(i,0)*b3(i,0); 
		coefsND(i,13) = a6(i,0)*b8(i,0)+a8(i,0)*b6(i,0); 
		coefsND(i,14) = a8(i,0)*b8(i,0); 
		coefsND(i,15) = a1(i,0)*b4(i,0)+a4(i,0)*b1(i,0); 
		coefsND(i,16) = a2(i,0)*b4(i,0)+a4(i,0)*b2(i,0)+a1(i,0)*b7(i,0)+a7(i,0)*b1(i,0); 
		coefsND(i,17) = a2(i,0)*b7(i,0)+a4(i,0)*b5(i,0)+a5(i,0)*b4(i,0)+a7(i,0)*b2(i,0); 
		coefsND(i,18) = a5(i,0)*b7(i,0)+a7(i,0)*b5(i,0); 
		coefsND(i,19) = a3(i,0)*b4(i,0)+a4(i,0)*b3(i,0)+a1(i,0)*b9(i,0)+a9(i,0)*b1(i,0); 
		coefsND(i,20) = a3(i,0)*b7(i,0)+a4(i,0)*b6(i,0)+a6(i,0)*b4(i,0)+a7(i,0)*b3(i,0)+a2(i,0)*b9(i,0)+a9(i,0)*b2(i,0); 
		coefsND(i,21) = a6(i,0)*b7(i,0)+a7(i,0)*b6(i,0)+a5(i,0)*b9(i,0)+a9(i,0)*b5(i,0); 
		coefsND(i,22) = a3(i,0)*b9(i,0)+a4(i,0)*b8(i,0)+a8(i,0)*b4(i,0)+a9(i,0)*b3(i,0); 
		coefsND(i,23) = a6(i,0)*b9(i,0)+a7(i,0)*b8(i,0)+a8(i,0)*b7(i,0)+a9(i,0)*b6(i,0); 
		coefsND(i,24) = a8(i,0)*b9(i,0)+a9(i,0)*b8(i,0); 
		coefsND(i,25) = a4(i,0)*b4(i,0)+a1(i,0)*b10(i,0)+a10(i,0)*b1(i,0); 
		coefsND(i,26) = a4(i,0)*b7(i,0)+a7(i,0)*b4(i,0)+a2(i,0)*b10(i,0)+a10(i,0)*b2(i,0); 
		coefsND(i,27) = a7(i,0)*b7(i,0)+a5(i,0)*b10(i,0)+a10(i,0)*b5(i,0); 
		coefsND(i,28) = a3(i,0)*b10(i,0)+a4(i,0)*b9(i,0)+a9(i,0)*b4(i,0)+a10(i,0)*b3(i,0); 
		coefsND(i,29) = a6(i,0)*b10(i,0)+a7(i,0)*b9(i,0)+a9(i,0)*b7(i,0)+a10(i,0)*b6(i,0); 
		coefsND(i,30) = a8(i,0)*b10(i,0)+a9(i,0)*b9(i,0)+a10(i,0)*b8(i,0); 
		coefsND(i,31) = a4(i,0)*b10(i,0)+a10(i,0)*b4(i,0); 
		coefsND(i,32) = a7(i,0)*b10(i,0)+a10(i,0)*b7(i,0); 
		coefsND(i,33) = a9(i,0)*b10(i,0)+a10(i,0)*b9(i,0); 
		coefsND(i,34) = a10(i,0)*b10(i,0); 
	}
}

//~ void QuEst::SolveQPData(const Eigen::MatrixXd C, Params params, Eigen::MatrixXd & X) 
void QuEst::SolveQPData(const Eigen::MatrixXd C, Eigen::MatrixXd & X) 
{
	// https://cvxgen.com/docs/c_interface.html
	// In this function, load all problem instance data.
	
	//~ params.A[i] = ...;
	int i = 0;
	int j = 0;
	Eigen::MatrixXd Ae;
	Ae.resize(10, 13);
	
	Ae << Eigen::MatrixXd::Zero(10, 3) , Eigen::MatrixXd::Identity(10,10);
	//~ std::cout << "Ae = " << std::endl << Ae << std::endl;
	
	for (i = 0; i < Ae.rows(); i++)
	{
		for (j = 0; j < Ae.cols(); j++)
		{
			//~ params.A[i+j*Ae.cols()] = Ae(i, j);
			params.A[i+j*Ae.rows()] = Ae(i, j);
		}
	}
	
	//~ std::cout << "A = " << std::endl;
	//~ for (i = 0; i < Ae.rows(); i++)
	//~ {
		//~ for (j = 0; j < Ae.cols(); j++)
		//~ {
			//~ printf("%f ", params.A[i+j*Ae.rows()]);
			
		//~ }
		//~ printf("\n ");
	//~ }
	
	//~ std::cout << "Ce = " << std::endl << C << std::endl;
	Eigen::MatrixXd Qe = ( C.transpose() ) * C;
	//~ std::cout << "Qe rows = " << Qe.rows() << " Qe Cols = " << Qe.cols() << std::endl;
	//~ std::cout << "Qe = " << std::endl << Qe << std::endl;
	
	for (i = 0; i < Qe.rows(); i++)
	{
		for (j = 0; j < Qe.cols(); j++)
		{
			//~ params.Q[i+j*Qe.cols()] = double(Qe(i, j));
			params.Q[i+j*Qe.rows()] = double(Qe(i, j));
		}
	}
	//~ std::cout << "Q = " << std::endl;
	//~ for (i = 0; i < Qe.rows(); i++)
	//~ {
		//~ for (j = 0; j < Qe.cols(); j++)
		//~ {
			//~ printf("%f ", params.Q[i+j*Qe.rows()]);
		//~ }
		//~ printf("\n ");
	//~ }
	
	// Set the scalar parameter lambda. Note the 0 array index.
	//~ params.lambda[0] = ...;
	
	int num_iters = solve();
	//~ ROS_INFO("%% QuadProg interations: %d \n", num_iters);
	if (PrintToScreen)
	{
		printf("%% QuadProg interations: %d \n", num_iters);
	}
	X.resize(13, 1);
	
	// Variables are retrieved in the same way, from the vars structure.
	if (work.converged == 1)
	{
		for (int i = 0; i < 13; i++)
		{
			X(i) = double(vars.x[i]);
			//~ ROS_INFO("X of %d : %f ", i, double(vars.x[i]));	
		}
	}
	else
	{
		for (int i = 0; i < 13; i++)
		{
			X(i) = double(vars.x[i]);
			//~ ROS_INFO("X of %d : %f \n", i, double(vars.x[i]));	
		}
		ROS_FATAL("QuadProg (CVXGEN) failed to converge!");
	}
	//~ std::cout << "X = " << std::endl << X << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "QuEstTransformPublisher");
	ros::NodeHandle nh;
	
	QuEst QM = QuEst(nh);
	
	ros::spin();
	
	return 0;
	
}
