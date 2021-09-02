#include <ros/ros.h>
#include <utari_sba_pkg/sba.h>
#include <utari_sba_pkg/visualization.h>


// For random seed.
#include <time.h>

#include <visualization_msgs/Marker.h>
#include "vbme_msgs/VbmeData_msg.h"

// basic file operations
#include <iostream>
#include <fstream>

//~ #include <time.h>

//~ #define DTTMFMT "%Y-%m-%d %H:%M:%S "
//~ #define DTTMSZ 21


using namespace sba;
using namespace std;

void ClearBuff(char * buff, int size)
{
	for (int i = 0; i < size; i++)
	{
		buff[i] = '\0';
	}
}
// convert quaternion into rotation matrix ---------- 
void Q2R(Eigen::MatrixXd& R_Q2R, const Eigen::Matrix4Xd& Q)
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

void Q2R_3by3(Eigen::Matrix3d& R_Q2R, const Eigen::Vector4d& Q)
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

void R2Q_3by3(const Eigen::Matrix3d& R,  Eigen::Vector4d& Q)
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

//~ void Eig2Htm(const Eigen::Matrix4Xd& Q, const Eigen::Matrix<double, 3, 1> T, Eigen::Matrix4d& Htm)
void Eig2Htm(const Eigen::Vector4d& Q, const Eigen::Matrix<double, 3, 1> T, Eigen::Matrix4d& Htm)
{
	Eigen::Matrix3d Rot; 
	Q2R_3by3(Rot, Q);
	
	Htm << Rot, T, 0,0,0,1;
}

void Ros2Htm(const geometry_msgs::Quaternion Q, const geometry_msgs::Vector3 T, Eigen::Matrix4d& Htm)
{
		//~ Eigen::Matrix4Xd QEig;
		Eigen::Vector4d QEig;
		QEig(0) = Q.w;
		QEig(1) = Q.x;
		QEig(2) = Q.y;
		QEig(3) = Q.z;
		Eigen::Matrix<double, 3, 1> TEig;
		TEig(0) = T.x;
		TEig(1) = T.y;
		TEig(2) = T.z;
		
		Eig2Htm(QEig, TEig, Htm);
}

class SbaTopics
{
public: // public vars
	
private: // private vars 
	ros::NodeHandle * n;
	ros::Subscriber VbmeInput;
	ros::Publisher VbmeOutput;
	ros::Publisher cam_marker_pub;
	ros::Publisher point_marker_pub;
	
	bool PrintToScreen;
	bool LoopSlow;
	float LoopSleep;
	// Create sba camera parameters.
	frame_common::CamParams cam_params;
	// Define dimensions of the image.
	int maxx; 							
	int maxy;
	bool NormalizeOutput; 										// rosparam, QuEstoutput with or without output normilized 
	// Create an empty SBA system.
	SysSBA sys;
	int CallbackN; 												// count of callbacks
	
	
public: // public methods
	SbaTopics(ros::NodeHandle nh) // constructor
	{
		n = &nh;
		
		CallbackN = -1; 											// count of callbacks
		GetParams();
		
		VbmeInput = nh.subscribe("/vbme/QuestData", 10, &SbaTopics::SBA_callback, this);  // input directly from QuEst
		//~ VbmeInput = nh.subscribe("/vbme/VestData", 10, &SbaTopics::SBA_callback, this); 	  // input from BigC Matrix
		VbmeOutput = nh.advertise<vbme_msgs::VbmeData_msg>("SbaData", 10);
		
		// Create publisher topics.
		cam_marker_pub = nh.advertise<visualization_msgs::Marker>("/sba/cameras", 10);
		point_marker_pub = nh.advertise<visualization_msgs::Marker>("/sba/points", 10);
		
		ros::Duration(0.1).sleep();
		//~ ros::spinOnce();
		
		
		
		ROS_INFO("sba ready");
	}
	
		
private: // private methods
	void GetParams()
	{
		n->param<bool>("PrintToScreen", PrintToScreen, true);
		n->param<bool>("LoopSlow", LoopSlow, true);
		n->param<float>("LoopSleep", LoopSleep, 1.0);
		n->param<bool>("NormalizeOutput", NormalizeOutput, false); 	// was QuEst output normalized?
		
		// Create sba camera parameters.
		//~ n->param<double>("cam_params/fx", cam_params.fx, 430.0); // Focal length in x
		//~ n->param<double>("cam_params/fy", cam_params.fx, 430.0); // Focal length in y
		n->param<double>("cam_params/fx", cam_params.fx, 984.2439); // Focal length in x
		n->param<double>("cam_params/fy", cam_params.fy, 980.8141); // Focal length in y
		n->param<double>("cam_params/cx", cam_params.cx, 690.0); 	// X position of principal point
		n->param<double>("cam_params/cy", cam_params.cx, 233.1966); // Y position of principal point
		n->param<double>("cam_params/tx", cam_params.tx, 0.0); 		// Baseline (no baseline since this is monocular)
		//~ n->param<double>("cam_params/fx", cam_params.fx, 0.0); 	// Focal length in x
		//~ n->param<double>("cam_params/fy", cam_params.fy, 0.0); 	// Focal length in y
		//~ n->param<double>("cam_params/cx", cam_params.cx, 1.0); 	// X position of principal point
		//~ n->param<double>("cam_params/cy", cam_params.cx, 1.0); 	// Y position of principal point
		//~ n->param<double>("cam_params/tx", cam_params.tx, 0.0); 	// Baseline (no baseline since this is monocular)
		
		n->param<int>("cam_params/image/sizex", maxx, 1392); 		// Define dimensions of the image.
		n->param<int>("cam_params/image/sizey", maxy, 512);
		
	}
	
	
	void SBA_callback(const vbme_msgs::VbmeData_msg::ConstPtr& msg)
	{
		ROS_INFO("Callback");
		std::cout << "Cam1 Trans Msg: " << std::endl << msg->transform << std::endl;
		CallbackN++;
		// If possible read these off each image
		// Create camera parameters.
		//~ frame_common::CamParams cam_params;
		//~ cam_params.fx = 430; // Focal length in x
		//~ cam_params.fy = 430; // Focal length in y
		//~ cam_params.cx = 320; // X position of principal point
		//~ cam_params.cy = 240; // Y position of principal point
		//~ cam_params.tx = 0;   // Baseline (no baseline since this is monocular)
		// Define dimensions of the image.
		//~ int maxx = 640;
		//~ int maxy = 480;
		
		// /////////////////////////////////////////////////////////////
		// populate nodes (cameras)
		// camera 0 
		Vector4d trans0;//(i/(nnodes-1.0)*path_length, 0, 0, 1);
		trans0(0) = msg->TransformCam0.translation.x;
		trans0(1) = msg->TransformCam0.translation.y;
		trans0(2) = msg->TransformCam0.translation.z;
		trans0(3) = 1;
		Quaterniond rot0;
		rot0.w() = msg->TransformCam0.rotation.w;
		rot0.x() = msg->TransformCam0.rotation.x;
		rot0.y() = msg->TransformCam0.rotation.y;
		rot0.z() = msg->TransformCam0.rotation.z;
		rot0.normalize();
		
		//~ std::cout << "Rot w org: " << rot.w() << std::endl;
		//~ std::cout << "Rot V org: " << rot.vec() << std::endl;
		//~ std::cout << "Rot w coj: " << rot.conjugate().w() << std::endl;
		//~ std::cout << "Rot V coj: " << rot.conjugate().vec() << std::endl;
		//~ std::cout << "Cam0 Trans: " << trans << std::endl;

		// Add a new node to the system.
		//~ sys.addNode(trans0, rot0, cam_params, false);
		// true means fixed camera, cannot move
		sys.addNode(trans0, rot0, cam_params, true);
      
		// camera 1 (transform(begining to end))
		Vector4d trans1;//(i/(nnodes-1.0)*path_length, 0, 0, 1);
		trans1(0) = msg->transform.translation.x;
		trans1(1) = msg->transform.translation.y;
		trans1(2) = msg->transform.translation.z;
		trans1(3) = 1;
		Quaterniond rot1; 										// rotation between cameras
		rot1.w() = msg->transform.rotation.w;
		rot1.x() = msg->transform.rotation.x;
		rot1.y() = msg->transform.rotation.y;
		rot1.z() = msg->transform.rotation.z;
		rot1.normalize();
		
		std::cout << "Cam1 Trans Msg: " << std::endl << msg->transform.translation << std::endl;
		std::cout << "Cam1 Trans Msg Z: " << std::endl << msg->transform.translation.z << std::endl;
		std::cout << "Cam1 Trans Z: " << std::endl << trans1(2) << std::endl;
		std::cout << "Cam1 Trans: " << std::endl << trans1 << std::endl;

		// Add a new node to the system.
		sys.addNode(trans1, rot1, cam_params, false);
		
		// /////////////////////////////////////////////////////////////
		// populate feature points. Add points into the system
		// 3D points
		
		// print depths
		//~ std::cout << "Depths: " << std::endl;
		//~ for (int i = 0; i < msg->Depths.size(); i++)
		//~ {
			//~ std::cout << msg->Depths[i] << std::endl;
		//~ }
		
		// calculate HTM (homogenious transform matrix) for cam0
		Eigen::Matrix4d Cam0Htm;
		Ros2Htm(msg->TransformCam0.rotation, msg->TransformCam0.translation, Cam0Htm);
		//~ std::cout << "CamoTrans: " << std::endl << msg->TransformCam0.translation << std::endl;
		
		for (int i = 0; i < (int)msg->X1.size(); i++)
		{
			// force depths to be positive 
			double Depth;
			if (msg->Depths[ (i*2) ] < 0.0)
			{
				Depth =  -1.0 * msg->Depths[ (i*2) ];
			}
			else
			{
				Depth = msg->Depths[ (i*2) ];
			}
			
			Eigen::Vector4d temppoint;
			//~ Eigen::Matrix<double, 3, 1> temppoint;
			// multiply calibrated feature points by depth to calculate 3d pts
			// this will be in reference to cam0
			//~ temppoint.x() = msg->X1[i].x * msg->Depths[ (i*2) ];
			//~ temppoint.y() = msg->X1[i].y * msg->Depths[ (i*2) ];
			//~ temppoint.z() = msg->Depths[ (i*2) ];
			//~ temppoint.w() = 1;
			temppoint(0) = msg->X1[i].x * Depth;
			temppoint(1) = msg->X1[i].y * Depth;
			temppoint(2) = Depth;
			temppoint(3) = 1;
			
			// calulate position in very first camera position (origin)
			Eigen::Vector4d Point3d;
			Point3d = Cam0Htm * temppoint;
			//~ std::cout << "Cam0: " << std::endl << Cam0Htm << std::endl;
			//~ std::cout << "PointCam: " << std::endl << temppoint << std::endl;
			//~ std::cout << "PointOri: " << std::endl << Point3d << std::endl;
			
			sys.addPoint(Point3d);
		}
		
		
		
		// /////////////////////////////////////////////////////////////
		// calibrated 2D points
		Vector2d proj;
		// Project points into nodes.
		// camera 0
		for (int i = 0; i < (int)msg->X1.size(); i++) 			// number of feature points
		{
			Vector2d proj;
			// Project the point into the node's image coordinate system.
			sys.nodes[0].setProjection();
			//~ sys.nodes[j].project2im(proj, points[i]);
				
			//~ proj(0) = msg->X1[i].x; 						// normalized calibrated points
			//~ proj(1) = msg->X1[i].y;
			proj(0) = msg->MatchList.Points[i].FlowArray.back().x; 	// raw feature points
			proj(1) = msg->MatchList.Points[i].FlowArray.back().y;
			// sys.addMonoProj (cam_num, pt_num, pt)
			//~ sys.addMonoProj(0, i, proj); 					// for 2 nodes only
			//~ sys.addMonoProj( (CallbackN*2) ,  ((CallbackN*2) + i ) , proj); // for CallbackN*2 nodes 
			sys.addMonoProj( (CallbackN*2) ,  ((CallbackN*5) + i ) , proj); // for CallbackN*2 nodes 
		}
		// camera 1
		for (int i = 0; i < (int)msg->X2.size(); i++) 			// number of feature points
		{
			Vector2d proj;
			// Project the point into the node's image coordinate system.
			sys.nodes[1].setProjection();
			//~ sys.nodes[j].project2im(proj, points[i]);
			//~ proj(0) = msg->X2[i].x;
			//~ proj(1) = msg->X2[i].y;
			proj(0) = msg->MatchList.Points[i].FlowArray.front().x;
			proj(1) = msg->MatchList.Points[i].FlowArray.front().y;
			//~ sys.addMonoProj(1, i, proj); 					// for 2 nodes only
			//~ sys.addMonoProj( ( (CallbackN*2) + 1 ) , ((CallbackN*2) + i ) , proj); // for CallbackN*2 nodes 
			sys.addMonoProj( ( (CallbackN*2) + 1 ) , ((CallbackN*5) + i ) , proj); // for CallbackN*2 nodes 
		}
		// /////////////////////////////////////////////////////////////
		
		// Provide some information about the data read in.
		ROS_INFO("Cameras (nodes): %d, Points: %d",
			(int)sys.nodes.size(), (int)sys.tracks.size());
		//~ // Publish markers
		drawGraph(sys, cam_marker_pub, point_marker_pub);
		//~ ros::spinOnce();
		ROS_INFO("Sleeping for 5 seconds to publish pre-SBA markers.");
		ros::Duration(5.0).sleep();
		
		// /////////////////////////////////////////////////////////////
		// to create log files
		
		//~ ofstream myfileNodes; 									
		//~ ofstream myfile3dPts;
		//~ ofstream myfile2dPts;
		//~ myfileNodes.open ("/tmp/sba_topics_in_nodes.csv", std::fstream::app);
		//~ myfile3dPts.open ("/tmp/sba_topics_in_3dpts.csv", std::fstream::app);
		//~ myfile2dPts.open ("/tmp/sba_topics_in_2dpts.csv", std::fstream::app);
		//~ int Size = 500;
		//~ char BuffData[Size];
		//~ ClearBuff(BuffData, Size);
		//~ // code will crash (later in code?) if you write to file without calling good before....
		//~ if (myfileNodes.good()) 								
		//~ {
			//~ for (int i = 0; i < 2; i++)
			//~ {
				//~ sprintf(BuffData, "node, %d, %f, %f, %f, %f, %f, %f, %f, ", i, sys.nodes[i].trans.x(), sys.nodes[i].trans.y(), sys.nodes[i].trans.z(), sys.nodes[i].qrot.w(), sys.nodes[i].qrot.x(), sys.nodes[i].qrot.y(), sys.nodes[i].qrot.z());
				//~ myfileNodes << BuffData;
			//~ }
			//~ ClearBuff(BuffData, Size);
			//~ sprintf(BuffData, "\n");
			//~ myfileNodes << BuffData;
			//~ ClearBuff(BuffData, Size);
		//~ }
		//~ else
		//~ {
			//~ ROS_WARN("Could not open myfileNodes");
		//~ }
		//~ if (myfile3dPts.good())
		//~ {
			//~ for (int i = 0; i < 5; i++)
			//~ {
				//~ sprintf(BuffData, "3d pts, %d, %f, %f, %f, %f, ", i, sys.tracks[i].point.x(), sys.tracks[i].point.y(), sys.tracks[i].point.z(), sys.tracks[i].point.w());
				//~ myfile3dPts << BuffData;
			//~ }
			//~ ClearBuff(BuffData, Size);
			//~ sprintf(BuffData, "\n");
			//~ myfile3dPts << BuffData;
			//~ ClearBuff(BuffData, Size);
			
		//~ }
		//~ else
		//~ {
			//~ ROS_WARN("Could not open myfile3dPts");
		//~ }	
		//~ if (myfile2dPts.good())
		//~ {
			//~ for (int i = 0; i < 5; i++)
			//~ {
				//~ for (int j = 0; j < 2; j++)
				//~ {
					//~ sprintf(BuffData, "2d pts, %d, from node, %d, is valid, %d,  %f, %f, %f, ", i, j, sys.tracks[i].projections[j].isValid, 
					//~ sys.tracks[i].projections[j].kp[0], sys.tracks[i].projections[j].kp[1], sys.tracks[i].projections[j].kp[2]);
					//~ myfile2dPts << BuffData;
				//~ }
			//~ }
			//~ ClearBuff(BuffData, Size);
			//~ sprintf(BuffData, "\n");
			//~ myfile2dPts << BuffData;
			//~ ClearBuff(BuffData, Size);
		//~ }
		//~ else
		//~ {
			//~ ROS_WARN("Could not open myfile2dPts");
		//~ }
		
		//~ myfileNodes.close();
		//~ myfile3dPts.close();
		//~ myfile2dPts.close();
		
		// /////////////////////////////////////////////////////////////
		// call SBA 
		
		// Perform SBA with 10 iterations, an initial lambda step-size of 1e-3, 
		// and using CSPARSE.
		// Run the LM algorithm that computes a nonlinear SBA estimate.
		// <niter> is the max number of iterations to perform; returns the
		// number actually performed.
		// <useCSparse> = 0 for dense Cholesky, 1 for sparse system, 
		//                2 for gradient system, 3 for block jacobian PCG
		// <initTol> is the initial tolerance for CG 
		// int SysSBA::doSBA(int niter, double sLambda, int useCSparse, double initTol, int maxCGiter)
		//~ ROS_INFO("  12.2 SBA type: %d", SBA_SPARSE_CHOLESKY);
		//~ sys.doSBA(10, 1e-3, SBA_SPARSE_CHOLESKY);
		//~ sys.doSBA(10, 1e-3, 1);
		sys.doSBA(20, 1e-3, 1);
		
		//~ sys.doSBA(10, 1e-3, 2, 1e-3, 50);
		
		int npts = sys.tracks.size();

		ROS_INFO("Bad projs (> 10 pix): %d, Cost without: %f", 
			(int)sys.countBad(10.0), sqrt(sys.calcCost(10.0)/npts));
		ROS_INFO("Bad projs (> 5 pix): %d, Cost without: %f", 
			(int)sys.countBad(5.0), sqrt(sys.calcCost(5.0)/npts));
		ROS_INFO("Bad projs (> 2 pix): %d, Cost without: %f", 
			(int)sys.countBad(2.0), sqrt(sys.calcCost(2.0)/npts));
		ROS_INFO("  14");
		ROS_INFO("Cameras (nodes): %d, Points: %d",
			(int)sys.nodes.size(), (int)sys.tracks.size());
			
		// Publish markers
		drawGraph(sys, cam_marker_pub, point_marker_pub);
		//~ ros::spinOnce();
		//~ ROS_INFO("Sleeping for 5 seconds to publish post-SBA markers.");
		//~ ros::Duration(5.0).sleep();
		ROS_INFO("Sleeping for 2 seconds to publish post-SBA markers.");
		ros::Duration(2.0).sleep();
		
		// /////////////////////////////////////////////////////////////
		//  publish output and create log files
		
		// data to simply copy over
		vbme_msgs::VbmeData_msg OutputMsg;
		OutputMsg.header = msg->header;
		OutputMsg.MatchList = msg->MatchList;
		OutputMsg.Depths = msg->Depths;
		OutputMsg.Rotation = msg->Rotation;
		OutputMsg.Velocity = msg->Velocity;
		
		//~ // output log files
		//~ ofstream myfileNodesOut;
		//~ ofstream myfile3dPtsOut;
		//~ ofstream myfile2dPtsOut;
		//~ myfileNodesOut.open ("/tmp/sba_topics_out_nodes.csv", std::fstream::app);
		//~ myfile3dPtsOut.open ("/tmp/sba_topics_out_3dpts.csv", std::fstream::app);
		//~ myfile2dPtsOut.open ("/tmp/sba_topics_out_2dpts.csv", std::fstream::app);
		
		//~ if (myfileNodesOut.good())
		//~ {
			//~ for (int j = 0; j < 2; j++)
			//~ {
				//~ int i = 2*CallbackN + j;
				//~ // sprintf(BuffData, "node, %d, %f, %f, %f, %f, %f, %f, %f, ", i, sys.nodes[i].trans.x(), sys.nodes[i].trans.y(), sys.nodes[i].trans.z(), sys.nodes[i].qrot.w(), sys.nodes[i].qrot.x(), sys.nodes[i].qrot.y(), sys.nodes[i].qrot.z());
				//~ int n = CallbackN + j;
				//~ sprintf(BuffData, "node, %d, %f, %f, %f, %f, %f, %f, %f, node, %d, %f, %f, %f, %f, %f, %f, %f, ", i, sys.nodes[i].trans.x(), sys.nodes[i].trans.y(), sys.nodes[i].trans.z(), sys.nodes[i].qrot.w(), sys.nodes[i].qrot.x(), sys.nodes[i].qrot.y(), sys.nodes[i].qrot.z(), n, sys.nodes[n].trans.x(), sys.nodes[n].trans.y(), sys.nodes[n].trans.z(), sys.nodes[n].qrot.w(), sys.nodes[n].qrot.x(), sys.nodes[n].qrot.y(), sys.nodes[n].qrot.z());
				//~ myfileNodesOut << BuffData;
			//~ }
			//~ ClearBuff(BuffData, Size);
			//~ sprintf(BuffData, "\n");
			//~ myfileNodesOut << BuffData;
			//~ ClearBuff(BuffData, Size);
		//~ }
		//~ else
		//~ {
			//~ ROS_WARN("Could not open myfileNodesOut");
		//~ }
		
		// cam position msg isnt loop friendly 
		OutputMsg.TransformCam0.translation.x = sys.nodes[0].trans.x();
		OutputMsg.TransformCam0.translation.y = sys.nodes[0].trans.y();
		OutputMsg.TransformCam0.translation.z = sys.nodes[0].trans.z();
		OutputMsg.TransformCam0.rotation.w = sys.nodes[0].qrot.w();
		OutputMsg.TransformCam0.rotation.x = sys.nodes[0].qrot.x();
		OutputMsg.TransformCam0.rotation.y = sys.nodes[0].qrot.y();
		OutputMsg.TransformCam0.rotation.z = sys.nodes[0].qrot.z();
		
		OutputMsg.transform.translation.x = sys.nodes[1].trans.x();
		OutputMsg.transform.translation.y = sys.nodes[1].trans.y();
		OutputMsg.transform.translation.z = sys.nodes[1].trans.z();
		OutputMsg.transform.rotation.w = sys.nodes[1].qrot.w();
		OutputMsg.transform.rotation.x = sys.nodes[1].qrot.x();
		OutputMsg.transform.rotation.y = sys.nodes[1].qrot.y();
		OutputMsg.transform.rotation.z = sys.nodes[1].qrot.z();
		
		//~ for (int j = 0; j < 5; j++)
		//~ {
			//~ int i = 2*CallbackN + j;
			//~ int n = CallbackN + j;
			//~ if (myfile3dPtsOut.good())
			//~ {
				//~ // sprintf(BuffData, "3d pts, %d, %f, %f, %f, %f, ", i, sys.tracks[i].point.x(), sys.tracks[i].point.y(), sys.tracks[i].point.z(), sys.tracks[i].point.w());
				//~ sprintf(BuffData, "3D pts, %d, %f, %f, %f, %f, 3d pts, %d, %f, %f, %f, %f, ", i, sys.tracks[i].point.x(), sys.tracks[i].point.y(), sys.tracks[i].point.z(), sys.tracks[i].point.w(), n, sys.tracks[n].point.x(), sys.tracks[n].point.y(), sys.tracks[n].point.z(), sys.tracks[n].point.w());
				//~ myfile3dPtsOut << BuffData;
			//~ }
			//~ else
			//~ {
				//~ ROS_WARN("Could not open myfile3dPtsOut");
			//~ }
			//~ ClearBuff(BuffData, Size);
			
			//~ geometry_msgs::Point32 temp;
			//~ temp.x = sys.tracks[i].point.x();
			//~ temp.y = sys.tracks[i].point.y();
			//~ temp.z = sys.tracks[i].point.z();
			//~ OutputMsg.Points3D.push_back(temp);
		//~ }
		//~ sprintf(BuffData, "\n");
		//~ myfile3dPtsOut << BuffData;
		//~ ClearBuff(BuffData, Size);
		
		//~ for (int i = 0; i < 5; i++)
		//~ {
			//~ if (myfile2dPtsOut.good())
			//~ {
				//~ for (int j = 0; j < 2; j++)
				//~ {
					//~ sprintf(BuffData, "2d pts, %d, from node, %d, is valid, %d,  %f, %f, %f, ", i, j, sys.tracks[i].projections[j].isValid, 
					//~ sys.tracks[i].projections[j].kp[0], sys.tracks[i].projections[j].kp[1], sys.tracks[i].projections[j].kp[2]);
					//~ myfile2dPtsOut << BuffData;
				//~ }
				//~ ClearBuff(BuffData, Size);
			//~ }
			//~ else
			//~ {
				//~ ROS_WARN("Could not open myfile2dPtsOut");
			//~ }
			//~ // 		track(time), projections(camera view), kp(x or y)
			//~ geometry_msgs::Point32 temp1, temp2;
			//~ temp1.x = sys.tracks[i].projections[0].kp[0];
			//~ temp1.y = sys.tracks[i].projections[0].kp[1];
			//~ temp2.x = sys.tracks[i].projections[1].kp[0];
			//~ temp2.y = sys.tracks[i].projections[1].kp[1];
			//~ OutputMsg.X1.push_back(temp1);
			//~ OutputMsg.X2.push_back(temp2);
		//~ }
		//~ sprintf(BuffData, "\n");
		//~ myfile2dPtsOut << BuffData;
		//~ ClearBuff(BuffData, Size);
		
		//~ myfileNodesOut.close();
		//~ myfile3dPtsOut.close();
		//~ myfile2dPtsOut.close();
		
		// /////////////////////////////////////////////////////////////
		// clear variables for next callback
		//~ sys.tracks.clear();
		//~ sys.nodes.clear();
		
		//~ ros::Duration(10.0).sleep(); 							// for testing
	}

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sba_system_topics");
    
    ros::NodeHandle node;
    
	SbaTopics SbaT = SbaTopics(node);
	
	ros::spin();
	
	return 0;
    
}
