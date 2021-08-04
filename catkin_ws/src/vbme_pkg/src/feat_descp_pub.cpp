// ROS headers
#include "ros/ros.h"
//~ #include "std_msgs/String.h"
#include "std_msgs/Header.h" 									// to copy headers and time stamps
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h> 
#include "vbme_pkg/KeyPointList_msg.h"
#include "vbme_pkg/KeyPoint_msg.h"

// opencv headers
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp> 

// custom headers
#include <vbme_pkg/PackFeatDescptMatch.h>
;
//~ class FeaturePublisher
class FeatDescpPub
{
public: // public vars
	
private: // private vars 
	ros::NodeHandle * n;
	
	bool PrintToScreen; 											// loaded from rosparam, flag to print
	bool DrawImages; 												// loaded from rosparam, flag to display windows
	int DetectMethod; 												// loaded from rosparam, choose to use 1 sift, 2 surf, or 3 orb 
	bool LoopSlow; 													// loaded from rosparam, flag to use sleep in loops to view images
	float LoopSleep; 												// rosparam, time to sleep in a loop secs
	
	// feature point variables for sift 
	int SiftNFeatures;
	int SiftNOctaveLayers;
	float SiftContrastThreshold;
	int SiftNPyramidOctaves;
	float SiftSigma;
	
	// feature point variables for surf 
	int SurfHessianThreshold;
	int SurfNPyramidOctaves;
	int SurfNOctaveLayers;
	bool SurfExtendedDescriptor;
	bool SurfUprightFlag;
	
	// feature point variables for orb
	int OrbNFeatures;
	float OrbScaleFactor;
	int OrbNPyramidLevels;
	int OrbEdgeThreshold;
	int OrbFirstLevel;
	int OrbWTA_k;
	int OrbScoreType;
	int OrbPatchSize;
	int OrbFastThreshold;
	
	ros::Subscriber SubImage;
	ros::Publisher PubFeat;
	//~ int Seq; 														// sequence to count images in headers
	//~ sensor_msgs::Image FirstImage; 								// holds the first image recorded
	//~ sensor_msgs::Image OldImage; 								// holds the last or second to newest image
	//~ sensor_msgs::Image NewImage; 								// holds the newest image
	//~ bool HaveFirstImage; 										// flag for first image
	//~ std::vector<cv::KeyPoint> FirstKeypoints; 					// holds the first keypoints recorded
	//~ std::vector<cv::KeyPoint> OldKeypoints; 					// holds the last or second to newest keypoints
	//~ cv::Mat FirstDescriptors; 									// holds the first descps recorded
	//~ cv::Mat OldDescriptors; 									// holds the last or second to newest descps
	
	PackFeatDescpt PackFD; 											// converts feats and desctps between opencv and ros msgs
	
public: // public methods
	FeatDescpPub(ros::NodeHandle nh, std::string Name) 				// constructor
	{
		n = &nh;
		//~ HaveFirstImage = false;
		//~ Seq = 0;
		
		SubImage = nh.subscribe("FeatPubImages", 5, &FeatDescpPub::CallbackImages, this);
		PubFeat = nh.advertise<vbme_pkg::KeyPointList_msg>("Keypoints", 50);
		
		GetParams(Name);
		//~ ROS_INFO("constructed");
	}
	
private: // private methods
	
	void GetParams(std::string Name)
	{
		n->param<bool>("PrintToScreen", PrintToScreen, true);
		n->param<bool>("DrawImages", DrawImages, true);
		n->param<int>("DetectMethod", DetectMethod, 3); 		// 1 sift, 2 surf, or 3 orb 
		n->param<bool>("LoopSlow", LoopSlow, true);
		n->param<float>("LoopSleep", LoopSleep, 1.0);
		
		// feature point variables for sift 
		n->param<int>("Sift/NFeatures", SiftNFeatures, 0); 			// nfeatures
		n->param<int>("Sift/NOctaveLayers", SiftNOctaveLayers, 3); 	// nOctaveLayers  
		n->param<float>("Sift/ContrastThreshold", SiftContrastThreshold, 0.04); // contrastThreshold  
		n->param<int>("Sift/NPyramidOctaves", SiftNPyramidOctaves, 10); // number of pyramid octaves
		n->param<float>("Sift/Sigma", SiftSigma,  1.6);    			// sigma
		
		// feature point variables for surf 
		n->param<int>("Surf/HessianThreshold", SurfHessianThreshold, 1600); // hessianThreshold 
		n->param<int>("Surf/NPyramidOctaves", SurfNPyramidOctaves, 3); // number of pyramid octaves
		n->param<int>("Surf/NOctaveLayers", SurfNOctaveLayers, 4); 	// number of octave layerss
		n->param<bool>("Surf/ExtendedDescriptor", SurfExtendedDescriptor, true); // extended descriptor
		n->param<bool>("Surf/UprightFlag", SurfUprightFlag, false); // upright flag
		
		// feature point variables for orb
		n->param<int>("Orb/NFeatures", OrbNFeatures, 500); 			// nfeatures
		n->param<float>("Orb/ScaleFactor", OrbScaleFactor, 1.2); 	// scaleFactor
		n->param<int>("Orb/NPyramidLevels", OrbNPyramidLevels, 8); 	// nlevels pyramid
		n->param<int>("Orb/EdgeThreshold", OrbEdgeThreshold, 31); 	// edgeThreshold
		n->param<int>("Orb/FirstLevel", OrbFirstLevel, 0); 			// firstLevel
		n->param<int>("Orb/WTA_k", OrbWTA_k, 2); 					// WTA_k 
		n->param<int>("Orb/ScoreType", OrbScoreType, 1); 			// HARRIS_SCORE or FAST_SCORE 
		n->param<int>("Orb/PatchSize", OrbPatchSize, 31); 			// patchSize
		n->param<int>("Orb/FastThreshold", OrbFastThreshold, 20); 	// fastThreshold 
		
	}
	
	void CallbackImages(const sensor_msgs::Image::ConstPtr& msg)
	{
		cv::Mat ImgNew = cv_bridge::toCvCopy(msg,"bgr8")->image; 	// convert from ROS image to cv image
		//~ cv::Mat ImgNew = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8); 			
		cv::Mat ImgNewGray;
		cvtColor(ImgNew, ImgNewGray, CV_BGR2GRAY); 					// convert to gray
		
		std::vector<cv::KeyPoint> keypoints_raw;
		cv::Mat descriptors;
		switch (DetectMethod) 										// 1 sift, 2 surf, or 3 orb 
		{
			case 1:
			{
				GetSiftPoints(ImgNewGray, keypoints_raw, descriptors, msg->header);
				break;
			}
			case 2:
			{
				GetSurfPoints(ImgNewGray, keypoints_raw, descriptors, msg->header);
				break;
			}
			case 3:
			{
				GetOrbPoints(ImgNewGray, keypoints_raw, descriptors, msg->header);
				break;
			}
			default:
			{
				GetOrbPoints(ImgNewGray, keypoints_raw, descriptors, msg->header);
			}
		}
		
		
	}
	
	void GetSiftPoints(const cv::Mat& img_input,
		std::vector<cv::KeyPoint>& keypoints_raw, 
		cv::Mat& descriptors, std_msgs::Header Header)
	{
		// get feature keypoints
		cv::Ptr<cv::Feature2D> detector = cv::xfeatures2d::SIFT::create
		(
			0, 		// nfeatures  
			3,		// nOctaveLayers  
			.04,	// contrastThreshold  
			10,		// edgeThreshold  
			1.6 	// sigma 
		); 
		
		detector->detect(img_input, keypoints_raw);  			// find the feature keypoints
		if (PrintToScreen)
		{
			ROS_INFO("SIFT points: %zd", keypoints_raw.size());
		}
		
		// show feature points in each image ---------- 
		if (DrawImages)
		{
			cv::Mat image_show; 
			drawKeypoints(img_input,
						keypoints_raw,
						image_show,
						//~ cv::Scalar::all(-1),	// random color
						cv::Scalar(20, 200, 20),	// green
						4							// draw rich points 
						); 
			namedWindow("keypoints", cv::WINDOW_NORMAL); 
			cvResizeWindow("keypoints", 1900, 600);
			imshow("keypoints",image_show); 
			if (LoopSlow)
			{
				cv::waitKey(LoopSleep * 1000.0);
			}
		}
		
		// get descriptors 
		detector->compute(img_input, keypoints_raw, descriptors);
		
		vbme_pkg::KeyPointList_msg Msg;
		//~ Msg.Header.seq = ++Seq;
		Msg.Header = Header;
		Msg.KeyType = "sift";
		PackFD.FeatDescptCv2Msg(keypoints_raw, descriptors, "sift", Msg);
		PubFeat.publish(Msg);
		
		// TEST DELETE ME v-v
		//~ std::vector<cv::KeyPoint> temp1;
		//~ cv::Mat temp2;
		//~ PackFD.FeatDescptMsg2Cv(Msg, temp1, temp2);
		//~ if (DrawImages)
		//~ {
			//~ cv::Mat image_show; 
			//~ drawKeypoints(img_input,
						//~ temp1,
						//~ image_show,
						// cv::Scalar::all(-1),
						//~ cv::Scalar(20, 200, 20),	// green
						//~ 4	// draw rich points 
						//~ ); 
			//~ namedWindow("keypoints2", cv::WINDOW_NORMAL); 
			//~ imshow("keypoints2",image_show); 
			
			//~ ROS_INFO("Descriptor image 1 ");
			//~ namedWindow("Descriptor image 1", cv::WINDOW_NORMAL); 
			//~ imshow("Descriptor image 1",descriptors); 
			
			//~ ROS_INFO("Descriptor image 2 ");
			//~ namedWindow("Descriptor image 2", cv::WINDOW_NORMAL); 
			//~ imshow("Descriptor image 2",temp2); 
			
			//~ if (LoopSlow)
			//~ {
				//~ cv::waitKey(1000);
			//~ }
		//~ }
		// TEST DELETE ME ^-^
		
	} // end GetSiftPoints
	
	void GetSurfPoints(const cv::Mat& img_input,
		std::vector<cv::KeyPoint>& keypoints_raw, 
		cv::Mat& descriptors, std_msgs::Header Header)
	{
		// // use SURF features ---------- 
		cv::Ptr<cv::Feature2D> detector = cv::xfeatures2d::SURF::create
		(
			1600, 	// hessianThreshold 
			3,		// number of pyramid octaves 
			4,		// number of octave layers 
			true,	// extended descriptor 
			false 	// upright flag 
		); 
		
		detector->detect(img_input, keypoints_raw);  			// find the feature keypoints
		if (PrintToScreen)
		{
			ROS_INFO("SURF points: %zd", keypoints_raw.size());
		}
		
		// show feature points in each image ---------- 
		if (DrawImages)
		{
			cv::Mat image_show; 
			drawKeypoints(img_input,
						keypoints_raw,
						image_show,
						//~ cv::Scalar::all(-1),	// random color
						cv::Scalar(20, 200, 20),	// green
						4							// draw rich points 
						); 
			namedWindow("keypoints", cv::WINDOW_NORMAL); 
			cvResizeWindow("keypoints", 1900, 600);
			imshow("keypoints",image_show); 
			if (LoopSlow)
			{
				cv::waitKey(1000);
			}
		}
		
		// get descriptors 
		detector->compute(img_input, keypoints_raw, descriptors);
		
		vbme_pkg::KeyPointList_msg Msg;
		//~ Msg.Header.seq = ++Seq;
		Msg.Header = Header;
		Msg.KeyType = "surf";
		PackFD.FeatDescptCv2Msg(keypoints_raw, descriptors, "surf", Msg);
		PubFeat.publish(Msg);
		
	} // end GetSurfPoints
	
	void GetOrbPoints(const cv::Mat& img_input,
		std::vector<cv::KeyPoint>& keypoints_raw, 
		cv::Mat& descriptors, std_msgs::Header Header)
	{
	// // use ORB features ---------- 
	cv::Ptr<cv::Feature2D> detector = cv::ORB::create(
		500, 				// nfeatures 
		1.2, 				// scaleFactor 
		8, 					// nlevels 
		31, 				// edgeThreshold
		0, 					// firstLevel 
		2, 					// WTA_k 
		1,				 	// HARRIS_SCORE or FAST_SCORE 
		31, 				// patchSize 
		20	 				// fastThreshold 
		); 
		//~ cv::Ptr<cv::ORB> ptrORB = cv::ORB::create(
		//~ 75, // total number of keypoints
		//~ 1.2, // scale factor between layers
		//~ 8);  // number of layers in pyramid
		
		detector->detect(img_input, keypoints_raw);  			// find the feature keypoints
		if (PrintToScreen)
		{
			ROS_INFO("ORB points: %zd", keypoints_raw.size());
		}
		
		// show feature points in each image ---------- 
		if (DrawImages)
		{
			cv::Mat image_show; 
			drawKeypoints(img_input,
						keypoints_raw,
						image_show,
						//~ cv::Scalar::all(-1),	// random color
						cv::Scalar(20, 200, 20),	// green
						4							// draw rich points 
						); 
			namedWindow("keypoints", cv::WINDOW_NORMAL); 
			cvResizeWindow("keypoints", 1900, 600);
			imshow("keypoints",image_show); 
			if (LoopSlow) 
			{
				cv::waitKey(1000);
			}
		}
		
		// get descriptors 
		detector->compute(img_input, keypoints_raw, descriptors);
		
		vbme_pkg::KeyPointList_msg Msg;
		//~ Msg.Header.seq = ++Seq;
		Msg.Header = Header;
		Msg.KeyType = "orb";
		PackFD.FeatDescptCv2Msg(keypoints_raw, descriptors, "orb", Msg);
		PubFeat.publish(Msg);
		
	} // end GetOrbPoints
	
	//~ void GetGoodPoints()
	//~ {
	//~ } 
	
	
};

int main(int argc, char **argv)
{
	std::string Name = "FeaturePublisher";
	ros::init(argc, argv, Name);
	ros::NodeHandle nh;
	
	FeatDescpPub FP = FeatDescpPub(nh, Name);
	
	ros::spin();
	
	return 0;
}
