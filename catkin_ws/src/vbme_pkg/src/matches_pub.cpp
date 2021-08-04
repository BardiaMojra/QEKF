#include "ros/ros.h"
//~ #include "std_msgs/String.h"
#include "vbme_pkg/KeyPointList_msg.h"
#include "vbme_pkg/KeyPoint_msg.h"
#include "vbme_pkg/MatchList_msg.h"
#include "vbme_pkg/Match_msg.h"

#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
//~ #include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp> 

// custom headers
#include <vbme_pkg/PackFeatDescptMatch.h>

class QuEstMatcher
{
public: // public vars
	
private: // private varsS
	ros::NodeHandle * n;
	bool PrintToScreen; 											// loaded from rosparam, flag to print
	bool DrawImages; 												// loaded from rosparam, flag to display windows
	bool LoopSlow; 													// loaded from rosparam, flag to use sleep in loops to view images
	float LoopSleep; 												// rosparam, time to sleep in a loop secs
	int MatchesKept; 												// rosparam, number of best matches to keep
	int MatchMethod; 												// rosparam, methods : 1 BFMatcher, 2 DescriptorMatcher, or 3 FlannBasedMatcher
	bool MatchSeparationCheck; 										// To test if match points are on top of each other
	double MatchSeparation; 										// euclidean distance match points must be away from each other
	ros::Subscriber SubKeypoints;
	ros::Publisher PubMatch;
	
	bool HaveFirstMsg; 												// flag for first msg (feat descrpt)
	int Count;
	//~ vbme_pkg::KeyPointList_msg 	FirstFeatDescpt;				// holds first set of keypoints and descriptors (ros form)
	std::vector<cv::KeyPoint> KeypointsFirstCv; 					// holds the first keypoints recorded, all matches are done against the first, in cv format
	vbme_pkg::KeyPointList_msg KeypointsFirstMsg; 					// holds the first keypoints recorded, all matches are done against the first, in ros msg format
	cv::Mat DescriptorsFirst; 										// holds the first descps recorded
	
	PackFeatDescpt PackFD; 											// converts features and descriptors between opencv and ros msgs
	//~ PackMatch PackMat; 											// converts matches between opencv and ros msgs
	
	// debugging vars
	ros::Subscriber SubImages; 										// only activated if we draw images, unused otherwise
	cv::Mat ImageFirst;
	cv::Mat ImageCurr;
	bool HaveFirstImage; 								 			// flag for first msg (image)
	
public: // public methods
	QuEstMatcher(ros::NodeHandle nh) // constructor
	{
		n = &nh;
		GetParams();
		SubKeypoints = nh.subscribe("Keypoints", 10, &QuEstMatcher::KeypointsCallback, this);
		if (DrawImages)
		{
			// we only need to get images if we want to draw, otherwise it is a waste. 
			SubImages = nh.subscribe("FeatPubImages", 5, &QuEstMatcher::CallbackImages, this);
		}
		PubMatch = nh.advertise<vbme_pkg::MatchList_msg>("Matches", 10);
		
		HaveFirstMsg = false;
		HaveFirstImage = false;
		Count = 0;
		
	}
	
private: // private methods
	void GetParams()
	{
		n->param<bool>("PrintToScreen", PrintToScreen, true);
		n->param<bool>("DrawImages", DrawImages, true);
		n->param<bool>("LoopSlow", LoopSlow, true);
		n->param<float>("LoopSleep", LoopSleep, 1.0);
		n->param<int>("MatchesKept", MatchesKept, 10);
		n->param<int>("MatchMethod", MatchMethod, 3); 				// methods : 1 BFMatcher, 2 DescriptorMatcher, or 3 FlannBasedMatcher
		n->param<bool>("MatchSeparationCheck", MatchSeparationCheck, true); // To test if match points are on top of each other
		n->param<double>("MatchSeparation", MatchSeparation, 2.0); 	// euclidean distance match points must be away from each other
	}
	
	void CallbackImages(const sensor_msgs::Image::ConstPtr& msg)
	{
		// Save first image to draw on
		if (!HaveFirstImage)
		{
			// convert image to opencv, save to first image
			ImageFirst = cv_bridge::toCvCopy(msg,"bgr8")->image; 	// convert from ROS image to cv image
			HaveFirstImage = true;
		}
		else
		{
			ImageCurr = cv_bridge::toCvCopy(msg,"bgr8")->image; 	// convert from ROS image to cv image
		}
	}
	
	void KeypointsCallback(const vbme_pkg::KeyPointList_msg::ConstPtr& msg)
	{
		// we need 2 msgs (features and descriptors) to do matching. 
		// on first msg we will save the first set and save them and wait
		// for the second set. 
		if (!HaveFirstMsg) 											
		{
			// convert from ros to opencv
			if ( PackFD.FeatDescptMsg2Cv(*msg, KeypointsFirstCv, DescriptorsFirst) )
			{
				// return of 1 is failure
				ROS_ERROR("Failed to convert ROS msg to opencv");
				return;
			}
			if (DescriptorsFirst.empty())
			{
				ROS_WARN("Descriptors image is empty");
				return;
			}
			// save keypoint list in ros msg form too, to repackage later
			KeypointsFirstMsg.KeyPointList = msg->KeyPointList;
			// put into match output variable
			//~ MatchMsg.KeyPtList1 = KeypointsFirstMsg.KeyPointList;
			HaveFirstMsg = true;
			return;
		}
		
		// convert from ros to opencv
		std::vector<cv::KeyPoint> KeypointsCurr; 
		cv::Mat DescriptorsCurr;
		PackFD.FeatDescptMsg2Cv(*msg, KeypointsCurr, DescriptorsCurr);
		
		// make matcher object and calculate matches
		std::vector<cv::DMatch> matches_raw;
		
		switch (MatchMethod)
		{
		case 1: 												// methods : 1 BFMatcher
			if ( (msg->KeyType.compare("sift") == 0) or (msg->KeyType.compare("surf") == 0))
			{
				cv::BFMatcher matcher(cv::NORM_L2, true); 		// true for cross match 
				matcher.match(DescriptorsFirst, DescriptorsCurr, matches_raw);
			}
			else if (msg->KeyType.compare("orb") == 0)
			{
				cv::BFMatcher matcher(cv::NORM_HAMMING, true);
				matcher.match(DescriptorsFirst, DescriptorsCurr, matches_raw);
			}
			else
			{
				cv::BFMatcher matcher(cv::NORM_L2, true);
				matcher.match(DescriptorsFirst, DescriptorsCurr, matches_raw);
			}
			break;
		case 2: 												// 2 DescriptorMatcher
			ROS_WARN("DescriptorMatcher not implemeted... using FlannBasedMatcher!");
			//~ break;
		default: 												// 3 FlannBasedMatcher
			if (msg->KeyType.compare("orb") == 0)
			{
				printf("Test 6 \n");
				//~ auto Params = cv::flann::LshIndexParams(20, 10, 2);
				//~ cv::FlannBasedMatcher matcher(&Params);
				cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20, 10, 2)); // if this isnt called with new a double free runtime error occurs?
				matcher.match(DescriptorsFirst, DescriptorsCurr, matches_raw); 

			}
			else
			{
				cv::FlannBasedMatcher matcher; 
				matcher.match(DescriptorsFirst, DescriptorsCurr, matches_raw); 
			}
			 
		}
		
		
		
		std::sort(matches_raw.begin(), matches_raw.end(), ComparatorDistance);
		
		// sort distance, select 20 best ones ---------- 
		//~ std::vector<double> distance; 
		//~ for(auto it=matches_raw.begin(); it<matches_raw.end();++it){
			//~ // this will be sorted by distance. If the images are the same, the distances will be zero... there will be no difference and this will break...
			//~ if ( !(abs(it->distance) < 0.00001) )
			//~ {
				//~ distance.push_back(it->distance); 
			//~ }
			//~ // ROS_INFO("Distance: %f, imgIdx: %d, queryIdx %d, trainIdx: %d", it->distance, it->imgIdx, it->queryIdx, it->trainIdx);
		//~ } 
		//~ if (distance.size() == 0)
		//~ {
			//~ ROS_WARN("All matches have distance zero, dropping message.");
			//~ return;
		//~ }
		//~ sort(distance.begin(), distance.end()); 
		
		// remove points that are too close
		if (MatchSeparationCheck)
		{
			//~ printf("Matches: %zd \n", matches_raw.size());
			unsigned int Passed = 0; 								// num of matches that have passed test
			auto it = matches_raw.begin(); 							// current test match
			while (Passed <= MatchesKept)
			{
				// we will check the next points. We only need to check up until
				// MatchesKept in total. From the current point we will check
				// matches kept minux Passed so in total we will have
				// matches kept
				for (unsigned int i = 1; i < MatchesKept-Passed; i++)
				{
					// calculate euclidean distance between query points, and train points
					// first match, first (query) image
					double X1q = KeypointsFirstCv[it->queryIdx].pt.x;
					double Y1q = KeypointsFirstCv[it->queryIdx].pt.y;
					
					// i match, first (query) image
					double X1t = KeypointsFirstCv[(it+i)->queryIdx].pt.x;
					double Y1t = KeypointsFirstCv[(it+i)->queryIdx].pt.y;
					
					// first match, second (train) image
					double X2q = KeypointsCurr[it->trainIdx].pt.x;
					double Y2q = KeypointsCurr[it->trainIdx].pt.y;
					
					// i match, second (train) image
					double X2t = KeypointsCurr[(it+i)->queryIdx].pt.x;
					double Y2t = KeypointsCurr[(it+i)->queryIdx].pt.y;
					
					double Dis1 = pow( pow(X1q-X2q, 2.0) + pow(Y1q-Y2q, 2.0), 0.5);
					//~ printf("Match Euclid distance 1: %f\n", Dis1);
					double Dis2 = pow( pow(X1t-X2t, 2.0) + pow(Y1t-Y2t, 2.0), 0.5);
					//~ printf("Match Euclid distance 2: %f\n", Dis2);
					if ((Dis1 < MatchSeparation) or (Dis2 < MatchSeparation))
					{
						printf("Matches are too close, deleting second point\n");
						matches_raw.erase(it+i);
					}
					//~ printf("Passed: %d, i: %d\n", Passed, i);
				}
				Passed++;
				//~ printf("Passed: %d\n", Passed)
			}
		}
		
		
		
		
		// only choose the best matches, the number of kept pulled from param server
		std::vector<cv::DMatch> matches_select(matches_raw.begin(), matches_raw.begin()+MatchesKept);
		
		//~ std::vector<cv::DMatch> matches_select;
		//~ for(int i=0;i<matches_raw.size();i++){
			//~ if(matches_raw[i].distance == distance[0]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[1]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[2]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[3]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[4]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[5]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[6]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[7]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[8]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[9]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[10]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[11]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[12]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[13]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[14]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[15]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[16]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[17]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[18]) matches_select.push_back(matches_raw[i]); 
			//~ if(matches_raw[i].distance == distance[19]) matches_select.push_back(matches_raw[i]); 
		//~ } 
		
		//~ printf("raw size: %zd, select size: %zd \n", matches_raw.size(), matches_select.size());
		//~ for (auto it = matches_raw.begin(); it < matches_raw.begin()+10; it++)
		//~ {
			//~ printf("%f ", it->distance);
		//~ }
		//~ printf("\n");
		//~ for (auto it = matches_select.begin(); it < matches_select.begin()+10; it++)
		//~ {
			//~ printf("%f ", it->distance);
		//~ }
		//~ printf("\n");
		//~ for (auto it = distance.begin(); it < distance.begin()+10; it++)
		//~ {
			//~ printf("%f ", *it);
		//~ }
		//~ printf("\n");
		
		 // create match msg to publish
		vbme_pkg::MatchList_msg MatchMsg; 							// output msg
		
		// now we will sort the keypoints so that KeyPtList1[0] is the match of KeyPtList2[0], KeyPtList1[1] to KeyPtList2[1], etc
		for(auto it=matches_select.begin(); it<matches_select.end(); ++it)
		{
			vbme_pkg::KeyPoint_msg tempM1;
			vbme_pkg::KeyPoint_msg tempM2;
			// i match from image 1
			tempM1.X = msg->KeyPointList[it->queryIdx].X;
			tempM1.Y = msg->KeyPointList[it->queryIdx].Y;
			// i match from image 2
			tempM2.X = msg->KeyPointList[it->trainIdx].X;
			tempM2.Y = msg->KeyPointList[it->trainIdx].Y;
			MatchMsg.KeyPtList1.push_back(tempM1);
			MatchMsg.KeyPtList2.push_back(tempM2);
		}
		
		MatchMsg.Header = msg->Header;
		
		// finally move the edited matches to msg. This is no longer needed. We are sorting the keypoints to match
		//~ PackMat.MatchCv2Msg(matches_select, MatchMsg);
		
		// for pack testing
		//~ PackMat.MatchMsg2Cv (MatchMsg, matches_select);
		
		PubMatch.publish(MatchMsg);
		
		//~ std::vector<cv::DMatch> matches_select_draw; 
		if (DrawImages) // only draw if specified 
		{
			ros::spinOnce(); 										// double check for new image
			// draw point matching, best 20 points, replaced with MatchesKept
			//~ int numPts = 20; 
			std::vector<cv::DMatch> matches_select_draw(matches_select.begin(), matches_select.begin()+MatchesKept);
			//~ for(int i=0;i<numPts;i++)
			//~ {
				//~ matches_select_draw.push_back(matches_select[i]); 
			//~ } 
			cv::Mat img_matches; 
			//~ printf("Image first: rows %d, cols %d, channels %d, type %d. \n", ImageFirst.rows, ImageFirst.cols, ImageFirst.channels(), ImageFirst.type());
			//~ printf("Image curr: rows %d, cols %d, channels %d, type %d. \n", ImageCurr.rows, ImageCurr.cols, ImageCurr.channels(), ImageCurr.type()); 
			cv::drawMatches(ImageFirst, KeypointsFirstCv,
						ImageCurr, KeypointsCurr,
						matches_select_draw,
						img_matches,
						//~ cv::Scalar::all(-1),cv::Scalar::all(-1),
						cv::Scalar(20, 225, 20),cv::Scalar(20, 225, 20),
						std::vector<char>(),
						cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
						); 
			cv::namedWindow("img_matches", cv::WINDOW_NORMAL); 
			cvResizeWindow("img_matches", 1900, 600);
			cv::imshow("img_matches",img_matches); ;
			if (LoopSlow)
			{
				cv::waitKey(LoopSleep * 1000.0); 
			}
			char FileName[60] = "\0";
			sprintf(FileName, "/tmp/MatchedImages/frame%04d.png", Count);
			cv::imwrite(FileName,img_matches);
		}
		
		if (PrintToScreen)
		{
			ROS_INFO("Matches all: %zd, select: %zd", matches_raw.size(), matches_select.size());
			ROS_INFO("Best match distance %f, worse kept match distance %f", matches_raw[0].distance, matches_raw[MatchesKept].distance);
		}
		
		Count++;
		
	} // end KeypointsCallback
	
	// simple comparator function to use std::sort on a vector<DMatch>
	static bool ComparatorDistance(cv::DMatch A, cv::DMatch B)
	{
			return A.distance < B.distance;
	}

};




int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener_class");
	ros::NodeHandle nh;
	
	QuEstMatcher QM = QuEstMatcher(nh);
	
	ros::spin();
	
	return 0;
	
}
