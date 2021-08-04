#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>

#include "vbme_pkg/KeyPointList_msg.h"
#include "vbme_pkg/KeyPoint_msg.h"
#include "vbme_pkg/MatchList_msg.h"
#include "vbme_pkg/Match_msg.h"

#include <opencv2/features2d.hpp>
//~ #include <opencv2/imgproc.hpp>
//~ #include <opencv2/xfeatures2d.hpp> 

class PackFeatDescpt
{
public: // public vars
	//~ PackFeatDescpt () 	// constructor 
	//~ {
		
	//~ }
	
private: // private vars 
	
	
public: // public methods
	
	int FeatDescptCv2Msg (const std::vector<cv::KeyPoint> & FeatCv, const cv::Mat Descrpt, const std::string FeatType, vbme_pkg::KeyPointList_msg & FeatMsg)
	{
		//~ printf("CV Size: %zd\n", FeatCv.size());
		//~ printf("CV Descrpt Original: \n" );
		//~ auto it1 = Descrpt.begin<double>();
		//~ for(;it1!=Descrpt.begin<double>()+200; it1++){
			//~ std::cout << *it1 << " ";
		//~ }
		//~ std::cout << std::endl<< std::endl;
		FeatMsg.KeyPointList.clear();
		for (auto it = FeatCv.begin(); it < FeatCv.end(); it++)
		{
			vbme_pkg::KeyPoint_msg temp;
			temp.X 			=  it->pt.x;
			temp.Y 			=  it->pt.y;
			temp.Size 		=  it->size;
			temp.Angle 		=  it->angle;
			temp.Response 	=  it->response;
			temp.Octave 	=  it->octave;
			temp.ClassID 	=  it->class_id;
			
			FeatMsg.KeyPointList.push_back(temp);
		}
		//~ if(Descrpt.type()!=CV_32F)  							// use only 32 float type/encoding
		//~ {
			//~ try
			//~ {
				//~ ROS_WARN("Descrpt.type() : %d", Descrpt.type());
				//~ ROS_WARN("Descriptors are not CV_32F, converting...");
				//~ Descrpt.convertTo(Descrpt, CV_32F);
			//~ }
			//~ catch (cv_bridge::Exception& e)
			//~ {
				//~ ROS_ERROR("In file PackFeatDescpt.h, method int int FeatDescptCv2Msg(), cv_bridge exception: %s", e.what());
				//~ return 1;
			//~ }
		//~ }
		try
		{
			//~ auto cv_bridge = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Descrpt);
			if (FeatType.compare("orb") == 0)
			{
				//~ auto cv_bridge = cv_bridge::CvImage(std_msgs::Header(), "MONO8", Descrpt);
				auto cv_bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, Descrpt);
				cv_bridge.toImageMsg(FeatMsg.Descriptors);
			}
			else
			{
				auto cv_bridge = cv_bridge::CvImage(std_msgs::Header(), "32FC1", Descrpt);
				cv_bridge.toImageMsg(FeatMsg.Descriptors);
			}
			
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("In file PackFeatDescpt.h, method int int FeatDescptCv2Msg(), cv_bridge exception: %s", e.what());
			return 1;
		}
		
		//~ ROS_INFO("CV Desc: Size %zd, rows %d, cols %d. Msg Desc: Size %zd, rows %d, cols %d.", Descrpt.size, Descrpt.rows, Descrpt.cols, FeatMsg.Descriptors.height, FeatMsg.Descriptors.width);
		//~ ROS_INFO("CV Desc:, rows %d, cols %d, channels %d, type %d. Msg Desc: rows %d, cols %d.", Descrpt.rows, Descrpt.cols, Descrpt.channels(), Descrpt.type(), FeatMsg.Descriptors.height, FeatMsg.Descriptors.width);
		//~ cv::Size sz = Descrpt.size();
		//~ std::cout<<sz<<std::endl;
		
		// print first 20 elements
		//~ printf("ROS Descrpt Packed: \n" );
		//~ auto it = FeatMsg.Descriptors.data.begin<double>();
		//~ auto it = FeatMsg.Descriptors.data.begin();
		//~ for(;it!=FeatMsg.Descriptors.data.begin()+200; it++){
			//~ std::cout << *it << " ";
		//~ }
		//~ std::cout << std::endl<< std::endl;
		
		return 0;
	}
	
	int FeatDescptMsg2Cv (const vbme_pkg::KeyPointList_msg FeatMsg, std::vector<cv::KeyPoint> & FeatCv, cv::Mat & Descrpt)
	{
		//~ printf("ROS Size: %zd \n", FeatMsg.KeyPointList.size());
		//~ printf("ROS Descrpt Original: \n" );
		//~ auto it1 = FeatMsg.Descriptors.data.begin<double>();
		//~ auto it1 = FeatMsg.Descriptors.data.begin();
		//~ for(;it1!=FeatMsg.Descriptors.data.begin()+200; it1++){
			//~ std::cout << *it1 << " ";
		//~ }
		//~ std::cout << std::endl<< std::endl;
		FeatCv.clear();
		for (auto it = FeatMsg.KeyPointList.begin(); it < FeatMsg.KeyPointList.end(); it++)
		{
			cv::KeyPoint temp;
			temp.pt.x 		=  it->X ;
			temp.pt.y 		=  it->Y;
			temp.size 		=  it->Size;
			temp.angle 		=  it->Angle;
			temp.response 	=  it->Response;
			temp.octave 	=  it->Octave;
			temp.class_id 	=  it->ClassID;
			
			FeatCv.push_back(temp);
		}
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			if (FeatMsg.KeyType.compare("orb") == 0)
			{
				cv_ptr = cv_bridge::toCvCopy(FeatMsg.Descriptors, sensor_msgs::image_encodings::TYPE_8UC1);
				Descrpt = cv_ptr->image;
			}
			else
			{
				// cv bridge varible = input image (sensor msgs image), and type
				//~ cv_ptr = cv_bridge::toCvCopy(FeatMsg.Descriptors, sensor_msgs::image_encodings::BGR8);
				cv_ptr = cv_bridge::toCvCopy(FeatMsg.Descriptors, sensor_msgs::image_encodings::TYPE_32FC1);
				Descrpt = cv_ptr->image;
			}
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("In file PackFeatDescpt.h, method int FeatDescptMsg2Cv(), cv_bridge exception: %s", e.what());
			ROS_INFO("ROS sensor_msgs/Image encoding used: %s", FeatMsg.Descriptors.encoding.c_str());
			//~ ROS_ERROR("In file PackFeatDescpt.h, method int FeatDescptMsg2Cv(), cv_bridge exception: %s", e.what());
			return 1;
		}
		
		//~ ROS_INFO("Msg Desc: rows %d, cols %d. CV Desc: rows %d, cols %d, channels %d, type %d. ", FeatMsg.Descriptors.height, FeatMsg.Descriptors.width, Descrpt.rows, Descrpt.cols, Descrpt.channels(), Descrpt.type());
		//~ cv::Size sz = Descrpt.size();
		//~ std::cout<< Descrpt.size() <<std::endl;
		
		//~ // print first 20 elements
		//~ printf("CV Descrpt Unpacked: \n" );
		//~ auto it = Descrpt.begin<double>();
		//~ for(;it!=Descrpt.begin<double>()+200; it++){
			//~ std::cout << *it << " ";
		//~ }
		//~ std::cout << std::endl<< std::endl;
		
		return 0;
	}
private: // private methods	

};




//~ class PackMatch
//~ {
//~ public: // public vars
	
//~ private: // private vars 
	
	
//~ public: // public methods
	
	//~ int MatchCv2Msg (const std::vector<cv::DMatch> CvMatches, vbme_pkg::MatchList_msg & RosMsg)
	//~ {
		//~ RosMsg.MatchList.clear();
		//~ for (auto it = CvMatches.begin(); it < CvMatches.end(); it++)
		//~ {
			//~ vbme_pkg::Match_msg temp;
			//~ temp.QueryIdx 		=  it->queryIdx;
			//~ temp.TrainIdx 		=  it->	trainIdx;
			//~ temp.ImgIdx 		=  it->imgIdx;
			//~ temp.Distance 		=  it->distance;
			
			//~ RosMsg.MatchList.push_back(temp);
		//~ }
		
		//~ return 0;
	//~ }
	
	//~ int MatchMsg2Cv (const vbme_pkg::MatchList_msg RosMsg, std::vector<cv::DMatch> & CvMatches)
	//~ {
		//~ CvMatches.clear();
		//~ for (auto it = RosMsg.MatchList.begin(); it < RosMsg.MatchList.end(); it++)
		//~ {
			//~ cv::DMatch temp;
			//~ temp.queryIdx 		=  it->QueryIdx;
			//~ temp.trainIdx 		=  it->TrainIdx;
			//~ temp.imgIdx 		=  it->ImgIdx;
			//~ temp.distance 		=  it->Distance;
			
			//~ CvMatches.push_back(temp);
		//~ }
		//~ return 0;
	//~ }
//~ };
	

