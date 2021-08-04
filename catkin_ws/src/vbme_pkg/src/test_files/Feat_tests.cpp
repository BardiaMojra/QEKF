#include <iostream>

//~ #include <opencv2/core/core.hpp>
//~ #include <opencv2/highgui/highgui.hpp>
//~ #include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "harrisDetector.h"

int GoodFeat(std::string ImageNameIn, std::vector<cv::KeyPoint> & KeyPtsOut, cv::Mat & descrtrOut)
{
	// Read input image
	cv::Mat image = cv::imread(ImageNameIn, 0);
	if (!image.data)
	{
		puts("Error: failed to read image");
		return 0; 
	}
	
	std::vector<cv::Point2f> p0;//, p1;
	cv::goodFeaturesToTrack(image, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);
	
	// Draw keypoints
	for(uint i = 0; i < p0.size(); i++)
    {
		cv::circle(image, p0[i], 4, 255, 1);
	}
	
	// Display the keypoints
    std::string WinName = "Good Features " + ImageNameIn;
	cv::namedWindow(WinName);
	cv::imshow(WinName,image);
	
	std::cout << "Number of Good keypoints: " << p0.size() << std::endl; 
	
	
	return 0;
}

int FindSurf(std::string ImageNameIn, std::vector<cv::KeyPoint> & KeyPtsOut, cv::Mat & descrtrOut)
{
	// SURF:
	
	// Read input image
	cv::Mat image = cv::imread(ImageNameIn, 0);
	if (!image.data)
	{
		puts("Error: failed to read image");
		return 0; 
	}

	//keypoints.clear();
	//~ std::vector<cv::KeyPoint> keypoints;
	
	// Construct the SURF feature detector object
	cv::Ptr<cv::xfeatures2d::SurfFeatureDetector> ptrSURF = cv::xfeatures2d::SurfFeatureDetector::create(2000.0);
	// detect the keypoints
	ptrSURF->detect(image, KeyPtsOut);
	
	// Detect the SURF features
	ptrSURF->detect(image,KeyPtsOut);
	
	cv::Mat featureImage;
	cv::drawKeypoints(image,KeyPtsOut,featureImage,cv::Scalar(255,255,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Display the keypoints
    std::string WinName = "SURF " + ImageNameIn;
	cv::namedWindow(WinName);
	cv::imshow(WinName,featureImage);

	std::cout << "Number of SURF keypoints: " << KeyPtsOut.size() << std::endl; 
	
	// Extract the descriptor
	ptrSURF->compute(image, KeyPtsOut, descrtrOut);
	//~ printf("1 Destripter 1 rows: %d \n", descrtrOut.rows);
	
	return 0;
}

int FindSift(std::string ImageNameIn, std::vector<cv::KeyPoint> & KeyPtsOut, cv::Mat & descrtrOut)
{
	// SIFT:
	
	// Read input image
	cv::Mat image= cv::imread(ImageNameIn, cv::IMREAD_GRAYSCALE);
	if (!image.data)
	{
		puts("Error: failed to read image");
		return 0; 
	}

	//keypoints.clear();
	//~ std::vector<cv::KeyPoint> keypoints;
	
	// Construct the SIFT feature detector object
	cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> ptrSIFT = cv::xfeatures2d::SiftFeatureDetector::create();
	// detect the keypoints
	ptrSIFT->detect(image, KeyPtsOut);

	// Detect the SIFT features
	ptrSIFT->detect(image,KeyPtsOut);
	cv::Mat featureImage;
	
	cv::drawKeypoints(image,KeyPtsOut,featureImage,cv::Scalar(255,255,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Display the keypoints
    std::string WinName = "SIFT " + ImageNameIn;
	cv::namedWindow(WinName);
	cv::imshow(WinName,featureImage);

	std::cout << "Number of SIFT keypoints: " << KeyPtsOut.size() << std::endl; 
	
	// Extract the descriptor
	ptrSIFT->compute(image, KeyPtsOut, descrtrOut);
	
	return 0;
}

int FindOrb(std::string ImageNameIn, std::vector<cv::KeyPoint> & KeyPtsOut, cv::Mat & descrtrOut)
{
	// ORB:

	// Read input image
	cv::Mat image= cv::imread(ImageNameIn,CV_LOAD_IMAGE_GRAYSCALE);
	if (!image.data)
	{
		puts("Error: failed to read image");
		return 0; 
	}
	//~ std::vector<cv::KeyPoint> keypoints;
	
	// Construct the BRISK feature detector object
	cv::Ptr<cv::ORB> ptrORB = cv::ORB::create(
		75, // total number of keypoints
		1.2, // scale factor between layers
		8);  // number of layers in pyramid
	// detect the keypoints
	ptrORB->detect(image, KeyPtsOut);
	cv::Mat featureImage;
	
	cv::drawKeypoints(image,KeyPtsOut,featureImage,cv::Scalar(255,255,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Display the keypoints
    std::string WinName = "ORB " + ImageNameIn;
	cv::namedWindow(WinName);
	cv::imshow(WinName,featureImage);

	std::cout << "Number of ORB keypoints: " << KeyPtsOut.size() << std::endl; 
	
	// Extract the descriptor
	ptrORB->compute(image, KeyPtsOut, descrtrOut);
	
	return 0;
}

int Matcher(std::string NameIn, cv::Mat Img1In, cv::Mat Img2In, std::vector<cv::KeyPoint> Keypts1In, std::vector<cv::KeyPoint> Keypts2In, cv::Mat Desctr1In, cv::Mat Desctr2In,  std::vector<cv::DMatch> MatchesOut)
{
	//~ printf("3 Keypits 1 size: %zd \n", Keypts1In.size());
	//~ printf("3 Keypits 2 size: %zd \n", Keypts2In.size());
	//~ printf("3 Destripter 1 rows: %d \n", Desctr1In.rows);
	//~ printf("3 Destripter 2 rows: %d \n", Desctr2In.rows);
	
	// Construction of the matcher 
	//~ cv::BFMatcher matcher(cv::NORM_L2);
	// to test with crosscheck (symmetry) test
	// note: must not be used in conjunction with ratio test
    cv::BFMatcher matcher(cv::NORM_L2, true); // with crosscheck
	// Match the two image descriptors
    //~ std::vector<cv::DMatch> matches;
    matcher.match(Desctr1In,Desctr2In, MatchesOut);

    // draw matches
    cv::Mat imageMatches;
    cv::drawMatches(
	   Img1In, Keypts1In, // 1st image and its keypoints
	   Img2In, Keypts2In, // 2nd image and its keypoints
	   MatchesOut,            // the matches
	   imageMatches,       // the image produced
	   cv::Scalar(255, 255, 255),  // color of lines
	   cv::Scalar(255, 255, 255),  // color of points
	   std::vector< char >(),      // masks if any 
	   cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Display the image of matches
    std::string WinName = NameIn + " Matches";
	//~ cv::namedWindow("SURF Matches");
	//~ cv::imshow("SURF Matches",imageMatches);
	cv::namedWindow(WinName);
	cv::imshow(WinName,imageMatches);

	std::cout << "Number of " << NameIn << " matches: " << MatchesOut.size() << std::endl; 
	
	return 0;
}

int main(int argc, char **argv)
{
	// First image
	//~ std::string ImageName1 = "m4_air_crop1.jpeg";
	std::string ImageName1 = "/home/codyl/Downloads/MMTS1_275_03DEC2014_Shot_3_-1173.png";
	//~ std::string ImageName1 = "/home/codyl/Downloads/lundberg-cody1.jpg";
	//~ cv::Mat image1= cv::imread(ImageName1,0);
	cv::Mat image1= cv::imread(ImageName1, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
	if (!image1.data)
	{
		puts("Error: failed to read image 1");
		return 0; 
	}
	
	// Variables
	std::vector<cv::KeyPoint> Img_1_GoodKeypts;
	std::vector<cv::KeyPoint> Img_1_SurfKeypts;
	std::vector<cv::KeyPoint> Img_1_SiftKeypts;
	std::vector<cv::KeyPoint> Img_1_OrbKeypts;
	cv::Mat Img_1_GoodDesctr;
	cv::Mat Img_1_SurfDesctr;
	cv::Mat Img_1_SiftDesctr;
	cv::Mat Img_1_OrbDesctr;
	
	//~ // Display the image
	cv::namedWindow("Original 1");
	cv::imshow("Original 1",image1);
	
	printf("First image %s..\n", ImageName1.c_str());
	
	GoodFeat(ImageName1, Img_1_GoodKeypts, Img_1_GoodDesctr);
	
	FindSurf(ImageName1, Img_1_SurfKeypts, Img_1_SurfDesctr);
	FindSift(ImageName1, Img_1_SiftKeypts, Img_1_SiftDesctr);
	FindOrb(ImageName1, Img_1_OrbKeypts, Img_1_OrbDesctr);
	
	// Second image
	//~ std::string ImageName2 = "m4_air_crop1.jpeg";
	//~ std::string ImageName2 = "m4_air_crop2.jpeg";
	//~ std::string ImageName2 = "m4_air_crop3.jpeg";
	//~ std::string ImageName2 = "m4_air_crop4.jpeg";
	//~ std::string ImageName2 = "m4_air_crop5.jpeg";
	//~ std::string ImageName2 = "m4_air_crop6.jpeg";
	std::string ImageName2 = "/home/codyl/Downloads/MMTS1_275_03DEC2014_Shot_3_-1177.png";
	cv::Mat image2= cv::imread(ImageName2,0);
	if (!image2.data)
	{
		puts("Error: failed to read image 2");
		return 0; 
	}
	
	// Variables
	std::vector<cv::KeyPoint> Img_2_GoodKeypts;
	std::vector<cv::KeyPoint> Img_2_SurfKeypts;
	std::vector<cv::KeyPoint> Img_2_SiftKeypts;
	std::vector<cv::KeyPoint> Img_2_OrbKeypts;
	cv::Mat Img_2_GoodDesctr;
	cv::Mat Img_2_SurfDesctr;
	cv::Mat Img_2_SiftDesctr;
	cv::Mat Img_2_OrbDesctr;
	
	//~ // Display the image
	cv::namedWindow("Original 2");
	cv::imshow("Original 2",image2);
	
	printf("Second image %s..\n", ImageName2.c_str());
	GoodFeat(ImageName2, Img_2_GoodKeypts, Img_2_GoodDesctr);
	FindSurf(ImageName2, Img_2_SurfKeypts, Img_2_SurfDesctr);
	FindSift(ImageName2, Img_2_SiftKeypts, Img_2_SiftDesctr);
	FindOrb(ImageName2, Img_2_OrbKeypts, Img_2_OrbDesctr);
	
	// matches
	std::vector<cv::DMatch> MatchesSurf;
	std::vector<cv::DMatch> MatchesSift;
	std::vector<cv::DMatch> MatchesOrb;
	
	Matcher("SURF", image1, image2, Img_1_SurfKeypts, Img_2_SurfKeypts, Img_1_SurfDesctr, Img_2_SurfDesctr, MatchesSurf);
	Matcher("SIFT", image1, image2, Img_1_SiftKeypts, Img_2_SiftKeypts, Img_1_SiftDesctr, Img_2_SiftDesctr, MatchesSift);
	Matcher("ORB", image1, image2, Img_1_OrbKeypts, Img_2_OrbKeypts, Img_1_OrbDesctr, Img_2_OrbDesctr, MatchesOrb);
	
	
	cv::waitKey();
	return 0;
	
}
