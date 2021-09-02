#include <ros/ros.h>

#include <utari_sba_pkg/sba.h>
#include <utari_sba_pkg/sba_file_io.h>
#include <utari_sba_pkg/visualization.h>

#include <visualization_msgs/Marker.h>

using namespace sba;
using namespace std;

void processSBAfile(char* filename, ros::NodeHandle node)
{
    // Create publisher topics.
    ros::Publisher cam_marker_pub = node.advertise<visualization_msgs::Marker>("/sba/cameras", 1);
    ros::Publisher point_marker_pub = node.advertise<visualization_msgs::Marker>("/sba/points", 1);
    ros::spinOnce();
    
    ROS_INFO("Sleeping for 2 seconds to publish topics...");
    ros::Duration(2.0).sleep();
    
    // Create an empty SBA system.
    SysSBA sys;
    
    // Read in information from the bundler file.
    readBundlerFile(filename, sys);
    
    // Provide some information about the data read in.
    ROS_INFO("Cameras (nodes): %d, Points: %d",
        (int)sys.nodes.size(), (int)sys.tracks.size());
        
    // Publish markers
    drawGraph(sys, cam_marker_pub, point_marker_pub);
    ros::spinOnce();
    
    ROS_INFO("Sleeping for 5 seconds to publish pre-SBA markers.");
    ros::Duration(5.0).sleep();
        
    // Perform SBA with 10 iterations, an initial lambda step-size of 1e-3, 
    // and using CSPARSE.
    sys.doSBA(10, 1e-3, 1);
    
    int npts = sys.tracks.size();

    ROS_INFO("Bad projs (> 10 pix): %d, Cost without: %f", 
        (int)sys.countBad(10.0), sqrt(sys.calcCost(10.0)/npts));
    ROS_INFO("Bad projs (> 5 pix): %d, Cost without: %f", 
        (int)sys.countBad(5.0), sqrt(sys.calcCost(5.0)/npts));
    ROS_INFO("Bad projs (> 2 pix): %d, Cost without: %f", 
        (int)sys.countBad(2.0), sqrt(sys.calcCost(2.0)/npts));
    
    ROS_INFO("Cameras (nodes): %d, Points: %d",
        (int)sys.nodes.size(), (int)sys.tracks.size());
        
    // Publish markers
    drawGraph(sys, cam_marker_pub, point_marker_pub);
    ros::spinOnce();
    ROS_INFO("Sleeping for 5 seconds to publish post-SBA markers.");
    ros::Duration(5.0).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sba_from_file");
    
    ros::NodeHandle node;

    if (argc < 2)
    {
      ROS_ERROR("Arguments are:  <input filename>");
      return -1;
    }
    char* filename = argv[1];
    
    processSBAfile(filename, node);
    ros::spinOnce();

    return 0;
}
