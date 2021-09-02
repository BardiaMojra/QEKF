#include <ros/ros.h>
#include <utari_sba_pkg/sba.h>
#include <utari_sba_pkg/visualization.h>


// For random seed.
#include <time.h>

#include <visualization_msgs/Marker.h>


// basic file operations
#include <iostream>
#include <fstream>

#include <time.h>


void ClearBuff(char * buff, int size)
{
	for (int i = 0; i < size; i++)
	{
		buff[i] = '\0';
	}
}

using namespace sba;
using namespace std;

void setupSBA(SysSBA &sys)
{
    // Create camera parameters.
    frame_common::CamParams cam_params;
    cam_params.fx = 430; // Focal length in x
    cam_params.fy = 430; // Focal length in y
    //~ cam_params.fx = 0; // Focal length in x
    //~ cam_params.fy = 0; // Focal length in y
    cam_params.cx = 320; // X position of principal point
    cam_params.cy = 240; // Y position of principal point
    cam_params.tx = 0;   // Baseline (no baseline since this is monocular)

    // Define dimensions of the image.
    int maxx = 640;
    int maxy = 480;

    // Create a plane containing a wall of points.
    int npts_x = 4; // Number of points on the plane in x
    int npts_y = 3;  // Number of points on the plane in y
    
    double plane_width = 5;     // Width of the plane on which points are positioned (x)
    double plane_height = 2.5;    // Height of the plane on which points are positioned (y)
    double plane_distance = 5; // Distance of the plane from the cameras (z)

    // Vector containing the true point positions.
    vector<Point, Eigen::aligned_allocator<Point> > points;

    for (int ix = 0; ix < npts_x ; ix++)
    {
      for (int iy = 0; iy < npts_y ; iy++)
      {
        // Create a point on the plane in a grid.
        points.push_back(Point(plane_width/npts_x*(ix+.5), -plane_height/npts_y*(iy+.5), plane_distance, 1.0));
      }
    }
    
    // Create nodes and add them to the system.
    unsigned int nnodes = 2; // Number of nodes.
    double path_length = 3; // Length of the path the nodes traverse.
    
    unsigned int i = 0, j = 0;
    
    for (i = 0; i < nnodes; i++)
    { 
      // Translate in the x direction over the node path.
      Vector4d trans(i/(nnodes-1.0)*path_length, 0, 0, 1);
            
      // Don't rotate.
      Quaterniond rot(1, 0, 0, 0);
      rot.normalize();
      
      // Add a new node to the system.
      sys.addNode(trans, rot, cam_params, false);
    }
        
    // Set the random seed.
    unsigned short seed = (unsigned short)time(NULL);
    seed48(&seed);
        
    // Add points into the system, and add noise.
    for (i = 0; i < points.size(); i++)
    {
      // Add up to .5 points of noise.
      Vector4d temppoint = points[i];
      temppoint.x() += drand48() - 0.5;
      temppoint.y() += drand48() - 0.5;
      temppoint.z() += drand48() - 0.5;
      sys.addPoint(temppoint);
    }
    
    Vector2d proj;
    
    // Project points into nodes.
    for (i = 0; i < points.size(); i++)
    {
      for (j = 0; j < sys.nodes.size(); j++)
      {
        // Project the point into the node's image coordinate system.
        sys.nodes[j].setProjection();
        sys.nodes[j].project2im(proj, points[i]);
        
        // If valid (within the range of the image size), add the monocular 
        // projection to SBA.
        if (proj.x() > 0 && proj.x() < maxx && proj.y() > 0 && proj.y() < maxy)
        {
			//~ std::cout << "proj " << i << ": " << proj << std::endl;
          sys.addMonoProj(j, i, proj);
        }
      }
    }
    
    // Add noise to node position.
    
    double transscale = 1.0;
    double rotscale = 0.2;
    
    // Don't actually add noise to the first node, since it's fixed.
    for (i = 1; i < sys.nodes.size(); i++)
    {
      Vector4d temptrans = sys.nodes[i].trans;
      Quaterniond tempqrot = sys.nodes[i].qrot;
      
      // Add error to both translation and rotation.
      temptrans.x() += transscale*(drand48() - 0.5);
      temptrans.y() += transscale*(drand48() - 0.5);
      temptrans.z() += transscale*(drand48() - 0.5);
      tempqrot.x() += rotscale*(drand48() - 0.5);
      tempqrot.y() += rotscale*(drand48() - 0.5);
      tempqrot.z() += rotscale*(drand48() - 0.5);
      tempqrot.normalize();
      
      sys.nodes[i].trans = temptrans;
      sys.nodes[i].qrot = tempqrot;
      
      // These methods should be called to update the node.
      sys.nodes[i].normRot();
      sys.nodes[i].setTransform();
      sys.nodes[i].setProjection();
      sys.nodes[i].setDr(true);
    }
        
}

void processSBA(ros::NodeHandle node)
{
    // Create publisher topics.
    ros::Publisher cam_marker_pub = node.advertise<visualization_msgs::Marker>("/sba/cameras", 1);
    ros::Publisher point_marker_pub = node.advertise<visualization_msgs::Marker>("/sba/points", 1);
    ros::spinOnce();
    
    ROS_INFO("Sleeping for 2 seconds to publish topics...");
    ros::Duration(2.0).sleep();
    
    // Create an empty SBA system.
    SysSBA sys;
    
    setupSBA(sys);
    
    // Provide some information about the data read in.
    ROS_INFO("Cameras (nodes): %d, Points: %d",
        (int)sys.nodes.size(), (int)sys.tracks.size());
	
	
	
	
	
	
	
	
	
	
	ofstream myfileNodes;
	ofstream myfile3dPts;
	ofstream myfile2dPts;
	myfileNodes.open ("/tmp/sba_topics_in_nodes.csv");
	myfile3dPts.open ("/tmp/sba_topics_in_3dpts.csv");
	myfile2dPts.open ("/tmp/sba_topics_in_2dpts.csv");
	int Size = 500;
	char buffdata[Size] = {'\0'};
	
	for (int i = 0; i < 2; i++)
	{
		//~ std::cout << i << " node: " << sys.nodes[i].qrot.w() << std::endl;
		//~ printf("node, %d, %f, %f, %f, %f, %f, %f, %f, \n", i, sys.nodes[i].trans.x(), sys.nodes[i].trans.y(), sys.nodes[i].trans.z(), sys.nodes[i].qrot.w(), sys.nodes[i].qrot.x(), sys.nodes[i].qrot.y(), sys.nodes[i].qrot.z());
		sprintf(buffdata, "node, %d, %f, %f, %f, %f, %f, %f, %f, \n", i, sys.nodes[i].trans.x(), sys.nodes[i].trans.y(), sys.nodes[i].trans.z(), sys.nodes[i].qrot.w(), sys.nodes[i].qrot.x(), sys.nodes[i].qrot.y(), sys.nodes[i].qrot.z());
		printf( "node, %d, %f, %f, %f, %f, %f, %f, %f, \n", i, sys.nodes[i].trans.x(), sys.nodes[i].trans.y(), sys.nodes[i].trans.z(), sys.nodes[i].qrot.w(), sys.nodes[i].qrot.x(), sys.nodes[i].qrot.y(), sys.nodes[i].qrot.z());
		myfileNodes << buffdata;
	}
	ClearBuff(buffdata, Size);
	for (int i = 0; i < 5; i++)
	{
		sprintf(buffdata, "3d pts, %d, %f, %f, %f, %f, \n", i, sys.tracks[i].point.x(), sys.tracks[i].point.y(), sys.tracks[i].point.z(), sys.tracks[i].point.w());
		myfile3dPts << buffdata;
	}
	ClearBuff(buffdata, Size);
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			sprintf(buffdata, "2d pts, %d, from node, %d, is valid, %d,  %f, %f, %f, \n", i, j, sys.tracks[i].projections[j].isValid, 
			sys.tracks[i].projections[j].kp[0], sys.tracks[i].projections[j].kp[1], sys.tracks[i].projections[j].kp[2]);
			printf( "2d pts, %d, from node, %d, is valid, %d,  %f, %f, %f, \n", i, j, sys.tracks[i].projections[j].isValid, 
			sys.tracks[i].projections[j].kp[0], sys.tracks[i].projections[j].kp[1], sys.tracks[i].projections[j].kp[2]);
			myfile2dPts << buffdata;
		}
	}
	
		
	myfileNodes.close();
	myfile3dPts.close();
	myfile2dPts.close();
	
	
    // Publish markers
    drawGraph(sys, cam_marker_pub, point_marker_pub);
    ros::spinOnce();
    
    ROS_INFO("Sleeping for 5 seconds to publish pre-SBA markers.");
    ros::Duration(5.0).sleep();
        
    // Perform SBA with 10 iterations, an initial lambda step-size of 1e-3, 
    // and using CSPARSE.
    //~ sys.doSBA(10, 1e-3, SBA_SPARSE_CHOLESKY);
    //~ sys.doSBA(10, 1e-3, 2);
    sys.doSBA(10, 1e-3, 2, 1e-3, 50);
    
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
    ros::init(argc, argv, "sba_system_setup");
    
    ros::NodeHandle node;
    
    processSBA(node);
    ros::spinOnce();

    return 0;
}
