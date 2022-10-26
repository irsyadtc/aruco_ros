/*****************************
 Copyright 2022 Muhammad Irsyad Sahalan. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Muhammad Irsyad Sahalan ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Muhammad Irsyad Sahalan.
 ********************************/
/**
 * @file single_reference.cpp
 * @original file simple_single.cpp
 * @author Muhammad Irsyad Sahalan
 * @date September 2022
 * @version 1.0
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
//#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

#include <cmath>
#include <std_msgs/String.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/TransformStamped.h>
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_broadcaster.h>
#include "tf2/buffer_core.h"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//#include <tf2_ros/transform_listener.h>
#include "include/markerslist.hpp"
#include "include/markerslist.cpp"



class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  bool useRectifiedImages;
  aruco::MarkerDetector mDetector;
  std::vector<aruco::Marker> markers;

  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  
  //ros::Publisher rpy_pub;
  ros::Publisher which_marker_pub;

  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;
  
  geometry_msgs::PoseStamped poseMsg;
	std_msgs::String whichMarkerMsg;
	//geometry_msgs::Vector3 rpyMsg;
	
  double marker_size;
  int marker_id;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;
  
  //map marker
  markerslist MarkersList;
  


public:

  /**********************
  	ArucoSimple()
  **********************/
	ArucoSimple() :
  	cam_info_received(false), nh("~"), it(nh)
  {

    if (nh.hasParam("corner_refinement"))
      ROS_WARN(
          "Corner refinement options have been removed in ArUco 3.0.0, corner_refinement ROS parameter is deprecated");

    aruco::MarkerDetector::Params params = mDetector.getParameters();
    std::string thresh_method;
    switch (params._thresMethod)
    {
      case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
        thresh_method = "THRESH_ADAPTIVE";
        break;
      case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
        thresh_method = "THRESH_AUTO_FIXED";
        break;
      default:
        thresh_method = "UNKNOWN";
        break;
    }

    // Print parameters of ArUco marker detector:
    ROS_INFO_STREAM("Threshold method: " << thresh_method);

    float min_marker_size; // percentage of image area
    nh.param<float>("min_marker_size", min_marker_size, 0.02);

    std::string detection_mode;
    nh.param<std::string>("detection_mode", detection_mode, "DM_FAST");
    if (detection_mode == "DM_FAST")
      mDetector.setDetectionMode(aruco::DM_FAST, min_marker_size);
    else if (detection_mode == "DM_VIDEO_FAST")
      mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
    else
      // Aruco version 2 mode
      mDetector.setDetectionMode(aruco::DM_NORMAL, min_marker_size);

    ROS_INFO_STREAM("Marker size min: " << min_marker_size << "% of image area");
    ROS_INFO_STREAM("Detection mode: " << detection_mode);

    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);		//HERE
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    
    which_marker_pub = nh.advertise<std_msgs::String>("which_marker",5);
    //rpy_pub = nh.advertise<geometry_msgs::Vector3>("rpy", 5);

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<std::string>("reference_frame", reference_frame, "map");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);

    ROS_INFO("ArUco node started with marker size of %f m", marker_size);
    ROS_INFO("ArUco node will publish pose to TF with %s as parent and %s as child.", marker_frame.c_str(), camera_frame.c_str());

    dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));
  }



  /************************
  	IMAGE_CALLBACK
  ************************/
	void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    /*
    if ((image_pub.getNumSubscribers() == 0) && (debug_pub.getNumSubscribers() == 0)
        && (pose_pub.getNumSubscribers() == 0) && (transform_pub.getNumSubscribers() == 0)
        && (position_pub.getNumSubscribers() == 0) && (marker_pub.getNumSubscribers() == 0)
        && (pixel_pub.getNumSubscribers() == 0))
    {
      ROS_DEBUG("No subscribers, not looking for ArUco markers");
      return;
    }
    */

		bool match_marker = false;
		static tf2_ros::TransformBroadcaster tfb;
		geometry_msgs::TransformStamped transformStamped;
    
    if (cam_info_received)
    {
      ros::Time curr_stamp = msg->header.stamp;
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        // detection results will go into "markers"
        markers.clear();
        // ok, let's detect
        mDetector.detect(inImage, markers, camParam, marker_size, false);
        
        // for each marker, draw info and its boundaries in the image
        for (std::size_t i = 0; i < markers.size(); ++i)
        {
					//detected marker id == marker on map?
					for(unsigned int j = 0; j < MarkersList.getSize(); j++)
					{
						//check if detected marker match with in the markerlist
						if(j+1 == markers[i].id)
						{
							match_marker == true;
							
							//localize
							tf2::Transform transform = aruco_ros::arucoMarker2Tf2(markers[i]);
							
							//inverse
							tf2::Transform inv_transform = transform.inverse();
							
							marker_frame = MarkersList.getName(j);
							transformStamped.header.frame_id = marker_frame;
							transformStamped.child_frame_id = camera_frame;
							transformStamped.header.stamp = curr_stamp;
							
							transformStamped.transform.translation.x = -1*inv_transform.getOrigin().x();
							transformStamped.transform.translation.y = -1*inv_transform.getOrigin().y();
	   					transformStamped.transform.translation.z = inv_transform.getOrigin().z();
	   					transformStamped.transform.rotation.x = -1*inv_transform.getRotation().x();
							transformStamped.transform.rotation.y = -1*inv_transform.getRotation().y();
							transformStamped.transform.rotation.z = inv_transform.getRotation().z();
							transformStamped.transform.rotation.w = inv_transform.getRotation().w();
						
							whichMarkerMsg.data = MarkersList.getName(j);			
							which_marker_pub.publish(whichMarkerMsg);
							
							//rpyMsg = euler_from_quaternion(transformStamped.transform.rotation);	//need to edit
							//rpy_pub.publish(rpyMsg);
							break;
						}
						if(match_marker)
						{	break;}
					}
          
          tfb.sendTransform(transformStamped);
            							
          // but drawing all the detected markers
          markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
        }

        // draw a 3d cube in each marker if there is 3d info
        if (camParam.isValid() && marker_size != -1)
        {
          for (std::size_t i = 0; i < markers.size(); ++i)
          {
            aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
        }

        if (image_pub.getNumSubscribers() > 0)
        {
          // show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if (debug_pub.getNumSubscribers() > 0)
        {
          // show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }
  
  /************************
  	QUARTERNION TO EULER
  *************************/
  geometry_msgs::Vector3 euler_from_quaternion(geometry_msgs::Quaternion q)
	{
        //Convert a quaternion into euler angles (roll, pitch, yaw)
        //roll is rotation around x in radians (counterclockwise)
        //pitch is rotation around y in radians (counterclockwise)
        //yaw is rotation around z in radians (counterclockwise)
		geometry_msgs::Vector3 v;
		double t0 = +2.0 * (q.w * q.x + q.y * q.z);
		double t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
		v.x = atan2(t0, t1); //roll

		double t2 = +2.0 * (q.w * q.y - q.z * q.x);
		
		if(t2 > 1.0)
		{ t2 = 1.0;	}

		if(t2 < -1.0)
		{ t2 = -1.0; }
		
		v.y = asin(t2); //pitch
		double t3 = +2.0 * (q.w * q.z + q.x * q.y);
    double t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
		v.z = atan2(t3, t4); //yaw
     
		return v; // in radians
	}

/***********************************
	CAM_INFO_CALLBACK
*************************************/

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CameraInfo documentation for details
    //rightToLeft.setIdentity();	//considered to be removed
    //rightToLeft.setOrigin(tf::Vector3(-msg.P[3] / msg.P[0], -msg.P[7] / msg.P[5], 0.0));	//considered to be removed

    cam_info_received = true;
    cam_info_sub.shutdown();
  }

  void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
  {
    mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
    if (config.normalizeImage)
    {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
  }
  
  /*****************************
  	Inverse matrix
  *****************************/
  
  
};

/*************************
	MAIN	
*************************/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_single_reference");

  ArucoSimple node;

  ros::spin();
}
