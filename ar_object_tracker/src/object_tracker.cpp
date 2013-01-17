/// object_tracker.cpp
///
/// Merges the pose data of multiple AR tags attached to an object to produce
/// the pose of the object.
/// Author: Max Pflueger
/// Date: Jan. 26, 2012

#include <fstream>
#include <math.h>
#include <string>

#include <ros/ros.h>
#include <ar_pose/ARMarkers.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include "yaml-cpp/yaml.h"

#include "../include/poseGraph.h"


using namespace std;

class ChairTrackingTestPublisher {
  public:
    ChairTrackingTestPublisher(tf::TransformBroadcaster* br): _br(br) {}

    void sendTestTransform(const ros::TimerEvent& e) {
      ros::Time stamp = ros::Time::now();

      tf::Transform baseToBack;
      baseToBack.setOrigin(tf::Vector3(0.6, 0.0, 0.7));
      //baseToBack.setRotation(tf::Quaternion(1.57, 0, 0));
      baseToBack.setRotation(tf::Quaternion(tf::Vector3(1, 0, 0), 1.57));

      _br->sendTransform(tf::StampedTransform(baseToBack, stamp,
                                              "base_link", "chair"));
      _br->sendTransform(tf::StampedTransform(baseToBack, stamp,
                                              "base_link", "back_observed"));

      // normally this node would publish transforms off of base_link, or
      // something else attached to the sensors it is using, for testing we will
      // provide a back->seat transform
      tf::Transform backToSeat;
      // the origin should be ignored
      backToSeat.setOrigin(tf::Vector3(0, -0.4355, -0.30));
      // the following should spin the seat slowly off axis, the spin should be
      // pulled back on-axis by the joint state publisher
      tf::Quaternion start(-1.570796, 3.1415927, 0);
      tf::Quaternion spin(tf::Vector3(1, 0, 0.5), fmod(stamp.toSec(), 6.28318) - 3.14159);
      backToSeat.setRotation(start * spin);

      _br->sendTransform(tf::StampedTransform(backToSeat, stamp,
                                              "back_observed", "seat_observed"));
    }

  private:
    tf::TransformBroadcaster* _br;
};

class MarkerPublisher {
  public:
    MarkerPublisher(ros::Publisher* marker_pub, PoseGraph* poseGraph):
        _marker_pub(marker_pub), _poseGraph(poseGraph) {}

    void publishMarkers(const ros::TimerEvent& e) {
      _marker_pub->publish(_poseGraph->getTagTreeMarkers());
    }

  private:
    ros::Publisher* _marker_pub;
    PoseGraph* _poseGraph;
};

/// Callback class to read observed markers and publish the transform to the
/// object
class ObjectTracker {
  public:
    ObjectTracker(tf::TransformBroadcaster* br,
                  PoseGraph* poseGraph):
        _br(br), _poseGraph(poseGraph) {}

    /// read from config the _object_frame and _root_marker_offset variables
    bool readOffsetConf(string offset_conf) {
      bool success = true;
      fstream fs;
      fs.open(offset_conf.c_str(), fstream::in);

      try {
        // create the parser
        YAML::Parser parser(fs);
        YAML::Node doc;
        parser.GetNextDocument(doc);

        // read the object frame
        doc["object_frame"] >> _object_frame;

        // read the marker offset
        const YAML::Node& origin = doc["pose"]["origin"];
        const YAML::Node& orientation = doc["pose"]["orientation"];
        double x,y,z,qx,qy,qz,qw;
        origin[0] >> x;
        origin[1] >> y;
        origin[2] >> z;
        orientation[0] >> qx;
        orientation[1] >> qy;
        orientation[2] >> qz;
        orientation[3] >> qw;
        tf::Transform pose(tf::Quaternion(qx, qy, qz, qw),
                           tf::Vector3(x, y, z));
        _root_marker_offset = pose;
      }
      catch(YAML::Exception& e) {
        ROS_ERROR("YAML Exception: [%s]", e.what());
        success = false;
      }

      fs.close();
      return success;
    }

    /// subscriber callback to track an object using incoming ARMarker arrays
    void trackObject(const ar_pose::ARMarkers::ConstPtr& markers) {
      // this assumes all markers have the same header, which should be true.
      // I don't like, but it may be unavoidable (could modify poseGraph to be
      // stamped).
      string base_frame;
      ros::Time stamp;
      if (markers->markers.size() > 0) {
        base_frame = markers->markers[0].header.frame_id;
        stamp = markers->markers[0].header.stamp;
      } else {
        ROS_DEBUG("trackObject recieved ARMarkers message with empty markers"
                  " element");
        return;
      }

      // call getObjectPose() with the message
      tf::Transform root_marker_pose = _poseGraph->getObjectPose(markers);

      // transform returned tf into object pose using offset config
      tf::Transform object_pose = root_marker_pose
                                  * _root_marker_offset.inverse();

      // publish the object transform
      ROS_DEBUG("publishing TF from [%s] to [%s]", base_frame.c_str(), _object_frame.c_str());
      _br->sendTransform(
          tf::StampedTransform(object_pose, stamp, base_frame, _object_frame));
    }

  private:
    tf::TransformBroadcaster* _br;
    PoseGraph* _poseGraph;
    string _object_frame;
    tf::Transform _root_marker_offset;
};

// ******* main *****************
int main(int argc, char** argv) {
  ros::init(argc, argv, "object_tracker");
  ros::NodeHandle nh("~");

  tf::TransformBroadcaster br;

  // Go to testing mode if necessry
  bool test = false;
  nh.getParam("test_mode", test);
  if (test) {
    ChairTrackingTestPublisher chairTrackingTestPublisher(&br);
    ros::Timer pub_timer = nh.createTimer(
                               ros::Duration(0.02),
                               &ChairTrackingTestPublisher::sendTestTransform,
                               &chairTrackingTestPublisher);
    ros::spin();
    return 0;
  }

  ROS_ERROR("check 1");
  // read object calibration config yaml
  string calibration_conf;
  if (!nh.getParam("object_calibration_conf", calibration_conf)) {
    ROS_FATAL("AR Object Tracker requires a filename in param:object_calibration_conf");
    return -1;
  }
  PoseGraph poseGraph;
  if (!poseGraph.readConf(calibration_conf)) {
    ROS_FATAL("AR Object Tracker failed to read config file [%s]",
              calibration_conf.c_str());
    return -1;
  }

  ROS_ERROR("check 2");
  // publish a visualization of our config, to see if it is any good
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "tag_config_vis", 10);
  MarkerPublisher markerPublisher(&marker_pub, &poseGraph);
  ros::Timer marker_pub_timer = nh.createTimer(
      ros::Duration(0.1),
      &MarkerPublisher::publishMarkers,
      &markerPublisher);

  ROS_ERROR("check 3");
  // read the object offset config yaml
  string offset_conf;
  if (!nh.getParam("object_offset_conf", offset_conf)) {
    ROS_FATAL("AR Object Tracker required a filename in param:object_offset_conf");
    return -1;
  }
  ObjectTracker objectTracker(&br, &poseGraph);
  if (!objectTracker.readOffsetConf(offset_conf)) {
    ROS_FATAL("AR Object Tracker failed to read config file [%s]",
              offset_conf.c_str());
    return -1;
  }

  ROS_ERROR("check 4");
  // listen to ar tag marker positions and publish object transform
  ros::Subscriber marker_sub = nh.subscribe("/ar_pose_marker",
                                            1,
                                            &ObjectTracker::trackObject,
                                            &objectTracker);

  ROS_ERROR("check 5");
  ros::spin();
  return 0;
}

