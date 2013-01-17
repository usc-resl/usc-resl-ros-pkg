/// calibrate.cpp
///
/// Measures the relative pose of multiple AR Tags and produces a config file
/// to be used for tracking the object to which they are attached.
/// Author: Max Pflueger
/// Date: March 15, 2012

//#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <string>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ar_pose/ARMarkers.h>

#include "../include/poseGraph.h"

using namespace std;

void stopper(bool* signal, boost::mutex* signal_lock);

int main(int argc, char** argv) {
  ros::init(argc, argv, "ar_object_tracker_calibrate");
  ros::NodeHandle nh("~");

  // get the destination filename
  string filename;
  if(!nh.getParam("output_file", filename)) {
    ROS_FATAL("AR Object Tracker Calibration requires a filename in param:output_file");
    //return -1;
  }

  PoseGraph poseGraph;

  // subscribe to the tag tracker topic
  ros::Subscriber sub = nh.subscribe("ar_pose_marker",
                                     30,
                                     &PoseGraph::processPose,
                                     &poseGraph);

  // spin and collect data until told to stop
  bool signal = true;
  boost::mutex signal_lock;
  boost::thread stop_thread(stopper, &signal, &signal_lock);
  ros::Rate r(30); // 30 hz
  while(ros::ok()) {
    // check for our stop signal
    boost::lock_guard<boost::mutex>* signal_guard
        = new boost::lock_guard<boost::mutex>(signal_lock);
    if(!signal) {
      delete signal_guard;
      break;
    }
    delete signal_guard;

    ros::spinOnce();
    r.sleep();
  }
  stop_thread.join();
  if(!ros::ok()) {
    return -1;
  }
  cout << "Done.\n";

  //TODO: average the transforms collected into a single tree for the object
  //  and write that tree to the output config file.
  poseGraph.genObjectTree();
  poseGraph.writeConf(filename);

  return 0;
}

void stopper(bool* signal, boost::mutex* signal_lock) {
  cout << "Press [space][enter] to stop collecting data...\n";

  // make stdin non-blocking
  int flags = fcntl(STDIN_FILENO, F_GETFL);
  fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

  unsigned char x = 0;
  ssize_t count = 0;
  while(ros::ok() && (count < 1 || x != ' ')) {
    count = read(STDIN_FILENO, &x, 1);
  }
  if(!ros::ok()) {
    return;
  }

  cout << "Stopping...";
  cout.flush();

  boost::lock_guard<boost::mutex>* signal_guard
      = new boost::lock_guard<boost::mutex>(*signal_lock);
  *signal = false;
  delete signal_guard;
}

