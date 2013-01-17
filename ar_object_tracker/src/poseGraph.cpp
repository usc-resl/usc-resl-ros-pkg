/// poseGraph.cpp
///
/// Author: Max Pflueger
/// Date: March 26, 2012

#include "../include/poseGraph.h"

#include <ros/ros.h>
#include "yaml-cpp/yaml.h"

#include <algorithm>
#include <fstream>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <utility>

using namespace std;
using namespace Eigen;

void PoseGraph::processPose(const ar_pose::ARMarkers::ConstPtr& markers) {
  ROS_DEBUG("processPose called");

  vector<ar_pose::ARMarker> my_markers;
  stringstream marker_ids;
  for(vector<ar_pose::ARMarker>::const_iterator i = markers->markers.begin();
      i != markers->markers.end(); i++) {
    marker_ids << i->id << ", ";
    //TODO: check if i is in our list
    // add it to a list
    my_markers.push_back(*i);
  }
  ROS_INFO("processPose: got marker ids: [%s]", marker_ids.str().c_str());

  // record a transform for every pair of poses in the list
  for(vector<ar_pose::ARMarker>::const_iterator i = my_markers.begin();
      i != my_markers.end(); i++) {
    for(vector<ar_pose::ARMarker>::const_iterator j = i + 1;
        j != my_markers.end(); j++) {
      if (i->header.frame_id != j->header.frame_id
          || i->header.stamp != j->header.stamp) {
        ROS_ERROR("processPose received two poses in different frames, this"
                  " is not supposed to be possible, and is not handled");
        continue;
      }
      PairwiseTransform ptf;
      ptf.parent = i->id;
      ptf.child = j->id;
      // calculate transform from i to j
      tf::Transform i_pose(btQuaternion(i->pose.pose.orientation.x,
                                        i->pose.pose.orientation.y,
                                        i->pose.pose.orientation.z,
                                        i->pose.pose.orientation.w),
                           btVector3(i->pose.pose.position.x,
                                     i->pose.pose.position.y,
                                     i->pose.pose.position.z));
      tf::Transform j_pose(btQuaternion(j->pose.pose.orientation.x,
                                        j->pose.pose.orientation.y,
                                        j->pose.pose.orientation.z,
                                        j->pose.pose.orientation.w),
                           btVector3(j->pose.pose.position.x,
                                     j->pose.pose.position.y,
                                     j->pose.pose.position.z));
      tf::Transform diff = i_pose.inverseTimes(j_pose);
      ptf.pose.pose = diff;

      // made up approximate covariance
      ptf.pose.cov << 0.02 * MatrixXd::Identity(3,3), MatrixXd::Zero(3,4),
                      MatrixXd::Zero(4,3), 0.2 * MatrixXd::Identity(4,4);
      _pairwise_tf_list.push_back(ptf);
    }
  }
};

tf::Transform PoseGraph::getObjectPose(
    const ar_pose::ARMarkers::ConstPtr& markers) const {
  TransformWithCov obj_pose;
  bool init = false;

  for(vector<ar_pose::ARMarker>::const_iterator it = markers->markers.begin();
      it != markers->markers.end(); it++) {
    // calcualate object pose implied by this marker
    tf::Transform marker_pose(tf::Quaternion(it->pose.pose.orientation.x,
                                             it->pose.pose.orientation.y,
                                             it->pose.pose.orientation.z,
                                             it->pose.pose.orientation.w),
                              tf::Vector3(it->pose.pose.position.x,
                                          it->pose.pose.position.y,
                                          it->pose.pose.position.z));
    TransformWithCov inc_obj_pose;
    inc_obj_pose.pose = marker_pose
                        * _object_tree.find(it->id)->second.inverse();
    // made up approximate covariance
    inc_obj_pose.cov << 0.02 * MatrixXd::Identity(3,3), MatrixXd::Zero(3,4),
                        MatrixXd::Zero(4,3), 0.2 * MatrixXd::Identity(4,4);

    // kalman update to object pose
    if (!init) {
      obj_pose = inc_obj_pose;
      init = true;
    } else {
      obj_pose = kalmanUpdate(obj_pose, inc_obj_pose);
    }
  }

  return obj_pose.pose;
};

int PoseGraph::genObjectTree() {
  ROS_WARN("genObjectTree is not implemented yet");

  //map<pair<int, int>, TransformWithCov> tf_graph;
  //inferTfGraph(tf_graph);
  //inferObjectTree(tf_graph);
  inferTfGraph();
  inferObjectTree();

  return 0;
};

//void PoseGraph::inferTfGraph(map<pair<int, int>, TransformWithCov>& tf_graph) {
void PoseGraph::inferTfGraph() {
  ROS_INFO("recorded %d pairwise transforms", (int) _pairwise_tf_list.size());

  _marker_set.clear();

  // compress the poses for each pair into a single transform using the Kalman
  // filter update rule
  for(vector<PairwiseTransform>::const_iterator i = _pairwise_tf_list.begin();
      i != _pairwise_tf_list.end(); i++) {
    int parent = i->parent;
    int child = i->child;
    TransformWithCov pose = i->pose;

    _marker_set.insert(parent);
    _marker_set.insert(child);

    if (parent > child) {
      ROS_WARN("parent > child ... ? (this shouldn't happen)");
      int swap = parent;
      parent = child;
      child = swap;
      pose.pose = pose.pose.inverse();
    }

    // check if this is the first transform for this pair
    if (_tf_graph.find(pair<int,int>(parent,child)) == _tf_graph.end()) {
      _tf_graph[pair<int,int>(parent, child)] = pose;
    } else {
      // apply Kalman filter update rule, no random motion or action
      // method from Probabilistic Robotics by Thrun et. al., p. 42
      TransformWithCov prev = _tf_graph[pair<int,int>(parent, child)];
      Matrix<double, 7, 1> mu_prev = tf2Vec(prev.pose);
      Matrix<double, 7, 1> z = tf2Vec(pose.pose);

      // compute Kalman gain, assume C = I
      Matrix<double, 7, 7> K = prev.cov * (prev.cov + pose.cov).inverse();

      Matrix<double, 7, 1> mu = mu_prev + K * (z - mu_prev);
      Matrix<double, 7, 7> cov = (MatrixXd::Identity(7,7) - K) * prev.cov;

      TransformWithCov update;
      update.pose = vec2Tf(mu);
      update.cov = cov;
      _tf_graph[pair<int,int>(parent, child)] = update;
    }
  }
  ROS_INFO("recorded transforms over %d unique markers",
           (int) _marker_set.size());
}

//void PoseGraph::inferObjectTree(
//    map<pair<int, int>, TransformWithCov>& tf_graph) {
void PoseGraph::inferObjectTree() {
  // need to be able to go id <-> index
  map<uint32_t, int> marker_index; // id -> index
  vector<uint32_t> marker_list; // index -> id
  for(set<uint32_t>::const_iterator i = _marker_set.begin();
      i != _marker_set.end(); i++) {
    marker_index[*i] = marker_list.size();
    marker_list.push_back(*i);
  }
  int dim_X = 7 * (_marker_set.size() - 1);

  // Build X
  // X is concatenated positions of all markers except the first
  VectorXd X = buildX(marker_index, marker_list);

  // Build D_bar from _tf_graph
  int dim_D = 7 * _tf_graph.size();
  VectorXd D_bar(dim_D);
  int block_index = 0;
  for(map<pair<int, int>, TransformWithCov>::const_iterator i
          = _tf_graph.begin();
      i != _tf_graph.end(); i++) {
    // use block operations to fill D_bar
    D_bar.block<7,1>(7*block_index, 0) = tf2Vec(i->second.pose);
    block_index ++;
  }

  // Build the H matrix using X
  MatrixXd H = MatrixXd::Zero(dim_D, dim_X);
  block_index = 0;
  for(map<pair<int, int>, TransformWithCov>::const_iterator i
          = _tf_graph.begin();
      i != _tf_graph.end(); i++) {
    int parent = (int) marker_index[i->first.first] - 1;
    int child = (int) marker_index[i->first.second] - 1;
    if (parent >= 0) {
      H.block<7,7>(7*block_index, 7*parent) = -MatrixXd::Identity(7,7);
    }
    // child is alwasys >= 0
    H.block<7,7>(7*block_index, 7*child) = MatrixXd::Identity(7,7);
    block_index ++;
  }

  // Build the offset vector Y using X
  VectorXd Y(dim_D);
  block_index = 0;
  for(map<pair<int, int>, TransformWithCov>::const_iterator i
          = _tf_graph.begin();
      i != _tf_graph.end(); i++) {
    int parent = (int) marker_index[i->first.first] - 1;
    int child = (int) marker_index[i->first.second] - 1;
    // set column 0 vectors to (ab - (b - a))
    VectorXd a;
    if (parent >= 0) {
      a = X.block<7,1>(7 * parent, 0);
    } else {
      VectorXd temp(7);
      temp << 0, 0, 0, 0, 0, 0, 1;
      a = temp;
    }
    VectorXd b = X.block<7,1>(7 * child, 0);
    VectorXd ab = tf2Vec(vec2Tf(a).inverseTimes(vec2Tf(b)));
    if (parent == -1) {
      a = MatrixXd::Zero(7,1);
    }
    Y.block<7,1>(7*block_index, 0) = ab - (b - a);
    block_index ++;
  }

  // Build C_inv from _tf_graph covariance values
  MatrixXd C_inv = MatrixXd::Zero(dim_D, dim_D);
  // first fill in as C
  block_index = 0;
  for(map<pair<int, int>, TransformWithCov>::const_iterator i
          = _tf_graph.begin();
      i != _tf_graph.end(); i++) {
    C_inv.block<7,7>(7*block_index, 7*block_index) = i->second.cov;
    block_index ++;
  }
  C_inv = C_inv.inverse(); // now invert

  // Calculate X=(H^t*C_inv*H)^-1 * H^t*C_inv*(D_bar - Y)
  X = (H.transpose() * C_inv * H).inverse()
      * H.transpose() * C_inv * (D_bar - Y);

  // record in _object_tree
  _object_tree.clear();
  // specify marker 0 position
  _object_tree[marker_list[0]] = tf::Transform(tf::Quaternion(0, 0, 0 ,1),
                                               tf::Vector3(0, 0 ,0));
  for(size_t i=0; i < (marker_list.size() - 1); i++) {
    //_object_tree[marker_list[i]] = vec2Tf(X.block<7,1>(7 * i, 0));
    //_object_tree[marker_list[i]] = _object_tree[marker_list[0]]
    //    * vec2Tf(X.block<7,1>(0,0)).inverseTimes(vec2Tf(X.block<7,1>(7*i, 0)));
    _object_tree[marker_list[i + 1]] = _object_tree[marker_list[0]]
        * vec2Tf(X.block<7,1>(7*i, 0));
  }
}

MatrixXd PoseGraph::buildX(std::map<uint32_t, int>& marker_index,
                           std::vector<uint32_t>& marker_list) {
  // Reference frame will be initial position of marker index 0
  // Tag with index 0 will be fixed at (0, 0, 0) (0, 0, 0, 1)
  // Use BFS to find initial positions of all tags X
  int dim_X = 7 * (_marker_set.size() - 1);
  VectorXd X(dim_X);
  VectorXd root(7);
  root << 0, 0, 0, 0, 0 ,0 ,1;
  //X.block<7,1>(0,0) << 0, 0, 0, 0, 0, 0, 1;

  // Use graph traversal to fill in X
  set<uint32_t> visited;
  queue<pair<uint32_t, uint32_t> > next; // parent, child

  // push the root, when parent == child we know it is the root
  next.push(pair<uint32_t, uint32_t>(marker_list[0], marker_list[0]));
  visited.insert(marker_list[0]);
  ROS_DEBUG("Root is id [%d]", marker_list[0]);
  while(!next.empty()) {
    // calculate tf to next.front().second and put it in X
    // check if this is the root node, it is already in X
    if (next.front().first == next.front().second) {
      if (marker_index[next.front().first] != 0) {
        ROS_ERROR("Got a non-zero index as a root node,"
                  " this should not happen.");
      }
    } else {
      // parent TF will already be stored in X
      int parent_index = (int) marker_index[next.front().first] - 1;
      tf::Transform parent_tf;
      if (parent_index >= 0) {
        parent_tf = vec2Tf(X.block<7,1>(7 * parent_index, 0));
      } else {
        parent_tf = vec2Tf(root);
      }

      // parent may have a greater or lesser id than child, this changes how
      // it is stored in _tf_graph
      tf::Transform parent_to_child;
      if (next.front().first < next.front().second) {
        parent_to_child = _tf_graph[pair<int,int>(
            next.front().first, next.front().second)].pose;
      } else {
        parent_to_child = _tf_graph[pair<int,int>(
            next.front().second, next.front().first)].pose.inverse();
      }

      // put the child tf into X
      X.block<7,1>(7 * (marker_index[next.front().second] - 1), 0)
          = tf2Vec(parent_tf * parent_to_child);
    }

    // push unvisited nodes accessible from front to next
    for(set<uint32_t>::const_iterator i = _marker_set.begin();
        i != _marker_set.end(); i++) {
      // check if visited
      if (visited.find(*i) == visited.end()) {
        // check if reachable
        int less = min(next.front().second, *i);
        int more = max(next.front().second, *i);
        if (_tf_graph.find(pair<int,int>(less, more)) != _tf_graph.end()) {
          ROS_DEBUG("inserting [%d] as child of [%d]", *i, next.front().second);
          next.push(pair<uint32_t, uint32_t>(next.front().second, *i));
          visited.insert(*i);
        }
      }
    }

    next.pop();
  }

  // Check for unreachable nodes
  for(set<uint32_t>::const_iterator i = _marker_set.begin();
      i != _marker_set.end(); i++) {
    if (visited.find(*i) == visited.end()) {
      ROS_ERROR("Marker graph is not fully connected!!!"
                " marker id [%d] not reached from root.", *i);
    }
  }

  return X;
}

visualization_msgs::MarkerArray PoseGraph::getTagTreeMarkers() {
  visualization_msgs::MarkerArray markers;
  for(set<uint32_t>::const_iterator it = _marker_set.begin();
      it != _marker_set.end(); it++) {
    // fill in a marker for each tag
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "object_tracker";
    marker.id = *it;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::MODIFY;

    marker.pose.position.x = _object_tree[*it].getOrigin().x();
    marker.pose.position.y = _object_tree[*it].getOrigin().y();
    marker.pose.position.z = _object_tree[*it].getOrigin().z();
    marker.pose.orientation.x = _object_tree[*it].getRotation().x();
    marker.pose.orientation.y = _object_tree[*it].getRotation().y();
    marker.pose.orientation.z = _object_tree[*it].getRotation().z();
    marker.pose.orientation.w = _object_tree[*it].getRotation().w();

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.01;

    marker.color.a = 1.0;
    // give the root a different color
    if (it == _marker_set.begin()) {
      marker.color.r = 0.7;
      marker.color.g = 0.0;
      marker.color.b = 0.7;
    } else {
      marker.color.r = 0.7;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
    //marker.duration = 0; //forever

    markers.markers.push_back(marker);
  }
  ROS_DEBUG("markers has [%d] elements, should have [%d].",
            markers.markers.size(),
            _marker_set.size());

  return markers;
}

bool PoseGraph::writeConf(string filename) {
  fstream fs;
  fs.open(filename.c_str(), fstream::out|fstream::trunc);
  if (fs.fail()) {
    ROS_ERROR("Failed to open output file");
    return false;
  }

  // write the file
  YAML::Emitter out;
  //out << YAML::Literal << "# This is an automatically generated config file.";
  out << YAML::BeginMap;
  out << YAML::Key << "tagCount" << YAML::Value  << _marker_set.size();
  out << YAML::Key << "tags";
  out << YAML::Value << YAML::BeginSeq;

  // print data for all tags in our set
  for(set<uint32_t>::const_iterator i = _marker_set.begin();
      i != _marker_set.end(); i++) {
    out << YAML::BeginMap;

    out << YAML::Key << "tagId";
    out << YAML::Value << *i;

    // write the pose for this tag, if we have it
    if (_object_tree.find(*i) != _object_tree.end()) {
      out << YAML::Key << "pose";
      out << YAML::Value;

      out << YAML::BeginMap;
      out << YAML::Key << "origin"
          << YAML::Value
          << YAML::Flow << YAML::BeginSeq
          << _object_tree[*i].getOrigin().x()
          << _object_tree[*i].getOrigin().y()
          << _object_tree[*i].getOrigin().z() << YAML::EndSeq;
      out << YAML::Key << "orientation"
          << YAML::Value
          << YAML::Flow << YAML::BeginSeq
          << _object_tree[*i].getRotation().x()
          << _object_tree[*i].getRotation().y()
          << _object_tree[*i].getRotation().z()
          << _object_tree[*i].getRotation().w() << YAML::EndSeq;
      out << YAML::EndMap;
    } else {
      ROS_WARN("Tag id [%d] was missing transform data in _object_tree", *i);
    }

    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;

  if (!out.good()) {
    ROS_ERROR("An error occuring writing the YAML for the PoseGraph config "
              "file: [%s]", out.GetLastError().c_str());
    return false;
  }

  fs << out.c_str();
  fs.close();
  return true;
};

bool PoseGraph::readConf(string filename) {
  bool success = true;
  fstream fs;
  fs.open(filename.c_str(), fstream::in);

  try {
    // create the parser
    YAML::Parser parser(fs);
    YAML::Node doc;
    parser.GetNextDocument(doc);

    // fill the _object_tree data structure
    const YAML::Node& tags = doc["tags"];
    for(size_t i=0; i < tags.size(); i++) {
      uint32_t tagId = 0;
      tags[i]["tagId"] >> tagId;

      const YAML::Node& origin = tags[i]["pose"]["origin"];
      const YAML::Node& orientation = tags[i]["pose"]["orientation"];
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

      _object_tree[tagId] = pose;
      _marker_set.insert(tagId);
    }

    // sanity check
    unsigned int numTags = 100000;  // big # to make the check more fragile
    doc["tagCount"] >> numTags;
    if (numTags != _object_tree.size()) {
      ROS_ERROR("tagCount does not match the size of _object_tree after parsing!");
      success = false;
    }
    if (numTags != _marker_set.size()) {
      ROS_ERROR("tagCount does not match the size of _marker_set after parsing!");
      success = false;
    }
    ROS_DEBUG("PoseGraph read config with [%d] tags", numTags);
  }
  catch(YAML::Exception& e) {
    ROS_ERROR("YAML Exception: [%s]", e.what());
    success = false;
  }

  fs.close();
  return success;
};

TransformWithCov PoseGraph::kalmanUpdate(TransformWithCov prev,
                                         TransformWithCov obs) {
  // apply Kalman filter update rule, no random motion or action
  // method from Probabilistic Robotics by Thrun et. al., p. 42
  Matrix<double, 7, 1> mu_prev = tf2Vec(prev.pose);
  Matrix<double, 7, 1> z = tf2Vec(obs.pose);

  // compute Kalman gain, assume C = I
  Matrix<double, 7, 7> K = prev.cov * (prev.cov + obs.cov).inverse();

  Matrix<double, 7, 1> mu = mu_prev + K * (z - mu_prev);
  Matrix<double, 7, 7> cov = (MatrixXd::Identity(7,7) - K) * prev.cov;

  TransformWithCov update;
  update.pose = vec2Tf(mu);
  update.cov = cov;
  return update;
}

Matrix<double, 7, 1> PoseGraph::tf2Vec(tf::Transform pose) {
  Matrix<double, 7, 1> ret;
  ret << pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
         pose.getRotation().x(), pose.getRotation().y(),
         pose.getRotation().z(), pose.getRotation().w();
  return ret;
};

tf::Transform PoseGraph::vec2Tf(Matrix<double, 7, 1> v) {
  tf::Transform ret(tf::Quaternion(v(3), v(4), v(5), v(6)),
                    tf::Vector3(v(0), v(1), v(2)));
  return ret;
};

