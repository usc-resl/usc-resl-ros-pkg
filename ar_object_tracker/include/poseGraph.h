/// poseGraph.h
///
/// Author: Max Pflueger
/// Date: March 26, 2012

#ifndef POSEGRAPH_H
#define POSEGRAPH_H

#include <map>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <ar_pose/ARMarkers.h>
#include <LinearMath/btTransform.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

struct TransformWithCov {
  tf::Transform pose;
  // covariance for the pose, 7x7 matrix,
  //           pos  | orientation
  // ordered: x y z   x  y  z  w
  Eigen::Matrix<double, 7, 7> cov;
};

struct PairwiseTransform {
  uint32_t parent;
  uint32_t child;
  TransformWithCov pose;
};

/// Class to store and manipulate the marker pairwise transforms that we care
///   about.
class PoseGraph {
  public:
    /// A callback function to record the relative poses from a set of
    /// detected markers
    void processPose(const ar_pose::ARMarkers::ConstPtr& markers);

    /// Use the recorded relative poses to infer the best consistent set of 
    /// transforms.
    int genObjectTree();

    /// Report the most likely object pose given the set of observed markers
    tf::Transform getObjectPose(const ar_pose::ARMarkers::ConstPtr& markers) const;

    /// Get a MarkerArray message to show the status of the current
    /// _object_tree
    visualization_msgs::MarkerArray getTagTreeMarkers();

    /// Write the relative poses to a config file
    bool writeConf(std::string filename);

    /// Read relative poses from a config file
    bool readConf(std::string filename);

    static TransformWithCov kalmanUpdate(TransformWithCov prev,
                                         TransformWithCov obs);
    static Eigen::Matrix<double, 7, 1> tf2Vec(tf::Transform pose);
    static tf::Transform vec2Tf(Eigen::Matrix<double, 7, 1> v);
  private:
    /// Infer the (probably) inconsistent tf graph from measured pairwise
    /// transforms.
    //void inferTfGraph(
    //    std::map<std::pair<int, int>, TransformWithCov>& tf_graph);
    void inferTfGraph();

    /// Perform linearized least squares optimization (from Lu & Milios) on
    /// the tf graph to find the tree for this object (written to _object_tree
    /// variable)
    //void inferObjectTree(
    //    std::map<std::pair<int, int>, TransformWithCov>& tf_graph);
    void inferObjectTree();

    /// Build a consistent set of positions X from _tf_graph
    /// (to be used as initial conditions for inference)
    Eigen::MatrixXd buildX(std::map<uint32_t, int>& marker_index,
                           std::vector<uint32_t>& marker_list);

    std::set<uint32_t> _marker_set;
    std::map<uint32_t, tf::Transform> _object_tree;
    std::vector<PairwiseTransform> _pairwise_tf_list;
    std::map<std::pair<int, int>, TransformWithCov> _tf_graph;
};

#endif // POSEGRAPH_H

