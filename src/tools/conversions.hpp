/** @file
Data-type conversion tools.
*/
#pragma once
#include <yaml-cpp/yaml.h>
#include "tools/general.hpp"
#include "tools/arithmetic.hpp"

#ifdef UNITAIR_USE_ROS
    #include <tf/transform_datatypes.h>
    #include <geometry_msgs/Point.h>
    #include <geometry_msgs/Vector3.h>
    #include <geometry_msgs/Quaternion.h>
#endif

namespace unitair {
namespace tools {

//////////////////////////////////////////////////

/// @name YAML to Unitair
//@{

/// Safely retrieves an entry from a YAML::Node with string keys
inline YAML::Node yget(Str const& key, YAML::Node const& node) {
    YAML::Node const& subnode = node[key];
    if (subnode) {return subnode;}
    throw std::runtime_error("YAML node is missing data with key '"+key+"'.");
}

/// Retrieves an entry from a YAML::Node with string keys if it exists, else uses default
template <class T>
inline YAML::Node yget(Str const& key, YAML::Node const& node, T const& default_value) {
    YAML::Node const& subnode = node[key];
    if(subnode.IsDefined() && !subnode.IsNull()) {return subnode;}
    return YAML::Node(default_value);
}

/// Converts YAML::Node to unitair::RealVecN with the specified dimensionality
inline RealVecN realvec_from_yaml(YAML::Node const& node, Nat rows) {
    if (!node.IsSequence() || node.size()!=rows) {
        throw std::runtime_error("YAML::Node could not be converted to RealVec of length "+std::to_string(rows)+".");
    }
    RealVecN realvec(node.size());
    for (Int i=0; i<realvec.rows(); ++i) {realvec(i) = node[i].as<Real>();}
    return realvec;
}

/// Converts YAML::Node to unitair::RealMatN with the specified dimensionality
inline RealMatN realmat_from_yaml(YAML::Node const& node, Nat rows, Nat cols) {
    if (!node.IsSequence() || node.size()!=rows || !node[0].IsSequence() || node[0].size()!=cols) {
        throw std::runtime_error("YAML::Node could not be converted to RealMat of shape ("+std::to_string(rows)+","+std::to_string(cols)+").");
    }
    RealMatN realmat(node.size(), node[0].size());
    for (Int col=0; col<realmat.cols(); ++col) {
        for (Int row=0; row<realmat.rows(); ++row) {
            realmat(row,col) = node[row][col].as<Real>();
        }
    }
    return realmat;
}

/// Converts YAML::Node to unitair::Quat (scaled to unit-norm)
inline Quat quat_from_yaml(YAML::Node const& node) {
    return Quat(yget("w", node).as<Real>(),
                yget("x", node).as<Real>(),
                yget("y", node).as<Real>(),
                yget("z", node).as<Real>()).normalized();
}

//@}

//////////////////////////////////////////////////

/// @name ROS to Unitair
//@{
#ifdef UNITAIR_USE_ROS

    /// Converts geometry_msgs::Point to unitair::RealVec<3>
    inline RealVec<3> realvec3_from_ros_point(geometry_msgs::Point const& msg) {
        return RealVec<3>(msg.x, msg.y, msg.z);
    }

    /// Converts geometry_msgs::Vector3 to unitair::RealVec<3>
    inline RealVec<3> realvec3_from_ros_vector3(geometry_msgs::Vector3 const& msg) {
        return RealVec<3>(msg.x, msg.y, msg.z);
    }

    /// Converts geometry_msgs::Quaternion to unitair::Quat (scaled to unit-norm)
    inline Quat quat_from_ros_quaternion(geometry_msgs::Quaternion const& msg) {
        return Quat(msg.w, msg.x, msg.y, msg.z).normalized();
    }

#endif
//@}

/////////////////////////

/// @name Unitair to ROS
//@{
#ifdef UNITAIR_USE_ROS

    /// Converts unitair::RealVec<3> to geometry_msgs::Point
    inline geometry_msgs::Point ros_point_from_realvec3(RealVec<3> const& v) {
        geometry_msgs::Point msg;
        msg.x = v(0); msg.y = v(1); msg.z = v(2);
        return msg;
    }

    /// Converts unitair::RealVec<3> to geometry_msgs::Vector3
    inline geometry_msgs::Vector3 ros_vector3_from_realvec3(RealVec<3> const& v) {
        geometry_msgs::Vector3 msg;
        msg.x = v(0); msg.y = v(1); msg.z = v(2);
        return msg;
    }

    /// Converts unitair::Quat to geometry_msgs::Quaternion
    inline geometry_msgs::Quaternion ros_quaternion_from_quat(Quat const& q) {
        geometry_msgs::Quaternion msg;
        msg.w = q.w(); msg.x = q.x(); msg.y = q.y(); msg.z = q.z();
        return msg;
    }

    /// Packages a position, orientation, frame names, and time into a tf::StampedTransform
    inline tf::StampedTransform make_tf(RealVec<3> const& pos, Quat const& ori, Str const& base, Str const& target, ros::Time const& when) {
        return tf::StampedTransform(tf::Transform(tf::Quaternion(ori.x(), ori.y(), ori.z(), ori.w()), tf::Vector3(pos(0), pos(1), pos(2))), when, base, target);
    }

#endif
//@}

//////////////////////////////////////////////////

} // namespace tools
} // namespace unitair
