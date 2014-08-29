/*
 * Copyright (c) 2014, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RobotLocalization_RosFilter_h
#define RobotLocalization_RosFilter_h

#include <robot_localization/filter_common.h>
#include <robot_localization/filter_base.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <XmlRpcException.h>

#include <Eigen/Dense>

#include <fstream>

// Handy methods for debug output
std::ostream& operator<<(std::ostream& os, const tf::Transform &trans)
{
  tf::Matrix3x3 orientation(trans.getRotation());

  double roll, pitch, yaw;
  orientation.getRPY(roll, pitch, yaw);

  os << "Origin: (" << std::setprecision(20) << trans.getOrigin().getX() << " " <<
        trans.getOrigin().getY() << " " << trans.getOrigin().getZ() << ")\n" <<
        "Rotation (RPY): (" << roll << ", " << pitch << ", " << yaw << ")\n";

  return os;
}

namespace RobotLocalization
{
  template<class Filter> class RosFilter
  {
    public:

      RosFilter();

      ~RosFilter();

      //! @brief Retrieves the EKF's output for broadcasting
      //! @param[out] message - The standard ROS odometry message to be filled
      //!
      bool getFilteredOdometryMessage(nav_msgs::Odometry &message);

      //! @brief Loads all parameters from file
      //!
      void loadParams();

      //! @brief Callback method for receiving all IMU messages
      //! @param[in] msg - The ROS IMU message to take in.
      //! @param[in] topicName - The name of the IMU data topic (we support many)
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //! @param[in] differential - Whether we integrate the pose portions of this message differentially
      //!
      void imuCallback(const sensor_msgs::Imu::ConstPtr &msg,
                       const std::string &topicName,
                       const std::vector<int> &updateVector,
                       const bool differential);

      //! @brief Callback method for receiving all odometry messages
      //! @param[in] msg - The ROS odometry message to take in.
      //! @param[in] topicName - The name of the odometry topic (we support many)
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //! @param[in] differential - Whether we integrate the pose portions of this message differentially
      //!
      void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg,
                            const std::string &topicName,
                            const std::vector<int> &updateVector,
                            const bool differential);

      //! @brief Callback method for receiving all pose messages
      //! @param[in] msg - The ROS stamped pose with covariance message to take in
      //! @param[in] topicName - The name of the pose topic (we support many)
      //! @param[in] targetFrame - The tf frame name into which we will transform this measurement
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //! @param[in] differential - Whether we integrate the pose portions of this message differentially
      //!
      void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                        const std::string &topicName,
                        const std::string &targetFrame,
                        const std::vector<int> &updateVector,
                        const bool differential);

      //! @brief Callback method for manually setting/resetting the internal pose estimate
      //! @param[in] msg - The ROS stamped pose with covariance message to take in
      //!
      void setPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

      //! @brief Callback method for receiving all twist messages
      //! @param[in] msg - The ROS stamped twist with covariance message to take in.
      //! @param[in] topicName - The name of the twist topic (we support many)
      //! @param[in] targetFrame - The tf frame name into which we will transform this measurement
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //!
      void twistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                         const std::string &topicName,
                         const std::string &targetFrame,
                         const std::vector<int> &updateVector);

      void run();

    protected:

      Filter filter_;

      //! @brief Loads fusion settings from the config file
      //! @param[in] topicName - The name of the topic for which to load settings
      //! @return The boolean vector of update settings for each variable for this topic
      //!
      std::vector<int> loadUpdateConfig(const std::string &topicName);

      //! @brief Prepares a pose message for integration into the filter
      //! @param[in] msg - The pose message to prepare
      //! @param[in] topicName - The name of the topic over which this message was received
      //! @param[in] targetFrame - The target tf frame
      //! @param[in, out] updateVector - The update vector for the data source
      //! @param[in, out] differential - Whether we're carrying out differential integration
      //! @param[in] measurement - The pose data converted to a measurement
      //! @param[in] measurementCovariance - The covariance of the converted measurement
      //!
      bool preparePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                       const std::string &topicName,
                       const std::string &targetFrame,
                       std::vector<int> &updateVector,
                       const bool differential,
                       Eigen::VectorXd &measurement,
                       Eigen::MatrixXd &measurementCovariance);

      //! @brief Prepares a twist message for integration into the filter
      //! @param[in] msg - The twist message to prepare
      //! @param[in] topicName - The name of the topic over which this message was received
      //! @param[in] targetFrame - The target tf frame
      //! @param[in] updateVector - The update vector for the data source
      //! @param[in] measurement - The twist data converted to a measurement
      //! @param[in] measurementCovariance - The covariance of the converted measurement
      //!
      bool prepareTwist(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                        const std::string &topicName,
                        const std::string &targetFrame,
                        std::vector<int> &updateVector,
                        Eigen::VectorXd &measurement,
                        Eigen::MatrixXd &measurementCovariance);

      //! @brief Utility method for converting quaternion to RPY
      //! @param[in] quat - The quaternion to convert
      //! @param[in] roll - The converted roll
      //! @param[in] pitch - The converted pitch
      //! @param[in] yaw - The converted yaw
      //!
      inline void quatToRPY(const tf::Quaternion &quat, double &roll, double &pitch, double &yaw);

      //! @brief The frequency of the run loop
      //!
      double frequency_;

      //! @brief tf prefix
      //!
      std::string tfPrefix_;

      //! @brief tf frame name for the robot's body frame
      //!
      std::string baseLinkFrameName_;

      //! @brief tf frame name for the robot's odometry (world) frame
      //!
      std::string odomFrameName_;

      //! @brief Store the last time a message from each topic was received
      //!
      //! If we're getting messages rapidly, we may accidentally get
      //! an older message arriving after a newer one. This variable keeps
      //! track of the most recent message time for each subscribed message
      //! topic. We also use it when listening to odometry messages to
      //! determine if we should be using messages from that topic.
      //!
      std::map<std::string, ros::Time> lastMessageTimes_;

      //! @brief Stores the last measurement from a given topic for
      //! differential integration
      //!
      //! To carry out differential integration, we have to (1) transform
      //! that into the target frame (probably the frame specified by
      //! \p odomFrameName_), (2) "subtract"  the previous measurement from
      //! the current measurement, and then (3) transform it again by the previous
      //! state estimate. This holds the measurements used for step (2).
      //!
      std::map<std::string, tf::Transform> previousMeasurements_;

      //! @brief Stores the last state estimate at the time the previous
      //! measurement from this sensor was captured
      //!
      //! To carry out differential integration, we have to (1) transform
      //! that into the target frame (probably the frame specified by
      //! \p odomFrameName_), (2)  "subtract" the previous measurement from
      //! the current measurement, and then (3) transform it again by the previous
      //! state estimate. This holds the measurements used for step (3).
      //!
      std::map<std::string, tf::Transform> previousStates_;

      //! @brief Vector to hold our subscriber objects so they don't go out
      //! of scope.
      //!
      std::vector<ros::Subscriber> topicSubs_;

      //! @brief Subscribes to the set_pose topic (usually published from rviz) - a geometry_msgs/PoseWithCovarianceStamped
      //!
      ros::Subscriber setPoseSub_;

      //! @brief Node handle
      //!
      ros::NodeHandle nh_;

      //! @brief Local node handle (for params)
      //!
      ros::NodeHandle nhLocal_;

      //! @brief Message that contains our latest transform (i.e., state)
      //!
      //! We use the vehicle's latest state in a number of places, and often
      //! use it as a transform, so this is the most convenient variable to
      //! use as our global "current state" object
      //!
      geometry_msgs::TransformStamped odomTrans_;

      //! @brief Transform listener for managing coordinate transforms
      //!
      tf::TransformListener tfListener_;

      //! @brief Used for outputting debug messages
      //!
      std::ofstream debugStream_;

  };
}

#endif
