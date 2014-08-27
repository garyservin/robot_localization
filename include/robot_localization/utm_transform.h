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

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace robot_localization {
  class UtmTransform {
    public:
      UtmTransform();
      ~UtmTransform();

      //! @brief Computes the transform from the odometry message's frame_id
      //! to the UTM message's frame_id
      //!
      //! The transform is computed from the odom frame to the UTM frame so that
      //! we don't accidentally attempt to give a second parent to the odom frame
      //! in the tf tree. Users may already have a parent for that frame_id, and
      //! we don't want to stomp on it.
      //!
      void computeOdomUtmTransform();

      //! @brief Computes the transform and, if available, publishes it
      //!
      //! Returns true iff the transform is available and was published
      bool publishTransform();

    private:
      //! @brief The TF broadcaster to publish our transform
      tf::TransformBroadcaster broadcaster_;

      //! @brief The utm_frame->odom_frame transform object
      tf::StampedTransform *odomUtmTransform_;

      //! @brief Whether or not we've computed a good heading
      bool transformGood_;

      //! @brief Used for calculating the utm_frame->odom_frame transform
      nav_msgs::Odometry *latestUtmMsg_;
      nav_msgs::Odometry *latestOdomMessage_;
      sensor_msgs::Imu *latestImuMsg_;

      //! @brief TF listener used for converting
      tf::TransformListener *tfListener_;

      //! @brief Used for updating the transform so as to
      //! remove altitude from the UTM measurements.
      tf::Pose originalOdomPose_;

      //! @brief Used for updating the transform so as to
      //! remove altitude from the UTM measurements.
      tf::Pose originalUtmPose_;

      //! @brief Parameter that specifies the magnetic decliation for the robot's
      //! environment.
      double magneticDeclination_;

      //! @brief Stores the roll we need to compute the transform
      double utmOdomTfRoll_;

      //! @brief Stores the pitch we need to compute the transform
      double utmOdomTfPitch_;

      //! @brief Stores the yaw we need to compute the transform
      double utmOdomTfYaw_;

      //! @brief Whether or not the GPS fix is usable
      bool hasFix_;

      //! @brief On level ground, your IMU should read 0 roll. If
      //! it doesn't, this (parameterized) value gives the offset
      double rollOffset_;

      //! @brief On level ground, your IMU should read 0 pitch. If
      //! it doesn't, this (parameterized) value gives the offset
      double pitchOffset_;

      //! @brief Your IMU should read 0 when facing *magnetic* north. If it
      //! doesn't, this (parameterized) value gives the offset (NOTE: if you
      //! have a magenetic declination, use the parameter setting for that).
      double yawOffset_;

      //! @brief If this parameter is true, we continually update the transform
      //! using the current altitude from the UTM message. This allows users to
      //! receive a (nearly) zero Z measurement when they use the transform.
      bool zeroAltitude_;

      //! @brief Callback for the UTM data
      //!
      void utmCallback(const nav_msgs::OdometryConstPtr& msg);

      //! @brief Callback for the IMU data
      //!
      void imuCallback(const sensor_msgs::ImuConstPtr& msg);

      //! @brief Callback for the odom data
      //!
      void odomCallback(const nav_msgs::OdometryConstPtr& msg);

      //! @brief Callback for the GPS fix data
      //!
      void gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg);


      //! @brief subscribers
      ros::Subscriber utmSub;
      ros::Subscriber imuSub;
      ros::Subscriber odomSub;
      ros::Subscriber gpsFixSub;
  };
};
