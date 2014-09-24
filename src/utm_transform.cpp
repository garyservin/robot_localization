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

#include <robot_localization/utm_transform.h>

namespace robot_localization {
  //! @brief Computes the transform from the odometry message's frame_id
  //! to the UTM message's frame_id
  //!
  //! The transform is computed from the odom frame to the UTM frame so that
  //! we don't accidentally attempt to give a second parent to the odom frame
  //! in the tf tree. Users may already have a parent for that frame_id, and
  //! we don't want to stomp on it.
  //!
  void UtmTransform::computeOdomUtmTransform()
  {
    // Only do this if:
    // 1. We haven't computed the odom_frame->utm_frame transform before
    // 2. We've received the messages we need already
    // 3. We have good GPS data
    if(!transformGood_ &&
       latestUtmMsg_ != NULL &&
       latestImuMsg_ != NULL &&
       latestOdomMessage_ != NULL &&
       hasFix_)
    {
      if(!std::isnan(latestUtmMsg_->pose.pose.position.x) &&
         !std::isnan(latestUtmMsg_->pose.pose.position.y) &&
         !std::isnan(latestUtmMsg_->pose.pose.position.z))
      {
        ROS_INFO_STREAM("Computing initial " << latestOdomMessage_->header.frame_id << "->" << latestUtmMsg_->header.frame_id << " transform");

        // Now that we have what we need, create the transform so it will be broadcast
        if(odomUtmTransform_ == NULL)
        {
          odomUtmTransform_ = new tf::StampedTransform();
        }

        // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
        tf::Matrix3x3 mat(tf::Quaternion(latestImuMsg_->orientation.x,
                                         latestImuMsg_->orientation.y,
                                         latestImuMsg_->orientation.z,
                                         latestImuMsg_->orientation.w));

        // Convert to RPY
        mat.getRPY(utmOdomTfRoll_, utmOdomTfPitch_, utmOdomTfYaw_);

        ROS_INFO_STREAM("Latest IMU orientation was: (" << std::fixed << utmOdomTfRoll_ << ", " << utmOdomTfPitch_ << ", " << utmOdomTfYaw_ << ")");

        // Compute the final yaw value that corrects for the difference between the
        // IMU's heading and the UTM grid's belief of where 0 heading should be (i.e.,
        // along the x-axis)
        utmOdomTfYaw_ += (magneticDeclination_ + yawOffset_ + (M_PI / 2.0));
        utmOdomTfPitch_ += pitchOffset_;
        utmOdomTfRoll_ += rollOffset_;

        ROS_INFO_STREAM("Corrected for magnetic declination of " << std::fixed << magneticDeclination_ <<
                        ", user-specified offset of " << yawOffset_ << ", and fixed offset of " << (M_PI / 2.0) <<
                        ". Transform heading factor is now " << utmOdomTfYaw_);

        // Convert to tf-friendly structures
        tf::Quaternion quat;
        quat.setRPY(utmOdomTfRoll_, utmOdomTfPitch_, utmOdomTfYaw_);
        tf::Vector3 pos;
        tf::pointMsgToTF(latestUtmMsg_->pose.pose.position, pos);

        // Put the transform together
        odomUtmTransform_->frame_id_ = latestOdomMessage_->header.frame_id;
        odomUtmTransform_->child_frame_id_ = latestUtmMsg_->header.frame_id;
        odomUtmTransform_->setRotation(quat);
        odomUtmTransform_->setOrigin(pos);

        ROS_INFO_STREAM("Before correcttion, " << odomUtmTransform_->frame_id_  << "->" <<
                        odomUtmTransform_->child_frame_id_ << " transform is: " << std::fixed <<
                        "\nPosition: (" << odomUtmTransform_->getOrigin().getX() << ", " <<
                                           odomUtmTransform_->getOrigin().getY() << ", " <<
                                           odomUtmTransform_->getOrigin().getZ() << ")" <<
                        "\nOrientation: (" << utmOdomTfRoll_ << ", " <<
                                              utmOdomTfPitch_ << ", " <<
                                              utmOdomTfYaw_ << ")");

        // If we started in a location without GPS (or with poor GPS), then we could be at some non-zero
        // (x, y) location in the odomFrame_ frame. For that reason, we need to move this transform to the
        // origin. To do that, we need to figure out what our odometry origin (the inverse
        // of our position) is in the UTM frame.

        // Convert the pose to a tf object
        tf::Pose odomPose;
        tf::poseMsgToTF(latestOdomMessage_->pose.pose, odomPose);

        // The transform order will be orig_odom_pos * orig_utm_pos_inverse * cur_utm_pos
        // Store these values so we can change and re-compose the transform later.
        originalOdomPose_ = odomPose;
        originalUtmPose_ = *odomUtmTransform_;
        odomUtmTransform_->mult(originalOdomPose_, originalUtmPose_.inverse());

        double odomRoll;
        double odomPitch;
        double odomYaw;
        double utmRoll;
        double utmPitch;
        double utmYaw;

        mat.setRotation(odomPose.getRotation());
        mat.getRPY(odomRoll, odomPitch, odomYaw);

        ROS_INFO_STREAM("Latest " << latestOdomMessage_->header.frame_id << " pose is: " << std::fixed <<
                        "\nPosition: (" << odomPose.getOrigin().getX() << ", " <<
                                           odomPose.getOrigin().getY() << ", " <<
                                           odomPose.getOrigin().getZ() << ")" <<
                        "\nOrientation: (" << odomRoll << ", " <<
                                              odomPitch << ", " <<
                                              odomYaw << ")");

        mat.setRotation(odomUtmTransform_->getRotation());
        mat.getRPY(utmRoll, utmPitch, utmYaw);

        ROS_INFO_STREAM(odomUtmTransform_->frame_id_  << "->" << odomUtmTransform_->child_frame_id_ <<
                         " transform is now: " << std::fixed <<
                         "\nPosition: (" << odomUtmTransform_->getOrigin().getX() << ", " <<
                                            odomUtmTransform_->getOrigin().getY() << ", " <<
                                            odomUtmTransform_->getOrigin().getZ() << ")" <<
                         "\nOrientation: (" << utmRoll << ", " <<
                                               utmPitch << ", " <<
                                               utmYaw << ")");

        transformGood_ = true;
      }
    }
  }

  //! @brief Callback for the UTM data
  //!
  void UtmTransform::utmCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    //HACK [tulku] - Always asume 0 altitude when transforming utm to nav.
    nav_msgs::Odometry hack;
    hack = *msg;
    hack.pose.pose.position.z = 0;

    if(latestUtmMsg_ == NULL)
    {
      ROS_INFO("Received initial UTM message");

      latestUtmMsg_ = new nav_msgs::Odometry();
    }

    *latestUtmMsg_ = hack;

    if(zeroAltitude_ && odomUtmTransform_ != NULL)
    {
      // Grab the original transforms, update the z position
      // in the original UTM transform, and then put it back
      // together.
      tf::Vector3 origin = originalUtmPose_.getOrigin();
      origin.setZ(hack.pose.pose.position.z);
      originalUtmPose_.setOrigin(origin);

      odomUtmTransform_->mult(originalOdomPose_, originalUtmPose_.inverse());
    }
  }

  //! @brief Callback for the IMU data
  //!
  void UtmTransform::imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    if(latestImuMsg_ == NULL)
    {
      ROS_INFO("Received initial IMU message");

      latestImuMsg_ = new sensor_msgs::Imu();
    }

    *latestImuMsg_ = *msg;
  }

  //! @brief Callback for the odom data
  //!
  void UtmTransform::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    if(latestOdomMessage_ == NULL)
    {
      ROS_INFO("Received initial relay odometry message");

      latestOdomMessage_ = new nav_msgs::Odometry();
    }

    *latestOdomMessage_ = *msg;
  }

  //! @brief Callback for the GPS fix data
  //!
  void UtmTransform::gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    hasFix_ = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
               !std::isnan(msg->altitude) &&
               !std::isnan(msg->latitude) &&
               !std::isnan(msg->longitude));
  }

  bool UtmTransform::publishTransform()
  {
    // No harm done if the transform had been already calculated: it will
    // be skipped
    computeOdomUtmTransform();

    if(transformGood_)
    {
      odomUtmTransform_->stamp_ = ros::Time::now();
      broadcaster_.sendTransform(*odomUtmTransform_);
    }

    return transformGood_;
  }

  UtmTransform::UtmTransform()
  {
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    latestUtmMsg_ = NULL;
    latestImuMsg_ = NULL;
    latestOdomMessage_ = NULL;
    odomUtmTransform_ = NULL;
    tfListener_ = new tf::TransformListener();

    magneticDeclination_ = 0;
    utmOdomTfYaw_ = 0;
    rollOffset_ = 0;
    pitchOffset_ = 0;
    yawOffset_ = 0;
    hasFix_ = false;
    transformGood_ = false;

    // Subscribe to the messages we need
    utmSub = nh.subscribe<nav_msgs::Odometry>("gps/gps_utm", 10, 
        boost::bind(&UtmTransform::utmCallback, this, _1));
    imuSub = nh.subscribe<sensor_msgs::Imu>("imu/data", 10, 
        boost::bind(&UtmTransform::imuCallback, this, _1));
    odomSub = nh.subscribe<nav_msgs::Odometry>("odometry/filtered", 10, 
        boost::bind(&UtmTransform::odomCallback, this, _1));
    gpsFixSub = nh.subscribe<sensor_msgs::NavSatFix>("gps/fix", 10,
        boost::bind(&UtmTransform::gpsFixCallback, this, _1));

    // Load the parameters we need
    nhPriv.getParam("magnetic_declination_radians", magneticDeclination_);
    nhPriv.getParam("roll_offset", rollOffset_);
    nhPriv.getParam("pitch_offset", pitchOffset_);
    nhPriv.getParam("yaw_offset", yawOffset_);
    nhPriv.param("zero_altitude", zeroAltitude_, false);
  }

  UtmTransform::~UtmTransform()
  {
    delete latestUtmMsg_;
    delete latestImuMsg_;
    delete latestOdomMessage_;
    delete odomUtmTransform_;
  }
}
