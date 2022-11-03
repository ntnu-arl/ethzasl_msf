/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef SENSORMANAGERROS_H
#define SENSORMANAGERROS_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// Message includes.
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#include <sensor_fusion_comm/DoubleMatrixStamped.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_fusion_comm/ExtEkf.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include <msf_core/MSF_CoreConfig.h>
#include <msf_core/msf_sensormanager.h>
#include <msf_core/msf_types.h>

namespace msf_core {

enum {
  ///< Number of states exchanged with external propagation.
  // Here: p, v, q, bw, bw = 16
  HLI_EKF_STATE_SIZE = 16
};

typedef dynamic_reconfigure::Server<msf_core::MSF_CoreConfig> ReconfigureServer;

/** \class MSF_SensorManagerROS
 * \brief Abstract class defining user configurable calculations for the msf_core
 * with ROS interfaces.
 */
template<typename EKFState_T>
struct MSF_SensorManagerROS : public msf_core::MSF_SensorManager<EKFState_T> {
 protected:
  msf_core::MSF_CoreConfig config_;  ///< Dynamic reconfigure config.
 private:

  typedef typename EKFState_T::StateDefinition_T StateDefinition_T;
  typedef typename EKFState_T::StateSequence_T StateSequence_T;

  ReconfigureServer *reconfServer_;  ///< Dynamic reconfigure server.

  ros::Publisher pubState_;  ///< Publishes all states of the filter.
  ros::Publisher pubPose_;  ///< Publishes 6DoF pose output.
  ros::Publisher pubOdometry_;  ///< Publishes odometry output.
  ros::Publisher pubOdometryVN100_; ///< Publishes odometry output for VN100.
  ros::Publisher pubMaplabOdometry_;  ///< Publishes odometry output.
  ros::Publisher pubPoseAfterUpdate_;  ///< Publishes 6DoF pose output after the update has been applied.
  ros::Publisher pubPoseCrtl_;  ///< Publishes 6DoF pose including velocity output.
  ros::Publisher pubCorrect_;  ///< Publishes corrections for external state propagation.
  ros::Publisher pubCovCore_;  ///< Publishes the covariance matrix for the core states.
  ros::Publisher pubCovAux_;  ///< Publishes the covariance matrix for the auxiliary states.
  ros::Publisher pubCovCoreAux_; ///< Publishes the covariance matrix for the cross-correlations between core and auxiliary states.

  std::string msf_output_frame_;

  mutable tf::TransformBroadcaster tf_broadcaster_;

  Eigen::Vector3d vn100_px4_pos_;
  Eigen::Quaterniond vn100_px4_quat_;

  sensor_fusion_comm::ExtEkf hl_state_buf_;  ///< Buffer to store external propagation data.

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MSF_SensorManagerROS(ros::NodeHandle pnh = ros::NodeHandle("~core")) {
    reconfServer_ = new ReconfigureServer(pnh);
    ReconfigureServer::CallbackType f = boost::bind(
        &MSF_SensorManagerROS::Config, this, _1, _2);
    reconfServer_->setCallback(f);

    pnh.param("data_playback", this->data_playback_, false);
    pnh.param("msf_output_frame", msf_output_frame_, std::string("world"));
    std::vector<double> vn100_px4_pos, vn100_px4_quat;
    std::vector<double> vn100_px4_pos_def = {0.00376, 0.05, -0.0416}, vn100_px4_quat_def = {0.0, 0.0, 0.0, 1.0};
    if(pnh.param("vn100_px4_tf/translation", vn100_px4_pos, vn100_px4_pos_def) && vn100_px4_pos.size() == 3) {
      vn100_px4_pos_ << vn100_px4_pos[0], vn100_px4_pos[1], vn100_px4_pos[2];
      std::cout << "---------------- vn100 px4 trans: " << vn100_px4_pos_.transpose() << "-----------" << std::endl;
    }
    else {
      vn100_px4_pos_ << vn100_px4_quat_def[0], vn100_px4_quat_def[1], vn100_px4_quat_def[2];
      std::cout << "---------------- NOT FOUND vn100 px4 trans: " << vn100_px4_pos_.transpose() << "-----------" << std::endl;
    }
    // pnh.param("vn100_px4_tf/rotation", vn100_px4_quat, vn100_px4_quat_def);
    if(pnh.param("vn100_px4_tf/rotation", vn100_px4_quat, vn100_px4_quat_def) && vn100_px4_quat.size() == 4) {
      // vn100_px4_quat_(vn100_px4_quat[3], vn100_px4_quat[0], vn100_px4_quat[1], vn100_px4_quat[2]);
      vn100_px4_quat_.x() = vn100_px4_quat[0];
      vn100_px4_quat_.y() = vn100_px4_quat[1];
      vn100_px4_quat_.z() = vn100_px4_quat[2];
      vn100_px4_quat_.w() = vn100_px4_quat[3];
      std::cout << "---------------- vn100 px4 rot: " << vn100_px4_quat_.x() << ", " << vn100_px4_quat_.y() << ", " << vn100_px4_quat_.z() << ", " << vn100_px4_quat_.w() << "-----------" << std::endl;
    }
    else {
      // vn100_px4_quat_(vn100_px4_quat_def[3], vn100_px4_quat_def[0], vn100_px4_quat_def[1], vn100_px4_quat_def[2]);
      vn100_px4_quat_.x() = vn100_px4_quat_def[0];
      vn100_px4_quat_.y() = vn100_px4_quat_def[1];
      vn100_px4_quat_.z() = vn100_px4_quat_def[2];
      vn100_px4_quat_.w() = vn100_px4_quat_def[3];
      std::cout << "---------------- NOT FOUND vn100 px4 rot: " << vn100_px4_quat_.x() << ", " << vn100_px4_quat_.y() << ", " << vn100_px4_quat_.z() << ", " << vn100_px4_quat_.w() << "-----------" << std::endl;
    }



    ros::NodeHandle nh("msf_core");

    pubState_ = nh.advertise < sensor_fusion_comm::DoubleArrayStamped
        > ("state_out", 100);
    pubCorrect_ = nh.advertise < sensor_fusion_comm::ExtEkf > ("correction", 1);
    pubPose_ = nh.advertise < geometry_msgs::PoseWithCovarianceStamped
        > ("pose", 100);
    pubOdometry_ = nh.advertise < nav_msgs::Odometry> ("odometry", 100);
    pubOdometryVN100_ = nh.advertise < nav_msgs::Odometry> ("VN100_odometry", 100);
    pubPoseAfterUpdate_ = nh.advertise
        < geometry_msgs::PoseWithCovarianceStamped > ("pose_after_update", 100);
    pubPoseCrtl_ = nh.advertise < sensor_fusion_comm::ExtState
        > ("ext_state", 1);
    pubCovCore_ = nh.advertise<sensor_fusion_comm::DoubleMatrixStamped>(
        "cov_core", 10);
    pubCovAux_ = nh.advertise<sensor_fusion_comm::DoubleMatrixStamped>(
        "cov_aux", 10);
    pubCovCoreAux_ = nh.advertise<sensor_fusion_comm::DoubleMatrixStamped>(
        "cov_core_aux", 10);

    hl_state_buf_.state.resize(HLI_EKF_STATE_SIZE, 0);

    // Print published/subscribed topics.
    ros::V_string topics;
    ros::this_node::getSubscribedTopics(topics);
    std::string nodeName = ros::this_node::getName();
    std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
    for (unsigned int i = 0; i < topics.size(); i++)
      topicsStr += ("\t\t" + topics.at(i) + "\n");

    topicsStr += "\tadvertised topics:\n";
    ros::this_node::getAdvertisedTopics(topics);
    for (unsigned int i = 0; i < topics.size(); i++)
      topicsStr += ("\t\t" + topics.at(i) + "\n");

    MSF_INFO_STREAM(""<< topicsStr);
  }

  virtual ~MSF_SensorManagerROS() {
    delete reconfServer_;
  }

  /**
   * \brief Gets called by the internal callback caller.
   */
  virtual void CoreConfigCallback(msf_core::MSF_CoreConfig &config, uint32_t level) {
    UNUSED(config);
    UNUSED(level);
  }

  /**
   * \brief Gets called by dynamic reconfigure and calls all registered
   * callbacks in callbacks_.
   */
  void Config(msf_core::MSF_CoreConfig &config, uint32_t level) {
    config_ = config;
    CoreConfigCallback(config, level);
  }

  // Set the latest HL state which is needed to compute the correction to be
  // send back to the HL.
  void SetHLControllerStateBuffer(const sensor_fusion_comm::ExtEkf& msg) {
    hl_state_buf_ = msg;
  }

  // Parameter getters.
  virtual bool GetParamFixedBias() const {
    return config_.core_fixed_bias;
  }
  virtual double GetParamNoiseAcc() const {
    return config_.core_noise_acc;
  }
  virtual double GetParamNoiseAccbias() const {
    return config_.core_noise_accbias;
  }
  virtual double GetParamNoiseGyr() const {
    return config_.core_noise_gyr;
  }
  virtual double GetParamNoiseGyrbias() const {
    return config_.core_noise_gyrbias;
  }
  virtual double GetParamFuzzyTrackingThreshold() const {
    return 0.1;
  }
  virtual void PublishStateInitial(const shared_ptr<EKFState_T>& state) const {
    /**
     * \brief Initialize the HLP based propagation.
     * \param State the state to send to the HLP.
     */
    sensor_fusion_comm::ExtEkf msgCorrect_;
    msgCorrect_.state.resize(HLI_EKF_STATE_SIZE, 0);
    msgCorrect_.header.stamp = ros::Time(state->time);
    msgCorrect_.header.seq = 0;
    msgCorrect_.angular_velocity.x = 0;
    msgCorrect_.angular_velocity.y = 0;
    msgCorrect_.angular_velocity.z = 0;
    msgCorrect_.linear_acceleration.x = 0;
    msgCorrect_.linear_acceleration.y = 0;
    msgCorrect_.linear_acceleration.z = 0;

    msgCorrect_.state.resize(HLI_EKF_STATE_SIZE);
    boost::fusion::for_each(
        state->statevars,
        msf_tmp::CoreStatetoDoubleArray<std::vector<float>, StateSequence_T>(
            msgCorrect_.state));

    msgCorrect_.flag = sensor_fusion_comm::ExtEkf::initialization;
    pubCorrect_.publish(msgCorrect_);
  }

  virtual void PublishStateAfterPropagation(
      const shared_ptr<EKFState_T>& state) const {

    if (pubPoseCrtl_.getNumSubscribers() || pubPose_.getNumSubscribers() 
        || pubOdometry_.getNumSubscribers() 
        || pubMaplabOdometry_.getNumSubscribers()) {
      static int msg_seq = 0;

      geometry_msgs::PoseWithCovarianceStamped msgPose;
      msgPose.header.stamp = ros::Time(state->time);
      msgPose.header.seq = msg_seq++;
      msgPose.header.frame_id = msf_output_frame_;
      state->ToPoseMsg(msgPose);
      pubPose_.publish(msgPose);
      sensor_fusion_comm::ExtState msgPoseCtrl;
      msgPoseCtrl.header = msgPose.header;
      state->ToExtStateMsg(msgPoseCtrl);
      pubPoseCrtl_.publish(msgPoseCtrl);

      nav_msgs::Odometry msgOdometry;
      msgOdometry.header.stamp = ros::Time(state->time);
      msgOdometry.header.seq = msg_seq++;
      msgOdometry.header.frame_id = msf_output_frame_;
      msgOdometry.child_frame_id = "vn100_imu";
      state->ToOdometryMsg(msgOdometry);
      pubOdometryVN100_.publish(msgOdometry);

      /* TODO: */
      // msf_core::Vector3 P_vn_px4(0.00376, 0.05, -0.0416);
      // msf_core::Matrix3 R_vn_px4 = Eigen::Matrix3d::Identity();
      msf_core::Vector3 P_vn_px4(vn100_px4_pos_(0), vn100_px4_pos_(1), vn100_px4_pos_(2));
      msf_core::Matrix3 R_vn_px4 = vn100_px4_quat_.toRotationMatrix();

      // std::cout << "In the function: " << std::endl;
      // std::cout << P_vn_px4.transpose() << std::endl;
      // std::cout << R_vn_px4 << std::endl;

      msf_core::Matrix4 T_vn_px4 = Eigen::Matrix4d::Identity();
      T_vn_px4.block<3,3>(0,0) = R_vn_px4;
      T_vn_px4.block<3,1>(0,3) = P_vn_px4;

      tf::Transform transform;
      const EKFState_T& state_const = *state;
      const Eigen::Matrix<double, 3, 1>& state_pos = state_const
          .template Get<StateDefinition_T::p>();
      const Eigen::Quaterniond& state_ori = state_const
          .template Get<StateDefinition_T::q>();
      
      msf_core::Matrix4 T_world_vn;
      T_world_vn.col(3) << state_pos, 1.0;

      msf_core::Quaternion q_w_vn = state_ori;
      msf_core::Matrix3 rot_mat = q_w_vn.toRotationMatrix();
      T_world_vn.block<3,3>(0,0) = rot_mat;

      msf_core::Vector3 v_vn_vn;
      v_vn_vn << msgOdometry.twist.twist.linear.x, msgOdometry.twist.twist.linear.y, msgOdometry.twist.twist.linear.z;

      msf_core::Vector3 w_vn_vn;
      w_vn_vn << msgOdometry.twist.twist.angular.x, msgOdometry.twist.twist.angular.y, msgOdometry.twist.twist.angular.z;

      msf_core::Matrix4 T_world_px4 = T_world_vn * T_vn_px4;

      msf_core::Vector3 p_w_px4 = T_world_px4.block<3, 1>(0,3);
      msf_core::Quaternion q_w_px4(T_world_px4.block<3,3>(0,0));

      msf_core::Vector3 v_px4_vn = v_vn_vn + w_vn_vn.cross(P_vn_px4);
      msf_core::Vector3 v_px4_px4 = R_vn_px4.inverse() * v_px4_vn;
      msf_core::Vector3 w_px4_px4 = R_vn_px4.inverse() * w_vn_vn;


      nav_msgs::Odometry msgOdometryPX4Eagle;
      msgOdometryPX4Eagle.header.stamp = ros::Time(state->time);
      msgOdometryPX4Eagle.header.seq = msg_seq;
      msgOdometryPX4Eagle.header.frame_id = msf_output_frame_;
      msgOdometryPX4Eagle.child_frame_id = "imu";
      eigen_conversions::Vector3dToPoint(p_w_px4, msgOdometryPX4Eagle.pose.pose.position);
      eigen_conversions::Vector3dToPoint(v_px4_px4, msgOdometryPX4Eagle.twist.twist.linear);
      eigen_conversions::Vector3dToPoint(w_px4_px4, msgOdometryPX4Eagle.twist.twist.angular);

      eigen_conversions::QuaternionToMsg(q_w_px4, msgOdometryPX4Eagle.pose.pose.orientation);
      msgOdometryPX4Eagle.pose.covariance = msgOdometry.pose.covariance;
      msgOdometryPX4Eagle.twist.covariance = msgOdometry.twist.covariance;

      pubOdometry_.publish(msgOdometryPX4Eagle);

    }
  }

  virtual void PublishStateAfterUpdate(
      const shared_ptr<EKFState_T>& state) const {
    static int msg_seq = 0;

    sensor_fusion_comm::ExtEkf msgCorrect_;
    msgCorrect_.state.resize(HLI_EKF_STATE_SIZE);
    msgCorrect_.header.stamp = ros::Time(state->time);
    msgCorrect_.header.seq = msg_seq;
    msgCorrect_.angular_velocity.x = 0;
    msgCorrect_.angular_velocity.y = 0;
    msgCorrect_.angular_velocity.z = 0;
    msgCorrect_.linear_acceleration.x = 0;
    msgCorrect_.linear_acceleration.y = 0;
    msgCorrect_.linear_acceleration.z = 0;

    // Prevent junk being sent to the external state propagation when data
    // playback is (accidentally) on.
    if (pubCorrect_.getNumSubscribers() > 0) {
      if (this->data_playback_) {
        for (int i = 0; i < HLI_EKF_STATE_SIZE; ++i) {
          msgCorrect_.state[i] = 0;
        }
        msgCorrect_.state[6] = 1;  //w
        msgCorrect_.flag = sensor_fusion_comm::ExtEkf::initialization;

        MSF_ERROR_STREAM_THROTTLE(
            1, __FUNCTION__ << " You have connected the external propagation "
            "topic but at the same time data_playback is on.");
      } else {
        const EKFState_T& state_const = *state;
        msgCorrect_.state[0] =
            state_const.template Get<StateDefinition_T::p>()[0]
                - hl_state_buf_.state[0];
        msgCorrect_.state[1] =
            state_const.template Get<StateDefinition_T::p>()[1]
                - hl_state_buf_.state[1];
        msgCorrect_.state[2] =
            state_const.template Get<StateDefinition_T::p>()[2]
                - hl_state_buf_.state[2];
        msgCorrect_.state[3] =
            state_const.template Get<StateDefinition_T::v>()[0]
                - hl_state_buf_.state[3];
        msgCorrect_.state[4] =
            state_const.template Get<StateDefinition_T::v>()[1]
                - hl_state_buf_.state[4];
        msgCorrect_.state[5] =
            state_const.template Get<StateDefinition_T::v>()[2]
                - hl_state_buf_.state[5];

        Eigen::Quaterniond hl_q(hl_state_buf_.state[6], hl_state_buf_.state[7],
                                hl_state_buf_.state[8], hl_state_buf_.state[9]);
        Eigen::Quaterniond qbuff_q = hl_q.inverse()
            * state_const.template Get<StateDefinition_T::q>();
        msgCorrect_.state[6] = qbuff_q.w();
        msgCorrect_.state[7] = qbuff_q.x();
        msgCorrect_.state[8] = qbuff_q.y();
        msgCorrect_.state[9] = qbuff_q.z();

        msgCorrect_.state[10] =
            state_const.template Get<StateDefinition_T::b_w>()[0]
                - hl_state_buf_.state[10];
        msgCorrect_.state[11] =
            state_const.template Get<StateDefinition_T::b_w>()[1]
                - hl_state_buf_.state[11];
        msgCorrect_.state[12] =
            state_const.template Get<StateDefinition_T::b_w>()[2]
                - hl_state_buf_.state[12];
        msgCorrect_.state[13] =
            state_const.template Get<StateDefinition_T::b_a>()[0]
                - hl_state_buf_.state[13];
        msgCorrect_.state[14] =
            state_const.template Get<StateDefinition_T::b_a>()[1]
                - hl_state_buf_.state[14];
        msgCorrect_.state[15] =
            state_const.template Get<StateDefinition_T::b_a>()[2]
                - hl_state_buf_.state[15];

        msgCorrect_.flag = sensor_fusion_comm::ExtEkf::state_correction;
      }

      if (state->CheckStateForNumeric()) {  // If not NaN.
        pubCorrect_.publish(msgCorrect_);
      } else {
        MSF_WARN_STREAM_THROTTLE(
            1, "Not sending updates to external EKF, because state NaN/inf.");
      }
    }

    // Publish state.
    sensor_fusion_comm::DoubleArrayStamped msgState;
    msgState.header = msgCorrect_.header;
    state->ToFullStateMsg(msgState);
    pubState_.publish(msgState);

    if (pubPoseAfterUpdate_.getNumSubscribers()) {
      // Publish pose after correction with covariance.
      geometry_msgs::PoseWithCovarianceStamped msgPose;
      msgPose.header.stamp = ros::Time(state->time);
      msgPose.header.seq = msg_seq;
      msgPose.header.frame_id = msf_output_frame_;

      state->ToPoseMsg(msgPose);
      pubPoseAfterUpdate_.publish(msgPose);
    }

    {
      tf::Transform transform;
      const EKFState_T& state_const = *state;
      const Eigen::Matrix<double, 3, 1>& pos = state_const
          .template Get<StateDefinition_T::p>();
      const Eigen::Quaterniond& ori = state_const
          .template Get<StateDefinition_T::q>();
      transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
      transform.setRotation(tf::Quaternion(ori.x(), ori.y(), ori.z(), ori.w()));
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(
              transform, ros::Time::now() /*ros::Time(latestState->time_)*/,
              msf_output_frame_, "vn100_state"));
    }

    if (pubCovCore_.getNumSubscribers()) {
      sensor_fusion_comm::DoubleMatrixStampedPtr msg(
          new sensor_fusion_comm::DoubleMatrixStamped);
      msg->header = msgCorrect_.header;
      state->GetCoreCovariance(*msg);
      pubCovCore_.publish(msg);
    }

    if (pubCovAux_.getNumSubscribers()) {
      sensor_fusion_comm::DoubleMatrixStampedPtr msg(
          new sensor_fusion_comm::DoubleMatrixStamped);
      msg->header = msgCorrect_.header;
      state->GetAuxCovariance(*msg);
      pubCovAux_.publish(msg);
    }

    if (pubCovCoreAux_.getNumSubscribers()) {
      sensor_fusion_comm::DoubleMatrixStampedPtr msg(
          new sensor_fusion_comm::DoubleMatrixStamped);
      msg->header = msgCorrect_.header;
      state->GetCoreAuxCovariance(*msg);
      pubCovCoreAux_.publish(msg);
    }
    msg_seq++;
  }
};

}
#endif  // SENSORMANAGERROS_H
