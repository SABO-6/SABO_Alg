//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <memory>
#include <vector>
#include <std_msgs/Int64.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"
#include <ocs2_msgs/mode_schedule.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class LeggedController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  LeggedController() = default;
  ~LeggedController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }
  void kpKdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void publishKpKdTotalDiff();
  void changeModeBaysian(const ocs2_msgs::mode_schedule::ConstPtr& msg);
  void publishModeChange();
  // void callback(const std_msgs::String::ConstPtr& msg);

 protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);
  
  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  ros::Publisher observationPublisher_;

  // Subcribe and Publish
  ros::Subscriber kpKdSubscriber_;
  ros::Publisher kpKdTotalDiffPublisher_;
  ros::Subscriber modeSubscriber_;
  ros::Publisher modeChangePublisher_;

  ros::Subscriber subscriber;
  ros::Publisher publisher;

  ros::Time collection_start_time_;
  bool collecting_ = false;
  double collected_kp_;
  double collected_kd_;
  int mode = 12;
  

  std::mutex data_mutex_;

  double kp_ = 4;
  double kd_ = 4;
  double total_diff_ = 6;
  double max_diff_ = 0;
  

 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
};

class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace legged
