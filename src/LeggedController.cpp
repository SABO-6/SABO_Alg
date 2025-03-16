//
// Created by qiayuan on 2022/6/24.
//
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <std_msgs/String.h>

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>
#include <ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h>
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_target_trajectories.h>

namespace legged {
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;


  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  ROS_INFO_STREAM("Controller is first init and the value of kp is: " << kp_);
  // Subscriber and Publisher

  // std::string node_name = ros::this_node::getName();
  // std::cout << "Node name: " << node_name << std::endl;
  kpKdSubscriber_ = controller_nh.subscribe("kp_kd_opt", 10, &LeggedController::kpKdCallback, this);
  kpKdTotalDiffPublisher_ = controller_nh.advertise<std_msgs::Float64MultiArray>("kp_kd_total_diff_topic", 100);
  // modeSubscriber_ = nh.subscribe<ocs2_msgs::mode_schedule>("legged_robot_mode_schedule", 1, &LeggedController::changeModeBaysian, this);
  // modeChangePublisher_ = controller_nh.advertise<std_msgs::Int64>("modeChange", 1);

  
  // subscriber = controller_nh.subscribe("topic", 10, &LeggedController::callback, this);
  // publisher = controller_nh.advertise<std_msgs::String>("topic", 10);

  return true;
}

void LeggedController::starting(const ros::Time& time) {
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {

  // State Estimate
  updateStateEstimation(time, period);
  // ROS_WARN("%f",hybridJointHandles_[2].getPosition());

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // static ros::Time last_print_time = time;
  // if ((time - last_print_time).toSec() >= 5.0) {
  //   ROS_INFO_STREAM("Current State: " << currentObservation_.state.transpose());
  //   last_print_time = time;
  // }

  // static ros::Time last_print_time = time;


  // double loop_duration = (time - last_print_time).toSec();
  // std::cout << "Time of loop_duration is: " << loop_duration << std::endl;
  // std::cout << "  " << std::endl;
  // last_print_time = time;

  // currentObservation_.state =  
	// -0.00562846 0.00637223 0.0244746 -6.73047e-05 0.000402518 -0.000216588 -0.0208596 -0.00322217 0.310232 -0.0232997 0.00201754 3.9777e-05 -0.132052 0.907906 -2.03426 -0.173705 0.768157 -1.58238 0.115971 0.723038 -1.56741 0.213382 0.949764 -2.05404
	// optimizedState =  
	// -0.00484025 0.00631556 0.0217619 -7.22362e-05 0.000200963 -0.000221127 -0.0208627 -0.00322072 0.310213 -0.0232636 0.00204426 4.37659e-05 -0.132112 0.906497 -2.03132 -0.173716 0.768052 -1.58237 0.11595 0.723029 -1.56754 0.213348 0.948188 -2.05083

  // Whole body control
  currentObservation_.input = optimizedInput;

  wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
  wbcTimer_.endTimer();

  vector_t torque = x.tail(12);

  vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  // std::cout << "optimizedState type: " << typeid(optimizedState).name() << std::endl;
  // std::cout << "currentObservation_.state type: " << typeid(currentObservation_.state).name() << std::endl;
  // std::cout << "optimizedState size: " << optimizedState.size() << std::endl;
  // std::cout << "currentObservation_.state size: " << currentObservation_.state.size() << std::endl;

  // static ros::Time collection_start_time = time;

  // if ((time - collection_start_time).toSec() < 2.0) {
  //     double diff = (optimizedState - currentObservation_.state).cwiseAbs().sum();
  //     total_diff_ += diff;
  //     std::cout << "Used params, kp is: " << kp_ << " and kd is: " << kd_ << std::endl;
  //   } else {
  //     publishKpKdTotalDiff();
  //     total_diff_ = 0;
  //     collection_start_time = time;
  //     timer_active = false;
  //     // std::string node_name = ros::this_node::getName();
  //     // std::cout << "Node name: " << node_name << std::endl;
  //   }
  // total_diff_ += (optimizedState - currentObservation_.state).cwiseAbs().sum();

  // if (timer_active)
  // {
  //   ros::Duration time_diff = time - last_kp_kd_time_;

  //   if (time_diff.toSec() < 2.0) {
  //     double diff = (optimizedState - currentObservation_.state).cwiseAbs().sum();
  //     total_diff_ += diff;
  //     std::cout << "Used params, kp is: " << kp_ << " and kd is: " << kd_ << std::endl;
  //   } else {
  //     std::cout << "The total diff value of last 2 second is: " << total_diff_ << std::endl;

  //     total_diff_ = 0;
  //     collection_start_time = time;
  //     timer_active = false;
  //   }
  // }


  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }
  float joint_kp_list[12]={8.0, 3.5, 2.5, 8.0, 3.5, 2.5, 8.0, 3.5, 2.5, 8.0, 3.5, 2.5};
  float joint_kd_list[12]={7.0, 8.5, 7.0, 7.0, 8.5, 7.0, 7.0, 8.5, 7.0, 7.0, 8.5, 7.0};
  
  // float kp = 0;
  // float kd = 3;
  // std::cout << "current kp: " << collected_kp_ << " kd: " << collected_kd_ << std::endl;

  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    hybridJointHandles_[j].setCommand(posDes(j), velDes(j), collected_kp_, collected_kd_, torque(j));
    // hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 3.0, 5.5, torque(j));
  }


  // Visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));

  // std::lock_guard<std::mutex> lock(data_mutex_);
  // std::cout << "collecting: " << collecting_ << std::endl;

  if (collecting_) {
    double elapsed = (time - collection_start_time_).toSec();
    // std::cout << elapsed << std::endl;
    double diff = (optimizedState - currentObservation_.state).cwiseAbs().sum();
    // ROS_INFO_STREAM("optimizedState is:");
    // ROS_INFO_STREAM(optimizedState);
    // ROS_INFO_STREAM("posDes");
    // ROS_INFO_STREAM(posDes);
    // ROS_INFO_STREAM("velDes");
    // ROS_INFO_STREAM(velDes);
    total_diff_ += diff;
    if (diff > max_diff_) max_diff_ = diff;

    if (elapsed >= 2.0) {
      std::cout << "Used params, kp is: " << collected_kp_ << " and kd is: " << collected_kd_ << " the total diff is: " << total_diff_ << "the max diff is: " << max_diff_ << std::endl;
      
      publishKpKdTotalDiff();
      max_diff_ = 0;
      collecting_ = false;
    }
  }
}

// int LeggedController::checkMode(ModeSequenceTemplate tmp) {
//   double stance[2] = {0.0, 0.5};
//   double trot[3] = {0.0, 0.3, 0.6};
//   double standing_trot[5] = {0.0, 0.25, 0.3, 0.55, 0.6};
//   double flying_trot[5] = {0.0, 0.15, 0.2, 0.35, 0.4};
//   double pace[5] = {0.0, 0.28, 0.3, 0.58, 0.6};
//   double standing_pace[5] = {0.0, 0.3, 0.35, 0.65, 0.7};
//   double dynamic_walk[7] = {0.0, 0.2, 0.3, 0.5, 0.7, 0.8, 1.0};
//   double static_walk[5] = {0.0, 0.3, 0.6, 0.9, 1.2};
//   double amble[5] = {0.0, 0.15, 0.4, 0.55, 0.8};
//   double lindyhop[13] = {0.0, 0.35, 0.45, 0.8, 0.9, 1.125, 1.35, 1.7, 1.8, 2.025, 2.25, 2.6, 2.7};
//   double skipping[9] = {0.0, 0.27, 0.3, 0.57, 0.6, 0.87, 0.9, 1.17, 1.2};
//   double pawup[11] = {0.0, 2.0};

//   std::vector<double> Stance(std::begin(stance), std::end(stance));
//   std::vector<double> Trot(std::begin(trot), std::end(trot));
//   std::vector<double> Standing_trot(std::begin(standing_trot), std::end(standing_trot));
//   std::vector<double> Flying_trot(std::begin(flying_trot), std::end(flying_trot));
//   std::vector<double> Pace(std::begin(pace), std::end(pace));
//   std::vector<double> Standing_pace(std::begin(standing_pace), std::end(standing_pace));
//   std::vector<double> Dynamic_walk(std::begin(dynamic_walk), std::end(dynamic_walk));
//   std::vector<double> Static_walk(std::begin(static_walk), std::end(static_walk));
//   std::vector<double> Amble(std::begin(amble), std::end(amble));
//   std::vector<double> Lindyhop(std::begin(lindyhop), std::end(lindyhop));
//   std::vector<double> Skipping(std::begin(skipping), std::end(skipping));
//   std::vector<double> Pawup(std::begin(pawup), std::end(pawup));

//   int tmp_mode = 12;

//   if (tmp.switchingTimesInput == Stance)
//     tmp_mode = 0;
//   else if (tmp.switchingTimesInput == Trot)
//     tmp_mode = 1;
//   else if (tmp.switchingTimesInput == Standing_trot)
//     tmp_mode = 2;
//   else if (tmp.switchingTimesInput == Flying_trot)
//     tmp_mode = 3;
//   else if (tmp.switchingTimesInput == Pace)
//     tmp_mode = 4;
//   else if (tmp.switchingTimesInput == Standing_pace)
//     tmp_mode = 5;
//   else if (tmp.switchingTimesInput == Dynamic_walk)
//     tmp_mode = 6;
//   else if (tmp.switchingTimesInput == Static_walk)
//     tmp_mode = 7;
//   else if (tmp.switchingTimesInput == Amble)
//     tmp_mode = 8;
//   else if (tmp.switchingTimesInput == Lindyhop)
//     tmp_mode = 9;
//   else if (tmp.switchingTimesInput == Skipping)
//     tmp_mode = 10;
//   else
//     tmp_mode = 11;

//   return tmp_mode;
// }

void LeggedController::changeModeBaysian(const ocs2_msgs::mode_schedule::ConstPtr& msg) {

  // ROS_INFO("1111111111111111111111111111111111111111111111111111111111111111111111111111111");

  // // auto eventTimes = msg->eventTimes;
  // // auto modeSequence = msg->modeSequence;

  // ModeSequenceTemplate tmp = readModeSequenceTemplateMsg(*msg);

  // mode = checkMode(tmp);

  // double stance[2] = {0.0, 0.5};
  // double trot[3] = {0.0, 0.3, 0.6};
  // double standing_trot[5] = {0.0, 0.25, 0.3, 0.55, 0.6};
  // double flying_trot[5] = {0.0, 0.15, 0.2, 0.35, 0.4};
  // double pace[5] = {0.0, 0.28, 0.3, 0.58, 0.6};
  // double standing_pace[5] = {0.0, 0.3, 0.35, 0.65, 0.7};
  // double dynamic_walk[7] = {0.0, 0.2, 0.3, 0.5, 0.7, 0.8, 1.0};
  // double static_walk[5] = {0.0, 0.3, 0.6, 0.9, 1.2};
  // double amble[5] = {0.0, 0.15, 0.4, 0.55, 0.8};
  // double lindyhop[13] = {0.0, 0.35, 0.45, 0.8, 0.9, 1.125, 1.35, 1.7, 1.8, 2.025, 2.25, 2.6, 2.7};
  // double skipping[9] = {0.0, 0.27, 0.3, 0.57, 0.6, 0.87, 0.9, 1.17, 1.2};
  // double pawup[11] = {0.0, 2.0};

  // std::vector<double> Stance(std::begin(stance), std::end(stance));
  // std::vector<double> Trot(std::begin(trot), std::end(trot));
  // std::vector<double> Standing_trot(std::begin(standing_trot), std::end(standing_trot));
  // std::vector<double> Flying_trot(std::begin(flying_trot), std::end(flying_trot));
  // std::vector<double> Pace(std::begin(pace), std::end(pace));
  // std::vector<double> Standing_pace(std::begin(standing_pace), std::end(standing_pace));
  // std::vector<double> Dynamic_walk(std::begin(dynamic_walk), std::end(dynamic_walk));
  // std::vector<double> Static_walk(std::begin(static_walk), std::end(static_walk));
  // std::vector<double> Amble(std::begin(amble), std::end(amble));
  // std::vector<double> Lindyhop(std::begin(lindyhop), std::end(lindyhop));
  // std::vector<double> Skipping(std::begin(skipping), std::end(skipping));
  // std::vector<double> Pawup(std::begin(pawup), std::end(pawup));

  // if (eventTimes == Stance)
  //   mode = 0;
  // else if (eventTimes == Trot)
  //   mode = 1;
  // else if (eventTimes == Standing_trot)
  //   mode = 2;
  // else if (eventTimes == Flying_trot)
  //   mode = 3;
  // else if (eventTimes == Pace)
  //   mode = 4;
  // else if (eventTimes == Standing_pace)
  //   mode = 5;
  // else if (eventTimes == Dynamic_walk)
  //   mode = 6;
  // else if (eventTimes == Static_walk)
  //   mode = 7;
  // else if (eventTimes == Amble)
  //   mode = 8;
  // else if (eventTimes == Lindyhop)
  //   mode = 9;
  // else if (eventTimes == Skipping)
  //   mode = 10;
  // else
  //   mode = 11;

  // std_msgs::Int64 modeChangeMsg;
  // modeChangeMsg.data = mode;
  // modeChangePublisher_.publish(modeChangeMsg);
  // ROS_INFO_STREAM("Controller has published change mode msg to optimizer." << " mode is: " << mode);
}

void LeggedController::publishModeChange() {
  std_msgs::Int64 modeChangeMsg;
  modeChangeMsg.data = mode;
  modeChangePublisher_.publish(modeChangeMsg);
  ROS_INFO_STREAM("Controller has published change mode msg to optimizer." << " mode is: " << mode);
}

void LeggedController::kpKdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {

  // if (msg->data.size() >= 2) {

  // std::lock_guard<std::mutex> lock(data_mutex_);
  kp_ = msg->data[0];
  kd_ = msg->data[1];

  if (!collecting_) {
    collecting_ = true;
    collection_start_time_ = ros::Time::now();
    total_diff_ = 0.0;
    collected_kp_ = kp_;
    collected_kd_ = kd_;
    std::cout << "Started 2-second collection window for total_diff_ and the current kp is: " << collected_kp_ << " and kd is: " << collected_kd_ << std::endl;
  }

  // if (kp_ != msg->data[0] && kd_ != msg->data[1]) 
  // {
  //   publishKpKdTotalDiff();
  //   total_diff_ = 0;
  //   kp_ = msg->data[0];
  //   kd_ = msg->data[1];
  //   ROS_INFO("Controller Received new kp: %f, kd: %f", kp_, kd_);
  // }
  
  // }
}

// void callback(const std_msgs::String::ConstPtr& msg) {
//   ROS_INFO(msg->data.c_str());
// }

void LeggedController::publishKpKdTotalDiff() {
  // std::cout << "publish kp kd to opt" << std::endl;
  // std::lock_guard<std::mutex> lock(data_mutex_);
  std_msgs::Float64MultiArray msg;
  msg.data.push_back(collected_kp_);
  // ROS_INFO("1");
  msg.data.push_back(collected_kd_);
  msg.data.push_back(total_diff_);
  msg.data.push_back(max_diff_);
  // std::cout << "publish kp kd to opt" << std::endl;
  kpKdTotalDiffPublisher_.publish(msg);
  ROS_INFO_STREAM("Controller has published msg to optimizer." << " kp is: " << kp_ << " kd is: " << kd_ << " total diff is: " << total_diff_ << " max diff is: " << max_diff_);
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }

  
  for (size_t i = 0; i < contacts.size(); ++i) {
    //contactFlag[i] = leggedInterface_->getSwitchedModelReferenceManagerPtr()->getContactFlags(currentObservation_.time)[i]; //使用时间序列估计接触
    contactFlag[i] = contactHandles_[i].isContact();
  }
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

LeggedController::~LeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
