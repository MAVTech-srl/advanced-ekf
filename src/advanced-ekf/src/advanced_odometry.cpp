#include <cstdio>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <deque>
#include <mutex>
#include <fstream>
#include "IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp"
#include "ikfom_model.hpp"
#include "esekfom/esekfom.hpp"
#include "advanced-ekf/utils.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <omp.h>

// using std::placeholders::_1;
using namespace Eigen;
namespace ph = std::placeholders;

// Log format config
IOFormat CommaInitFormatBase(StreamPrecision, DontAlignCols, ", ", ", ", "", "", "", ", ");

// Global measurement struct so it can be seen throughout this script
Measurements measurement;
bool extrinsic_estim_enable;
// Defining this function outside of the main class due to esekfom complaining about the function type...
Eigen::Matrix<double, Eigen::Dynamic, 1> measurement_model(state_ikfom &state, esekfom::dyn_share_datastruct<double> &dyn_share_data)
  {
    dyn_share_data.h.resize(15);
    dyn_share_data.h.setZero();
    dyn_share_data.h.head(3) = state.rot_gps_imu * state.pos + state.pos_gps_imu;
    dyn_share_data.h.segment(3, 3) = state.rot_gps_imu * state.vel;
    Eigen::Matrix3d gps_rot = state.rot_gps_imu.toRotationMatrix() * state.rot.toRotationMatrix();
    dyn_share_data.h.segment(6, 3) = Eigen::Quaterniond(gps_rot).normalized().vec();
    dyn_share_data.h.segment(9, 3) = state.acc + state.ba;
    dyn_share_data.h.segment(12, 3) = state.omega + state.bg;

    dyn_share_data.z.resize(15);
    dyn_share_data.z.setZero();
    dyn_share_data.z.head(3) = measurement.px4_position;
    dyn_share_data.z.segment(3, 3) = measurement.px4_velocity;
    dyn_share_data.z.segment(6, 3) = measurement.px4_orientation.normalized().vec();
    dyn_share_data.z.segment(9, 3) = measurement.imu_acc + state.ba;
    dyn_share_data.z.segment(12, 3) = measurement.imu_gyr + state.bg;

    dyn_share_data.R.resize(15, 15);
    dyn_share_data.R.setZero();
    dyn_share_data.R.block<3, 3>(0, 0) = measurement.px4_position_cov;
    dyn_share_data.R.block<3, 3>(3, 3) = measurement.px4_velocity_cov;
    dyn_share_data.R.block<3, 3>(6, 6) = measurement.px4_orientation_cov;
    dyn_share_data.R.block<3, 3>(9, 9) = measurement.imu_acc_cov;
    dyn_share_data.R.block<3, 3>(12, 12) = measurement.imu_gyr_cov;

    // h_x
    dyn_share_data.h_x.resize(15, 38);
    dyn_share_data.h_x.setZero();
    dyn_share_data.h_x.topLeftCorner(3, 3) << state.rot_gps_imu.toRotationMatrix();
    dyn_share_data.h_x.block<3, 3>(6, 3) << state.rot_gps_imu.toRotationMatrix();
    dyn_share_data.h_x.block<3, 3>(3, 12) << state.rot_gps_imu.toRotationMatrix();
    dyn_share_data.h_x.block<3, 3>(12, 15) << Matrix3d::Identity();
    dyn_share_data.h_x.block<3, 3>(9, 18) << Matrix3d::Identity();
    if (extrinsic_estim_enable)
    {
      dyn_share_data.h_x.block<3, 3>(0, 23) << -state.rot_gps_imu.toRotationMatrix() * skew_symm(state.pos);
      dyn_share_data.h_x.block<3, 3>(3, 23) << state.rot.toRotationMatrix();
      dyn_share_data.h_x.block<3, 3>(6, 23) << -state.rot.toRotationMatrix() * skew_symm(state.vel);
      dyn_share_data.h_x.block<3, 3>(0, 26) << Matrix3d::Identity();
    }
    dyn_share_data.h_x.block<3, 3>(9, 29) << Matrix3d::Identity();
    dyn_share_data.h_x.block<3, 3>(6, 35) << Matrix3d::Identity();

    return dyn_share_data.h;
  }

class AdvancedOdometry : public rclcpp::Node
{
public:
	explicit AdvancedOdometry() : Node("advanced_odometry_node"), CommaInitFormat(CommaInitFormatBase), log_file()
	{
    // Get all parameters
    RCLCPP_DEBUG_STREAM(node_logger,"Getting parameters...");
    this->declare_parameter<double>("ekf.frequency");
    this->get_parameter_or<double>("ekf.frequency", EKF_frequency, 10.0);

    this->declare_parameter<double>("ekf.acc_noise_covariance");
    this->get_parameter_or<double>("ekf.acc_noise_covariance", acc_noise_cov, 0.001);

    this->declare_parameter<double>("ekf.gyr_noise_covariance");
    this->get_parameter_or<double>("ekf.gyr_noise_covariance", gyr_noise_cov, 0.001);

    this->declare_parameter<double>("ekf.jitter_noise_covariance");
    this->get_parameter_or<double>("ekf.jitter_noise_covariance", jitter_noise_cov, 0.00001);

    this->declare_parameter<bool>("ekf.extrinsic_estim_enable");
    this->get_parameter_or<bool>("ekf.extrinsic_estim_enable", extrinsic_estim_enable, false);

    this->declare_parameter<bool>("save_log");
    this->get_parameter_or<bool>("save_log", save_log, false);

    this->declare_parameter<std::string>("log_filepath");
    this->get_parameter_or<std::string>("log_filepath", log_filepath, "./ekf_log.csv");

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
    RCLCPP_DEBUG_STREAM(node_logger,"Subscribing to topics...");
		odometry_subscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&AdvancedOdometry::px4_odometry_listener, this, ph::_1));
    imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>("/livox/lidar/imu", qos, std::bind(&AdvancedOdometry::IMU_listener, this, ph::_1));
    // Devo aggiungere un subscriber per la velocità dei motori!
    force_torque0_subscriber = this->create_subscription<geometry_msgs::msg::Wrench>("/world/forest/model/x500_lidar_3d_0/joint/rotor_0_joint/sensor/force_torque/forcetorque", qos,
                                std::bind(&AdvancedOdometry::force0_listener, this, ph::_1));
    force_torque1_subscriber = this->create_subscription<geometry_msgs::msg::Wrench>("/world/forest/model/x500_lidar_3d_0/joint/rotor_1_joint/sensor/force_torque/forcetorque", qos,
                                std::bind(&AdvancedOdometry::force1_listener, this, ph::_1));
    force_torque2_subscriber = this->create_subscription<geometry_msgs::msg::Wrench>("/world/forest/model/x500_lidar_3d_0/joint/rotor_2_joint/sensor/force_torque/forcetorque", qos,
                                std::bind(&AdvancedOdometry::force2_listener, this, ph::_1));
    force_torque3_subscriber = this->create_subscription<geometry_msgs::msg::Wrench>("/world/forest/model/x500_lidar_3d_0/joint/rotor_3_joint/sensor/force_torque/forcetorque", qos,
                                std::bind(&AdvancedOdometry::force3_listener, this, ph::_1));

    odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 20);

    VectorXd epsi = 0.001 * VectorXd::Ones(current_state.DIM);
    ikfom.init_dyn_share(f, df_dx, df_dw, measurement_model, 3, epsi.data());

    auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1.0e3 / EKF_frequency));
    timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&AdvancedOdometry::step_ekf, this));

    if (save_log)
    {
      RCLCPP_INFO_STREAM(node_logger, "Opening log file at " << log_filepath);
      log_file.open(log_filepath);
      log_file << "timestamp_s, elapsed_one_step_ms, pos_x, pos_y, pos_z, rot_w, rot_x, rot_y, rot_z, rot_I_L_w, rot_I_L_x, rot_I_L_y, rot_I_L_z, \
pos_I_L_x, pos_I_L_y, pos_I_L_z, vel_x, vel_y, vel_z, bw_x, bw_y, bw_z, ba_x, ba_y, ba_z, g_x, g_y, g_z, \
rot_O_I_w, rot_O_I_x, rot_O_I_y, rot_O_I_z, pos_O_I_x, pos_O_I_y, pos_O_I_z, acc_x, acc_y, acc_z, j_x, j_y, j_z, \
omega_x, omega_y, omega_z, force_x, force_y, force_z, torque_x, torque_y, torque_z" << std::endl;
      log_file.flush();
      // log_file << std::unitbuf;
    }

    RCLCPP_INFO(this->get_logger(), "Advanced EKF node initialized.");
	}

  void close_log_file(void)
  {
    log_file.close();
    RCLCPP_DEBUG(node_logger, "Log file saved!");
  }

private:
  // Subscriber callbacks
  void px4_odometry_listener(px4_msgs::msg::VehicleOdometry msg)
  {
    odom_lock.lock();
    odom_msg_buffer.push_front(msg);
    odom_lock.unlock();
  }
  void IMU_listener(sensor_msgs::msg::Imu msg)
  {
    imu_lock.lock();
    imu_msg_buffer.push_front(msg);
    imu_lock.unlock();
  }
  void force0_listener(geometry_msgs::msg::Wrench msg)
  {
    auto time = this->now();
    force0_time = time.nanoseconds();
    force0_lock.lock();
    force0_buffer.push_front(msg);
    force0_lock.unlock();
  }
  void force1_listener(geometry_msgs::msg::Wrench msg)
  {
    auto time = this->now();
    force1_time = time.nanoseconds();
    force1_lock.lock();
    force1_buffer.push_front(msg);
    force1_lock.unlock();
  }
  void force2_listener(geometry_msgs::msg::Wrench msg)
  {
    auto time = this->now();
    force2_time = time.nanoseconds();
    force2_lock.lock();
    force2_buffer.push_front(msg);
    force2_lock.unlock();
  }
  void force3_listener(geometry_msgs::msg::Wrench msg)
  {
    auto time = this->now();
    force3_time = time.nanoseconds();
    force3_lock.lock();
    force3_buffer.push_front(msg);
    force3_lock.unlock();
  }

  int sync_msgs(void)
  {

    measurements_updated = false;
    // Synchronize the messages when the timer callback fires and fill measurement struct.
    if (force0_buffer.empty() || force1_buffer.empty() || force2_buffer.empty() || force3_buffer.empty())
    {
      RCLCPP_DEBUG(node_logger, "Force buffers empty!");
      return 1;
    }
    force0_lock.lock();
    force1_lock.lock();
    force2_lock.lock();
    force3_lock.lock();
    odom_lock.lock();
    imu_lock.lock();
    
    if (force_msg_count == 0)
    {
      // Offsetting initial time
      offset_force0_time = force0_time;
      offset_force1_time = force1_time;
      offset_force2_time = force2_time;
      offset_force3_time = force3_time;
      force_msg_count++;
      force0_buffer.clear();
      force1_buffer.clear();
      force2_buffer.clear();
      force3_buffer.clear();
      force0_lock.unlock();
      force1_lock.unlock();
      force2_lock.unlock();
      force3_lock.unlock();
      odom_lock.unlock();
      imu_lock.unlock();
      return 1;   // Discard the first force message so that I can compute the delta time when next message arrives
    }
    int64_t current_input_time = 0.25 * (force0_time + force1_time + force2_time + force3_time - offset_force0_time - offset_force1_time - offset_force2_time - offset_force3_time);
    input_dt = (current_input_time - old_input_time) * 1.0e-9;
    old_input_time = current_input_time;
    RCLCPP_DEBUG_STREAM(node_logger, "Force mean time: " << current_input_time / 1e6 << " ms, dt = " << input_dt << " s.");

    Vector3d total_force =  Vector3d(force0_buffer.front().force.x*0, force0_buffer.front().force.y*0, force0_buffer.front().force.z + 0.15755) +
                            Vector3d(force1_buffer.front().force.x*0, force1_buffer.front().force.y*0, force1_buffer.front().force.z + 0.15755) +
                            Vector3d(force2_buffer.front().force.x*0, force2_buffer.front().force.y*0, force2_buffer.front().force.z + 0.15755) +
                            Vector3d(force3_buffer.front().force.x*0, force3_buffer.front().force.y*0, force3_buffer.front().force.z + 0.15755);
    current_input.force = total_force;

    Vector3d total_torque = Vector3d( (force1_buffer.front().force.z + force2_buffer.front().force.z - force0_buffer.front().force.z - force3_buffer.front().force.z) * arm_dist_y,
                                      (force1_buffer.front().force.z + force3_buffer.front().force.z - force0_buffer.front().force.z - force2_buffer.front().force.z) * arm_dist_x,
                                      force0_buffer.front().torque.z + force1_buffer.front().torque.z + force2_buffer.front().torque.z + force3_buffer.front().torque.z);
    current_input.torque = total_torque;
    force_msg_count++;
    force0_buffer.pop_front();
    force1_buffer.pop_front();
    force2_buffer.pop_front();
    force3_buffer.pop_front();
    // For now we clear the buffer as we use only the latest value
    force0_buffer.clear();
    force1_buffer.clear();
    force2_buffer.clear();
    force3_buffer.clear();

    // If odom or IMU buffers are empty, just propagate the model
    if (odom_msg_buffer.empty() || imu_msg_buffer.empty())
    {
      force0_lock.unlock();
      force1_lock.unlock();
      force2_lock.unlock();
      force3_lock.unlock();
      odom_lock.unlock();
      imu_lock.unlock();
      RCLCPP_DEBUG_STREAM(node_logger, "Processing only forces\n-------------------------------------------------------------------------------------------------------");
      return 0;
    }

    // IMU measurements
    current_imu_time = rclcpp::Time(imu_msg_buffer.front().header.stamp).nanoseconds();
    imu_dt = (current_imu_time - old_imu_time) * 1.0e-9;
    old_imu_time = current_imu_time;
    RCLCPP_DEBUG_STREAM(node_logger, "IMU delta t: " << imu_dt << " s.");
    measurement.imu_acc = Vector3d( imu_msg_buffer.front().linear_acceleration.x,
                                    imu_msg_buffer.front().linear_acceleration.y,
                                    imu_msg_buffer.front().linear_acceleration.z);
    measurement.imu_acc_cov = acc_noise_cov * Matrix3d::Identity();
    measurement.imu_gyr = Vector3d( imu_msg_buffer.front().angular_velocity.x,
                                    imu_msg_buffer.front().angular_velocity.y,
                                    imu_msg_buffer.front().angular_velocity.z);
    measurement.imu_gyr_cov = gyr_noise_cov * Matrix3d::Identity();
    imu_msg_count++;
    
    // Odometry measurements
    current_odom_time = 1e3 * odom_msg_buffer.front().timestamp;
    odom_dt = (current_odom_time - old_odom_time) * 1.0e-9;
    old_odom_time = current_odom_time;
    RCLCPP_DEBUG_STREAM(node_logger, "Odom delta t: " << odom_dt << " s.");
    // Position and linear velocity are rotated into NWU RF
    // Position and linear velocity covariances are rotated into NWU RF
    Matrix3f px4_position_covariance = Map<const Vector3f>(odom_msg_buffer.front().position_variance.data(), 3).asDiagonal();
    // NOTE: the orientation variance is in BODY FRAME!
    Matrix3f px4_orientation_covariance = Map<const Vector3f>(odom_msg_buffer.front().orientation_variance.data(), 3).asDiagonal();
    Matrix3f px4_velocity_covariance = Map<const Vector3f>(odom_msg_buffer.front().velocity_variance.data(), 3).asDiagonal();

    if (odom_msg_count == 0)
    {
        measurement.px4_position = VectorXd::Zero(3);
        measurement.px4_position_cov = ned2nwu * px4_position_covariance.cast<double>() * ned2nwu.transpose();
        measurement.px4_velocity = Vector3f(odom_msg_buffer.front().velocity.data()).cast<double>();
        measurement.px4_velocity_cov = ned2nwu * px4_velocity_covariance.cast<double>() * ned2nwu.transpose();
        // Save initial position bias
        initial_position = Vector3f(odom_msg_buffer.front().position.data()).cast<double>();
        // Save initial orientation bias
        initial_orientation = Quaternionf(Vector4f(odom_msg_buffer.front().q.data())).cast<double>();
        measurement.px4_orientation = Quaterniond(1.0, 0.0, 0.0, 0.0);
        measurement.px4_orientation_cov = ned2nwu * px4_orientation_covariance.cast<double>() * ned2nwu.transpose();
        
        RCLCPP_DEBUG_STREAM(node_logger, "Initial position:\n" << initial_position.transpose() << "\nInitial pose:\n" << initial_orientation.coeffs().transpose() );
        odom_msg_buffer.pop_front();
        odom_msg_count++;

    }
    else
    {
      // if (odom_time <= lidar_end_time)     // This in SITL was always false!!! Maybe also in real experiments? CHECK THIS
      // {
          Vector3d actual_position = Vector3f(odom_msg_buffer.front().position.data()).cast<double>();
          measurement.px4_position = actual_position - initial_position;
          measurement.px4_position_cov = ned2nwu * px4_position_covariance.cast<double>() * ned2nwu.transpose();
          // Populate linear velocity
          measurement.px4_velocity = Vector3f(odom_msg_buffer.front().velocity.data()).cast<double>();
          measurement.px4_velocity_cov = ned2nwu * px4_velocity_covariance.cast<double>() * ned2nwu.transpose();
          // Populate orientation
          measurement.px4_orientation = Quaternionf(Vector4f(odom_msg_buffer.front().q.data())).cast<double>().normalized();
          measurement.px4_orientation_cov = ned2nwu * px4_orientation_covariance.cast<double>() * ned2nwu.transpose();
          // NOTE: SUBTRACT THE INITIAL ORIENTATION!
          // This is an extrinsic rotation because I want to apply the initial rotation (inversed) to the actual axes wrt inertial RF
          Matrix3d actual_pose = initial_orientation.normalized().toRotationMatrix().transpose() * measurement.px4_orientation.normalized().toRotationMatrix();
          measurement.px4_orientation = Quaterniond(actual_pose).normalized() ;
          // measurement.px4_orientation.normalize();
          // // Populate angular velocity
          // measurement.px4_angular_speed = Vector3d(odom_buffer.front()->twist.twist.angular.x,
          //                                     odom_buffer.front()->twist.twist.angular.y,
          //                                     odom_buffer.front()->twist.twist.angular.z);    // BUG: Maybe this is wrong because it does not take into account the initial orientation bias!!
          // measurement.px4_angular_speed_cov = px4_twist_covariance.bottomRightCorner(3, 3);
          // // Now rotate it to align with the lidar RF
          // Eigen::Quaterniond q(-0.4231, 0.5666, 0.5666, 0.4231);     // Order: w, x, y, z
          // Matrix3d rot_mat = q.normalized().toRotationMatrix();
          // meas.px4_position = rot_mat.transpose() * meas.px4_position;
          // std::cout << "Just read from mavlink:\n" << meas.px4_position << std::endl;
          odom_msg_buffer.pop_front();
          odom_msg_count++;
      // }
    }

    odom_msg_buffer.clear();
    imu_msg_buffer.clear();

    force0_lock.unlock();
    force1_lock.unlock();
    force2_lock.unlock();
    force3_lock.unlock();
    odom_lock.unlock();
    imu_lock.unlock();
    measurements_updated = true;
    RCLCPP_DEBUG_STREAM(node_logger, "Processing both forces & measures\n-------------------------------------------------------------------------------------------------------");
    return 0;
  }

  void step_ekf(void)
  {
    RCLCPP_DEBUG_STREAM(node_logger, "EKF stepped " << ++num_steps << " times.");
    int64_t time_begin = this->now().nanoseconds();
    // Synchronize incoming messages
    if (sync_msgs())
    {
      RCLCPP_DEBUG(node_logger, "Buffers empty, stepping ekf!");
      return;
    }
    // Predict and update
    Matrix<double, 9, 9> Q = process_noise_cov();
    Q.bottomRightCorner<3, 3>() = jitter_noise_cov * Matrix3d::Identity();
    ikfom.predict(input_dt, Q, current_input);
    current_state = ikfom.get_x();
    VectorXd state_vector;
    RCLCPP_DEBUG_STREAM(node_logger, "Predicted state: \n" << current_state);
    if (measurements_updated)
    {
      // Update propagated state with measurements
      double update_elapsed = 0.0;
      ikfom.update_iterated_dyn_share_modified(update_elapsed);

      // Compute elapsed times
      auto time_post_update = this->now();
      double elapsed_one_step = (time_post_update.nanoseconds() - time_begin) * 1.0e-6; // In milliseconds
      current_state = ikfom.get_x();
      RCLCPP_DEBUG_STREAM(node_logger, "Updated state: " << current_state << " total elapsed: " << elapsed_one_step << "ms.");

      // Publish odometry
      nav_msgs::msg::Odometry odometry_out;
      odometry_out.header.frame_id = "map";
      odometry_out.header.stamp = time_post_update;
      odometry_out.pose.pose.position.x = current_state.pos.x();
      odometry_out.pose.pose.position.y = current_state.pos.y();
      odometry_out.pose.pose.position.z = current_state.pos.z();
      odometry_out.pose.pose.orientation.w = current_state.rot.w();
      odometry_out.pose.pose.orientation.x = current_state.rot.x();
      odometry_out.pose.pose.orientation.y = current_state.rot.y();
      odometry_out.pose.pose.orientation.z = current_state.rot.z();
      odometry_out.twist.twist.linear.x = current_state.vel.x();
      odometry_out.twist.twist.linear.y = current_state.vel.y();
      odometry_out.twist.twist.linear.z = current_state.vel.z();
      odometry_out.twist.twist.angular.x = current_state.omega.x();
      odometry_out.twist.twist.angular.y = current_state.omega.y();
      odometry_out.twist.twist.angular.z = current_state.omega.z();
      odometry_publisher->publish(odometry_out);

      if (save_log)
      {
        if (!log_file.is_open())
        {
          std::cout << "Log file is closed!!" << std::endl;
        }
        log_file  << time_post_update.nanoseconds() * 1.0e-9 << ", "
                  << elapsed_one_step << ", "
                  << current_state.pos.format(CommaInitFormat) 
                  << current_state.rot.coeffs().format(CommaInitFormat)
                  << current_state.offset_R_L_I.coeffs().format(CommaInitFormat)
                  << current_state.offset_T_L_I.format(CommaInitFormat)
                  << current_state.vel.format(CommaInitFormat)
                  << current_state.bg.format(CommaInitFormat)
                  << current_state.ba.format(CommaInitFormat)
                  << current_state.grav.get_vect().format(CommaInitFormat)
                  << current_state.rot_gps_imu.coeffs().format(CommaInitFormat)
                  << current_state.pos_gps_imu.format(CommaInitFormat)
                  << current_state.acc.format(CommaInitFormat)
                  << current_state.jitter.format(CommaInitFormat)
                  << current_state.omega.format(CommaInitFormat)
                  << current_input.force.format(CommaInitFormat)
                  << current_input.torque.format(IOFormat(StreamPrecision, DontAlignCols, ", ", ", ", "", "", "", ""))
                  << std::endl;
        log_file.flush();
        // log_file << std::unitbuf;
      }
    }
    else
    {
      RCLCPP_DEBUG_STREAM(node_logger, "Only prediction this loop\n======================================================================");
      return;
    }
    
  }

  // Subscribers
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr force_torque0_subscriber, force_torque1_subscriber, force_torque2_subscriber, force_torque3_subscriber;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
  // Message buffers/queues
  std::deque<px4_msgs::msg::VehicleOdometry> odom_msg_buffer;
  std::deque<sensor_msgs::msg::Imu> imu_msg_buffer;
  std::deque<geometry_msgs::msg::Wrench> force0_buffer, force1_buffer, force2_buffer, force3_buffer;
  std::mutex odom_lock, imu_lock, force0_lock, force1_lock, force2_lock, force3_lock;

  double EKF_frequency;
  double acc_noise_cov, gyr_noise_cov, jitter_noise_cov;
  // Measurements z;
  input_ikfom current_input;
  state_ikfom current_state;
  esekfom::esekf<state_ikfom, 9, input_ikfom> ikfom;
  const Matrix3d ned2nwu = AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix();    // NED-to-NWU rotation
  uint64_t old_input_time, old_imu_time, old_odom_time;
  int64_t force0_time, force1_time, force2_time, force3_time;
  int64_t offset_force0_time, offset_force1_time, offset_force2_time, offset_force3_time;
  double input_dt;                                                                    // Time difference in seconds between to consecutive input measurements
  double imu_dt, odom_dt;                                                             // Time difference in seconds between to consecutive output measurements (z)
  uint64_t force_msg_count = 0, odom_msg_count = 0, imu_msg_count = 0, num_steps = 0;
  uint64_t current_imu_time, current_odom_time;
  Vector3d initial_position;
  Quaterniond initial_orientation;
  bool measurements_updated = false;
  rclcpp::Logger node_logger = this->get_logger();
  IOFormat CommaInitFormat;
  std::ofstream log_file;
  bool save_log;
  std::string log_filepath;
};

int main(int argc, char *argv[])
{
  // omp_set_num_threads(8);
	rclcpp::init(argc, argv);
  std::shared_ptr<AdvancedOdometry> adv_odom = std::make_shared<AdvancedOdometry>();
  rclcpp::on_shutdown( [&adv_odom](){adv_odom->close_log_file();} );
  try
	{
    rclcpp::spin(adv_odom);
  }
  catch (...)
  {
    adv_odom->close_log_file();
    throw;
  }

  // if (!rclcpp::ok())
  // {
  //   adv_odom->close_log_file();
  // }

	rclcpp::shutdown();
  // std::cout << "Pre-shutdown actions..." << std::endl;
  // adv_odom->close_log_file();
	return 0;
}
