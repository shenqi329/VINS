#include <msckf_mono/ros_interface.h>

namespace msckf_mono
{
  RosInterface::RosInterface() :
    imu_calibrated_(false),
    prev_imu_time_(0.0)
  {
    load_parameters();
    setup_track_handler();
  }

  void RosInterface::imuCallback()
  {
//    double cur_imu_time = imu->header.stamp.toSec();
//    if(prev_imu_time_ == 0.0){
//      prev_imu_time_ = cur_imu_time;
//      done_stand_still_time_ = cur_imu_time + stand_still_time_;
//      return;
//    }
//
//    imuReading<float> current_imu;
//
//    current_imu.a[0] = imu->linear_acceleration.x;
//    current_imu.a[1] = imu->linear_acceleration.y;
//    current_imu.a[2] = imu->linear_acceleration.z;
//
//    current_imu.omega[0] = imu->angular_velocity.x;
//    current_imu.omega[1] = imu->angular_velocity.y;
//    current_imu.omega[2] = imu->angular_velocity.z;
//
//    current_imu.dT = cur_imu_time - prev_imu_time_;
//
//    imu_queue_.emplace_back(cur_imu_time, current_imu);
//
//    prev_imu_time_ = cur_imu_time;
  }

  void RosInterface::imageCallback()
  {
//    double cur_image_time = msg->header.stamp.toSec();
//    cv_bridge::CvImagePtr cv_ptr;
//    try
//    {
//      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//      ROS_ERROR("cv_bridge exception: %s", e.what());
//      return;
//    }
//
//    if(!imu_calibrated_){
//      if(imu_queue_.size() % 100 == 0){
//        ROS_INFO_STREAM("Has " << imu_queue_.size() << " readings");
//      }
//
//      if(can_initialize_imu()){
//        initialize_imu();
//
//        imu_calibrated_ = true;
//        imu_queue_.clear();
//
//        setup_msckf();
//      }
//
//      return;
//    }
//    ros::Time t1 = ros::Time::now();
//
//    std::vector<imuReading<float>> imu_since_prev_img;
//    imu_since_prev_img.reserve(10);
//
//    // get the first imu reading that belongs to the next image
//    auto frame_end = std::find_if(imu_queue_.begin(), imu_queue_.end(),
//        [&](const auto& x){return std::get<0>(x) > cur_image_time;});
//
//    std::transform(imu_queue_.begin(), frame_end,
//        std::back_inserter(imu_since_prev_img),
//        [](auto& x){return std::get<1>(x);});
//
//    imu_queue_.erase(imu_queue_.begin(), frame_end);
//
//    for(auto& reading : imu_since_prev_img){
//      msckf_.propagate(reading);
//
//      Vector3<float> gyro_measurement = R_imu_cam_ * (reading.omega - init_imu_state_.b_g);
//      track_handler_->add_gyro_reading(gyro_measurement);
//    }
//
//    track_handler_->set_current_image( cv_ptr->image, cur_image_time );
//
//    std::vector<Vector2<float>,
//      Eigen::aligned_allocator<Vector2<float>>> cur_features;
//    corner_detector::IdVector cur_ids;
//    track_handler_->tracked_features(cur_features, cur_ids);
//
//    std::vector<Vector2<float>,
//      Eigen::aligned_allocator<Vector2<float>>> new_features;
//    corner_detector::IdVector new_ids;
//    track_handler_->new_features(new_features, new_ids);
//
//    msckf_.augmentState(state_k_, (float)cur_image_time);
//    msckf_.update(cur_features, cur_ids);
//    msckf_.addFeatures(new_features, new_ids);
//    msckf_.marginalize();
//    // msckf_.pruneRedundantStates();
//    msckf_.pruneEmptyStates();
//
//    ros::Time t2 = ros::Time::now();
//    ROS_INFO_STREAM("time = " << t2.toSec() - t1.toSec());
//
//    publish_core(msg->header.stamp);
//    publish_extra(msg->header.stamp);
  }

//  void RosInterface::publish_core(const ros::Time& publish_time)
//  {
//    auto imu_state = msckf_.getImuState();
//
//    nav_msgs::Odometry odom;
//    odom.header.stamp = publish_time;
//    odom.header.frame_id = "map";
//    odom.twist.twist.linear.x = imu_state.v_I_G[0];
//    odom.twist.twist.linear.y = imu_state.v_I_G[1];
//    odom.twist.twist.linear.z = imu_state.v_I_G[2];
//
//    odom.pose.pose.position.x = imu_state.p_I_G[0];
//    odom.pose.pose.position.y = imu_state.p_I_G[1];
//    odom.pose.pose.position.z = imu_state.p_I_G[2];
//    Quaternion<float> q_out = imu_state.q_IG.inverse();
//    odom.pose.pose.orientation.w = q_out.w();
//    odom.pose.pose.orientation.x = q_out.x();
//    odom.pose.pose.orientation.y = q_out.y();
//    odom.pose.pose.orientation.z = q_out.z();
//
//    odom_pub_.publish(odom);
//  }

//  void RosInterface::publish_extra(const ros::Time& publish_time)
//  {
//    if(track_image_pub_.getNumSubscribers() > 0){
//      cv_bridge::CvImage out_img;
//      out_img.header.frame_id = "cam0";
//      out_img.header.stamp = publish_time;
//      out_img.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
//      out_img.image = track_handler_->get_track_image();
//      track_image_pub_.publish(out_img.toImageMsg());
//    }
//  }

  bool RosInterface::can_initialize_imu()
  {
    if(imu_calibration_method_ == TimedStandStill){
      return prev_imu_time_ > done_stand_still_time_;
    }

    return false;
  }

  void RosInterface::initialize_imu()
  {
    Eigen::Vector3f accel_accum;
    Eigen::Vector3f gyro_accum;
    int num_readings = 0;

    accel_accum.setZero();
    gyro_accum.setZero();

    for(const auto& entry : imu_queue_){
      auto imu_time = std::get<0>(entry);
      auto imu_reading = std::get<1>(entry);

      accel_accum += imu_reading.a;
      gyro_accum += imu_reading.omega;
      num_readings++;
    }

    Eigen::Vector3f accel_mean = accel_accum / num_readings;
    Eigen::Vector3f gyro_mean = gyro_accum / num_readings;

    init_imu_state_.b_g = gyro_mean;
    init_imu_state_.g << 0.0, 0.0, -9.81;
    init_imu_state_.q_IG = Quaternion<float>::FromTwoVectors(
        -init_imu_state_.g, accel_mean);

    init_imu_state_.b_a = init_imu_state_.q_IG*init_imu_state_.g + accel_mean;

    init_imu_state_.p_I_G.setZero();
    init_imu_state_.v_I_G.setZero();
    const auto q = init_imu_state_.q_IG;

//    LOGI("\nInitial IMU State" <<
//      "\n--p_I_G " << init_imu_state_.p_I_G.transpose() <<
//      "\n--q_IG " << q.w() << "," << q.x() << "," << q.y() << "," << q.x() <<
//      "\n--v_I_G " << init_imu_state_.v_I_G.transpose() <<
//      "\n--b_a " << init_imu_state_.b_a.transpose() <<
//      "\n--b_g " << init_imu_state_.b_g.transpose() <<
//      "\n--g " << init_imu_state_.g.transpose());

  }

  void RosInterface::setup_track_handler()
  {
    track_handler_.reset( new corner_detector::TrackHandler(K_, dist_coeffs_, distortion_model_) );
    track_handler_->set_grid_size(n_grid_rows_, n_grid_cols_);
    track_handler_->set_ransac_threshold(ransac_threshold_);
  }

  void RosInterface::setup_msckf()
  {
    state_k_ = 0;
    msckf_.initialize(camera_, noise_params_, msckf_params_, init_imu_state_);
  }

  void RosInterface::load_parameters()
  {
    // 相机内参
    std::vector<float> intrinsics(4);
    intrinsics[0] = 458.654;
    intrinsics[1] = 457.296;
    intrinsics[2] = 367.215;
    intrinsics[3] = 248.375;

    K_ = cv::Mat::eye(3,3,CV_32F);
    K_.at<float>(0,0) = intrinsics[0];
    K_.at<float>(1,1) = intrinsics[1];
    K_.at<float>(0,2) = intrinsics[2];
    K_.at<float>(1,2) = intrinsics[3];

    distortion_model_ = "radtan";

    // 畸变
    dist_coeffs_ = cv::Mat::zeros(4 , 1, CV_32F);
    dist_coeffs_.at<float>(0) = -0.28340811;
    dist_coeffs_.at<float>(1) = 0.07395907;
    dist_coeffs_.at<float>(2) = 0.00019359;
    dist_coeffs_.at<float>(3) = 1.76187114e-05;

    Matrix4<float> T_cam_imu;

    T_cam_imu(0, 0) = 0.0148655429818;
    T_cam_imu(0, 1) = -0.999880929698;
    T_cam_imu(0, 2) = 0.00414029679422;
    T_cam_imu(0, 3) = -0.021640145497;

    T_cam_imu(1, 0) = 0.999557249008;
    T_cam_imu(1, 1) = 0.0149672133247;
    T_cam_imu(1, 2) = 0.025715529948;
    T_cam_imu(1, 3) = -0.064676986768;

    T_cam_imu(2, 0) = -0.0257744366974;
    T_cam_imu(2, 1) = 0.00375618835797;
    T_cam_imu(2, 2) = 0.999660727178;
    T_cam_imu(2, 3) = 0.009810730590;

    T_cam_imu(3, 0) = 0.0;
    T_cam_imu(3, 1) = 0.0;
    T_cam_imu(3, 2) = 0.0;
    T_cam_imu(3, 3) = 1.0;

    R_cam_imu_ =  T_cam_imu.block<3,3>(0,0);
    p_cam_imu_ =  T_cam_imu.block<3,1>(0,3);

    R_imu_cam_ = R_cam_imu_.transpose();
    p_imu_cam_ = R_imu_cam_ * (-1. * p_cam_imu_);

    // setup camera parameters
    camera_.f_u = intrinsics[0];
    camera_.f_v = intrinsics[1];
    camera_.c_u = intrinsics[2];
    camera_.c_v = intrinsics[3];

    camera_.q_CI = Quaternion<float>(R_cam_imu_).inverse(); // TODO please check it
    camera_.p_C_I = p_cam_imu_;

    // Feature tracking parameteres
    n_grid_rows_ = 10;
    n_grid_cols_ = 10;

    //    float ransac_threshold_;
    ransac_threshold_ = 0.00000002;

    // MSCKF Parameters
    float feature_cov = 2;

    Eigen::Matrix<float,12,1> Q_imu_vars;
    float w_var = 1e-4, dbg_var = 3.6733e-5, a_var = 1e-2, dba_var = 7e-2;
    Q_imu_vars << w_var, 	w_var, 	w_var,
                  dbg_var,dbg_var,dbg_var,
                  a_var,	a_var,	a_var,
                  dba_var,dba_var,dba_var;

    Eigen::Matrix<float,15,1> IMUCovar_vars;
    float q_var_init = 1e-5, bg_var_init = 1e-2, v_var_init = 1e-2, ba_var_init = 1e-2, p_var_init = 1e-12;
    IMUCovar_vars << q_var_init, q_var_init, q_var_init,
                     bg_var_init,bg_var_init,bg_var_init,
                     v_var_init, v_var_init, v_var_init,
                     ba_var_init,ba_var_init,ba_var_init,
                     p_var_init, p_var_init, p_var_init;

    // Setup noise parameters
    noise_params_.initial_imu_covar = IMUCovar_vars.asDiagonal();
    noise_params_.Q_imu = Q_imu_vars.asDiagonal();
    noise_params_.u_var_prime = pow(feature_cov/camera_.f_u,2);
    noise_params_.v_var_prime = pow(feature_cov/camera_.f_v,2);

    msckf_params_.max_gn_cost_norm = 7;
    msckf_params_.max_gn_cost_norm = pow(msckf_params_.max_gn_cost_norm/camera_.f_u, 2);
    msckf_params_.translation_threshold = 0.1;
    msckf_params_.min_rcond = 3e-12;
    msckf_params_.redundancy_angle_thresh = 0.5;
    msckf_params_.redundancy_distance_thresh = 0.5;
    msckf_params_.max_track_length = 50;
    msckf_params_.min_track_length = 5;
    msckf_params_.max_cam_states = 30;

    // Load calibration time
    int method = 0;
    if(method == 0){
      imu_calibration_method_ = TimedStandStill;
    }
    stand_still_time_ = 8.0;

//    ROS_INFO_STREAM("Loaded " << kalibr_camera);
//    ROS_INFO_STREAM("-Intrinsics " << intrinsics[0] << ", "
//                                   << intrinsics[1] << ", "
//                                   << intrinsics[2] << ", "
//                                   << intrinsics[3] );
//    ROS_INFO_STREAM("-Distortion " << distortion_coeffs[0] << ", "
//                                   << distortion_coeffs[1] << ", "
//                                   << distortion_coeffs[2] << ", "
//                                   << distortion_coeffs[3] );
    const auto q_CI = camera_.q_CI;
//    ROS_INFO_STREAM("-q_CI \n" << q_CI.x() << "," << q_CI.y() << "," << q_CI.z() << "," << q_CI.w());
//    ROS_INFO_STREAM("-p_C_I \n" << camera_.p_C_I.transpose());
  }

}
