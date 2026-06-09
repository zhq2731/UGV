#include "ndt.h"

const double D2R = (M_PI / 180.0);
const double R2D = (180.0 / M_PI);
ROSMultiLidarCalibratorApp app;
int temp_vaid = 1;
std::string path_txt;
std::string ins_txt;

std::vector<localization_msgs::Localization> ins_v;
std::vector<geometry_msgs::PoseStamped> pose_v;
std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> pose_r;

NdtLocalizer::NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh):nh_(nh), private_nh_(private_nh), tf2_listener_(tf2_buffer_){

  key_value_stdmap_["state"] = "Initializing";
  init_params();

  // Publishers
  sensor_aligned_pose_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_aligned", 10);
  ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 10);
  exe_time_pub_ = nh_.advertise<std_msgs::Float32>("exe_time_ms", 10);
  transform_probability_pub_ = nh_.advertise<std_msgs::Float32>("transform_probability", 10);
  iteration_num_pub_ = nh_.advertise<std_msgs::Float32>("iteration_num", 10);
  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 10);
  ins_pub_ = nh_.advertise<localization_msgs::Localization>("/odomData", 10);

  // Subscribers
  initial_pose_sub_ = nh_.subscribe("initialpose", 100, &NdtLocalizer::callback_init_pose, this);
  map_points_sub_ = nh_.subscribe("points_map", 1, &NdtLocalizer::callback_pointsmap, this);
  sensor_points_sub_ = nh_.subscribe("filtered_points", 1, &NdtLocalizer::callback_pointcloud, this);
  result_path_sub_ = nh_.subscribe("ndt_pose", 1, &NdtLocalizer::callback_pointpath, this);
  //imu_sub_ = nh_.subscribe(input_imu_topic, 100, &NdtLocalizer::callback_imu, this);

  diagnostic_thread_ = std::thread(&NdtLocalizer::timer_diagnostic, this);
  diagnostic_thread_.detach();
}

NdtLocalizer::~NdtLocalizer() {}

//发布ndt的pose
void NdtLocalizer::callback_pointpath(const geometry_msgs::PoseStamped pose){
    static nav_msgs::Path path;
    path.header = pose.header;
    path.poses.push_back(pose);
    path_pub_.publish(path);
    pose_v.push_back(pose);

    //TODO local2global
    localization_msgs::Localization ins;
    ins.timestamp = pose.header.stamp.toSec();

    Eigen::Vector3d pos_local;
    pos_local(0) = pose.pose.position.x;
    pos_local(1) = pose.pose.position.y;
    pos_local(2) = pose.pose.position.z;

    Eigen::Vector3d vel;
    vel = (pos_local - last_pos) / 0.1;
    last_pos = pos_local;

    Eigen::Matrix3d R;
    R << 1.0,0.0,0.0,
         0.0, -1.0, 0.0,
         0.0, 0.0, -1.0;
    pos_local = R * pos_local;
    Vector3d pos = Earth::local2global(origin, pos_local);
    pos.segment(0, 2) *= R2D;
   // ins.latitude = -1.0 *  pose.pose.position.y;
   // ins.longitude = pose.pose.position.x;
   // ins.altitude = pose.pose.position.z;
   ins.original_ins.latitude = pos(0);
   ins.original_ins.longitude = pos(1);
   ins.original_ins.altitude = pos(2);

    ins.original_ins.east_speed = -1.0 * vel(1);
    ins.original_ins.north_speed = vel(0);
    ins.original_ins.sky_speed = vel(2);
    ins.original_ins.ground_speed = sqrt(vel(1) * vel(1) + vel(0) * vel(0));

    Eigen::Quaterniond  qua;
    qua.x() = pose.pose.orientation.x;
    qua.y() = pose.pose.orientation.y;
    qua.z() = pose.pose.orientation.z;
    qua.w() = pose.pose.orientation.w;
    Eigen::Matrix3d R_qua;
    R_qua = qua.toRotationMatrix();
    R_qua = R * R_qua;
    Vector3d att = Earth::matrix2euler(R_qua) * R2D;
    att(2) = -1.0 * att(2) + 90;

    if(att(2) >= 180.0)
        att(2) -= 360.0;

    if(att(2) <= -180.0)
        att(2) += 360.0;

    ins.original_ins.yaw = att(2) * D2R;
    ins.original_ins.pitch = att(0) * D2R;
    ins.original_ins.roll = att(1) * D2R;

    ins.original_ins.ins_state = temp_vaid;

    sensor_msgs::Imu last_msg;
    if(imu_v.empty())
    {
        ins.original_ins.acc_x = 0.0;
        ins.original_ins.acc_y = 0.0;
        ins.original_ins.acc_z = 0.0;
        ins.original_ins.angular_x = 0.0;
        ins.original_ins.angular_y= 0.0;
        ins.original_ins.angular_z = 0.0;
        ins_pub_.publish(ins);
        return;
    }

    ins.original_ins.acc_x = last_msg.linear_acceleration.x;
    ins.original_ins.acc_y = last_msg.linear_acceleration.y;
    ins.original_ins.acc_z = last_msg.linear_acceleration.z;
    ins.original_ins.angular_x = last_msg.angular_velocity.x;
    ins.original_ins.angular_y= last_msg.angular_velocity.x;
    ins.original_ins.angular_z = last_msg.angular_velocity.x;
    ins_v.push_back(ins);
    ins_pub_.publish(ins);

}

void NdtLocalizer::timer_diagnostic()
{
  ros::Rate rate(100);
  while (ros::ok()) {
    diagnostic_msgs::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "ndt_scan_matcher";
    diag_status_msg.hardware_id = "";

    for (const auto & key_value : key_value_stdmap_) {
      diagnostic_msgs::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing") {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_status_msg.message += "Initializing State. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1) {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5) {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }

    diagnostic_msgs::DiagnosticArray diag_msg;
    diag_msg.header.stamp = ros::Time::now();
    diag_msg.status.push_back(diag_status_msg);

    diagnostics_pub_.publish(diag_msg);

    rate.sleep();
  }
}

void NdtLocalizer::callback_imu(const sensor_msgs::Imu::ConstPtr & imu_msg_ptr)
{
    sensor_msgs::Imu imu_msg = *imu_msg_ptr;
    imu_v.push(imu_msg);
}

void NdtLocalizer::callback_init_pose(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & initial_pose_msg_ptr)
{
  if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
    initial_pose_cov_msg_ = *initial_pose_msg_ptr;
  } else {
    // get TF from pose_frame to map_frame
    geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
    get_transform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    // transform pose_frame to map_frame
    geometry_msgs::PoseWithCovarianceStamped::Ptr mapTF_initial_pose_msg_ptr(
      new geometry_msgs::PoseWithCovarianceStamped);
    tf2::doTransform(*initial_pose_msg_ptr, *mapTF_initial_pose_msg_ptr, *TF_pose_to_map_ptr);
    // mapTF_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
    initial_pose_cov_msg_ = *mapTF_initial_pose_msg_ptr;
  }
  // if click the initpose again, re init！
  init_pose = false;
}

void NdtLocalizer::callback_pointsmap(
  const sensor_msgs::PointCloud2::ConstPtr & map_points_msg_ptr)
{
  const auto trans_epsilon = ndt_.getTransformationEpsilon();
  const auto step_size = ndt_.getStepSize();
  const auto resolution = ndt_.getResolution();
  const auto max_iterations = ndt_.getMaximumIterations();

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_new;

  ndt_new.setTransformationEpsilon(trans_epsilon);
  ndt_new.setStepSize(step_size);
  ndt_new.setResolution(resolution);
  ndt_new.setMaximumIterations(max_iterations);

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
  ndt_new.setInputTarget(map_points_ptr);
  // create Thread
  // detach
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt_new.align(*output_cloud, Eigen::Matrix4f::Identity());
  map_cloud = *map_points_ptr;

  // swap
  ndt_map_mtx_.lock();
  ndt_ = ndt_new;
  ndt_map_mtx_.unlock();
}

void NdtLocalizer::callback_pointcloud(
  const sensor_msgs::PointCloud2::ConstPtr & sensor_points_sensorTF_msg_ptr)
{
  const auto exe_start_time = std::chrono::system_clock::now();
  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  //const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
  const std::string sensor_frame = "rslidar";
  const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
  // get TF base to sensor
  geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr(new geometry_msgs::TransformStamped);
  get_transform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);

  Eigen::Matrix4f initial_pose_matrix;

  if(first_lidar && online_calib)
  {

      if(map_cloud.points.size() == 0)
      {
          std::cout << "wrong map grid path" << std::endl;
          return;
      }

      bool index = auto_calib(*sensor_points_sensorTF_ptr);

      if(!index)
      {
          std::cout << "init is failed!" << std::endl;
          return;
      }

      first_lidar = false;

      //R = R_i.inverse();
      initial_pose_matrix = app.current_guess_;
      pre_trans = initial_pose_matrix;
      init_pose = true;

      std::cout << "-------------------------" << std::endl;
      std::cout << "online calib finish!!!!" << std::endl;
      std::cout << "online calib finish!!!!" << std::endl;
      std::cout << "online calib finish!!!!" << std::endl;
      std::cout << "online calib finish!!!!" << std::endl;
      std::cout << "-------------------------" << std::endl;
     // std::cout << TF_base_to_sensor_ptr->transform.translation.x << ", " << TF_base_to_sensor_ptr->transform.translation.y << ", " << TF_base_to_sensor_ptr->transform.translation.z << std::endl;
     // std::cout << temp.x() << ", " << temp.y() << ", " << temp.z() << ", " << temp.z() << std::endl;


  }
  else
  {
      // use predicted pose as init guess (currently we only impl linear model)
      initial_pose_matrix = pre_trans * delta_trans;
  }

  const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
  const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_baselinkTF_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(
    *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);

  // set input point cloud
  ndt_.setInputSource(sensor_points_baselinkTF_ptr);

  if (ndt_.getInputTarget() == nullptr) {
    ROS_WARN_STREAM_THROTTLE(1, "No MAP!");
    return;
  }
  // align
//  Eigen::Matrix4f initial_pose_matrix;
//  if (!init_pose ){
//    Eigen::Affine3d initial_pose_affine;
//    tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
//    initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
//    // for the first time, we don't know the pre_trans, so just use the init_trans,
//    // which means, the delta trans for the second time is 0
//    pre_trans = initial_pose_matrix;
//    init_pose = true;
//  }else
//  {
//    // use predicted pose as init guess (currently we only impl linear model)
//    initial_pose_matrix = pre_trans * delta_trans;
//  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  const auto align_start_time = std::chrono::system_clock::now();
  key_value_stdmap_["state"] = "Aligning";
  ndt_.align(*output_cloud, initial_pose_matrix);
  key_value_stdmap_["state"] = "Sleeping";
  const auto align_end_time = std::chrono::system_clock::now();
  const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() /1000.0;

  const Eigen::Matrix4f result_pose_matrix = ndt_.getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

  const float transform_probability = ndt_.getTransformationProbability();
  const int iteration_num = ndt_.getFinalNumIteration();

  bool is_converged = true;
  static size_t skipping_publish_num = 0;

  if (
    iteration_num >= ndt_.getMaximumIterations() + 2 ||
    transform_probability < converged_param_transform_probability_) {
    is_converged = false;
    ++skipping_publish_num;
    double temp_vaid_d = (1.0 * transform_probability) / (1.0 * converged_param_transform_probability_);
    if(temp_vaid_d >= 0.5)
         temp_vaid = 1;
    else
    {
        std::cout << "vaif is zeros !!!!!!!!!!!!!!" << std::endl;
        temp_vaid = 0;
    } 
    std::cout << "Not Converged" << std::endl;
  } else {
    skipping_publish_num = 0;
    temp_vaid = 1;
  }
  // calculate the delta tf from pre_trans to current_trans
  delta_trans = pre_trans.inverse() * result_pose_matrix;

  Eigen::Vector3f delta_translation = delta_trans.block<3, 1>(0, 3);
  std::cout<<"delta x: "<<delta_translation(0) << " y: "<<delta_translation(1)<<
             " z: "<<delta_translation(2)<<std::endl;

  Eigen::Matrix3f delta_rotation_matrix = delta_trans.block<3, 3>(0, 0);
  Eigen::Vector3f delta_euler = delta_rotation_matrix.eulerAngles(2,1,0);
  std::cout<<"delta yaw: "<<delta_euler(0) << " pitch: "<<delta_euler(1)<<
             " roll: "<<delta_euler(2)<<std::endl;

  pre_trans = result_pose_matrix;

  // publish
  geometry_msgs::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;

  //if (is_converged) 
  {
    ndt_pose_pub_.publish(result_pose_stamped_msg);
  }

  // publish tf(map frame to base frame)
  publish_tf(map_frame_, base_frame_, result_pose_stamped_msg);

  // publish aligned point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(
    *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
  sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = map_frame_;
  sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);


  std_msgs::Float32 exe_time_msg;
  exe_time_msg.data = exe_time;
  exe_time_pub_.publish(exe_time_msg);

  std_msgs::Float32 transform_probability_msg;
  transform_probability_msg.data = transform_probability;
  transform_probability_pub_.publish(transform_probability_msg);

  std_msgs::Float32 iteration_num_msg;
  iteration_num_msg.data = iteration_num;
  iteration_num_pub_.publish(iteration_num_msg);

  key_value_stdmap_["seq"] = std::to_string(sensor_points_sensorTF_msg_ptr->header.seq);
  key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
  key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);

  std::cout << "------------------------------------------------" << std::endl;
  std::cout << "align_time: " << align_time << "ms" << std::endl;
  std::cout << "exe_time: " << exe_time << "ms" << std::endl;
  std::cout << "trans_prob: " << transform_probability << std::endl;
  std::cout << "iter_num: " << iteration_num << std::endl;
  std::cout << "skipping_publish_num: " << skipping_publish_num << std::endl;
}

bool NdtLocalizer::auto_calib(pcl::PointCloud<pcl::PointXYZ>& local_lidar)
{
    //std::cout << "auto calib" << std::endl;

    // calibration
    //标定
    pcl::PointCloud<pcl::PointXYZ> local_lidar_ = local_lidar;

    double cost = app.PerformNdtOptimize(map_cloud, local_lidar_);

    return cost;
}

//bool NdtLocalizer::auto_calib(pcl::PointCloud<pcl::PointXYZ>& local_lidar)
//{
//    //std::cout << "auto calib" << std::endl;
//
//    // calibration
//    //标定
//    pcl::PointCloud<pcl::PointXYZ> local_lidar_ = local_lidar;
//
//    for(int i = 0; i < pose_r.size(); i++)
//    {
//        double cost = app.PerformNdtOptimize(map_cloud, local_lidar_, pose_r[i]);
//
//        if(cost)
//        {
//            std::cout << "find position" <<std::endl;
//            std::cout << i << std::endl;
//            return cost;
//        }
//
//    }
//    return 0;
//}

void NdtLocalizer::init_params(){

  private_nh_.getParam("base_frame", base_frame_);
  ROS_INFO("base_frame_id: %s", base_frame_.c_str());

  double trans_epsilon = ndt_.getTransformationEpsilon();
  double step_size = ndt_.getStepSize();
  double resolution = ndt_.getResolution();
  int max_iterations = ndt_.getMaximumIterations();

  private_nh_.getParam("trans_epsilon", trans_epsilon);
  private_nh_.getParam("step_size", step_size);
  private_nh_.getParam("resolution", resolution);
  private_nh_.getParam("max_iterations", max_iterations);
  private_nh_.getParam("init_latitude", init_latitude);
  private_nh_.getParam("init_longitude", init_longitude);
  private_nh_.getParam("init_altitude", init_altitude);
  private_nh_.getParam("input_imu_topic", input_imu_topic);
  private_nh_.param<bool>("online_calib", online_calib, true);
  private_nh_.param<std::string>("map_grid_path", map_grid_path, "");
  private_nh_.param<int>("max_score", app.max_score, 100);
  private_nh_.param<std::string>("path_txt", path_txt, "/home/cf206/Desktop/ndt_localizer-master-new/pose.txt");
    private_nh_.param<std::string>("ins_txt", ins_txt, "/home/cf206/Desktop/ndt_localizer-master-new/ins.txt");
  app.ndt_epsilon_=trans_epsilon;
  app.ndt_step_size_ = step_size;
  app.ndt_resolution_ = resolution;
  app.ndt_iterations_=max_iterations;

  origin(0) = init_latitude;
  origin(1) = init_longitude;
  origin(2) = init_altitude;
  origin *= D2R;
  last_pos << 0.0, 0.0, 0.0;

  std::cout << origin(0) << ",  " << origin(1) << ", " << origin(2) << std::endl;

  map_frame_ = "map";

  ndt_.setTransformationEpsilon(trans_epsilon);
  ndt_.setStepSize(step_size);
  ndt_.setResolution(resolution);
  ndt_.setMaximumIterations(max_iterations);

  ROS_INFO(
    "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon,
    step_size, resolution, max_iterations);

  private_nh_.getParam(
    "converged_param_transform_probability", converged_param_transform_probability_);
}


bool NdtLocalizer::get_transform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr, const ros::Time & time_stamp)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, time_stamp);
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool NdtLocalizer::get_transform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

void NdtLocalizer::publish_tf(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::PoseStamped & pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_localizer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    //read_path();

    NdtLocalizer ndt_localizer(nh, private_nh);

    ros::spin();

    return 0;
}
