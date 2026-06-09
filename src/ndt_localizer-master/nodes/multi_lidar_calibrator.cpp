#include "multi_lidar_calibrator.h"


bool ROSMultiLidarCalibratorApp::PerformNdtOptimize(const pcl::PointCloud<PointT> map, const pcl::PointCloud<PointT> source_point, const Eigen::Matrix4d pose){

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    current_guess_ << pose(0,0), pose(0,1), pose(0,2), pose(0,3),
                      pose(1,0), pose(1,1), pose(1,2), pose(1,3),
                      pose(2,0), pose(2,1), pose(2,2), pose(2,3),
                      0.0, 0.0, 0.0, 1.0;

    ndt.setTransformationEpsilon(ndt_epsilon_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setResolution(ndt_resolution_);

    ndt.setMaximumIterations(ndt_iterations_);

    pcl::PointCloud<PointT>::Ptr in_parent_cloud_ (new pcl::PointCloud<PointT>);
    *in_parent_cloud_ = map;
    pcl::PointCloud<PointT>::Ptr in_child_cloud_ (new pcl::PointCloud<PointT>);
    *in_child_cloud_ = source_point;
    //pcl::io::savePCDFile("/media/cf206/512/multi_lidars_calibration-NDT_ws/map.pcd", *in_parent_cloud_);

    //滤波后的点云
    ndt.setInputSource(in_child_cloud_);
    //配准目标点云
    ndt.setInputTarget(in_parent_cloud_);

    pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);

    //如果当前变换为单位矩阵,即无初始估计,则从文件中读取的构建齐次变换矩阵
    if(current_guess_ == Eigen::Matrix4f::Identity())
    {
        //三维空间表示变换矩阵,即旋转与平移
        //初始平移
        Eigen::Translation3f init_translation(0.0,
                0.0, 0.0);
        //初始旋转
        Eigen::AngleAxisf init_rotation_x(0.0, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(0.0, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(0.0, Eigen::Vector3f::UnitZ());

        Eigen::Matrix4f init_guess_ = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

        current_guess_ = init_guess_;
    }

    //采用current_guess_初始估计,来初步对齐点云
    ndt.align(*output_cloud, current_guess_);

    const float transform_Score = ndt.getFitnessScore();
    const int iteration_num = ndt.getFinalNumIteration();
    bool is_converged = false;

    if (iteration_num <= ndt.getMaximumIterations() + 2 && (abs(ndt.getTransformationProbability() - 4.0) <= 0.001 || ndt.getTransformationProbability() > 4.0))
           // transform_Score <= max_score) 
{
            is_converged = true;
    }

   std::cout  << max_score << std::endl;

    std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged ()
              << " score: " << ndt.getFitnessScore () << " prob:" << ndt.getTransformationProbability() << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    //
    pcl::transformPointCloud (*in_child_cloud_, *output_cloud, ndt.getFinalTransformation());
   // pcl::io::savePCDFile("/media/cf206/512/multi_lidars_calibration-NDT_ws/transformed_map.pcd", *output_cloud);
    
    //获取最终的配准的转化矩阵，即原始点云到目标点云的刚体变换，返回Matrix4数据类型，该数据类型采用了另一个专门用于矩阵计算的开源c++库eigen
    current_guess_ = ndt.getFinalTransformation();
    
    //欧式变换矩阵提取旋转矩阵  0,0是指从矩阵的第0行第0列位置开始,取3×3列,是旋转矩阵
    Eigen::Matrix3f rotation_matrix = current_guess_.block(0,0,3,3);
    //欧式变换矩阵提取平移矩阵
    Eigen::Vector3f translation_vector = current_guess_.block(0,3,3,1);

    std::cout << "This transformation can be replicated using:" << std::endl;
    std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
              << " " << rotation_matrix.eulerAngles(2,1,0).transpose()   << std::endl;

    std::cout << "Corresponding transformation matrix:" << std::endl
              << std::endl << current_guess_ << std::endl << std::endl;

    return is_converged;

}


bool ROSMultiLidarCalibratorApp::PerformNdtOptimize(const pcl::PointCloud<PointT> map, const pcl::PointCloud<PointT> source_point){

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    current_guess_ = Eigen::Matrix4f::Identity();

    ndt.setTransformationEpsilon(ndt_epsilon_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setResolution(ndt_resolution_);

    ndt.setMaximumIterations(ndt_iterations_);

    pcl::PointCloud<PointT>::Ptr in_parent_cloud_ (new pcl::PointCloud<PointT>);
    *in_parent_cloud_ = map;
    pcl::PointCloud<PointT>::Ptr in_child_cloud_ (new pcl::PointCloud<PointT>);
    *in_child_cloud_ = source_point;
    //pcl::io::savePCDFile("/media/cf206/512/multi_lidars_calibration-NDT_ws/map.pcd", *in_parent_cloud_);

    //滤波后的点云
    ndt.setInputSource(in_child_cloud_);
    //配准目标点云
    ndt.setInputTarget(in_parent_cloud_);

    pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);

    //如果当前变换为单位矩阵,即无初始估计,则从文件中读取的构建齐次变换矩阵
    if(current_guess_ == Eigen::Matrix4f::Identity())
    {
        //三维空间表示变换矩阵,即旋转与平移
        //初始平移
        Eigen::Translation3f init_translation(0.0,
                                              0.0, 0.0);
        //初始旋转
        Eigen::AngleAxisf init_rotation_x(0.0, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(0.0, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(0.0, Eigen::Vector3f::UnitZ());

        Eigen::Matrix4f init_guess_ = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

        current_guess_ = init_guess_;
    }

    //采用current_guess_初始估计,来初步对齐点云
    ndt.align(*output_cloud, current_guess_);

    const float transform_Score = ndt.getFitnessScore();
    const int iteration_num = ndt.getFinalNumIteration();
    bool is_converged = false;

    if (iteration_num <= ndt.getMaximumIterations() + 2 &&ndt.getTransformationProbability() >= 5)
        // transform_Score <= max_score)
    {
        is_converged = true;
    }

    std::cout  << max_score << std::endl;

    std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged ()
              << " score: " << ndt.getFitnessScore () << " prob:" << ndt.getTransformationProbability() << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    //
    pcl::transformPointCloud (*in_child_cloud_, *output_cloud, ndt.getFinalTransformation());
    // pcl::io::savePCDFile("/media/cf206/512/multi_lidars_calibration-NDT_ws/transformed_map.pcd", *output_cloud);

    //获取最终的配准的转化矩阵，即原始点云到目标点云的刚体变换，返回Matrix4数据类型，该数据类型采用了另一个专门用于矩阵计算的开源c++库eigen
    current_guess_ = ndt.getFinalTransformation();

    //欧式变换矩阵提取旋转矩阵  0,0是指从矩阵的第0行第0列位置开始,取3×3列,是旋转矩阵
    Eigen::Matrix3f rotation_matrix = current_guess_.block(0,0,3,3);
    //欧式变换矩阵提取平移矩阵
    Eigen::Vector3f translation_vector = current_guess_.block(0,3,3,1);

    std::cout << "This transformation can be replicated using:" << std::endl;
    std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
              << " " << rotation_matrix.eulerAngles(2,1,0).transpose()   << std::endl;

    std::cout << "Corresponding transformation matrix:" << std::endl
              << std::endl << current_guess_ << std::endl << std::endl;

    return is_converged;

}

ROSMultiLidarCalibratorApp::ROSMultiLidarCalibratorApp()
{
     
}
