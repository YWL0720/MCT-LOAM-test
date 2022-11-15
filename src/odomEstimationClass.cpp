#include "odomEstimationClass.h"
#include "LidarFactor.hpp"

void OdomEstimationClass::init(lidar::Lidar lidar_param, double map_resolution){
    //init local map
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    //downsampling size
    downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
    downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

    //kd-tree
    kdtreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    

    // 初始化当前帧里程计和前一帧里程计
    odom_curr_middle = Eigen::Isometry3d::Identity();
    odom_curr_end = Eigen::Isometry3d::Identity();
    odom_last_end = Eigen::Isometry3d::Identity();


    optimization_count=2;
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in){
    *laserCloudCornerMap += *edge_in;
    *laserCloudSurfMap += *surf_in;
    optimization_count=12;
}


void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in){

    static int frame = 0;
    ROS_INFO("=======fram %d======", frame);
    frame++;

    if(optimization_count>2)
        optimization_count--;
/*
    Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
    last_odom = odom;
    odom = odom_prediction;

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();
*/
    /**********************************************************************/
    // 更新前一帧位姿 当前帧成为前一帧
    odom_last_middle = odom_curr_middle;
    odom_last_end = odom_curr_end;

    q_w_last_end = Eigen::Quaterniond(odom_last_end.rotation());
    t_w_last_end = odom_last_end.translation();

    /**
     * 当前帧中间时刻和结束时刻的速度预测
     * 匀速模型假设
     * 当前帧中间时刻的预测位姿 = 上一帧结束时刻的位姿 * 速度
     * 当前帧结束时刻的预测位姿 = 当前帧中间时刻的预测位姿 * 速度
     * 速度 = (上一帧中间时刻的位姿.inverse * 上一帧结束时刻的位姿)
     */
    Eigen::Isometry3d velocity = odom_last_middle.inverse() * odom_last_end;
    Eigen::Isometry3d odom_curr_middle_prediction = odom_last_end * velocity;
    Eigen::Isometry3d odom_curr_end_prediction = odom_last_end * velocity * velocity;

    odom_curr_middle = odom_curr_middle_prediction;
    odom_curr_end = odom_curr_end_prediction;

    // 对优化变量更新预测初值
    q_w_curr_middle = Eigen::Quaterniond(odom_curr_middle.rotation());
    t_w_curr_middle = odom_curr_middle.translation();

    q_w_curr_end = Eigen::Quaterniond(odom_curr_end.rotation());
    t_w_curr_end = odom_curr_end.translation();
    /**********************************************************************/

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
    downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud);
    //ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
    if(laserCloudCornerMap->points.size()>10 && laserCloudSurfMap->points.size()>50)
    {
        kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

        for (int iterCount = 0; iterCount < optimization_count; iterCount++)
        {
            ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            // 添加参数块 旋转采用EigenQuaternionParameterization
            // 四个参数 分别为当前帧中间时刻的姿态 位置 当前帧结束时刻的姿态 位置
            problem.AddParameterBlock(parameters_middle_q, 4, new ceres::EigenQuaternionParameterization);
            problem.AddParameterBlock(parameters_middle_t, 3);
            problem.AddParameterBlock(parameters_end_q, 4, new ceres::EigenQuaternionParameterization);
            problem.AddParameterBlock(parameters_end_t, 3);

            addEdgeCostFactor(downsampledEdgeCloud,laserCloudCornerMap,problem,loss_function);
            addSurfCostFactor(downsampledSurfCloud,laserCloudSurfMap,problem,loss_function);

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 10;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << std::endl;



        }

        // 优化结束 更新当前帧的里程计状态量
        odom_curr_middle = Eigen::Isometry3d::Identity();
        odom_curr_middle.linear() = q_w_curr_middle.toRotationMatrix();
        odom_curr_middle.translation() = t_w_curr_middle;

        odom_curr_end = Eigen::Isometry3d::Identity();
        odom_curr_end.linear() = q_w_curr_end.toRotationMatrix();
        odom_curr_end.translation() = t_w_curr_end;
    }
    else
        ROS_WARN("not enough points in map to associate, map error");


    addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud);

}

/*!
 * @brief 将雷达坐标系下的点对齐到世界坐标系下
 * @param pi
 * @param po
 */
void OdomEstimationClass::pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    double time = pi->intensity;
    Eigen::Vector3d curr_point(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w;
    if (time <= 0.5)
    {
        // 位于sweep的前半段
        time = time * 2;
        Eigen::Quaterniond q_w_curr_point = q_w_last_end.slerp(time, q_w_curr_middle);
        Eigen::Vector3d t_w_curr_point = t_w_last_end + time * (t_w_curr_middle - t_w_last_end);
        point_w = q_w_curr_point * curr_point + t_w_curr_point;
    }
    else
    {
        // 位于sweep的后半段
        time = (time * 2) - 1;
        Eigen::Quaterniond q_w_curr_point = q_w_curr_middle.slerp(time, q_w_curr_end);
        Eigen::Vector3d t_w_curr_point = t_w_curr_middle + time * (t_w_curr_end - t_w_curr_middle);
        point_w = q_w_curr_point * curr_point + t_w_curr_point;
    }

    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;


}

void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out){
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);    
}

void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int corner_num=0;
    int corner_num_first = 0;
    int corner_num_second = 0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
        // 调整阈值 1.0 改为 2.0
        if (pointSearchSqDis[4] < 1.0)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            double time = pc_in->points[i].intensity;
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            { 
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                //ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
                //problem.AddResidualBlock(cost_function, loss_function, parameters);
                if (time <= 0.5)
                {
                    ceres::CostFunction* cost_function = LidarFirstEdgeFactor::Create(curr_point, point_a, point_b, q_w_last_end, t_w_last_end, time);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_middle_q, parameters_middle_t);

                    ceres::CostFunction* velocity_factor = VelocityConsistentyFactor::Create(q_w_last_end, t_w_last_end);
                    problem.AddResidualBlock(velocity_factor, loss_function, parameters_middle_q, parameters_middle_t, parameters_end_q, parameters_end_t);

                    corner_num_first++;
                }
                else
                {
                    ceres::CostFunction* cost_function = LidarSecondEdgeFactor::Create(curr_point, point_a, point_b, time);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_middle_q, parameters_middle_t, parameters_end_q, parameters_end_t);
                    corner_num_second++;

                    ceres::CostFunction* velocity_factor = VelocityConsistentyFactor::Create(q_w_last_end, t_w_last_end);
                    problem.AddResidualBlock(velocity_factor, loss_function, parameters_middle_q, parameters_middle_t, parameters_end_q, parameters_end_t);

                }

                corner_num++;

            }                           
        }
    }
    if(corner_num_first < 10)
        ROS_ERROR("Only %d correct edge points in first sweep", corner_num_first);
    else if (corner_num_second < 10)
        ROS_ERROR("Only %d correct edge points in second sweep", corner_num_second);
    else
    {
        ROS_INFO("There are %d correct edge points in first sweep ", corner_num_first);
        ROS_INFO("There are %d correct edge points in second sweep ", corner_num_second);

    }

}

void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int surf_num=0;
    int surf_num_first = 0;
    int surf_num_second = 0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 2.0)
        {
            
            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            double time = pc_in->points[i].intensity;
            if (planeValid)
            {
                // ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                // problem.AddResidualBlock(cost_function, loss_function, parameters);
                if (time <= 0.5)
                {
                    ceres::CostFunction* cost_function = LidarFirstPlaneFactor::Create(curr_point, norm, negative_OA_dot_norm, q_w_last_end, t_w_last_end, time);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_middle_q, parameters_middle_t);

                    ceres::CostFunction* velocity_factor = VelocityConsistentyFactor::Create(q_w_last_end, t_w_last_end);
                    problem.AddResidualBlock(velocity_factor, loss_function, parameters_middle_q, parameters_middle_t, parameters_end_q, parameters_end_t);
                    surf_num_first++;
                }
                else
                {
                    ceres::CostFunction* cost_function = LidarSecondPlaneFactor::Create(curr_point, norm, negative_OA_dot_norm, time);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_middle_q, parameters_middle_t, parameters_end_q, parameters_end_t);

                    ceres::CostFunction* velocity_factor = VelocityConsistentyFactor::Create(q_w_last_end, t_w_last_end);
                    problem.AddResidualBlock(velocity_factor, loss_function, parameters_middle_q, parameters_middle_t, parameters_end_q, parameters_end_t);
                    surf_num_second++;
                }


                surf_num++;

            }

        }

    }
    if(surf_num_first < 10)
        ROS_ERROR("Only %d surf correct points in first sweep", surf_num_first);
    else if(surf_num_second < 10)
        ROS_ERROR("Only %d surf correct points in second sweep", surf_num_second);
    else
    {
        ROS_INFO("There are %d correct surf points in first sweep", surf_num_first);
        ROS_INFO("There are %d correct surf points in second sweep", surf_num_second);
    }


}


void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud){

    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp);
        laserCloudCornerMap->push_back(point_temp); 
    }
    
    for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp);
        laserCloudSurfMap->push_back(point_temp);
    }
    
    double x_min = +odom.translation().x()-100;
    double y_min = +odom.translation().y()-100;
    double z_min = +odom.translation().z()-100;
    double x_max = +odom.translation().x()+100;
    double y_max = +odom.translation().y()+100;
    double z_max = +odom.translation().z()+100;
    
    //ROS_INFO("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
    cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    cropBoxFilter.setNegative(false);    

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZI>());
    cropBoxFilter.setInputCloud(laserCloudSurfMap);
    cropBoxFilter.filter(*tmpSurf);
    cropBoxFilter.setInputCloud(laserCloudCornerMap);
    cropBoxFilter.filter(*tmpCorner);

    downSizeFilterSurf.setInputCloud(tmpSurf);
    downSizeFilterSurf.filter(*laserCloudSurfMap);
    downSizeFilterEdge.setInputCloud(tmpCorner);
    downSizeFilterEdge.filter(*laserCloudCornerMap);

}

void OdomEstimationClass::getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap){
    
    *laserCloudMap += *laserCloudSurfMap;
    *laserCloudMap += *laserCloudCornerMap;
}

OdomEstimationClass::OdomEstimationClass(){

}
