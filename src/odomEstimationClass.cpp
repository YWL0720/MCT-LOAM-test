#include "odomEstimationClass.h"
#include "lidarOptimization.hpp"

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

    odom_m = Eigen::Isometry3d::Identity();
    odom_e = Eigen::Isometry3d::Identity();
    last_odom_m = Eigen::Isometry3d::Identity();
    last_odom_e = Eigen::Isometry3d::Identity();
    velocity = Eigen::Isometry3d::Identity();

    optimization_count=2;
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in){
    *laserCloudCornerMap += *edge_in;
    *laserCloudSurfMap += *surf_in;
    optimization_count=10;
}


void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in){

    if(optimization_count>2)
        optimization_count--;

    static int optimize_num = 1;
    std::cout << "Frame " << optimize_num << std::endl;
    /****************************************************/
    // 更新当前帧中间时刻和结束时刻位姿的预测值
    // 利用前一帧两段速度的平均值进行更新
    Eigen::Isometry3d velocity_first = last_odom_e.inverse() * odom_m;
    Eigen::Isometry3d velocity_second = odom_m.inverse() * odom_e;

    Eigen::Quaterniond velocity_first_q(velocity_first.rotation());
    Eigen::Vector3d velocity_first_t = velocity_first.translation();

    Eigen::Quaterniond velocity_second_q(velocity_second.rotation());
    Eigen::Vector3d velocity_second_t = velocity_second.translation();

    Eigen::Quaterniond velocity_average_q = velocity_first_q.slerp(0.5, velocity_second_q);
    Eigen::Vector3d velocity_average_t = velocity_first_t + 0.5 * (velocity_second_t - velocity_first_t);

    velocity = Eigen::Isometry3d::Identity();
    velocity.rotate(velocity_average_q);
    velocity.translation() = velocity_average_t;

    velocity_history.push_back(velocity);

    Eigen::Isometry3d odom_predicition_middle = odom_e * velocity;
    Eigen::Isometry3d odom_predicition_end = odom_e * velocity * velocity;

    // Eigen::Isometry3d odom_predicition_middle = odom_e * (odom_m.inverse() * odom_e);
    // Eigen::Isometry3d odom_prediction_end = odom_e * (odom_m.inverse() * odom_e) ;


    last_odom_m = odom_m;
    last_odom_e = odom_e;

    odom_m = odom_predicition_middle;
    odom_e = odom_predicition_end;

    q_w_curr_m = Eigen::Quaterniond(odom_m.rotation());
    t_w_curr_m = odom_m.translation();

    q_w_curr_e = Eigen::Quaterniond(odom_e.rotation());
    t_w_curr_e = odom_e.translation();

    q_w_last_end = Eigen::Quaterniond(last_odom_e.rotation());
    t_w_last_end = last_odom_e.translation();

    q_w_last_middle = Eigen::Quaterniond(last_odom_m.rotation());
    t_w_last_middle = last_odom_m.translation();
    /****************************************************/

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
    downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud);
    //ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
    if(laserCloudCornerMap->points.size()>10 && laserCloudSurfMap->points.size()>50){
        kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        kdtreeSurfMap->setInputCloud(laserCloudSurfMap);


        /********************************************************************************/
        // 优化部分
        for (int iterCount = 0; iterCount < optimization_count; iterCount++)
        {
            ceres::LossFunction* lossFunction = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(parameters_middle, 7, new PoseSE3Parameterization());
            problem.AddParameterBlock(parameters_end, 7, new PoseSE3Parameterization());
            addEdgeCostFactorNew(downsampledEdgeCloud, laserCloudCornerMap, problem, lossFunction);
            addSurfCostFactorNew(downsampledSurfCloud, laserCloudSurfMap, problem, lossFunction);
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            if (optimization_count > 2)
                options.max_num_iterations = 15;
            else
                options.max_num_iterations = 10;

            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);

            std::cout << summary.BriefReport()<< std::endl;



        }
        /********************************************************************************/

    }else{
        ROS_WARN("not enough points in map to associate, map error");
    }

    odom_m = Eigen::Isometry3d::Identity();
    odom_m.linear() = q_w_curr_m.toRotationMatrix();
    odom_m.translation() = t_w_curr_m;

    odom_e = Eigen::Isometry3d ::Identity();
    odom_e.linear() = q_w_curr_e.toRotationMatrix();
    odom_e.translation() = t_w_curr_e;


    /*
    std::cout << "Current sweep odom_middle = " << std::endl << odom_m.matrix() << std::endl;
    std::cout << "Current sweep odom_end = " << std::endl << odom_e.matrix() << std::endl;

    Eigen::Isometry3d velocity_ = odom_m.inverse() * odom_e;

    std::cout << "Current sweep velocity = " << std::endl << velocity_.matrix() << std::endl;

    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

    Eigen::Isometry3d odom_m_next_p = odom_e * velocity_;
    Eigen::Isometry3d odom_e_next_p = odom_e * velocity_ * velocity_;
    std::cout << "Next sweep odom_middle = " << std::endl << odom_m_next_p.matrix() << std::endl;
    std::cout << "Next sweep odom_end = " << std::endl << odom_e_next_p.matrix() << std::endl;
    std::cout << "=====================================================" << std::endl;
    */
    optimize_num++;
    addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud);


}

/***********************************************************************************************************/
/*!
 * @brief 根据当前帧中间时刻和结束时刻的位姿，将当前帧特征点对齐到世界坐标系下
 * @param pi
 * @param po
 */
void OdomEstimationClass::pointAssociateToMapNew(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    double time = pi->intensity;
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);

    if (time <= 0.5)
    {
        // 当前点位于sweep前半段，用中间时刻位姿表示
        // 当前点时刻的世界位姿
        Eigen::Quaterniond q_w_point = q_w_last_end.slerp(time * 2, q_w_curr_m);
        Eigen::Vector3d t_w_point = t_w_last_end + time * 2 * (t_w_curr_m - t_w_last_end);

        Eigen::Vector3d point_w = q_w_point * point_curr + t_w_point;

        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }
    else
    {
        // 当前点位于sweep后半段，用中间时刻位姿和结束时刻位姿表示
        Eigen::Quaterniond q_w_point = q_w_curr_m.slerp(2 * time - 1, q_w_curr_e);
        Eigen::Vector3d t_w_point = t_w_curr_m + (2 * time - 1) * (t_w_curr_e - t_w_curr_m);

        Eigen::Vector3d point_w = q_w_point * point_curr + t_w_point;

        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }

}
/***********************************************************************************************************/

void OdomEstimationClass::pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    // 修改
    double time = pi->intensity;
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w;
    if (time <= 0.5)
    {
        time = time * 2;
        Eigen::Quaterniond q_w_curr_point = q_w_last_end.slerp(time, q_w_curr_m);
        Eigen::Vector3d t_w_curr_point = t_w_last_end + time * (t_w_curr_m - t_w_last_end);
        point_w = q_w_curr_point * point_curr + t_w_curr_point;
    }
    else
    {
        time = time * 2 -1;
        Eigen::Quaterniond q_w_curr_point = q_w_curr_m.slerp(time, q_w_curr_e);
        Eigen::Vector3d t_w_curr_point = t_w_curr_m + time * (t_w_curr_e - t_w_curr_m);
        point_w = q_w_curr_point * point_curr + t_w_curr_point;
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
/*********************************************************************************************************************************************/
/*!
 * @brief 添加线特征约束
 * @param pc_in
 * @param map_in
 * @param problem
 * @param loss_function
 */
void OdomEstimationClass::addEdgeCostFactorNew(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction* loss_function)
{
    int corner_num = 0;
    int corner_first_num = 0;
    int corner_second_num = 0;
    for (int i=0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMapNew(&pc_in->points[i], &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
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

            double curr_time = pc_in->points[i].intensity;

            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            {
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                if (curr_time <= 0.5)
                {
                    // 前半段 添加中间位姿的约束
                    ceres::CostFunction* cost_function = LidarFirstEdgeFactor::Create(curr_point, point_a, point_b, q_w_last_end, t_w_last_end, curr_time);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_middle);
                    corner_first_num++;
                }
                else
                {
                    // 后半段 添加中间位姿和结束位姿的约束
                    ceres::CostFunction* cost_function = LidarSecondEdgeFactor::Create(curr_point, point_a, point_b, curr_time);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_middle, parameters_end);

                    ceres::CostFunction* cost_velocity = VelocityConsistentyFactor::Create(q_w_last_end, t_w_last_end);
                    //problem.AddResidualBlock(cost_velocity, loss_function, parameters_middle, parameters_end);
                    corner_second_num++;
                }

                corner_num++;
            }
        }
    }
    if(corner_num<20)
    {
        printf("not enough correct points");
    }

    //std::cout << "First corner num = " << corner_first_num << std::endl;
    //std::cout << "Second corner num = " << corner_second_num << std::endl;

}

/*!
 * @brief 添加面特征约束
 * @param pc_in
 * @param map_in
 * @param problem
 * @param loss_function
 */
void OdomEstimationClass::addSurfCostFactorNew(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction* loss_function)
{
    int surf_num=0;
    int surf_first_num = 0;
    int surf_second_num = 0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMapNew(&(pc_in->points[i]), &point_temp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 1.0)
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
            double curr_time = pc_in->points[i].intensity;
            if (planeValid)
            {
                // ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                // problem.AddResidualBlock(cost_function, loss_function, parameters);

                if (curr_time <= 0.5)
                {
                    ceres::CostFunction* cost_function = LidarFirstPlaneFactor::Create(curr_point, norm, negative_OA_dot_norm, q_w_last_end, t_w_last_end, curr_time);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_middle);
                    surf_first_num++;
                }
                else
                {
                    ceres::CostFunction* cost_function = LidarSecondPlaneFactor::Create(curr_point, norm, negative_OA_dot_norm, curr_time);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_middle, parameters_end);
                    ceres::CostFunction* cost_velocity = VelocityConsistentyFactor::Create(q_w_last_end, t_w_last_end);
                    // problem.AddResidualBlock(cost_velocity, loss_function, parameters_middle, parameters_end);
                    surf_second_num++;
                }

                surf_num++;
            }
        }

    }
    if(surf_num<20){
        printf("not enough correct points");
    }

    //std::cout << "First surf num = " << surf_first_num << std::endl;
    //std::cout << "Second surf num = " << surf_second_num << std::endl;




}
/*********************************************************************************************************************************************/
void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int corner_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis); 
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
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            { 
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);  
                problem.AddResidualBlock(cost_function, loss_function, parameters);
                corner_num++;   
            }                           
        }
    }
    if(corner_num<20){
        printf("not enough correct points");
    }

}

void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int surf_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 1.0)
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
            if (planeValid)
            {
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);    
                problem.AddResidualBlock(cost_function, loss_function, parameters);

                surf_num++;
            }
        }

    }
    if(surf_num<20){
        printf("not enough correct points");
    }

}

void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud){

    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMapNew(&downsampledEdgeCloud->points[i], &point_temp);
        laserCloudCornerMap->push_back(point_temp); 
    }
    
    for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMapNew(&downsampledSurfCloud->points[i], &point_temp);
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
