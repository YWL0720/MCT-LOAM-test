#ifndef _ODOM_ESTIMATION_CLASS_H_
#define _ODOM_ESTIMATION_CLASS_H_

//std lib
#include <string>
#include <math.h>
#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include "lidar.h"
#include "lidarOptimization.h"
#include <ros/ros.h>

class OdomEstimationClass 
{

    public:
    	OdomEstimationClass();
    	
		void init(lidar::Lidar lidar_param, double map_resolution);	
		void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in);
		void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in);
		void getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap);

		Eigen::Isometry3d odom;
        // 里程计变量

        Eigen::Isometry3d odom_curr_middle;
        Eigen::Isometry3d odom_curr_end;

        Eigen::Isometry3d odom_last_end;
        Eigen::Isometry3d odom_last_middle;
        // 上一帧结束时刻的位姿
        Eigen::Quaterniond q_w_last_end;
        Eigen::Vector3d t_w_last_end;


		pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerMap;
		pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfMap;
	private:
		//optimization variable
		double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
		Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
		Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

        // 优化变量 分别为当前帧中间时刻的位姿和当前帧结束时刻的位姿
        double parameters_middle_q[4] = {0, 0, 0, 1};
        double parameters_middle_t[3] = {0, 0, 0};

        double parameters_end_q[4] = {0, 0, 0, 1};
        double parameters_end_t[3] = {0, 0, 0};

        Eigen::Map<Eigen::Quaterniond> q_w_curr_middle = Eigen::Map<Eigen::Quaterniond>(parameters_middle_q);
        Eigen::Map<Eigen::Vector3d> t_w_curr_middle = Eigen::Map<Eigen::Vector3d>(parameters_middle_t);
        Eigen::Map<Eigen::Quaterniond> q_w_curr_end = Eigen::Map<Eigen::Quaterniond>(parameters_end_q);
        Eigen::Map<Eigen::Vector3d> t_w_curr_end = Eigen::Map<Eigen::Vector3d>(parameters_end_t);



		Eigen::Isometry3d last_odom;

		//kd-tree
		pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgeMap;
		pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfMap;

		//points downsampling before add to map
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterEdge;
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;

		//local map
		pcl::CropBox<pcl::PointXYZI> cropBoxFilter;

		//optimization count 
		int optimization_count;

		//function
		void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		void addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud);
		void pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po);
		void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out);
};

#endif // _ODOM_ESTIMATION_CLASS_H_

