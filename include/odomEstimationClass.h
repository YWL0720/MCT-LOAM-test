// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
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
        /************************************/
        // 里程计
        // odom_m, odom_e 分别为当前帧中间时刻和当前帧结束时刻
        // last_odom_m, last_odom_e 分别为上一帧中间时刻和上一帧结束时刻
        Eigen::Isometry3d odom_m;
        Eigen::Isometry3d odom_e;
        Eigen::Isometry3d last_odom_m;
        Eigen::Isometry3d last_odom_e;

        Eigen::Quaterniond q_w_last_end;
        Eigen::Vector3d t_w_last_end;

        Eigen::Quaterniond q_w_last_middle;
        Eigen::Vector3d t_w_last_middle;
        // 前一帧两段时间内的平均速度
        Eigen::Isometry3d velocity;
        std::vector<Eigen::Isometry3d> velocity_history;
        /************************************/

		pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerMap;
		pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfMap;
	private:
		//optimization variable

        /************************************************************************************/
        // 优化变量
        // parameters_middle-当前帧中间时刻在世界坐标系下的位姿
        // parameters_end-当前帧结束时刻在世界坐标系下的位姿
        double parameters_middle[7] = {0, 0, 0, 1, 0, 0, 0};
        double parameters_end[7] = {0, 0, 0, 1, 0, 0, 0};

        Eigen::Map<Eigen::Quaterniond> q_w_curr_m = Eigen::Map<Eigen::Quaterniond>(parameters_middle);
        Eigen::Map<Eigen::Vector3d> t_w_curr_m = Eigen::Map<Eigen::Vector3d>(parameters_middle + 4);
        Eigen::Map<Eigen::Quaterniond> q_w_curr_e = Eigen::Map<Eigen::Quaterniond>(parameters_end);
        Eigen::Map<Eigen::Vector3d> t_w_curr_e = Eigen::Map<Eigen::Vector3d>(parameters_end + 4);
        /************************************************************************************/

		double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
		Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
		Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);



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
        /********************************************************************/
        void addEdgeCostFactorNew(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction* loss_function);

        void addSurfCostFactorNew(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction* loss_function);
        void pointAssociateToMapNew(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po);
        /********************************************************************/

		void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		void addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud);
		void pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po);
		void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out);
};

#endif // _ODOM_ESTIMATION_CLASS_H_

