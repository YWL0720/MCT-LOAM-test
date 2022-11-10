//
// Created by ywl on 22-11-7.
//
#ifndef MCT_LOAM_LIDAROPTIMIZATION_HPP
#define MCT_LOAM_LIDAROPTIMIZATION_HPP
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>

/*!
 * @brief 位于当前帧前半段的线特征约束
 */
struct LidarFirstEdgeFactor
{
    LidarFirstEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_,
                         Eigen::Quaterniond q_w_last_end_, Eigen::Vector3d t_w_last_end_, double time_)
                         : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_),
                         q_w_last_end(q_w_last_end_), t_w_last_end(t_w_last_end_), time(time_ * 2) {};

    template <typename T>
    bool operator()(const T* parameters_middle, T* residual) const
    {
        // 优化变量 当前帧中间时刻的位姿
        Eigen::Quaternion<T> q_w_curr_middle{parameters_middle[3], parameters_middle[0], parameters_middle[1], parameters_middle[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr_middle{parameters_middle[4], parameters_middle[5], parameters_middle[6]};

        // 上一帧结束时刻的位姿，为已知固定
        Eigen::Quaternion<T> q_w_last_end_T{T(q_w_last_end.w()), T(q_w_last_end.x()), T(q_w_last_end.y()), T(q_w_last_end.z())};
        Eigen::Matrix<T, 3, 1> t_w_last_end_T{T(t_w_last_end.x()), T(t_w_last_end.y()), T(t_w_last_end.z())};

        // 计算点和相关量
        Eigen::Matrix<T, 3, 1> curr_point_T{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> last_point_a_T{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
        Eigen::Matrix<T, 3, 1> last_point_b_T{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

        // 当前点的世界坐标，用当前帧中间时刻的位姿一个优化变量表示
        Eigen::Matrix<T, 3, 1> curr_point_world;
        // 当前点所在时刻的位姿
        Eigen::Quaternion<T> q_w_curr_point;
        Eigen::Matrix<T, 3, 1> t_w_curr_point;

        q_w_curr_point = q_w_last_end_T.slerp(T(time), q_w_curr_middle);
        t_w_curr_point = t_w_last_end_T + T(time) * (t_w_curr_middle - t_w_last_end_T);

        // 当前点的世界坐标，用当前帧中间时刻的位姿一个优化变量表示
        curr_point_world = q_w_curr_point * curr_point_T + t_w_curr_point;

        Eigen::Matrix<T, 3, 1> nu = (curr_point_world - last_point_a_T).cross(curr_point_world - last_point_b_T);
        Eigen::Matrix<T, 3, 1> de = last_point_a_T - last_point_b_T;

        residual[0] = nu.x() / de.norm();
        residual[1] = nu.y() / de.norm();
        residual[2] = nu.z() / de.norm();


        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                                       const Eigen::Vector3d last_point_b_, const Eigen::Quaterniond q_w_last_end_,
                                       const Eigen::Vector3d t_w_last_end_, double time_)
    {
        return (new ceres::AutoDiffCostFunction<LidarFirstEdgeFactor, 3, 7>(
                new LidarFirstEdgeFactor(curr_point_, last_point_a_, last_point_b_, q_w_last_end_, t_w_last_end_, time_)
                ));
    }


    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    Eigen::Quaterniond q_w_last_end;
    Eigen::Vector3d t_w_last_end;
    double time;
};


/*!
 * @brief 位于当前帧后半段的线特征约束
 */
struct LidarSecondEdgeFactor
{
    LidarSecondEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_, double time_)
    : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), time((time_ * 2) - 1) {};

    template <typename T>
    bool operator()(const T* parameters_middle, const T* parameters_end, T* residual) const
    {
        // 优化变量，当前帧中间时刻和结束时刻的位姿
        Eigen::Quaternion<T> q_w_curr_middle{parameters_middle[3], parameters_middle[0], parameters_middle[1], parameters_middle[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr_middle{parameters_middle[4], parameters_middle[5], parameters_middle[6]};

        Eigen::Quaternion<T> q_w_curr_end{parameters_end[3], parameters_end[0], parameters_end[1], parameters_end[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr_end{parameters_end[4], parameters_end[5], parameters_end[6]};

        // 计算点和关联点
        Eigen::Matrix<T, 3, 1> curr_point_T{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> last_point_a_T{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
        Eigen::Matrix<T, 3, 1> last_point_b_T{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

        // 当前的世界坐标，用当前帧中间时刻和结束时刻两个优化变量来表示
        Eigen::Matrix<T, 3, 1> curr_point_world;

        // 当前点所在时刻的位姿
        Eigen::Quaternion<T> q_w_curr_point;
        Eigen::Matrix<T, 3, 1> t_w_curr_point;

        q_w_curr_point = q_w_curr_middle.slerp(T(time), q_w_curr_end);
        t_w_curr_point = t_w_curr_middle + T(time) * (t_w_curr_end - t_w_curr_middle);

        curr_point_world = q_w_curr_point * curr_point_T + t_w_curr_point;

        Eigen::Matrix<T, 3, 1> nu = (curr_point_world - last_point_a_T).cross(curr_point_world - last_point_b_T);
        Eigen::Matrix<T, 3, 1> de = last_point_a_T - last_point_b_T;
        residual[0] = nu.x() / de.norm();
        residual[1] = nu.y() / de.norm();
        residual[2] = nu.z() / de.norm();

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                                       const Eigen::Vector3d last_point_b_, double time_)
    {
        return (new ceres::AutoDiffCostFunction<LidarSecondEdgeFactor, 3, 7, 7>(
                new LidarSecondEdgeFactor(curr_point_, last_point_a_, last_point_b_, time_)
        ));
    }

    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    double time;

};

/*!
 * @brief 位于当前帧前半段的面特征约束
 */
struct LidarFirstPlaneFactor
{
    LidarFirstPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_,
                          Eigen::Quaterniond q_w_last_end_, Eigen::Vector3d t_w_last_end, double time_) :
                          curr_point(curr_point_), plane_unit_norm(plane_unit_norm_), negative_OA_dot_norm(negative_OA_dot_norm_),
                          q_w_last_end(q_w_last_end_), t_w_last_end(t_w_last_end), time(time_ * 2) {};


    template <typename T>
    bool operator()(const T* parameters_middle, T* residual) const
    {
        // 优化变量，当前帧中间时刻的位姿
        Eigen::Quaternion<T> q_w_curr_middle{parameters_middle[3], parameters_middle[0], parameters_middle[1], parameters_middle[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr_middle{parameters_middle[4], parameters_middle[5], parameters_middle[6]};

        // 上一帧结束时刻的位姿，为已知固定
        Eigen::Quaternion<T> q_w_last_end_T{T(q_w_last_end.w()), T(q_w_last_end.x()), T(q_w_last_end.y()), T(q_w_last_end.z())};
        Eigen::Matrix<T, 3, 1> t_w_last_end_T{T(t_w_last_end.x()), T(t_w_last_end.y()), T(t_w_last_end.z())};

        // 计算点和相关量
        Eigen::Matrix<T, 3, 1> curr_point_T{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> plane_unit_norm_T{T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z())};

        // 当前点在世界坐标系下的坐标
        Eigen::Matrix<T, 3, 1> curr_point_world;

        // 当前点时刻在世界坐标系下的位姿， 用当前帧中间时刻的位姿一个优化变量来表示
        Eigen::Quaternion<T> q_w_curr_point;
        Eigen::Matrix<T, 3, 1> t_w_curr_point;

        q_w_curr_point = q_w_last_end_T.slerp(T(time), q_w_curr_middle);
        t_w_curr_point = t_w_last_end_T + T(time) * (t_w_curr_middle - t_w_last_end_T);

        curr_point_world = q_w_curr_point * curr_point_T + t_w_curr_point;

        residual[0] = plane_unit_norm_T.dot(curr_point_world) + T(negative_OA_dot_norm);

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_, const double negative_OA_dot_norm_,
                                       const Eigen::Quaterniond q_w_last_end_, const Eigen::Vector3d t_w_last_end_, const double time_)
    {
        // 自动求导模板
        return (new ceres::AutoDiffCostFunction<LidarFirstPlaneFactor, 1, 7>(new LidarFirstPlaneFactor(curr_point_, plane_unit_norm_,
                                                                                                       negative_OA_dot_norm_, q_w_last_end_,
                                                                                                       t_w_last_end_, time_)));
    }




    Eigen::Vector3d curr_point, plane_unit_norm;
    Eigen::Quaterniond q_w_last_end;
    Eigen::Vector3d t_w_last_end;
    double negative_OA_dot_norm;
    double time;
};

/*!
 * @brief 位于当前帧后半段的面特征约束
 */
struct LidarSecondPlaneFactor
{
    LidarSecondPlaneFactor(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_, const double negative_OA_dot_norm_, const double time_)
    : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_), negative_OA_dot_norm(negative_OA_dot_norm_), time(2 * time_ - 1) {};

    template <typename T>
    bool operator()(const T* parameters_middle, const T* parameters_end, T* residual) const
    {
        // 优化变量，当前帧中间时刻和结束时刻的位姿
        Eigen::Quaternion<T> q_w_curr_middle{parameters_middle[3], parameters_middle[0], parameters_middle[1], parameters_middle[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr_middle{parameters_middle[4], parameters_middle[5], parameters_middle[6]};

        Eigen::Quaternion<T> q_w_curr_end{parameters_end[3], parameters_end[0], parameters_end[1], parameters_end[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr_end{parameters_end[4], parameters_end[5], parameters_end[6]};

        // 计算点和关联点
        Eigen::Matrix<T, 3, 1> curr_point_T{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> plane_unit_norm_T{T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z())};

        // 当前点在世界坐标系下的坐标
        Eigen::Matrix<T, 3, 1> curr_point_world;

        // 当前时刻的世界坐标系下的位姿
        Eigen::Quaternion<T> q_w_curr_point;
        Eigen::Matrix<T, 3, 1> t_w_curr_point;

        q_w_curr_point = q_w_curr_middle.slerp(T(time), q_w_curr_end);
        t_w_curr_point = t_w_curr_middle + T(time) * (t_w_curr_end - t_w_curr_middle);

        curr_point_world = q_w_curr_point * curr_point_T + t_w_curr_point;

        residual[0] = plane_unit_norm_T.dot(curr_point_world) + T(negative_OA_dot_norm);

        return true;
    }


    static ceres::CostFunction* Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_, double time_)
    {
        return (new ceres::AutoDiffCostFunction<LidarSecondPlaneFactor, 1, 7, 7>(new LidarSecondPlaneFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_, time_)));
    }

    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    double negative_OA_dot_norm;
    double time;
};

// TODO ？
/*!
 * @brief 恒定速度约束
 */
struct VelocityConsistentyFactor
{
    VelocityConsistentyFactor(const Eigen::Quaterniond q_w_last_end_, const Eigen::Vector3d t_w_last_end_) : q_w_last_end(q_w_last_end_), t_w_last_end(t_w_last_end_) {};

    template <typename T>
    bool operator()(const T* parameters_middle, const T* parameters_end, T* residual) const
    {
        // 优化变量，当前帧中间时刻和结束时刻的位姿
        Eigen::Quaternion<T> q_w_curr_middle{parameters_middle[3], parameters_middle[0], parameters_middle[1], parameters_middle[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr_middle{parameters_middle[4], parameters_middle[5], parameters_middle[6]};

        Eigen::Quaternion<T> q_w_curr_end{parameters_end[3], parameters_end[0], parameters_end[1], parameters_end[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr_end{parameters_end[4], parameters_end[5], parameters_end[6]};

        // 上一帧结束时刻的位姿，为已知固定
        Eigen::Quaternion<T> q_w_last_end_T{T(q_w_last_end.w()), T(q_w_last_end.x()), T(q_w_last_end.y()), T(q_w_last_end.z())};
        Eigen::Matrix<T, 3, 1> t_w_last_end_T{T(t_w_last_end.x()), T(t_w_last_end.y()), T(t_w_last_end.z())};


        // 计算两段速度
        Eigen::Quaternion<T> q_velocity_first = q_w_last_end_T.inverse() * q_w_curr_middle;
        Eigen::Matrix<T, 3, 1> t_velocity_first = t_w_curr_middle - t_w_last_end_T;

        Eigen::Quaternion<T> q_velocity_second = q_w_curr_middle.inverse() * q_w_curr_end;
        Eigen::Matrix<T, 3, 1> t_velocity_second = t_w_curr_end - t_w_curr_middle;

        Eigen::Quaternion<T> delta_q = q_velocity_first.inverse() * q_velocity_second;
        Eigen::Matrix<T, 3, 1> delta_t = t_velocity_second - t_velocity_first;


        residual[0] = delta_t[0];
        residual[1] = delta_t[1];
        residual[2] = delta_t[2];




        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Quaterniond q_w_last_end_, const Eigen::Vector3d t_w_last_end)
    {
        return (new ceres::AutoDiffCostFunction<VelocityConsistentyFactor, 3, 7, 7>(new VelocityConsistentyFactor(q_w_last_end_,
                                                                                                                  t_w_last_end)));
    }

    Eigen::Quaterniond q_w_last_end;
    Eigen::Vector3d t_w_last_end;


};

// TODO ？
/*!
 * @brief 初始约束，用于初始化阶段
 */
struct LidarInitEdgeFactor
{
    LidarInitEdgeFactor(Eigen::Vector3d curr_point){};
};

#endif //MCT_LOAM_LIDAROPTIMIZATION_HPP
