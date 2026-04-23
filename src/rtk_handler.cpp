/*
 * ====================================================================
 * RTK Handler Implementation (FAST_LIO RTK扩展)
 * ====================================================================
 *
 * 本文件实现了三种RTK数据格式的处理器:
 *   - RtkHandlerOdometry:  解析 nav_msgs/Odometry (ENU位置 + 姿态)
 *   - RtkHandlerNavSatFix: 解析 sensor_msgs/NavSatFix (LLA → ENU)
 *   - RtkHandlerInspvax:   解析 novatel_msgs/INSPVAX (待实现)
 *
 * 所有处理器的 process() 方法都会自动将外参填充到 RtkData 中,
 * 调用方无需手动设置 T_imu_ant / R_imu_ant。
 * ====================================================================
 */

#include "rtk_handler.h"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <iostream>

// =====================================================================
// LLA→ENU 坐标转换 (WGS84椭球模型)
// 输入/输出: 角度单位为度, 高程单位为米
// =====================================================================
Eigen::Vector3d RtkHandlerBase::LLA2ENU(double lat, double lon, double alt) const
{
    if (!has_ref_) {
        std::cerr << "RTK Handler: Reference position not set for LLA->ENU conversion!" << std::endl;
        return Eigen::Vector3d::Zero();
    }

    // WGS84椭球参数
    const double a = 6378137.0;          // 长半轴 (m)
    const double b = 6356752.314245;     // 短半轴 (m)
    const double e2 = (a * a - b * b) / (a * a);  // 第一偏心率的平方

    // 角度→弧度
    const double deg2rad = M_PI / 180.0;
    double ref_lat_rad = ref_lat_ * deg2rad;
    double ref_lon_rad = ref_lon_ * deg2rad;
    double lat_rad = lat * deg2rad;
    double lon_rad = lon * deg2rad;

    // 弧度差值
    double d_lat = lat_rad - ref_lat_rad;
    double d_lon = lon_rad - ref_lon_rad;
    double d_alt = alt - ref_alt_;

    // 卯酉圈曲率半径 N, 子午圈曲率半径 M
    double sin_ref_lat = std::sin(ref_lat_rad);
    double N = a / std::sqrt(1 - e2 * sin_ref_lat * sin_ref_lat);
    double M = a * (1 - e2) / std::pow(1 - e2 * sin_ref_lat * sin_ref_lat, 1.5);

    // ENU坐标
    Eigen::Vector3d enu;
    enu[0] = (N + ref_alt_) * std::cos(ref_lat_rad) * d_lon;  // East
    enu[1] = (M + ref_alt_) * d_lat;                            // North
    enu[2] = d_alt;                                              // Up

    return enu;
}

// =====================================================================
// RtkHandlerOdometry - nav_msgs/Odometry 格式处理器
// 输入: ENU位置 + 四元数姿态 (来自gps_driver等节点)
// 有效性判断: 协方差对角线元素 > 0
// =====================================================================
class RtkHandlerOdometry : public RtkHandlerBase {
public:
    RtkData process(const void* msg, double timestamp) override {
        RtkData data;
        data.timestamp = timestamp;

        const auto* odom = static_cast<const nav_msgs::msg::Odometry*>(msg);

        data.position = Eigen::Vector3d(
            odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z
        );

        data.rotation = Eigen::Quaterniond(
            odom->pose.pose.orientation.w,
            odom->pose.pose.orientation.x,
            odom->pose.pose.orientation.y,
            odom->pose.pose.orientation.z
        );

        // 有效性判断: 协方差对角线存在正值
        bool has_valid_cov = false;
        for (int i = 0; i < 6; i++) {
            if (odom->pose.covariance[i * 6 + i] > 0) {
                has_valid_cov = true;
                break;
            }
        }
        data.valid = has_valid_cov;

        // 自动填充外参 (由setExtrinsic设置)
        data.T_imu_ant = T_imu_ant_;
        data.R_imu_ant = R_imu_ant_;

        return data;
    }
};

// =====================================================================
// RtkHandlerNavSatFix - sensor_msgs/NavSatFix 格式处理器
// 输入: LLA坐标 (纬度/经度/高度)
// 特殊处理:
//   - 无姿态信息, rotation = Identity (假设IMU与ENU对齐)
//   - 首条数据自动设为LLA参考点 (若未配置)
// 有效性判断: status.status > STATUS_NO_FIX
// =====================================================================
class RtkHandlerNavSatFix : public RtkHandlerBase {
public:
    RtkData process(const void* msg, double timestamp) override {
        RtkData data;
        data.timestamp = timestamp;

        const auto* fix = static_cast<const sensor_msgs::msg::NavSatFix*>(msg);

        // 检查定位状态: 低于NO_FIX视为无效
        if (fix->status.status <= sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
            data.valid = false;
            return data;
        }

        // LLA → ENU 转换
        if (has_ref_) {
            data.position = LLA2ENU(fix->latitude, fix->longitude, fix->altitude);
            data.rotation = Eigen::Quaterniond::Identity();  // NavSatFix无姿态
            data.valid = true;
        } else {
            // 首条有效数据作为LLA参考点
            setRefLLA(fix->latitude, fix->longitude, fix->altitude);
            data.position = Eigen::Vector3d::Zero();
            data.rotation = Eigen::Quaterniond::Identity();
            data.valid = true;
            std::cout << "RTK Handler: Using first fix as reference LLA" << std::endl;
        }

        // 自动填充外参 (由setExtrinsic设置)
        data.T_imu_ant = T_imu_ant_;
        data.R_imu_ant = R_imu_ant_;

        return data;
    }
};

// =====================================================================
// RtkHandlerInspvax - novatel_msgs/INSPVAX 格式处理器 (待实现)
// =====================================================================
class RtkHandlerInspvax : public RtkHandlerBase {
public:
    RtkData process(const void* msg, double timestamp) override {
        RtkData data;
        data.timestamp = timestamp;
        data.valid = false;  // Not implemented yet
        return data;
    }
};

// =====================================================================
// 工厂函数: 根据格式创建对应的RTK处理器
// =====================================================================
RtkHandlerBase::Ptr createRtkHandler(RtkFormat format) {
    switch (format) {
        case RtkFormat::ODOMETRY:
            return std::make_shared<RtkHandlerOdometry>();
        case RtkFormat::NAVSAT:
            return std::make_shared<RtkHandlerNavSatFix>();
        case RtkFormat::INSPVAX:
            return std::make_shared<RtkHandlerInspvax>();
        default:
            std::cerr << "RTK Handler: Invalid format requested" << std::endl;
            return nullptr;
    }
}
