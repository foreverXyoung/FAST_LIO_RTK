/*
 * ====================================================================
 * RTK Handler - 模块化RTK数据处理 (FAST_LIO RTK扩展)
 * ====================================================================
 *
 * 模块职责:
 *   1. RtkData       - 统一RTK数据结构 (位置/姿态/外参/天线→IMU转换)
 *   2. RtkHandlerBase - RTK数据解析抽象基类 (多格式支持)
 *   3. RtkFormat      - RTK数据格式枚举
 *
 * 支持的RTK数据格式:
 *   - ODOMETRY: nav_msgs/Odometry (ENU位置 + 姿态)
 *   - NAVSAT:   sensor_msgs/NavSatFix (LLA坐标, 需转换为ENU)
 *   - INSPVAX:  novatel_msgs/INSPVAX (完整导航解, 待实现)
 *
 * 数据流:
 *   RTK消息 → rtk_handler.process() → RtkData → rtk_buffer → IMU初始化
 *
 * 与原始FAST_LIO的改动对比:
 *   - 新增: rtk_handler.h/cpp (整个文件)
 *   - 修改: common_lib.h (添加 #include <rtk_handler.h>)
 *   - 修改: laserMapping.cpp (RTK回调/同步/参数)
 *   - 修改: IMU_Processing.hpp (RTK初始化逻辑)
 * ====================================================================
 */

#ifndef RTK_HANDLER_H
#define RTK_HANDLER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <string>

// Forward declarations
namespace nav_msgs { namespace msg { class Odometry; } }
namespace sensor_msgs { namespace msg { class NavSatFix; } }

// =====================================================================
// RtkData - 统一RTK数据结构
// =====================================================================
struct RtkData {
    double timestamp;
    Eigen::Vector3d position;      // RTK天线位置 (ENU, meters)
    Eigen::Quaterniond rotation;   // RTK姿态 (NavSatFix格式下为Identity)
    bool valid;

    // IMU-RTK天线外参 (由RtkHandlerBase::process()自动填充)
    // 用于将RTK天线位置转换为IMU位置
    Eigen::Vector3d T_imu_ant;   // RTK天线在IMU坐标系中的平移 (IMU→天线)
    Eigen::Matrix3d R_imu_ant;   // RTK天线到IMU的旋转矩阵 (天线→IMU)

    RtkData() : timestamp(0), valid(false) {
        position = Eigen::Vector3d::Zero();
        rotation = Eigen::Quaterniond::Identity();
        T_imu_ant = Eigen::Vector3d::Zero();
        R_imu_ant = Eigen::Matrix3d::Identity();
    }

    void reset() {
        timestamp = 0;
        position.setZero();
        rotation = Eigen::Quaterniond::Identity();
        valid = false;
        T_imu_ant = Eigen::Vector3d::Zero();
        R_imu_ant = Eigen::Matrix3d::Identity();
    }

    // 将RTK天线位置转换为IMU位置
    // 公式: P_imu = P_rtk - R_world_imu * R_imu_ant^T * T_imu_ant
    //
    // 说明:
    //   当RTK提供姿态时(Odometry格式): R_world_imu = rotation
    //   当RTK无姿态时(NavSatFix格式):  R_world_imu = Identity,
    //     即假设IMU坐标系与ENU世界坐标系对齐, 天线偏移直接在ENU中表达
    Eigen::Vector3d toImuPosition() const {
        Eigen::Matrix3d R_world_imu = rotation.toRotationMatrix();
        Eigen::Matrix3d R_ant_to_imu = R_imu_ant.transpose();
        Eigen::Vector3d T_ant_in_world = R_world_imu * (R_ant_to_imu * T_imu_ant);
        return position - T_ant_in_world;
    }
};

// =====================================================================
// RtkFormat - RTK数据格式枚举
// =====================================================================
enum class RtkFormat {
    ODOMETRY = 0,    // nav_msgs/Odometry
    NAVSAT = 1,      // sensor_msgs/NavSatFix (LLA -> ENU)
    INSPVAX = 2,     // novatel_msgs/INSPVAX
    INVALID = -1
};

// =====================================================================
// RtkHandlerBase - RTK数据解析抽象基类
// =====================================================================
// 职责:
//   - 从不同格式的RTK消息中提取位置/姿态
//   - LLA→ENU坐标转换 (NAVSAT格式)
//   - 管理IMU-RTK天线外参, 自动填充到RtkData
//
// 使用方式:
//   auto handler = createRtkHandler(RtkFormat::ODOMETRY);
//   handler->setExtrinsic(T, R);        // 设置外参
//   handler->setRefLLA(lat, lon, alt);   // 设置LLA参考点(NAVSAT格式)
//   RtkData data = handler->process(msg, timestamp);
// =====================================================================
class RtkHandlerBase {
public:
    using Ptr = std::shared_ptr<RtkHandlerBase>;

    RtkHandlerBase() : ref_lat_(0), ref_lon_(0), ref_alt_(0), has_ref_(false) {
        R_imu_ant_ = Eigen::Matrix3d::Identity();
        T_imu_ant_ = Eigen::Vector3d::Zero();
    }
    virtual ~RtkHandlerBase() = default;

    // 解析RTK消息, 返回标准化的RtkData (外参自动填充)
    virtual RtkData process(const void* msg, double timestamp) = 0;

    // 设置LLA参考点 (NAVSAT格式必需, Odometry格式不需要)
    void setRefLLA(double lat, double lon, double alt) {
        ref_lat_ = lat;
        ref_lon_ = lon;
        ref_alt_ = alt;
        has_ref_ = true;
    }

    // 设置IMU-RTK天线外参 (会自动填充到process()返回的RtkData中)
    void setExtrinsic(const Eigen::Vector3d &T_imu_ant, const Eigen::Matrix3d &R_imu_ant) {
        T_imu_ant_ = T_imu_ant;
        R_imu_ant_ = R_imu_ant;
    }

    bool hasRef() const { return has_ref_; }

    // LLA→ENU坐标转换 (角度单位: 度)
    Eigen::Vector3d LLA2ENU(double lat, double lon, double alt) const;

protected:
    double ref_lat_, ref_lon_, ref_alt_;  // 参考点LLA (度)
    bool has_ref_;
    Eigen::Vector3d T_imu_ant_;   // IMU→天线 平移
    Eigen::Matrix3d R_imu_ant_;   // 天线→IMU 旋转
};

// 工厂函数: 根据格式创建对应的RTK处理器
RtkHandlerBase::Ptr createRtkHandler(RtkFormat format);

#endif
