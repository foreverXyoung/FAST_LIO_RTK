# FAST-LIO-ROS2-RTK

> ROS2 Fork maintainer: [Ericsiii](https://github.com/Ericsii)
> RTK Extension by: [foreverXyoung](https://github.com/foreverXyoung)

## 简介

本项目是基于 [FAST-LIO-ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) 的扩展，增加了 **RTK (实时动态定位) 初始化功能**。

RTK 初始化可以在系统启动时利用 RTK GNSS 提供绝对位置和姿态，从而获得更精确的初始位姿，避免 FAST-LIO 在启动阶段因缺乏先验信息而导致的位置/姿态偏差。

## 主要特性

- **模块化 RTK 处理器**：支持多种 RTK 数据格式
  - `Odometry` (nav_msgs/Odometry) - ENU 坐标直接使用
  - `NavSatFix` (sensor_msgs/NavSatFix) - LLA 坐标自动转换为 ENU
  - `INSPVAX` (novatel_msgs/INSPVAX) - 预留接口
- **可配置开关**：通过参数控制是否启用 RTK 功能，禁用时与原版 FAST-LIO 完全一致
- **灵活的坐标系转换**：支持自定义参考点进行 LLA 到 ENU 转换
- **天线-IMU 外参标定**：支持配置 RTK 天线到 IMU 的平移和旋转外参

## 工作原理

### RTK 初始化流程

```
系统启动
  │
  ├── 1. 订阅 RTK 话题 → rtk_handler.process() → RtkData → rtk_buffer
  │
  ├── 2. 第一帧 LiDAR 数据到达 (flg_first_scan)
  │       └── 若 rtk_buffer 中有有效数据 → p_imu->set_rtk_init() 保存 RTK 位姿
  │
  ├── 3. IMU 初始化阶段 (前 ~10 帧)
  │       └── 正常执行 IMU 静态初始化 (重力/零偏估计)
  │
  ├── 4. IMU 初始化完成 (init_iter_num > MAX_INI_COUNT)
  │       └── 用 RTK 位姿覆盖 EKF 初始状态:
  │             - pos  = RTK天线位置 → IMU位置 (外参转换)
  │             - rot  = RTK姿态
  │             - grav = rot × (0, 0, -G)  (根据新姿态重新计算重力)
  │             - vel  = (0, 0, 0)          (静态初始化假设)
  │
  └── 5. 后续正常运行 FAST-LIO 激光里程计
```

### 数据流

```
RTK消息 (Odometry/NavSatFix/INSPVAX)
    │
    ▼
rtk_handler.process()  ─── 解析 + LLA→ENU + 有效性检查 + 自动填充外参
    │
    ▼
RtkData {position, rotation, T_imu_ant, R_imu_ant, valid}
    │
    ▼
rtk_buffer (mutex保护)
    │
    ▼
sync_packages()  ─── 按时间戳匹配 RTK 数据到当前 LiDAR 帧
    │
    ▼
timer_callback()  ─── 首帧时传递给 ImuProcess::set_rtk_init()
    │
    ▼
ImuProcess::Process()  ─── IMU初始化完成后覆盖 EKF 初始状态
```

## 快速开始

### 1. 安装依赖

```bash
cd <ros2_ws>/src
git clone https://github.com/foreverXyoung/FAST_LIO_RTK.git --recursive
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
. ./install/setup.bash
```

### 2. 配置参数

编辑 `config/avia.yaml` 中的 RTK 配置：

```yaml
rtk:
    use_rtk: true                              # 启用 RTK 初始化
    rtk_topic: "/gps_odom"                     # RTK 话题
    rtk_format: 0                               # 0: Odometry, 1: NavSatFix, 2: INSPVAX
    ref_lla: [22.5, 113.0, 10.0]             # 参考点 LLA (仅 NavSatFix 需要)
```

### 3. 运行

```bash
# 启动 FAST-LIO-RTK
ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml

# 启动 RTK 设备或播放 RTK 数据
```

## RTK 配置说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `use_rtk` | bool | false | 是否启用 RTK 初始化 |
| `rtk_topic` | string | "/gps_odom" | RTK 数据话题名 |
| `rtk_format` | int | 0 | 数据格式：0=Odometry, 1=NavSatFix, 2=INSPVAX |
| `ref_lla` | vector | [0,0,0] | 参考点坐标 [lat, lon, alt]，用于 LLA→ENU 转换（NavSatFix格式需要） |
| `T_imu_ant` | vector | [0,0,0] | RTK天线在IMU坐标系中的位置 [x, y, z]，单位：米 |
| `R_imu_ant` | vector | 单位矩阵 | RTK天线到IMU的旋转矩阵 [r11,r12,...,r33]，行主序 |

### 坐标系说明

本系统涉及三个坐标系：

```
世界坐标系 (World/ENU Frame)
    │
    ├── RTK 天线坐标系 (RTK Antenna Frame)
    │       │
    │       └── 天线相位中心位置
    │
    └── IMU 坐标系 (IMU Body Frame)
            │
            └── 惯导测量单元
```

**坐标系定义：**

| 坐标系 | 说明 |
|--------|------|
| 世界坐标系 | ENU 东北天坐标系，RTK 输出的位置和姿态基于此坐标系 |
| RTK天线坐标系 | 以 RTK 天线相位中心为原点的坐标系 |
| IMU坐标系 | 以 IMU 测量单元为原点的坐标系 |

**RTK天线到IMU的外参定义：**

```yaml
rtk:
    # RTK天线在IMU坐标系中的位置 (从IMU原点指向天线中心的向量)
    T_imu_ant: [0.0, 0.0, 0.0]    # 单位：米

    # RTK天线到IMU的旋转矩阵 (天线坐标系 → IMU坐标系)
    # 默认单位矩阵表示天线与IMU同方向
    R_imu_ant: [1., 0., 0.,
                0., 1., 0.,
                0., 0., 1.]
```

**坐标变换公式：**

RTK输出的位置是天线相位中心在世界坐标系中的位置 $P_{rtk}$。要获得IMU在世界坐标系中的位置 $P_{imu}$：

```
P_imu = P_rtk - R_world_imu × R_imu_ant^T × T_imu_ant

其中：
  - P_rtk: RTK天线位置 (世界坐标系)
  - R_world_imu: IMU在世界坐标系中的旋转 (从IMU到世界)
  - T_imu_ant: 天线在IMU坐标系中的位置
  - R_imu_ant: 天线到IMU的旋转矩阵
```

> **NavSatFix 格式注意**：NavSatFix 没有姿态信息，系统假设 IMU 坐标系与 ENU 世界坐标系对齐（`R_world_imu = Identity`）。如果实际安装有旋转偏移，建议使用 Odometry 格式。

**配置示例：**

假设 RTK 天线安装在 IMU 上方 0.1m，偏左 0.05m，偏前 0.02m：

```yaml
rtk:
    use_rtk: true
    rtk_topic: "/gps_odom"
    rtk_format: 0
    T_imu_ant: [0.02, -0.05, 0.1]   # 天线在IMU坐标系中的位置
    R_imu_ant: [1., 0., 0.,         # 假设天线朝上，与IMU同方向
                0., 1., 0.,
                0., 0., 1.]
```

### 使用示例

**1. Odometry 格式 (默认)**
```yaml
rtk:
    use_rtk: true
    rtk_topic: "/gps_odom"
    rtk_format: 0
```

**2. NavSatFix 格式 (需要设置参考点)**
```yaml
rtk:
    use_rtk: true
    rtk_topic: "/gps/fix"
    rtk_format: 1
    ref_lla: [22.596512, 113.971123, 10.0]  # 您的参考点坐标
```

> 如果不设置 `ref_lla`，首条有效 NavSatFix 数据将自动作为参考点。

**3. 禁用 RTK**
```yaml
rtk:
    use_rtk: false
```

## 与原版 FAST-LIO 的改动对比

### 新增文件

| 文件 | 说明 |
|------|------|
| `include/rtk_handler.h` | RTK 数据结构 (RtkData) + 抽象处理器基类 + 格式枚举 |
| `src/rtk_handler.cpp` | RTK 处理器实现 (Odometry/NavSatFix/INSPVAX) + LLA2ENU |

### 修改文件

| 文件 | 改动区域 | 说明 |
|------|----------|------|
| `include/common_lib.h` | 1行 | 添加 `#include <rtk_handler.h>` |
| `src/laserMapping.cpp` | RTK Extension (START/END) 标记区域 | RTK全局变量、回调函数、数据同步、参数加载、订阅初始化 |
| `src/IMU_Processing.hpp` | RTK Extension (START/END) 标记区域 | `set_rtk_init()` 方法 + EKF 初始状态覆盖 |
| `config/avia.yaml` | rtk 配置段 | 添加 RTK 相关参数 |
| `CMakeLists.txt` | 1行 | 编译目标添加 `src/rtk_handler.cpp` |

所有 RTK 相关改动均以 `RTK Extension (START/END)` 标记，便于与原版对比和合并。

### 已删除文件

| 文件 | 原因 |
|------|------|
| `include/rtk_data.h` | 与 `rtk_handler.h` 中的 `RtkData` 定义重复，已删除以避免 ODR (One Definition Rule) 违规 |

## 扩展新的 RTK 数据格式

项目采用模块化设计，添加新格式只需：

1. 在 `rtk_handler.h` 的 `RtkFormat` 枚举添加新类型
2. 创建新的 Handler 类继承 `RtkHandlerBase`
3. 在 `createRtkHandler()` 工厂函数添加 case
4. 在 `rtk_handler.cpp` 实现 `process()` 方法
5. 在 `laserMapping.cpp` 添加对应的回调函数和订阅

```cpp
// 示例：添加新格式
class RtkHandlerNewFormat : public RtkHandlerBase {
public:
    RtkData process(const void* msg, double timestamp) override {
        // 实现数据解析逻辑
        // 注意: process() 会自动将 T_imu_ant_/R_imu_ant_ 填充到返回的 RtkData 中
    }
};
```

## 注意事项

1. **静态初始化假设**：RTK 初始化时假设载体处于静止状态，速度被置零。如果启动时载体在运动，初始速度会存在偏差。
2. **RTK 数据时间同步**：RTK 数据需要与 LiDAR/IMU 数据时间同步。如果 RTK 数据延迟较大，可能无法在首帧时获取有效数据。
3. **NavSatFix 无姿态**：NavSatFix 格式不包含姿态信息，系统默认 `rotation = Identity`，适用于 IMU 与 ENU 对齐的场景。
4. **LLA 参考点**：使用 NavSatFix 格式时，建议手动配置 `ref_lla`，以确保 LLA→ENU 转换的精度。如果不配置，首条有效数据将作为参考点。
5. **仅初始化使用**：RTK 数据仅在 EKF 初始化阶段使用一次，后续运行完全依赖 FAST-LIO 的激光里程计。

## 相关项目

**原始项目:**
- [FAST-LIO-ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) - ROS2 版本的 FAST-LIO

**SLAM 相关:**
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree) - 动态 KD-Tree
- [R2LIVE](https://github.com/hku-mars/r2live) - LiDAR-IMU-Visual 融合
- [LI_Init](https://github.com/hku-mars/LiDAR_IMU_Init) - LiDAR-IMU 外参初始化

**控制与规划:**
- [IKFOM](https://github.com/hku-mars/IKFoM) - 流形卡尔曼滤波工具箱
- [UAV Obstacle Avoidance](https://github.com/hku-mars/dyn_small_obs_avoidance) - 动态障碍物避障

## 致谢

感谢 LOAM、Livox_Mapping、LINS、Loam_Livox 以及 FAST-LIO 原作者。

---

## Original README

原版 README 请查看: [README_original.md](README_original.md)

原版 FAST-LIO 说明请参考: [FAST-LIO-ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
