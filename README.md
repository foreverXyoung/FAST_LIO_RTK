# FAST-LIO-ROS2-RTK

> ROS2 Fork maintainer: [Ericsiii](https://github.com/Ericsii)
> RTK Extension by: [foreverXyoung](https://github.com/foreverXyoung)

## 简介

本项目基于 [FAST-LIO-ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)，增加了 **RTK (Real-Time Kinematic) 初始化功能**。

FAST-LIO 启动时仅靠 IMU 静态初始化估计重力和零偏，初始位置默认从原点开始。加入 RTK 后，系统可以在启动时利用 RTK GNSS 提供的**绝对位置和姿态**覆盖初始状态，使建图起点直接位于真实世界坐标，无需后续配准对齐。

## 主要特性

- **模块化 RTK 处理器**：支持多种 RTK 数据格式，可扩展
  - `Odometry` (nav_msgs/Odometry) — ENU 坐标直接使用
  - `NavSatFix` (sensor_msgs/NavSatFix) — LLA 坐标自动转换为 ENU
  - `INSPVAX` (novatel_msgs/INSPVAX) — 预留接口
- **RTK 等待机制**：可配置等待时间，确保 RTK 数据在初始化前积累到缓冲区，避免空缓冲导致初始化失败
- **零侵入设计**：通过 `use_rtk: false` 参数关闭后，行为与原版 FAST-LIO 完全一致
- **灵活的坐标系转换**：支持自定义参考点进行 LLA→ENU 转换
- **天线-IMU 外参标定**：支持配置 RTK 天线到 IMU 的平移和旋转外参

## RTK 初始化流程

```
系统启动
  │
  ├── 1. 订阅 RTK 话题 → rtk_handler.process() → RtkData → rtk_buffer
  │
  ├── 2. 第一帧 LiDAR 到达 (flg_first_scan)
  │       └── 记录 rtk_wait_start_time, 进入等待阶段
  │
  ├── 3. RTK 等待阶段 (elapsed < rtk_wait_time)
  │       ├── 继续调用 Process() 积累 IMU 数据 (IMU 初始化正常进行)
  │       └── 跳过建图 (return), 等待 RTK 数据积累
  │
  ├── 4. 等待结束 (elapsed >= rtk_wait_time)
  │       ├── 若 cur_rtk.valid → p_imu->set_rtk_init() 保存 RTK 位姿
  │       └── 若 !cur_rtk.valid → 跳过 RTK 初始化, 回退 IMU 初始化
  │
  ├── 5. 首帧建图: p_imu->Process()
  │       └── RTK 覆盖 EKF 初始状态:
  │             - pos  = RTK天线位置 → IMU位置 (外参转换)
  │             - rot  = RTK姿态
  │             - grav = rot × (0, 0, -G)  (根据新姿态重新计算重力)
  │             - vel  = (0, 0, 0)          (静态初始化假设)
  │
  └── 6. 后续正常运行 FAST-LIO 激光里程计
```

> **关键设计**：RTK 等待期间不建图，确保首帧建图时 EKF 状态已被 RTK 覆盖，避免用原点 (0,0,0) 开始建图后再跳变的问题。

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
rtk_buffer (mutex 保护)
    │
    ▼
sync_packages()  ─── 按时间戳匹配 RTK 数据到当前 LiDAR 帧 [lidar_beg, lidar_end]
    │
    ▼
timer_callback()  ─── 等待期结束后传递给 ImuProcess::set_rtk_init()
    │
    ▼
ImuProcess::Process()  ─── IMU 初始化完成后覆盖 EKF 初始状态
```

## 快速开始

### 1. 安装依赖

```bash
cd <ros2_ws>/src
git clone https://github.com/foreverXyoung/FAST_LIO_RTK.git --recursive
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

### 2. 配置参数

编辑对应雷达的 yaml 配置文件（如 `config/mid360_imu.yaml`）：

```yaml
rtk:
    use_rtk: true                              # 启用 RTK 初始化
    rtk_topic: "/gps/rtk_odom"                # RTK 话题
    rtk_format: 0                               # 0: Odometry, 1: NavSatFix, 2: INSPVAX
    rtk_wait_time: 3.0                          # RTK 等待时间(秒), 等待RTK数据积累
    ref_lla: [22.5, 113.0, 10.0]               # 参考点 LLA (仅 NavSatFix 需要)
```

### 3. 运行

```bash
# 启动 FAST-LIO-RTK
ros2 launch fast_lio mapping.launch.py config_file:=mid360_imu.yaml

# 启动 RTK 设备或播放包含 RTK 数据的 rosbag
```

### 4. 验证初始化

运行后观察终端输出：

```
RTK: Waiting for data accumulation, wait time = 3.000s    ← 等待阶段
IMU Initial Done                                           ← IMU 初始化完成
[RTK DEBUG] Wait done. elapsed=3.100s ...                  ← 等待结束
[RTK DEBUG] Init SUCCESS: pos= ... rot_w= ... t= ...      ← RTK 初始化成功
[RTK DEBUG] State override:                                ← EKF 状态覆盖
  pos: 0.009 0.002 -0.002 -> 3.774 -2.479 0.035          ← 位置从原点跳到真实坐标
  grav: 0.049 0.217 -9.806 -> -0.043 0.011 -9.809        ← 重力方向更新
Initialize the map kdtree                                  ← 建图开始
```

## RTK 配置说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `use_rtk` | bool | false | 是否启用 RTK 初始化 |
| `rtk_topic` | string | "/gps_odom" | RTK 数据话题名 |
| `rtk_format` | int | 0 | 数据格式：0=Odometry, 1=NavSatFix, 2=INSPVAX |
| `rtk_wait_time` | double | 3.0 | RTK 等待时间(秒)，等待 RTK 数据积累后再初始化 |
| `ref_lla` | vector | [0,0,0] | 参考点坐标 [lat, lon, alt]，用于 LLA→ENU 转换（NavSatFix 格式需要） |
| `T_imu_ant` | vector | [0,0,0] | RTK 天线在 IMU 坐标系中的位置 [x, y, z]，单位：米 |
| `R_imu_ant` | vector | 单位矩阵 | RTK 天线到 IMU 的旋转矩阵 [r11,r12,...,r33]，行主序 |

### `rtk_wait_time` 说明

RTK 数据通常比 LiDAR/IMU 数据延迟到达。如果 IMU 先完成初始化而 RTK 缓冲区仍为空，则 RTK 初始化会失败，系统回退到默认原点。

`rtk_wait_time` 控制在首帧 LiDAR 到达后，额外等待多少秒再执行初始化：

- **等待期间**：IMU 数据正常积累，但不建图
- **等待结束**：检查 RTK 数据是否有效，有效则覆盖 EKF 状态，无效则回退 IMU 初始化
- **建议值**：2~5 秒，取决于 RTK 设备的数据延迟

### 坐标系说明

```
世界坐标系 (World/ENU Frame)
    │
    ├── RTK 天线坐标系 (RTK Antenna Frame)
    │       └── 天线相位中心位置
    │
    └── IMU 坐标系 (IMU Body Frame)
            └── 惯导测量单元
```

**坐标变换公式：**

RTK 输出的位置是天线相位中心在世界坐标系中的位置 $P_{rtk}$，IMU 在世界坐标系中的位置 $P_{imu}$：

```
P_imu = P_rtk - R_world_imu × R_imu_ant^T × T_imu_ant
```

> **NavSatFix 格式注意**：NavSatFix 没有姿态信息，系统假设 IMU 坐标系与 ENU 世界坐标系对齐（`R_world_imu = Identity`）。如果实际安装有旋转偏移，建议使用 Odometry 格式。

### 配置示例

**1. Odometry 格式 (推荐)**

```yaml
rtk:
    use_rtk: true
    rtk_topic: "/gps/rtk_odom"
    rtk_format: 0
    rtk_wait_time: 3.0
```

**2. NavSatFix 格式 (需要设置参考点)**

```yaml
rtk:
    use_rtk: true
    rtk_topic: "/gps/fix"
    rtk_format: 1
    rtk_wait_time: 3.0
    ref_lla: [22.596512, 113.971123, 10.0]
```

> 如果不设置 `ref_lla`，首条有效 NavSatFix 数据将自动作为参考点。

**3. 天线-IMU 有偏移**

假设 RTK 天线安装在 IMU 上方 0.1m，偏左 0.05m，偏前 0.02m：

```yaml
rtk:
    use_rtk: true
    rtk_topic: "/gps/rtk_odom"
    rtk_format: 0
    rtk_wait_time: 3.0
    T_imu_ant: [0.02, -0.05, 0.1]       # 天线在 IMU 坐标系中的位置
    R_imu_ant: [1., 0., 0.,              # 天线与 IMU 同方向
                0., 1., 0.,
                0., 0., 1.]
```

**4. 禁用 RTK**

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
| `include/common_lib.h` | 1 行 | 添加 `#include <rtk_handler.h>` |
| `src/laserMapping.cpp` | RTK Extension (START/END) 标记区域 | RTK 全局变量、回调函数、等待机制、数据同步、参数加载、订阅初始化 |
| `src/IMU_Processing.hpp` | RTK Extension (START/END) 标记区域 | `set_rtk_init()` 方法 + EKF 初始状态覆盖 |
| `config/*.yaml` | rtk 配置段 | 所有配置文件添加 RTK 参数 |
| `CMakeLists.txt` | 1 行 | 编译目标添加 `src/rtk_handler.cpp` |

所有 RTK 相关改动均以 `RTK Extension (START/END)` 标记，便于与原版对比和合并。

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
2. **RTK 数据时间同步**：RTK 数据需要与 LiDAR/IMU 数据时间同步。系统通过 `rtk_wait_time` 参数等待 RTK 数据积累，如果等待超时仍无有效数据，则回退到 IMU 初始化。
3. **NavSatFix 无姿态**：NavSatFix 格式不包含姿态信息，系统默认 `rotation = Identity`，适用于 IMU 与 ENU 对齐的场景。
4. **LLA 参考点**：使用 NavSatFix 格式时，建议手动配置 `ref_lla`，以确保 LLA→ENU 转换的精度。如果不配置，首条有效数据将作为参考点。
5. **仅初始化使用**：RTK 数据仅在 EKF 初始化阶段使用一次，后续运行完全依赖 FAST-LIO 的激光里程计。

## 相关项目

**原始项目：**
- [FAST-LIO-ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) — ROS2 版本的 FAST-LIO

**SLAM 相关：**
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree) — 动态 KD-Tree
- [R2LIVE](https://github.com/hku-mars/r2live) — LiDAR-IMU-Visual 融合
- [LI_Init](https://github.com/hku-mars/LiDAR_IMU_Init) — LiDAR-IMU 外参初始化

**工具库：**
- [IKFOM](https://github.com/hku-mars/IKFoM) — 流形卡尔曼滤波工具箱

## 致谢

感谢 LOAM、Livox_Mapping、LINS、Loam_Livox 以及 FAST-LIO 原作者。

---

## Original README

原版 README 请查看: [README_original.md](README_original.md)

原版 FAST-LIO 说明请参考: [FAST-LIO-ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
