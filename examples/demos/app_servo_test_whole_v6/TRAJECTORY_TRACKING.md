# Trajectory Tracking Application

这是一个Crazyflie无人机的轨迹跟踪应用程序，支持以下功能：

## 功能特性

1. **Waypoint-based Trajectory Following** - 按照预定义的航点追踪轨迹
2. **Collision Detection** - 基于名义动力学的碰撞检测
3. **End-Effector Control** - 碰撞时激活IO_3/PB4端口以打开电机装置
4. **Autonomous Landing** - 碰撞后自动后退和降落

## 系统架构

### 主要文件

- **trajectory_tracking.h** - 数据结构和API定义
- **trajectory_tracking.c** - 核心状态机实现
- **app_main.c** - 应用程序主入口
- **trajectory_client.py** - Python地面站客户端

### 状态机流程

```
IDLE 
  ↓ (接收轨迹数据)
TAKEOFF 
  ↓ (上升到第一个航点高度)
FOLLOWING 
  ↓ (按照航点追踪)
  ├→ 到达最后航点 → FINAL_FORWARD
  └→ 检测到碰撞 → COLLISION_DETECTED
     ↓
     DEPLOYING (激活IO_3/PB4)
     ↓
     BACKING_UP (后退)
     ↓
     LANDING (降落)
```

## 通信协议

### 从Python发送数据格式

```
[Command ID] [Waypoint Data]

Command ID:
  0xA1 - Waypoint command

Waypoint Data (每个航点20字节):
  [float32: x (m)]
  [float32: y (m)]
  [float32: z (m)]
  [float32: yaw (radians)]
  [uint32: duration (ms)]
```

### 示例数据包

对于一个航点 (0.5, 0.0, 1.0, 0.0, 2000):
```
0xA1 3E00003F 00000000 3F800000 00000000 D0070000
```

## 编译和刷写

### 编译应用程序

```bash
cd /home/chaoxiangye/crazyflie-bl/crazyflie-firmware
make -j4 APP=1
```

### 使用cfcompile或Makefile

```bash
# 完整编译
make clean
make all -j4 APP=1

# 只编译应用程序
make app_api
```

### 刷写固件到Crazyflie

使用Crazyflie客户端或cfloader：

```bash
cfloader flash $(build path)/cf2.bin
```

## 使用方法

### 1. 启动Crazyflie

1. 打开Crazyflie电源
2. 等待蓝牙连接
3. 无人机应准备好接收轨迹数据

### 2. 发送轨迹数据（Python）

#### 预设示例

**示例1：方形轨迹**
```bash
python3 trajectory_client.py --uri radio://0/80/2M --example 1
```

**示例2：墙壁靠近轨迹**
```bash
python3 trajectory_client.py --uri radio://0/80/2M --example 2
```

#### 自定义轨迹

**示例3：交互式输入**
```bash
python3 trajectory_client.py --uri radio://0/80/2M --example 3
```

然后按提示输入每个航点：
```
Waypoint: 0.0 0.0 0.5 0.0 1000
Waypoint: 0.2 0.0 0.5 0.0 1000
Waypoint: 0.4 0.0 0.5 0.0 1000
(按Ctrl+D完成)
```

### 3. 执行流程

1. 无人机收到轨迹数据
2. 自动起飞到第一个航点的高度
3. 按照航点顺序追踪
4. 到达最后一个航点后，直线向前飞行
5. **碰撞检测：**
   - 基于位置变化监控
   - 如果发送前进命令但位置不变，认为发生碰撞
   - 连续3帧检测到碰撞条件后激活
6. **碰撞处理：**
   - 停止所有运动（200ms）
   - 激活IO_3/PB4端口（打开电机）
   - 保持激活状态500ms
   - 向后飞行3秒
   - 停用IO_3/PB4端口
   - 下降降落

## 硬件连接

### IO_3/PB4 端口

| 引脚 | 功能 |
|------|------|
| PB4  | 数字输出（0=关闭，1=打开） |
| 3.3V | 电源 |
| GND  | 地 |

**接线示例：**
```
PB4 ──→ 继电器/MOSFET ──→ 电机驱动
3.3V ──→ 继电器/MOSFET 电源
GND  ──→ 共地
```

## 参数配置

可在 `trajectory_tracking.h` 中调整以下参数：

```c
// 碰撞检测
#define COLLISION_THRESHOLD_X 0.2f          // 碰撞距离阈值 (m)
#define COLLISION_COUNT_THRESHOLD 3         // 触发前的帧数

// 航点容差
#define WAYPOINT_REACH_DISTANCE 0.05f       // 到达航点的距离 (m)
#define WAYPOINT_YAW_TOLERANCE 0.1f         // 偏航容差 (rad)

// 速度
#define FORWARD_SPEED 0.3f                  // 前进速度 (m/s)
#define BACKWARD_SPEED -0.3f                // 后退速度 (m/s)
#define LANDING_SPEED -0.2f                 // 降落速度 (m/s)
```

## 调试和日志

### 启用调试输出

应用程序使用 `DEBUG_PRINT()` 宏输出日志。确保编译时包含了debug支持。

### 关键日志消息

```
[TRAJECTORY] Trajectory tracking initialized
[TRAJECTORY] Received N waypoints
[TRAJECTORY] Takeoff complete. Starting trajectory following.
[TRAJECTORY] Waypoint N reached
[TRAJECTORY] All waypoints reached. Starting final forward movement.
[TRAJECTORY] Collision detected!
[TRAJECTORY] Activating end-effector on IO_3 (PB4)
[TRAJECTORY] End-effector activated
[TRAJECTORY] Deactivating end-effector on IO_3 (PB4)
[TRAJECTORY] Landing...
```

## 高级功能

### 集成到现有控制回路

如果需要将轨迹跟踪集成到现有的稳定器或控制器中，可以：

1. 初始化轨迹跟踪模块：
```c
trajectoryTrackingInit();
```

2. 定期更新当前位置：
```c
trajectoryTrackingUpdateState(x, y, z, yaw);
```

3. 获取控制命令：
```c
float vx, vy, vz, yaw_rate;
trajectoryTrackingGetControl(&vx, &vy, &vz, &yaw_rate);
```

4. 发送给控制器：
```c
setpoint_t setpoint;
setpoint.mode = modeVelocityWorld;
setpoint.velocity.x = vx;
setpoint.velocity.y = vy;
setpoint.velocity.z = vz;
setpoint.attitudeRate.yaw = yaw_rate;
commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_HIGHLEVEL);
```

### 外部数据源

可以从以下来源获取位置信息：

- **Loco位置系统** - Ultra-Wideband定位
- **运动捕捉系统** - 外部估计器
- **视觉系统** - 摄像头或flow deck
- **IMU融合** - 内置姿态估计

## 故障排除

### 问题：无人机不响应轨迹

**解决方案：**
1. 检查Crazyflie是否连接
2. 验证app_channel通信
3. 检查固件是否正确编译和刷写
4. 查看debug输出确认数据接收

### 问题：碰撞检测不工作

**解决方案：**
1. 验证位置估计的准确性
2. 调整 `COLLISION_COUNT_THRESHOLD` 参数
3. 检查 `FORWARD_SPEED` 是否足够高以产生明显的位置变化

### 问题：IO_3端口没有激活

**解决方案：**
1. 检查硬件连接
2. 验证deck configuration中启用了IO_3
3. 确认 `deckIsUsingIO3()` 返回true
4. 检查IO_3是否被其他deck使用

## 性能指标

- **更新频率：** 100 Hz
- **最大航点数：** 100
- **航点精度：** ±5 cm（取决于位置估计）
- **反应时间：** ~300 ms（碰撞检测）

## 未来改进

1. **自适应速度控制** - 基于航点距离调整速度
2. **更高级的碰撞检测** - 使用范围传感器（如果可用）
3. **路径优化** - 自动生成光滑的轨迹曲线
4. **轨迹存储** - 在板载闪存中存储多条轨迹
5. **远程参数调整** - 通过app_channel动态调整参数

## 参考

- [Crazyflie固件文档](https://github.com/bitcraze/crazyflie-firmware)
- [App API示例](../examples/)
- [Commander系统](../src/modules/interface/commander.h)
- [Estimator系统](../src/modules/interface/estimator.h)
