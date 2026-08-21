# 快速开始指南 - 轨迹跟踪系统

## 概述

这个系统允许Crazyflie无人机：
1. 从Python地面站接收航点轨迹
2. 自动追踪航点
3. 到达最后航点后直线向前飞行
4. 检测碰撞（通过位置监测）
5. 碰撞时激活IO_3/PB4端口打开电机装置
6. 后退并自动降落

## 5分钟快速开始

### 第一步：编译固件

```bash
cd /home/chaoxiangye/crazyflie-bl/crazyflie-firmware

# 清理旧编译
make clean

# 编译（这会花费几分钟）
make -j4 APP=1

# 编译成功后会看到：
# ...
# Generating binary....
# cf2.bin - Crazyflie 2.0 firmware
```

### 第二步：刷写到无人机

使用Crazyflie客户端或cfloader：

```bash
# 使用cfloader
cfloader flash build/cf2.bin

# 或使用Crazyflie GUI：
# 1. 打开Crazyflie GUI
# 2. 连接到无人机
# 3. 进入"Firmware"标签
# 4. 选择build/cf2.bin
# 5. 点击"Flash"
```

### 第三步：运行地面站

```bash
# 发送示例轨迹（方形路径）
python3 trajectory_client.py --example 1

# 或发送墙壁靠近轨迹（会触发碰撞）
python3 trajectory_client.py --example 2

# 或交互式输入自定义轨迹
python3 trajectory_client.py --example 3
```

## 关键文件说明

| 文件 | 功能 |
|------|------|
| `app_api/src/trajectory_tracking.h` | 数据结构和API定义 |
| `app_api/src/trajectory_tracking.c` | 核心算法实现（状态机、碰撞检测） |
| `app_api/src/app_main.c` | 主应用程序（接收数据、控制回路） |
| `trajectory_client.py` | Python地面站客户端 |
| `TRAJECTORY_TRACKING.md` | 详细文档 |

## 工作流程解释

### 1. 数据接收
- Python脚本通过USB/蓝牙发送航点数据
- 格式：`[0xA1]` + 航点数据
- 每个航点：x, y, z (float), yaw (float), duration (int32)

### 2. 轨迹执行
```
启动 → 起飞 → 追踪航点 → 最后航点后直飞 → 碰撞 → 激活电机 → 后退 → 降落
```

### 3. 碰撞检测
- **方法：** 监测无人机位置变化
- **原理：** 发送前进命令但位置不变 = 碰撞
- **阈值：** 连续3帧检测到后激活

### 4. 电机激活
- 激活IO_3/PB4端口（输出3.3V）
- 使用继电器/MOSFET驱动电机
- 后退3秒后关闭

## 代码修改要点

### 如果修改碰撞检测阈值

编辑 `app_api/src/trajectory_tracking.h`：

```c
// 原值
#define COLLISION_COUNT_THRESHOLD 3

// 改为更敏感（更快检测）
#define COLLISION_COUNT_THRESHOLD 1

// 改为更迟钝（更少误触发）
#define COLLISION_COUNT_THRESHOLD 5
```

### 如果修改飞行速度

编辑 `app_api/src/trajectory_tracking.h`：

```c
#define FORWARD_SPEED 0.3f      // 改为0.5f更快，0.1f更慢
#define BACKWARD_SPEED -0.3f    
#define LANDING_SPEED -0.2f
```

### 如果修改航点到达距离

编辑 `app_api/src/trajectory_tracking.h`：

```c
#define WAYPOINT_REACH_DISTANCE 0.05f  // 改为0.1f容差更大，0.02f更严格
```

## 常见问题排查

### Q: 编译出错 "estimator.h not found"

**A:** 检查header文件位置，可能需要添加include路径：
```bash
# 编辑app_api/src/app_main.c
# 找到include部分，添加完整路径或确保源文件在正确位置
```

### Q: 无人机收不到轨迹数据

**A:** 
1. 确认Python脚本中的URI正确 (`radio://0/80/2M`)
2. 检查Crazyflie是否正确连接
3. 查看app_main中的DEBUG_PRINT输出

### Q: IO_3端口没有输出

**A:**
1. 检查硬件连接（PB4引脚接线正确）
2. 验证 `deckIsUsingIO3()` 返回true
3. 检查是否有其他deck使用IO_3

### Q: 碰撞检测不工作

**A:**
1. 确认位置估计准确（检查debug输出中的位置值）
2. 增加 `COLLISION_COUNT_THRESHOLD` 值
3. 确认飞行速度足够高（`FORWARD_SPEED >= 0.2f`）

## 测试清单

在部署到实际环境前，检查以下项：

- [ ] 编译成功无error
- [ ] 固件刷写成功
- [ ] Crazyflie可以正常起飞和着陆
- [ ] app_channel通信正常（可以发送接收数据）
- [ ] 位置估计准确（使用Loco或其他定位系统）
- [ ] IO_3硬件连接正确并测试
- [ ] 在安全区域首次测试碰撞功能
- [ ] 验证电机激活时序正确

## 性能数据

| 指标 | 值 |
|------|-----|
| 控制循环频率 | 100 Hz |
| 轨迹数据处理延迟 | <10 ms |
| 碰撞检测延迟 | ~300 ms（3帧@100Hz） |
| 位置精度需求 | ±5 cm |
| 最大航点数 | 100 |
| 默认前进速度 | 0.3 m/s |

## 安全建议

1. **始终在室内（围栏内）测试** - 防止无人机走丢
2. **在碰撞测试前准备好停止无人机** - 以防万一
3. **确保电机装置安全** - 不要让旋转部件接近人员
4. **定期检查电池** - 碰撞和后退会消耗额外能量
5. **在开阔区域测试** - 确保足够的飞行空间

## 下一步

1. **阅读详细文档** - 参考 `TRAJECTORY_TRACKING.md`
2. **自定义轨迹** - 使用 `trajectory_client.py --example 3` 创建自己的路径
3. **集成到生产系统** - 修改protocol或添加额外功能
4. **优化性能** - 调整参数以适应特定应用

## 支持

如有问题，检查：
- Debug日志输出（uart或日志系统）
- Python脚本的控制台输出
- 硬件连接和配置
- 编译时的warning和error

---

**最后修改：** 2026年5月6日
**版本：** 1.0
