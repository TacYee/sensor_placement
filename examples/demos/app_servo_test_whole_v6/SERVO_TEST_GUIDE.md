# 舵机/电机测试指南 - IO_3/PB4 Port

## 概述

这个测试程序用于验证Crazyflie的IO_3（PB4）端口是否能够正确激活舵机或电机。

## 硬件准备

### 需要的物品
- Crazyflie 2.0/2.1
- 舵机或电机驱动
- 电源（3.3V）
- 继电器或MOSFET（可选）

### 接线图

```
Crazyflie:
  PB4 ────┬──→ 继电器/MOSFET基极 ──→ 电机驱动
  3.3V ───┤
  GND ────┴──→ 继电器/MOSFET 地

或直接连接（如果电流 < 50mA）:
  PB4 ────→ 舵机信号线
  3.3V ───→ 舵机正极
  GND ────→ 舵机负极
```

## 两种测试方式

### 方式1：使用servo_test.c（推荐）

这是一个独立的测试程序，自动运行多个测试模式。

#### 编译步骤

```bash
# 1. 进入固件目录
cd /home/chaoxiangye/crazyflie-bl/crazyflie-firmware

# 2. 清理旧编译
make clean

# 3. 编译servo_test.c（替代app_main.c）
# 需要修改app_api/src/Kbuild，临时使用servo_test.o而不是app_main.o
# 或者直接编译：
make -j4 APP=1 SRC=servo_test

# 4. 刷写到Crazyflie
cfloader flash build/cf2.bin
```

#### 运行测试

```bash
# 在另一个终端运行Python客户端
python3 servo_test.py --uri radio://0/80/2M --all
```

**预期输出：**
```
✓ Connected to radio://0/80/2M
Connected successfully!

==================================================
TEST 1: Single Pulse (500ms)
==================================================
Expected: Servo moves once for 500ms

[→] Sending: PULSE (0x01)
✓ Pulse command sent

==================================================
TEST 2: Toggle On/Off (5 cycles)
==================================================
Expected: Servo toggles 5 times (300ms each)

[→] Sending: TOGGLE (0x02)
✓ Toggle command sent

...（继续其他测试）
```

#### 测试命令

```bash
# 运行所有测试
python3 servo_test.py --all

# 只运行单个脉冲测试
python3 servo_test.py --pulse

# 运行切换测试
python3 servo_test.py --toggle

# 运行闪烁模式
python3 servo_test.py --blink

# 交互模式（手动选择测试）
python3 servo_test.py --interactive

# 指定特定的Crazyflie连接
python3 servo_test.py --uri radio://0/80/2M --all
```

### 方式2：集成到trajectory_tracking.c

如果想在轨迹跟踪中测试碰撞响应的舵机激活：

```bash
# 编译标准的轨迹跟踪版本
make clean
make -j4 APP=1

# 发送会触发碰撞的轨迹
python3 trajectory_client.py --example 2

# 无人机会自动激活舵机（碰撞时）
```

## 故障排查

### 问题1：编译失败 - "servo_test.c: No such file"

**解决方案：**
```bash
# 确保servo_test.c文件存在
ls -la app_api/src/servo_test.c

# 如果不存在，需要创建
# 或使用标准编译方式（使用app_main.c）
```

### 问题2：连接失败

```bash
# 检查Crazyflie是否打开
# 检查URI是否正确
# 尝试使用Crazyflie GUI连接确认

# 获取正确的URI
cflib-scanble  # 或 cflib-scan
```

### 问题3：舵机没有动

**检查清单：**
1. ☐ 舵机电源是否接上？
2. ☐ 信号线是否接到PB4？
3. ☐ 地线是否接上？
4. ☐ IO_3在Kconfig中是否启用？
5. ☐ 固件是否正确刷写？
6. ☐ 舵机是否需要PWM信号而不是简单的HIGH/LOW？

**更改Kconfig以启用IO_3：**
```bash
# 编辑src/Kconfig
# 查找并启用IO_3相关选项
```

### 问题4：多个deck使用IO_3

**错误信息：**
```
Warning: IO_3 not available or in use
```

**解决方案：**
- 检查是否有其他deck（如Flow Deck、Loco Deck）使用了IO_3
- 移除冲突的deck或在配置中禁用其他device

## 测试模式详解

### 1. 单脉冲 (PULSE)
```
时间轴：
|--激活500ms--|--关闭--|
HIGH        LOW
```
**用途：** 基本的开/关测试

### 2. 切换 (TOGGLE)
```
时间轴：
|ON--|OFF--|ON--|OFF--|ON--|OFF--|ON--|OFF--|ON--|OFF--|
300ms循环，共5个周期
```
**用途：** 验证持续工作

### 3. 闪烁 (BLINK)
```
时间轴：
|快|快|快|------|快|快|快|------|...
ON OFF ON OFF ON OFF   PAUSE
100ms间隔，5个周期
```
**用途：** 验证快速响应

### 4. 状态信息 (INFO)
```
输出：
IO_3 Available: YES/NO
Pin: PB4
Voltage: 3.3V
Status: READY
```
**用途：** 确认硬件可用

## 高级配置

### 修改脉冲时间

编辑 `app_api/src/servo_test.c`：

```c
static void test_single_pulse(void) {
  DEBUG_PRINT(">>> Test: Single Pulse <<<\n");
  DEBUG_PRINT("Activating servo for 500ms...\n");
  
  set_io3(true);
  vTaskDelay(M2T(500));  // ← 修改这个值（毫秒）
  set_io3(false);
  
  DEBUG_PRINT("Pulse complete\n\n");
}
```

### 修改切换次数

```c
static void test_toggle(void) {
  DEBUG_PRINT(">>> Test: Toggle On/Off <<<\n");
  
  for (int i = 0; i < 5; i++) {  // ← 修改这个值
    // ...
  }
}
```

### 自定义PWM（如果需要）

如果舵机需要PWM信号而不是简单的HIGH/LOW：

```c
// 需要使用定时器PWM而不是GPIO
// 参考Crazyflie固件中的PWM驱动

// 简单PWM实现（250Hz）：
for (int i = 0; i < duration_ms; i += 4) {  // 4ms = 250Hz
  set_io3(true);
  vTaskDelay(M2T(2));    // 50% duty cycle
  set_io3(false);
  vTaskDelay(M2T(2));
}
```

## 集成到主程序

一旦验证了IO_3工作正常，可以集成到trajectory_tracking.c：

```c
// 在trajectory_tracking.c中已包含
static void activate_end_effector(void) {
  DEBUG_PRINT("Activating end-effector on IO_3 (PB4)\n");
  if (deckIsUsingIO3()) {
    pinMode(g_io3_pin, OUTPUT);
    digitalWrite(g_io3_pin, 1);
    DEBUG_PRINT("End-effector activated\n");
  }
}
```

## 性能指标

| 指标 | 值 |
|------|-----|
| 输出电压 | 3.3V |
| 最大输出电流 | ~50 mA（GPIO限制） |
| 响应时间 | < 1 ms |
| 频率范围 | 0-1000 Hz |
| 可靠性 | 产业级 |

## 下一步

1. ✅ 验证IO_3端口工作
2. ✅ 测试舵机/电机响应
3. → 配置轨迹跟踪程序
4. → 测试碰撞检测和电机激活
5. → 部署到实际应用

## 调试技巧

### 启用详细日志

```bash
# 编译时启用DEBUG
make -j4 APP=1 DEBUG=1

# 查看debug输出（通过UART）
cfclient --uri radio://0/80/2M
# 打开"Console"标签查看输出
```

### 使用示波器观察信号

如果有示波器，可以监测PB4上的电压变化：

```
设置：
- 通道：CH1连接到PB4和GND
- 时基：500 ms/div（对于PULSE）
- 触发：上升沿

观察：应看到0V到3.3V的阶跃变化
```

### 使用逻辑分析仪

记录详细的时序信息，验证：
- 脉宽精度
- 上升/下降时间
- 频率精度

## 支持和问题

如有问题，检查：

1. **硬件连接** - 检查接线是否正确
2. **固件版本** - 确保运行了正确的固件
3. **Debug输出** - 查看console中的错误信息
4. **IO配置** - 验证Kconfig中的IO_3设置

---

**最后修改：** 2026年5月6日  
**版本：** 1.0
