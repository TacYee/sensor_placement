# 舵机测试 - 5分钟快速开始

## ⚡ 最快的方式

### 步骤1：编译servo_test（2分钟）

```bash
cd /home/chaoxiangye/crazyflie-bl/crazyflie-firmware

# 一条命令编译servo_test版本
sed -i 's/app_main.o/servo_test.o/' app_api/src/Kbuild && \
make clean && make -j4 APP=1 && \
sed -i 's/servo_test.o/app_main.o/' app_api/src/Kbuild && \
echo "✓ 编译完成！二进制文件：build/cf2.bin"
```

### 步骤2：刷写到Crazyflie（1分钟）

```bash
# 方式A：使用cfloader
cfloader flash build/cf2.bin

# 方式B：使用Crazyflie GUI
# 1. 打开GUI → Firmware
# 2. 选择 build/cf2.bin
# 3. 点击 Flash
```

### 步骤3：运行测试（2分钟）

```bash
# 在另一个终端运行
python3 servo_test.py --uri radio://0/80/2M --all
```

## 📊 预期结果

```
✓ Connected to radio://0/80/2M

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

==================================================
TEST 3: Blink Pattern (3x blink, 5 cycles)
==================================================
Expected: Servo blinks 3 times, pauses, repeats 5 times

[→] Sending: BLINK (0x03)
✓ Blink command sent

==================================================
TEST 4: Status Information
==================================================
[→] Sending: INFO (0x04)
✓ Info command sent

█████████████████████████████████████████████████
█  All Tests Complete!
█████████████████████████████████████████████████

Result Summary:
  If the servo/motor responded to the above patterns,
  then the IO_3 (PB4) port is working correctly!
```

## 🔌 硬件连接

```
Crazyflie PB4 (GPIO) ──→ [继电器/MOSFET] ──→ 电机驱动

或直接连接（电流 < 50mA）：
Crazyflie:
  ├─ PB4 ──→ 舵机信号线
  ├─ 3.3V ──→ 舵机正极
  └─ GND  ──→ 舵机负极
```

## 🎮 交互模式

如果想手动控制：

```bash
# 启动交互模式
python3 servo_test.py --interactive

# 然后按提示输入：
# 1 = 单个脉冲
# 2 = 开关切换
# 3 = 闪烁模式
# 4 = 状态信息
# q = 退出
```

## 🔧 如果舵机没有反应

**检查清单：**

- [ ] Crazyflie已打开？
- [ ] USB/蓝牙连接正常？
- [ ] 舵机电源接上？
- [ ] 信号线接到PB4？
- [ ] 地线接好？

**快速诊断：**

```bash
# 检查编译结果
ls -la build/cf2.bin

# 查看debug输出（需要在固件中连接UART）
cfclient --uri radio://0/80/2M
# 打开Console标签，应该看到：
# "IO_3 (PB4) initialized successfully"
```

## ✅ 成功标志

**您应该看到/听到：**

- ✓ 舵机或电机在TEST 1时运动一次（500ms）
- ✓ 在TEST 2时上下运动（切换5次）
- ✓ 在TEST 3时快速闪烁（3x快速运动，然后暂停）
- ✓ Python客户端显示所有命令都成功发送

## 🚀 下一步

### 如果测试成功：

```bash
# 1. 回到标准轨迹跟踪版本
cd /home/chaoxiangye/crazyflie-bl/crazyflie-firmware
make clean && make -j4 APP=1

# 2. 刷写
cfloader flash build/cf2.bin

# 3. 测试碰撞响应
python3 trajectory_client.py --example 2
# 无人机会：
#   - 向前飞
#   - 遇到墙（或预设的碰撞点）
#   - 激活舵机（通过IO_3）
#   - 后退
#   - 降落
```

### 如果测试失败：

1. **检查Kbuild**
   ```bash
   cat app_api/src/Kbuild
   # 应该看到：obj-y += app_main.o
   ```

2. **重新编译**
   ```bash
   make clean
   make -j4 APP=1
   cfloader flash build/cf2.bin
   ```

3. **检查连接**
   ```bash
   # 验证舵机电源和信号线
   # 使用示波器观察PB4的3.3V高电平
   ```

## 📝 文件说明

- **servo_test.c** - 测试程序（包含4个测试模式）
- **servo_test.py** - Python客户端（发送命令）
- **SERVO_TEST_GUIDE.md** - 详细文档
- **BUILD_REFERENCE.md** - 编译参考

## 💡 Tips

- **快速切换版本**：使用 `sed` 修改Kbuild
- **保存两个版本**：编译后用 `cp` 保存二进制文件
- **调试信息**：在Crazyflie GUI的Console标签查看
- **完整功能**：测试成功后使用trajectory_tracking版本

---

**耗时：** 约5分钟  
**难度：** ⭐ 简单  
**成功率：** 如果硬件接线正确，99%成功

**最后修改：** 2026年5月6日
