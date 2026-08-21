# 快速编译参考 - servo_test vs trajectory_tracking

## 编译servo_test（舵机测试）

### 方法1：临时修改Kbuild（推荐）

```bash
# 1. 进入固件目录
cd /home/chaoxiangye/crazyflie-bl/crazyflie-firmware

# 2. 临时修改app_api/src/Kbuild
#    将：obj-y += app_main.o
#    改为：obj-y += servo_test.o

vi app_api/src/Kbuild

# 3. 编译
make clean
make -j4 APP=1

# 4. 刷写
cfloader flash build/cf2.bin

# 5. 完成后恢复Kbuild
#    将：obj-y += servo_test.o
#    改回：obj-y += app_main.o
```

### 方法2：使用sed命令（自动化）

```bash
# 快速编译servo_test版本
cd /home/chaoxiangye/crazyflie-bl/crazyflie-firmware

# 修改
sed -i 's/app_main.o/servo_test.o/' app_api/src/Kbuild

# 编译
make clean && make -j4 APP=1

# 恢复
sed -i 's/servo_test.o/app_main.o/' app_api/src/Kbuild
```

### 方法3：一行命令

```bash
cd /home/chaoxiangye/crazyflie-bl/crazyflie-firmware && \
sed -i 's/app_main.o/servo_test.o/' app_api/src/Kbuild && \
make clean && make -j4 APP=1 && \
sed -i 's/servo_test.o/app_main.o/' app_api/src/Kbuild && \
echo "Build complete! Binary: build/cf2.bin"
```

---

## 编译trajectory_tracking（标准轨迹跟踪）

```bash
cd /home/chaoxiangye/crazyflie-bl/crazyflie-firmware

# 确保Kbuild中是app_main.o
make clean
make -j4 APP=1

# 刷写
cfloader flash build/cf2.bin
```

---

## 测试servo_test

编译成功后：

```bash
# 运行所有测试
python3 servo_test.py --uri radio://0/80/2M --all

# 或使用默认URI
python3 servo_test.py --all

# 或交互模式
python3 servo_test.py --interactive
```

**预期：舵机/电机会按照模式运动**

---

## 返回trajectory_tracking

```bash
# 确保app_api/src/Kbuild中是app_main.o
# 重新编译
make clean
make -j4 APP=1

# 刷写
cfloader flash build/cf2.bin

# 运行轨迹（如果要测试碰撞响应）
python3 trajectory_client.py --example 2
```

---

## Kbuild文件当前内容

```makefile
# 当前内容（app_api/src/Kbuild）
obj-y += app_main.o
obj-y += trajectory_tracking.o

# 编译servo_test时应改为：
# obj-y += servo_test.o
# obj-y += trajectory_tracking.o
```

---

## 快速状态检查

```bash
# 查看当前Kbuild
cat /home/chaoxiangye/crazyflie-bl/crazyflie-firmware/app_api/src/Kbuild

# 查看编译输出中使用的源文件
# 应该看到 "servo_test.o" 或 "app_main.o"
```

---

## 常见问题

**Q: 编译后发现用了错误的文件怎么办？**
```bash
# 检查Kbuild
cat app_api/src/Kbuild

# 如果内容不对，手动恢复或使用sed修复
sed -i 's/servo_test.o/app_main.o/' app_api/src/Kbuild
```

**Q: 如何同时拥有两个版本的二进制文件？**
```bash
# 编译并保存servo_test版本
sed -i 's/app_main.o/servo_test.o/' app_api/src/Kbuild
make clean && make -j4 APP=1
cp build/cf2.bin servo_test.bin

# 恢复并编译trajectory版本
sed -i 's/servo_test.o/app_main.o/' app_api/src/Kbuild
make clean && make -j4 APP=1
cp build/cf2.bin trajectory_tracking.bin

# 现在有两个二进制文件
ls -la *.bin
```

---

## 推荐流程

1. **第一步：测试硬件（servo_test）**
   ```bash
   # 编译servo_test，验证IO_3工作
   sed -i 's/app_main.o/servo_test.o/' app_api/src/Kbuild
   make clean && make -j4 APP=1
   cfloader flash build/cf2.bin
   
   # 运行测试
   python3 servo_test.py --all
   # 观察舵机/电机的反应
   ```

2. **第二步：回到轨迹跟踪**
   ```bash
   # 恢复并编译轨迹跟踪版本
   sed -i 's/servo_test.o/app_main.o/' app_api/src/Kbuild
   make clean && make -j4 APP=1
   cfloader flash build/cf2.bin
   ```

3. **第三步：测试完整功能**
   ```bash
   # 测试轨迹跟踪和碰撞响应
   python3 trajectory_client.py --example 2
   # 观察碰撞时舵机是否激活
   ```

---

## Shell脚本自动化（可选）

创建 `build_servo.sh`：

```bash
#!/bin/bash
cd /home/chaoxiangye/crazyflie-bl/crazyflie-firmware

echo "Building servo_test version..."
sed -i 's/app_main.o/servo_test.o/' app_api/src/Kbuild
make clean && make -j4 APP=1

if [ -f build/cf2.bin ]; then
    echo "✓ Build successful!"
    echo "  Binary: build/cf2.bin"
    echo "  Next: cfloader flash build/cf2.bin"
else
    echo "✗ Build failed"
    exit 1
fi

# Optional: auto-flash
if command -v cfloader &> /dev/null; then
    read -p "Flash to Crazyflie? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cfloader flash build/cf2.bin
    fi
fi

# Restore Kbuild
sed -i 's/servo_test.o/app_main.o/' app_api/src/Kbuild
```

使用：
```bash
chmod +x build_servo.sh
./build_servo.sh
```

---

**版本：** 1.0  
**最后更新：** 2026年5月6日
