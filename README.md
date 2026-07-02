# florid-usb-sdk

Ragtime 机械臂 USB 主机 SDK — 提供基于 USB CDC 的 MIT 直接控制，支持 **C++** 和 **Python**。

## 架构

```
用户代码
  │
  ▼
florid_usb (Arm 类)
  │  ├── ProtocolStack (序列化 / 反序列化 / 可靠会话)
  │  └── HySerial (USB 串口传输)
  │
  ▼
STM32H7 固件 (USB CDC)
```

- 固件内置重力补偿（CasADi 生成），`tau` 参数是在重力补偿之上的**额外**前馈力矩
- `dt_us` 和 `seq` 由 SDK 根据两次调用之间的壁钟时间自动计算
- 两次调用间隔 > 200ms 视为新轨迹（`dt_us = 1000µs`）

## C++ 快速开始

```cpp
#include <florid/usb/Arm.hpp>

using namespace florid::usb;
using namespace std::chrono_literals;

Arm::Config cfg;
cfg.device = "/dev/ttyACM0";
cfg.baud_rate = 115200;

Arm arm(cfg);
arm.connect();

// 启动 USB 会话（阻塞，等待固件确认）
arm.startSession(1s);

// 读取当前关节位置
auto status = arm.getArmStatus();

// 发送 MIT 位置指令（fire-and-forget，非阻塞）
float q[6]  = {0, 0, 0, 0, 0, 0.5};
float dq[6] = {};
float tau[6]= {};
float kp[6] = {8, 8, 8, 8, 8, 8};
float kd[6] = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7};
arm.sendMitCommand(q, dq, tau, kp, kd);

// 停止会话
arm.stopSession(1s);
```

### CMake 集成

```cmake
add_subdirectory(path/to/florid-usb-sdk)

target_link_libraries(my_app PRIVATE florid_usb)
```

`sdk/usb` 目录下 `protocols/` 子目录也会被自动构建。

### C++ API 参考

#### `Arm::Config`

| 字段 | 默认值 | 说明 |
|---|---|---|
| `device` | `/dev/ttyACM0` | 串口设备路径 |
| `baud_rate` | 115200 | 波特率 |
| `session_timeout` | 500ms | 每个可靠请求的超时 |
| `max_retries` | 3 | 最大重试次数 |

#### `Arm`

| 方法 | 阻塞/非阻塞 | 说明 |
|---|---|---|
| `connect()` | 非阻塞 | 打开串口，启动后台工作线程 |
| `disconnect()` | 阻塞 | 关闭串口，停止线程 |
| `isConnected()` | 非阻塞 | 是否已连接且持续收到遥测 |
| `startSession(timeout)` | 阻塞 | 发送 `UsbSessionStart` 并等待响应 |
| `stopSession(timeout)` | 阻塞 | 发送 `UsbSessionStop` 并等待响应 |
| `sendMitCommand(q, dq, tau, kp, kd, mode)` | 非阻塞 | MIT 直接控制（fire-and-forget） |
| `setMotorControlMode(joint_id, mode, timeout)` | 阻塞 | 切换电机控制模式 |
| `sendPosVelCommand(q, dq, enabled_mask)` | 非阻塞 | POSVEL 模式控制 |
| `sendVelocityCommand(dq, enabled_mask)` | 非阻塞 | 速度模式控制 |
| `sendHybridCommand(q, dq_limit, current_limit, enabled_mask)` | 非阻塞 | 混合力位控制 |
| `sendGripperCommand(q, dq, tau, kp, kd, mode)` | 非阻塞 | 夹爪 MIT 控制 |
| `emergencyStop()` | 非阻塞 | 急停（fire-and-forget） |
| `getArmStatus()` | 非阻塞 | 获取缓存的机械臂状态 |
| `getGripperStatus()` | 非阻塞 | 获取缓存的夹爪状态 |
| `getMotorFeedback(timeout)` | 阻塞 | 请求电机详细反馈 |
| `homeAll(timeout)` | 阻塞 | 归零所有关节 |
| `clearFaults(timeout)` | 阻塞 | 清除所有故障 |

---

## Python 快速开始

### 安装

```bash
pip install git+https://github.com/Ragtime-LAB/florid-usb-sdk.git
```

或从本地构建：

```bash
pip install -e .
```

### 使用

```python
import numpy as np
from florid_usb import Arm, Config

# 创建配置
cfg = Config()
cfg.device = "/dev/ttyACM0"
cfg.baud_rate = 115200

# 连接
arm = Arm(cfg)
arm.connect()

# 启动会话
arm.start_session(timeout=1.0)

# 发送 MIT 指令
q  = np.zeros(6, dtype=np.float32)
dq = np.zeros(6, dtype=np.float32)
tau = np.zeros(6, dtype=np.float32)
kp = np.full(6, 8.0, dtype=np.float32)
kd = np.full(6, 0.7, dtype=np.float32)
q[5] = 0.5  # J5 偏转 0.5 rad

arm.send_mit_command(q, dq, tau, kp, kd, control_mode=1)

# 读取状态
status = arm.get_arm_status()
print(status["q"])       # numpy array, 6 floats
print(status["mode"])    # int: 0=INIT, 1=IDLE, 2=RUNNING, 3=FAULT, 4=ESTOP
print(status["gripper"]) # dict: q, dq, tau, temp_c, enabled

# 切换电机模式
arm.set_motor_control_mode(joint_id=0, mode="posvel", timeout=0.5)

# 停止
arm.stop_session()
arm.disconnect()
```

### Python API 参考

`florid_usb` 模块导出 `Arm` 和 `Config` 两个类。

#### `Config`

| 属性 | 默认值 | 说明 |
|---|---|---|
| `device` | `/dev/ttyACM0` | 串口设备路径 |
| `baud_rate` | 115200 | 波特率 |
| `session_timeout_ms` | 500 | 每个可靠请求的超时（毫秒） |
| `max_retries` | 3 | 最大重试次数 |

#### `Arm`

| 方法 | 阻塞/非阻塞 | 说明 |
|---|---|---|
| `connect()` | 否 | 打开串口并启动通信 |
| `disconnect()` | 是 | 关闭串口 |
| `is_connected()` | 否 | 连接状态 |
| `start_session(timeout=0.5)` | 是 | 启动 USB 会话，timeout 单位秒 |
| `stop_session(timeout=0.5)` | 是 | 停止 USB 会话 |
| `send_mit_command(q, dq, tau, kp, kd, control_mode=1)` | 否 | MIT 控制，参数为长度为 6 的 numpy 数组 |
| `set_motor_control_mode(joint_id, mode, timeout=0.5)` | 是 | 切换电机模式。`mode` 可以是字符串 `'mit' | 'posvel' | 'vel' | 'hybrid'` 或整数 `1..4` |
| `send_posvel_command(q, dq, enabled_mask=0x3f)` | 否 | POSVEL 控制 |
| `send_velocity_command(dq, enabled_mask=0x3f)` | 否 | 速度控制 |
| `send_hybrid_command(q, dq_limit, current_limit_norm, enabled_mask=0x3f)` | 否 | 混合力位控制 |
| `send_gripper_command(q, dq, tau, kp, kd, control_mode=1)` | 否 | 夹爪控制，参数为单个 float |
| `emergency_stop()` | 否 | 急停 |
| `get_arm_status()` | 否 | 返回 `dict`，包含 `mode, seq, timestamp_us, q, dq, tau, gripper` |
| `get_gripper_status()` | 否 | 返回 `dict`，包含 `q, dq, tau, temp_c, enabled` |
| `get_motor_feedback(timeout=0.5)` | 是 | 返回 `dict {motors: [...]}`，每个电机包含 `joint_id, position_rad, speed_rad_s, torque_nm, temp_c` |
| `home_all(timeout=0.5)` | 是 | 归零 |
| `clear_faults(timeout=0.5)` | 是 | 清除故障 |

## 构建（C++）

```bash
git clone --recursive git@github.com:Ragtime-LAB/florid-usb-sdk.git
cd florid-usb-sdk
cmake -B build
cmake --build build
```

Python 绑定通过 `pip install .` 自动构建，无需手动运行 CMake。

## 依赖

| 依赖 | 来源 | 用途 |
|---|---|---|
| [HySerial](https://github.com/RoboMaster-DLMU-CONE/HySerial) | git submodule (`3rdparty/HySerial`) | USB 串口传输 |
| [florid-usb-protocols](https://github.com/Ragtime-LAB/florid-usb-protocols) | git submodule (`protocols/`) | 协议栈（ProtocolStack, ReliableSession, 包定义） |
| [rpl](https://github.com/C-One-Studio/RPL) | 内置于 `protocols/3rdparty/rpl` | RPL 序列化框架 |
| [unordered_dense](https://github.com/martinus/unordered_dense) | CMake FetchContent | 哈希表 |
| [pybind11](https://github.com/pybind/pybind11) | CMake FetchContent（仅 Python 绑定） | C++/Python 绑定 |
