# control_demo — Zsibot 机器狗手机网页控制器

通过 ROS2 Humble + Zsibot Genisom L1 SDK 控制机器狗。手机连接机器狗热点后，在浏览器输入运行控制服务的 PC 的 IP，即可使用双摇杆网页控制器。

> **安全第一。** 本项目的"急停"是软件急停（`CMD_EMERGENCY_STOP`，机器狗会趴下/软停），**不是硬件急停**。实体遥控器、机器狗电源开关始终是最终安全手段。首次测试请在空旷场地、机器狗站稳、有人看护的情况下进行，并从低速小幅动作开始。

## 架构

```
手机浏览器 ──HTTP/JSON──> control_demo_web (Python, http.server + TCP 客户端)
                               │ 本机 TCP 127.0.0.1:8765（换行分隔文本协议）
                               ▼
                     control_demo_driver (C++/rclcpp + Zsibot SDK)
                               │ UDP 192.168.234.1:8081
                               ▼
                            机器狗
```

- `control_demo_driver`：唯一接触 SDK 的进程。负责 30 Hz `SetRemote`、控制权获取、命令白名单、急停锁存与超时归零。
- `control_demo_web`：提供手机网页与 HTTP API，通过**本机 TCP 控制桥**把操作发给驱动、接收状态。
- 驱动仍是正常 ROS2 节点，并额外发布 `Joy/UInt8/Bool/String` 话题供调试或其他 ROS2 节点使用；Web 节点不依赖这些话题。
- **为什么用 TCP 桥**：在 WSL2 + Fast DDS 环境下，同一 `ros2 launch` 内的两个本地节点偶发无法发现彼此的端点（状态收不到、指令发不出）。改成仅监听 `127.0.0.1` 的 TCP 桥后，控制链路稳定，且不影响驱动的 ROS2/SDK 功能。

## 目录结构

```
control_demo/
├── README.md
├── .gitignore
├── src/
│   ├── control_demo_driver/        # ament_cmake, C++20
│   │   ├── include/control_demo_driver/
│   │   ├── src/                    # control_node / executor_holder / status_builder
│   │   └── config/control_demo.params.yaml
│   └── control_demo_web/           # ament_python
│       ├── control_demo_web/       # web_node / http_server / ros_bridge / urls
│       ├── launch/control_demo.launch.py
│       └── web/                    # index.html / style.css / app.js / joystick.js
└── tools/
    ├── fake_robot.py               # 仅测试用 UDP 模拟机器狗
    ├── test_http.sh                # HTTP API 冒烟测试
    └── check_axis.py               # 从假机器狗日志断言 SDK 轴映射
```

## 功能

- 双虚拟摇杆：
  - 左摇杆：前进、后退、原地左转、原地右转。
  - 右摇杆：左平移、右平移（仅水平方向，上下拖动不响应）。
- 姿态按钮：站立、趴下、平衡站立、移动模式、锁定。
- 速度档位：慢 / 正常 / 快。
- 大号软件急停，以及急停后的"恢复控制"流程。
- 状态 HUD：连接、控制权、电量/电压、模式、故障数量。
- 移动端适配：Pointer Events 多点触控、安全区、横竖屏、防缩放/滚动。
- 安全兜底：
  - SDK 没有自动重发/自动归零，驱动每 33 ms 发送一次 `SetRemote`（包括零）。
  - 摇杆松开、触摸取消、页面隐藏、切后台、横竖屏切换、请求失败都会立即归零。
  - 驱动在 0.35 s 没有收到新摇杆消息时归零。
  - 驱动退出时发送 5 次零 `SetRemote`。
  - 默认 `max_speed_scale=0.5`，限制最大速度。
  - 急停会锁存并重发；恢复需重新取得控制权后发送站立命令。

## 依赖

- ROS2 Humble（`/opt/ros/humble`）
- colcon、CMake ≥ 3.22、GCC ≥ 11（C++20）
- Python 3.10 + `rclpy`、`ament_index_python`（Web 节点只用 rclpy 读取启动参数和处理退出信号）
- Zsibot SDK：`../zsibot-sdk/genisom_L1_sdk`（相对 `control_demo/`）；包含 `lib/x86_64|aarch64/libzsibot.a`
- 不需要 rosbridge、Flask、cpp-httplib、nlohmann/json 或 pip 包。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/snail/project/Development-course/control_demo
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

如 SDK 路径或架构不同：

```bash
colcon build --packages-select control_demo_driver --cmake-args \
  -DZSIBOT_SDK_DIR=/绝对路径/genisom_L1_sdk -DZSIBOT_ARCH=x86_64
```

切换 `ZSIBOT_SDK_DIR`/`ZSIBOT_ARCH` 后建议删除 `build/ install/ log/` 再构建。

## 运行（真机热点场景）

1. 给机器狗上电，开启其 WiFi 热点。
2. 让运行本项目的 **PC 连接机器狗热点**；手机也连接同一个热点。
3. 在 PC 上启动：
   ```bash
   source /opt/ros/humble/setup.bash
   source /home/snail/project/Development-course/control_demo/install/setup.bash
   ros2 launch control_demo_web control_demo.launch.py robot_ip:=192.168.234.1 role:=sdk
   ```
4. 终端会打印访问地址，例如：
   ```
   [control_demo_web]: Web 服务已启动: http://0.0.0.0:8000/
   [control_demo_web]: 推荐手机访问: http://192.168.234.44:8000/ （与机器狗 192.168.234.1 同网段）
   ```
5. 手机浏览器输入该 URL（必须与 PC 在同一热点网络；网页不需要互联网）。
6. 等待 HUD 显示 `已连接` 和 `SDK · STAY`。若显示"正在获取控制权"，检查实体遥控器是否占用控制权，或尝试 `role:=remote`。
7. 先按"站立"，确认机器狗站稳，再用左/右摇杆做小幅动作。

> 网页无登录鉴权。同一热点内的任何人都能打开并控制，请只在可信热点下使用。

### 手机访问 IP 不在同一网段时

Web 服务绑定 `0.0.0.0:8000` 并打印全部本机 IPv4。选择与机器狗热点同网段的那个地址（通常是 `192.168.234.x`）。若手机无法访问 PC：

- 确认手机和 PC 连的是同一个热点，而不是手机流量。
- 关闭热点的"客户端隔离/AP isolation"。
- Windows 防火墙允许 TCP 8000。
- WSL2 建议使用 mirrored 网络模式（见下文）。

## 控制说明

| 操作 | 效果 |
|---|---|
| 左摇杆上/下 | 前进/后退 |
| 左摇杆左/右 | 原地左转/右转 |
| 右摇杆左/右 | 左/右平移 |
| 站立 | `CMD_STAND_UP (122)` |
| 趴下 | `CMD_SIT_DOWN (106)` |
| 平衡 | `CMD_BALANCE_STAND_MODE (154)` |
| 移动 | `CMD_MOVE_MODE (138)` |
| 锁定 | `CMD_LOCK_MODE (155)` |
| 慢/正常/快 | `CMD_SLOW_SPEED (174)` / `CMD_NORMAL_SPEED (175)` / `CMD_FAST_SPEED (176)` |
| 急停 | `CMD_EMERGENCY_STOP (90)`，本地锁存 |
| 恢复控制 | 重新取得控制权后发送站立命令；5 s 未恢复则保持锁定并报错 |

右摇杆被限制为水平轴：纵向拖动会被忽略，旋钮只沿水平引导线移动；`head` 轴固定为 0。

松开摇杆会立即发送零；手机断流 0.35 s 后驱动也会归零。摇杆输出默认乘以 `max_speed_scale=0.5`。

## Web ↔ 驱动控制桥（TCP 127.0.0.1:8765）

换行分隔文本协议；只绑定回环地址，手机无法直接访问。

| 方向 | 报文 | 含义 |
|---|---|---|
| web → driver | `J:forward,yaw,strafe,head,seq` | 摇杆，四轴各 `[-1,1]`；Web 控制器中 `head` 固定为 0 |
| web → driver | `C:<code>` | 一次性命令码 |
| web → driver | `EC` | 请求解除急停 |
| driver → web | `S:<status json>` | 2 Hz 状态快照 |

驱动同时保留以下 ROS2 话题，供调试/扩展使用（Web 节点不使用）：

| 话题 | 类型 | QoS |
|---|---|---|
| `/control_demo/joy` | `sensor_msgs/msg/Joy` | best effort, depth 1 |
| `/control_demo/command` | `std_msgs/msg/UInt8` | reliable, depth 10 |
| `/control_demo/estop_clear` | `std_msgs/msg/Bool` | reliable, depth 10 |
| `/control_demo/status` | `std_msgs/msg/String` | reliable, depth 1, transient_local |

`Joy.axes` 对外的命名顺序保持 `[forward, yaw, strafe, head]`，范围 `[-1, 1]`；驱动在 SDK 边界重排为 `[forward, strafe, yaw, head]`。Web 控制器只写前三个轴，`head` 始终为 0。状态 JSON 示例：

```json
{
  "executor_ready": true, "connected": true, "in_control": true,
  "role": "sdk", "model": "XG",
  "battery": {"power": 86, "volt": 24.1, "current": 1.23, "temp": 30.5, "error": 0},
  "speed_level": 2, "speed_level_name": "normal",
  "function_mode": 3, "function_mode_name": "sdk",
  "control_mode": 6, "control_mode_name": "stay",
  "motion_mode": 2, "motion_mode_name": "rest",
  "estop_latched": false, "estop_clear_pending": false,
  "last_joy_age_ms": 42, "control_tx_hz": 30.3, "joy_rx_hz": 20.0,
  "fault_count": 0, "faults": [], "last_error": ""
}
```

## 参数

驱动节点：

| 参数 | 默认 | 说明 |
|---|---|---|
| `robot_ip` | `192.168.234.1` | 机器狗 IP |
| `send_port` | `8081` | SDK UDP 发送端口 |
| `recv_port` | `8080` | SDK UDP 接收端口（会被驱动独占） |
| `role` | `sdk` | `sdk` 或 `remote` |
| `control_rate_hz` | `30.0` | `SetRemote` 频率，5~100 |
| `joy_timeout_sec` | `0.35` | 摇杆超时归零，0.05~5 |
| `status_rate_hz` | `2.0` | 状态发布频率，0.5~20 |
| `auto_request_control` | `true` | `role=sdk` 时自动请求 179 |
| `control_request_period_sec` | `1.0` | 控制权请求重发周期 |
| `max_speed_scale` | `0.5` | 安全限速比例，0.1~1.0 |
| `command_debounce_ms` | `300` | 命令去抖 |
| `estop_recovery_cmd` | `122` | 解除急停时发送的姿态命令 |
| `bridge_port` | `8765` | 本机 TCP 控制桥端口（仅 127.0.0.1） |
| `warn_period_sec` | `5.0` | 日志限流 |

Web 节点：

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` | `8000` | HTTP 端口（TCP） |
| `bind_address` | `0.0.0.0` | 监听地址 |
| `robot_ip` | `192.168.234.1` | 仅用于计算"推荐手机访问"地址 |
| `status_timeout_sec` | `2.0` | 驱动状态过期判定 |
| `bridge_host` | `127.0.0.1` | 驱动 TCP 控制桥地址 |
| `bridge_port` | `8765` | 驱动 TCP 控制桥端口 |

启动参数示例：

```bash
ros2 launch control_demo_web control_demo.launch.py \
  robot_ip:=192.168.234.1 role:=sdk max_speed_scale:=0.35 port:=8000
```

## 网络与 WSL2

- 机器狗默认控制地址 `192.168.234.1`，SDK 发送端口 `8081`、接收端口 `8080`。
- 运行服务的 PC 必须能访问 `192.168.234.0/24`，并且手机必须能访问 PC 的 HTTP 端口。
- **WSL2 mirrored 模式**（推荐）：Windows 11 `.wslconfig` 的 `[wsl2]` 下设置 `networkingMode=mirrored`，然后 `wsl --shutdown`。
- **WSL2 NAT 模式**：Windows 访问 WSL 服务需要端口转发（仅 TCP 8000 可行；SDK 的 UDP 仍需 mirrored 或原生 Linux）：
  ```powershell
  netsh interface portproxy add v4tov4 listenport=8000 listenaddress=0.0.0.0 connectport=8000 connectaddress=<WSL_IP>
  netsh advfirewall firewall add rule name="control_demo_8000" dir=in action=allow protocol=TCP localport=8000
  ```
- 若手机无法访问 PC，优先检查热点客户端隔离、Windows 防火墙、以及是否连错 WiFi。
- Web 与驱动之间走本机 TCP（`127.0.0.1:8765`），不依赖 ROS2 DDS；驱动的 ROS2 话题仅用于调试/扩展。
- 如果 8765 被其他程序占用，可通过 `bridge_port:=...` 同时修改驱动和 Web 节点的端口。

## 模拟验证（不连真机）

`tools/fake_robot.py` 只允许 loopback，默认不会接触真实机器狗。它按 SDK 协议回复心跳，并记录所有 `remote`/`cmd` 包。

```bash
# 终端 1：先启动假机器狗（它会立刻发送 dog_state，解开 SDK 构造函数的阻塞）
python3 -I tools/fake_robot.py --duration-sec 300 --log-file /tmp/fake_robot.jsonl

# 终端 2：启动驱动 + 网页，目标指向回环
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch control_demo_web control_demo.launch.py robot_ip:=127.0.0.1 role:=sdk

# 终端 3：HTTP 冒烟测试
./tools/test_http.sh

# 查看状态
curl -s http://127.0.0.1:8000/api/status | python3 -m json.tool
```

预期结果：

- 驱动日志出现 `TCP 控制桥监听 127.0.0.1:8765` 和 `Web 控制桥已连接`；`/api/status` 的 `bridge_connected=true`。
- 假机器狗收到 `cmd 179` 后把 `function` 切为 `"sdk"`，驱动状态变为 `in_control=true`。
- 假机器狗统计到约 **30 包/秒** 的 `remote`。
- HTTP 测试全部通过（18/18），包括 400/404/413/415 错误路径。
- 发送 `/api/estop` 后所有 `remote` 为零，状态 `estop_latched=true`；驱动会立即发送 `cmd 90` 并再重发 2 次。`/api/estop/clear` 后观察到 `cmd 122` 并恢复。
- 停止 launch 时，假机器狗观察到 **5 个零 `remote` 包**（驱动关闭兜底）。

**坐标轴说明（实测）**：网页/ROS 对外仍使用 `[forward, yaw, strafe, head]`；SDK API 的实际顺序是 `[forward, strafe, yaw, head]`（用户实测 a/d 原地转向、q/e 左右平移，与 `remote_control.cpp` 注释相反）。驱动在 `axis_mapping.hpp` 的 SDK 边界把 `yaw` 放到索引 2、`strafe` 放到索引 1；SDK 打包时再把四轴取反并交换前两轴，线上为 `[-strafe, -forward, -yaw, -head]`。因此 `yaw=0.5`（`max_speed_scale=0.5`）对应线上 `[0,0,-0.25,0]`，`strafe=0.5` 对应 `[-0.25,0,0,0]`。可用 `tools/check_axis.py` 验证。

## 排错

| 现象 | 处理 |
|---|---|
| 手机打不开网页 | 检查手机/PC 是否在同一热点；PC IP 是否为打印的地址；防火墙 TCP 8000；WSL2 mirrored/portproxy；热点客户端隔离 |
| 一直显示"正在获取控制权" | `role=sdk` 时实体遥控器可能占用了控制权；释放实体遥控器，或改用 `role:=remote` |
| 页面显示"未连接" | 驱动未启动/状态过期；检查 launch 终端、`/api/health`、PC 与手机网络 |
| 摇杆动了但机器狗不动 | 未取得控制权、机器狗未站立、`max_speed_scale` 太小、摇杆超时；查看状态 JSON 的 `in_control/control_mode/last_error` |
| 网页显示"未连接"或摇杆/按钮无反应 | 检查驱动日志是否有 `TCP 控制桥监听 127.0.0.1:8765`；确认 8765 未被占用、驱动与 Web 的 `bridge_port` 一致；查看 `/api/status` 的 `bridge_connected` |
| 驱动日志 `TCP 控制桥无法监听` | 8765 被其他程序占用；停掉占用进程，或通过 `bridge_port:=...` 同时修改驱动和 Web 的端口 |
| 急停后无法恢复 | 恢复需要重新取得控制权并发送站立命令；5 s 超时会保持锁定。若仍失败，使用实体遥控器/断电 |
| `model is empty, please wait..` | SDK 正在等待机器狗 model 信息；检查网络/机器狗电源。Ctrl-C 仍可退出 |
| 创建 ZsibotExecutor 失败/bind 错误 | UDP 8080 被占用；停止 `read_state` 或其他 SDK 消费者；端口可参数化 |
| 静态页面 404 | 使用 `--symlink-install` 构建；确认 `install/control_demo_web/share/control_demo_web/web/` 存在文件 |
| 构建报 SDK 路径/架构错误 | 检查 `ZSIBOT_SDK_DIR`/`ZSIBOT_ARCH`；删除 `build/ install/ log/` 重建 |
| `ros2 run/launch` 找不到包 | 先 `source install/setup.bash` |
| 中文/°C 乱码 | 使用 UTF-8 终端/浏览器 |

## 已知限制

- 网页无鉴权，任何能访问该热点的人都可控制；仅用于可信环境。
- 软件急停依赖控制权和网络，可能被实体遥控器抢占或链路中断；实体遥控器/断电仍是最终安全手段。
- 不包含摄像头图传、地图/建图、特技动作（跳/招手/后空翻等）。
- 右摇杆仅支持水平左右平移；头部角度控制未开放，`head` 轴固定为 0。
- 同一时间只建议一个手机客户端控制；多个客户端同时发送时以最后到达为准。
- `read_state` 与 `control_demo` 不能同时运行（都独占 UDP 8080）；先停止一个再启动另一个。

## 许可证

MIT。
