# Gimbal Control Component

## 项目简介

本组件提供用户态云台控制驱动与示例程序，当前支持通过 UDP 接入 TZ0xxx 系列云台与 Skydroid C12 云台。

组件封装了模式切换、角度控制、速度控制、缩放控制、状态获取等通用接口，便于上层应用通过统一 API 控制不同云台设备。

## 功能特性

### 支持能力

- UDP 方式创建云台设备实例。
- 云台模式控制：锁定、跟随、角度模式、速度模式、校准等，具体能力由驱动和设备协议决定。
- 角度目标控制与速度目标控制。
- 缩放控制。
- 状态反馈读取与稳定状态判断。
- C12 TOP 协议控制，默认设备地址为 `192.168.144.108:5000`。
- C12 取流地址解析，可基于 RTSP 取流，例如：

```bash
gst-launch-1.0 rtspsrc location=rtsp://192.168.144.108:554/stream=1 ! rtph265depay ! h265parse ! video/x-h265,width=1280,height=720,framerate=30/1 ! spacemitdec ! glimagesink sync=false
```

### 当前限制

- C12 光学变焦速度控制暂不支持。当前 C12 缩放映射为 DZM 数码变焦步进命令。
- C12 roll 轴控制下发暂不支持。当前只解析 roll 反馈，控制下发以 yaw/pitch 为主。

## 快速开始

以下以 `test/test_gimbal_udp.c` 为例，说明最短路径运行方式。

### 环境准备

- Linux 设备具备网络连接能力，并能访问云台设备 IP。
- C12 默认控制地址：`192.168.144.108:5000`。
- TZ0xxx 默认控制端口：`4900`。
- 具备 GCC、CMake 编译环境。

C12 网络配置示例：

```bash
# bianbu-minima: 根据实际接入的网口配置 IP
ifconfig eth0 192.168.144.100 netmask 255.255.255.0 up

# bianbu-linux: 配置 IP 后，还需要添加路由
route add -net 192.168.144.0 netmask 255.255.255.0 dev eth0
```

### 构建编译

以下为脱离 SDK 的独立构建方式：

```bash
mkdir -p build
cd build
cmake ..
make -j
```

如需指定编译驱动，可通过 CMake 变量选择：

```bash
cmake -DSROBOTIS_CONTROL_GIMBAL_ENABLED_DRIVERS="drv_udp_c12" ..
cmake -DSROBOTIS_CONTROL_GIMBAL_ENABLED_DRIVERS="drv_udp_tz0xxx;drv_udp_c12" ..
```

### 运行示例

C12 默认测试：

```bash
./test_gimbal_udp --driver drv_udp_c12 --ip 192.168.144.108 --port 5000 --bind-port 5000
```

C12 简写测试，默认驱动为 `drv_udp_c12`：

```bash
./test_gimbal_udp --ip 192.168.144.108 --port 5000 --bind-port 5000
```

TZ0xxx 测试：

```bash
./test_gimbal_udp --driver drv_udp_tz0xxx --ip 192.168.44.160 --port 4900 --bind-port 4900
```

查看参数：

```bash
./test_gimbal_udp --help
```

### 关键代码示例

初始化、调用接口与释放：

```c
gimbal_udp_config_t cfg = {
    .bind_ip = "0.0.0.0",
    .bind_port = 5000,
    .device_ip = "192.168.144.108",
    .device_port = 5000,
    .resend_period_s = 0.02f,
};

struct gimbal_dev *dev = gimbal_alloc_udp("drv_udp_c12", &cfg);
if (!dev) {
    fprintf(stderr, "alloc gimbal failed\n");
    return -1;
}

gimbal_set_mode(dev, GIMBAL_MODE_LOCK);
gimbal_set_mode(dev, GIMBAL_MODE_ANGLE_ABS);

const gimbal_euler_t target = {
    .pitch = -15.0f,
    .yaw = 25.0f,
    .roll = 0.0f,
};
gimbal_set_target(dev, &target);

gimbal_set_zoom(dev, GIMBAL_ZOOM_IN, 0);
gimbal_tick(dev, 0.02f);

gimbal_free(dev);
```

## 详细使用

`test_gimbal_udp` 是统一测试程序，通过 `--driver` 选择具体驱动。默认驱动为 `drv_udp_c12`。

### C12 测试流程

C12 测试流程会依次执行：

- 使能/等待反馈。
- 回中。
- 绝对角度控制。
- 相对角度控制。
- 速度控制。
- Zoom in / Zoom stop / Zoom out / Zoom stop。
- 切换锁定模式。

### C12 缩放命令

C12 缩放使用 DZM 数码变焦命令：

- `GIMBAL_ZOOM_IN` 发送 `#TPUD2wDZM0A65`。
- `GIMBAL_ZOOM_OUT` 发送 `#TPUD2wDZM0B66`。
- `GIMBAL_ZOOM_STOP` 驱动内不发送命令、直接返回 `GIMBAL_OK`；调用者仍可按统一缩放接口正常调用 stop，因为 C12 DZM 是步进命令，不是连续变焦命令。

### C12 控制发送方式

C12 控制发送方式与 `skydroid_c12_control.c` 对齐：每条 TOP 控制命令使用临时 UDP socket 发送，不绑定本地端口，短命令按 20 字节补 0 发送。驱动内部绑定的 socket 仍用于接收姿态反馈。

### 日志说明

C12 发送日志示例：

```text
[GIMBAL-C12-UDP][TX-SKYDROID] frame_len=14 udp_len=20 cmd=#TPUD2wDZM0B66 payload=23 54 50 55 44 32 77 44 5A 4D 30 42 36 36 00 00 00 00 00 00
```

反馈日志说明：

- 出现 `[RX-STATE] pitch=... yaw=...` 表示已经收到姿态反馈。
- 一直出现 `[RX-STATE] waiting feedback...` 表示未收到新鲜反馈，不代表控制命令发送失败。

## 常见问题

- **C12 能控制但一直 waiting feedback**：确认运行参数包含 `--bind-port 5000`，并确认设备是否已经接受 GAA 姿态主动输出使能命令。
- **C12 缩放不能工作**：确认运行的是最新 `libgimbal.so`，可执行 `strings build/libgimbal.so | grep TX-SKYDROID`；同时确认设备地址为 `192.168.144.108:5000`。
- **手动链接 so 后 undefined reference**：`-L` 后面应填写库目录，不能填写 `libgimbal.so` 文件本身。示例：`gcc test_gimbal_udp.c -Iinclude -Lbuild -lgimbal -o test_gimbal_udp`。
- **运行时找不到 libgimbal.so**：设置 `LD_LIBRARY_PATH` 或使用 `-Wl,-rpath` 指定运行时库路径。
- **C12 取流地址不明确**：当前 `c12.pdf` 未说明 RTSP/RTP 取流 URL，需要通过厂家资料或实机抓包确认。

## 版本与发布

| 版本   | 日期       | 说明 |
| ------ | ---------- | ---- |
| 0.1.0  | 2026-06-09 | 初始版本，支持 TZ0xxx UDP 驱动与 C12 TOP UDP 驱动，提供统一测试程序。 |

## 贡献方式

欢迎参与贡献：提交 Issue 反馈问题，或通过 Pull Request 提交代码。

- **编码规范**：本组件 C 代码遵循 [Linux Kernel 编码风格](https://www.kernel.org/doc/html/latest/process/coding-style.html)，请按该规范编写与修改代码，并尽量补充测试与说明。
- **提交前检查**：请在提交前运行本仓库的 lint 脚本，确保通过风格检查：

  ```bash
  # 在仓库根目录执行（检查全仓库）
  bash scripts/lint/lint_cpp.sh

  # 仅检查本组件
  bash scripts/lint/lint_cpp.sh components/control/gimbal
  ```

  脚本路径：`scripts/lint/lint_cpp.sh`。若未安装 `cpplint`，可先执行：`pip install cpplint` 或 `pipx install cpplint`。
- **提交说明**：提交前请描述云台型号、设备 IP/端口、测试命令与复现步骤。

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
