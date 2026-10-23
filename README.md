# Go2 on SpacemiT K3

本仓库仅包含 Go2 应用，检出到 SDK 的 `application/ros2/unitree-go2`。
`clients/android` 和 `clients/web` 分别保存 Android 客户端与控制网页。
模型通过 `scripts/download_models_go2.py` 从 SpacemiT 模型库下载到应用内的 `policy/`，该目录不纳入 Git。
离线模型包 `unitree-go2.tar.gz` 只包含 ONNX 权重，解压到同一目录。
下载、解压与安装命令见[首次构建](#首次构建)。构建时模型安装到应用共享目录，启动时可用 `model:=...` 指定其他目录。
文件清单与校验值保存在 `config/model_manifest.yaml`，权重与本仓库的策略配置、参考动作配套使用。
各模型的来源见 [模型声明](MODEL_SOURCES.md)，版权及许可汇总见 [NOTICE](NOTICE)，原始许可保存在 `LICENSES/`。
参考动作保存在 `resources/motions/`，原始模型参数保存在 `config/model_metadata/`，随应用安装。
RL、motor、IMU、SBUS 和 LED 分别使用 SDK 的 `components/model_zoo/rl`、
`components/peripherals/{motor,imu,rc_receiver,led}` 独立仓库，不在本仓库复制维护。
依赖由 `package.xml` 声明，CMake 通过 SDK 的 `Components` 和 `motor` 导出目标链接安装后的库。
图传保留 native 接入口，默认禁用。

`go2_node` 只负责 Go2 本体控制、状态和模式服务；网页、SBUS 遥控器等输入源各自运行
为独立节点。网页与 SBUS 可同时启动，双启用时分别使用 `/cmd_vel_web` 和 `/cmd_vel_rc`，
由 Go2 节点选择当前控制端，再转发到 `/cmd_vel_direct`。单独启动时保留原输入路径。SBUS
底层协议库放在 `components/peripherals/rc_receiver`，独立输入节点为
`sbus_teleop_node`。

## 板型与硬件配置

所有机器狗使用相同启动入口。`config/go2.yaml` 保存通用参数，
`config/robots/` 保存已有机器狗的部署参考。每块板实际读取
`/etc/unitree-go2/robot.yaml`，其中保存身份、通信域、电机标定、IMU 安装补偿和本机硬件差异。
板端文件不随代码同步或构建覆盖。`GO2_LOCAL_CONFIG` 可指定其他本机配置路径。

| 参考文件 | 对应机器狗 | `ROS_DOMAIN_ID` |
| --- | --- | --- |
| [go2edu.yaml](config/robots/go2edu.yaml) | Go2 Edu（`go2edu`） | 217 |
| [go2air1.yaml](config/robots/go2air1.yaml) | Go2 Air1（`go2air1`） | 218 |
| [go2air2.yaml](config/robots/go2air2.yaml) | Go2 Air2（`go2air2`） | 219 |
| [go2air3.yaml](config/robots/go2air3.yaml) | Go2 Air3（`go2air3`） | 220 |

这些参考均使用 `new_bigcore`，默认沿用板子当前连接的 Wi-Fi。
首次部署或更换标定时，在对应机器狗的板端选择匹配的文件安装。例如 Go2 Air2：

```bash
cd /root/spacemit_robot/application/ros2/unitree-go2
install -D -m 600 config/robots/go2air2.yaml /etc/unitree-go2/robot.yaml
```

其他新机器狗使用 [robot.example.yaml](config/robot.example.yaml) 填写唯一的身份、通信域和本机实测标定，
再安装到同一位置。参考文件中的电机零偏、启动转子参考和 IMU 补偿对应各自机器狗，不能直接套用到其他机器。
配置安装好后，统一使用下文的启动命令。

通过本机配置的 `robot.board_profile` 选择板型，launch 参数 `board_profile:=...` 可临时覆盖。
公共设备路径和 IMU 安装方向在 `config/board_profiles.yaml`，本机 `go2.ros__parameters` 中的字段优先。
模型推理固定在大核 Linux 上；`board_profile` 选择电机链路与板级设备。

| 板型 | 电机链路 | IMU | 设备配置 |
| --- | --- | --- | --- |
| `legacy_usb` | 四路 USB 转 RS485 | `iwt603` 或 `bmi270` | 旧板 USB 路径 |
| `new_bigcore` | Linux 板载 UART | 固定 BMI270 | `/dev/ttyS3/5/6/8`、`/dev/iio:device0`、`/dev/ttyS7` |
| `new_usbmotor` | FT4232H USB 四路串口 | 固定 BMI270 | `/dev/go2/motor-*` 部署别名、`/dev/iio:device0`、`/dev/ttyS7` |
| `new_smallcore` | 四路 G2MR v2 RPMsg 服务 | 固定 BMI270 | `go2-motor-*`、`/dev/iio:device0`、`/dev/ttyS7` |

## 直接运行

同一块板只启动一组 Go2 程序，避免重复占用电机串口、SBUS 串口和网页端口。
已启用开机自启时，手动启动前先在板端停止服务：

```bash
systemctl stop go2-sdk.service
```

停止当前程序不取消下次开机自启。

在任意机器狗的板端执行相同的启动命令：

```bash
cd /root/spacemit_robot
bash application/ros2/unitree-go2/scripts/start-go2.sh
```

默认使用板子当前连接的 Wi-Fi，不指定网络名称；未连接时按系统优先级尝试已保存的自动连接网络。
本机配置的 `robot.wifi_connection` 可选填首选连接，省略或留空即可使用默认行为。
手机或 PC 连接同一 Wi-Fi 后，打开 [统一控制网页](http://go2-sdk.local:8082/)，
在右上角选择实际在线的机器狗。Android App 也可自动发现服务或手动设置板端地址。

手动使用热点时，先停止已有的 Go2 程序，再在板端后台启动：

```bash
cd /root/spacemit_robot
nohup env GO2_NETWORK_MODE=ap bash application/ros2/unitree-go2/scripts/start-go2.sh >/tmp/go2-ap.log 2>&1 </dev/null &
```

后台启动可避免切换热点时 Wi-Fi SSH 断开导致程序退出，日志保存在 `/tmp/go2-ap.log`。
热点名称来自本机配置的 `robot.ap_ssid`，Air3 为 `Go2-go2air3`；默认密码为 `go2control`。
手机或 PC 连上热点后，打开 [热点控制网页](http://10.0.91.250:8082/)。

关闭手动热点时，PC 连着热点，另开终端执行（以 Air3 为例）：

```bash
ssh -o HostName=10.0.91.250 Ogo2air3
pgrep -f '[/]ros2 launch unitree_go2 go2.launch.py' | xargs -r kill -INT
```

程序退出会关闭热点并尝试恢复原来的 Wi-Fi，SSH 随之断开。重新使用 Wi-Fi 模式时，执行上面的普通启动命令。

### 更换 Wi-Fi

将 K3 调试串口（UART0 / Console）连接到 Linux PC，并给板子上电。
Go2 定制板按板子丝印与 USB 转串口模块同名连接：**RX 接 RX、TX 接 TX、GND 接 GND**。

1. **PC：查找串口设备。**

   ```bash
   ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
   ```

   查看设备名称与串口编号的对应关系，确认连接 K3 调试串口的设备：

   ```bash
   ls -l /dev/serial/by-id/
   ```

2. **PC：打开串口终端。** 将 `/dev/ttyUSB0` 替换为上一步确认的设备。
   未安装 minicom 时，先执行 `sudo apt install minicom`。

   ```bash
   sudo minicom -D /dev/ttyUSB0 -b 115200 -8
   ```

   按 `Ctrl+A`，再按 `O`，进入 **Serial port setup**，确认 **115200 8N1**，
   将 **Hardware Flow Control** 和 **Software Flow Control** 都设为 **No**，返回串口终端。
   按回车显示 Linux 的 `login:` 提示后，输入默认用户名 `root`；
   在 `Password:` 提示后输入默认密码 `bianbu`，密码输入不回显。已修改密码时使用修改后的密码。
   出现 `root@主机名:~#` 命令提示符后，再执行以下板端命令。

3. **板端：更换 Wi-Fi。** 在串口终端执行：

   ```bash
   nmcli radio wifi on
   PAGER='' nmcli --colors no -f SSID,SIGNAL,SECURITY device wifi list --rescan yes
   ```

   扫描结果直接输出，不进入分页界面；`SSID` 为 Wi-Fi 名称，`SIGNAL` 为信号强度，
   `SECURITY` 为加密方式。输出结束回到命令提示符后，打开网络选择界面：

   ```bash
   nmtui connect
   ```

   若已打开的 nmtui 只显示当前连接，先通过“返回”退出，重新扫描后再打开，刷新可选网络列表。
   `nmtui connect` 直接打开 **启用连接** 界面；如果执行的是 `nmtui`，
   在主菜单选择第二项 **启用连接**，再按回车。
   用 `↑↓` 选择目标 Wi-Fi，按 `Tab` 切换到右侧“激活”按钮，再按回车，按提示输入 Wi-Fi 密码并确认。
   连接成功后选择“返回”并退出。已有 Wi-Fi 配置可保留。

4. **板端串口：保存连接并查看 IP。** 查看当前连接名：

   ```bash
   PAGER='' nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
   ```

   找到 `TYPE` 为 `wifi`、状态为 `connected` 的网卡。
   将下面的“Wi-Fi 连接名”替换为 `CONNECTION` 列的实际名称，保存自动连接设置：

   ```bash
   nmcli connection modify "Wi-Fi 连接名" connection.autoconnect yes connection.autoconnect-priority 100
   ```

   `/etc/unitree-go2/robot.yaml` 中的 `robot.wifi_connection` 保持为空（`wifi_connection: ''`），
   启动时沿用当前连接；若已指定其他网络，只清空这一项，保留其他身份和标定配置。

   在串口终端查询板端 IP：

   ```bash
   ip -br addr
   ```

   找到 Wi-Fi 网卡（例如 `wlx140a02ffbfcc`）这一行，读取 IPv4 地址。
   例如显示 `192.168.1.80/24`，SSH 使用 `192.168.1.80`，不带 `/24`。

5. **PC：通过 Wi-Fi 登录板子。** PC 连接与板子相同的 Wi-Fi，在 PC 新开终端，
   将下面的 IP 替换为上一步查到的地址：

   ```bash
   ssh root@192.168.1.80
   ```

   首次连接出现主机指纹确认提示时输入 `yes`，在 `Password:` 提示后输入默认密码 `bianbu`，
   密码输入不回显；已修改密码时使用修改后的密码。此流程使用密码登录，无需配置 SSH 免密。
   Wi-Fi 配置完成后即可断开串口，后续操作在 SSH 终端进行。

6. **SSH 终端：启动 Go2。** 机器狗摆回启动蹲姿后执行：

   ```bash
   cd /root/spacemit_robot
   bash application/ros2/unitree-go2/scripts/start-go2.sh
   ```

   使用自启服务时，在 SDK 根目录改为执行 `bash application/ros2/unitree-go2/scripts/wifi.sh`，
   保存 Wi-Fi 模式并启动服务；两种启动方式选一种。

7. **手机或 PC：访问网页或 App。** 连接与板子相同的 Wi-Fi，Go2 启动后打开
   `http://go2-sdk.local:8082/`，在网页右上角选择机器狗；Android App 可直接自动发现并连接。
   也可直接打开 `http://192.168.1.80:8082/`（替换为第 4 步查到的 IP），App 的手动地址填写同一地址。
   地址需包含 `http://` 和端口 `:8082`。PC 开启系统代理时，将 `go2-sdk.local` 和板端当前 IP
   加入“忽略代理 / 直连”列表。

### 开机自启

所有板子使用同一个 `go2-sdk.service`，从 `/etc/unitree-go2/robot.yaml` 读取本机配置。
服务启动主控、SBUS 和网页，默认使用 Wi-Fi；动作仍由遥控器或网页触发。

先准备本机身份和标定，停止手动启动的程序，再安装并启用服务。
SDK 路径不同时，修改服务中的 `SROBOTIS_ROOT`、`WorkingDirectory` 和 `ExecStart`。

```bash
cd /root/spacemit_robot/application/ros2/unitree-go2
install -m 644 config/systemd/go2-sdk.service /etc/systemd/system/go2-sdk.service
systemd-analyze verify /etc/systemd/system/go2-sdk.service
systemctl daemon-reload
systemctl enable --now go2-sdk.service
```

主控或网页退出时整组退出，服务等待 5 秒后重启。手动停止服务后不会自动重启。
运动 CSV 默认写入 `/dev/null`。

在板端选择需要的管理命令执行：

```bash
systemctl status go2-sdk.service         # 查看状态
journalctl -u go2-sdk.service -f         # 查看实时日志
systemctl stop go2-sdk.service           # 停止当前程序
systemctl start go2-sdk.service          # 启动当前程序
systemctl disable --now go2-sdk.service  # 取消开机自启并停止程序
```

已安装服务时，在当前板端选择一种网络模式；脚本会保存模式并重启服务：

```bash
cd /root/spacemit_robot/application/ros2/unitree-go2/scripts
bash wifi.sh  # 切换为 Wi-Fi
# bash ap.sh  # 切换为热点
```

模式保存在 `/etc/systemd/system/go2-sdk.service.d/network.conf`，更新代码不会覆盖。
热点连接信息与上面的手动启动相同；切换网络时原来的 Wi-Fi SSH 连接可能断开。

### SBUS 接线与按键

停止时 **H 高 + D（紧急制动）** 立即蹲下并进入被动；其他位置按 **D（正常休眠）** 先完成当前模式已有的退出/恢复动作，再蹲下并进入被动。最后在启动终端按 `Ctrl+C`。

B、F、E 选档，H 低/中时按 C 进入模式；**H 高 + C** 切换白灯。网页与 SBUS 同时启用时，切到 **4 号位**立即交给网页，
无需按 C；切出 4 号位后恢复遥控器控制，关闭遥控器或持续断连也会交给网页。

**5 号位**导航接收 `/cmd_vel`，**6 号位** AI 模式接收 `/cmd_vel_ai`，均使用平地行走模型。

| &emsp;B&emsp; | &emsp;F&emsp; | &emsp;E 低&emsp; | &emsp;E 中&emsp; | &emsp;E 高&emsp; |
| --- | --- | --- | --- | --- |
| &emsp;暗&emsp; | &emsp;低&emsp; | &emsp;**1** 综合地形&emsp; | &emsp;**2** 招手&emsp; | &emsp;**3** 坐下&emsp; |
| &emsp;暗&emsp; | &emsp;中&emsp; | &emsp;**4** 网页模式&emsp; | &emsp;**5** 导航&emsp; | &emsp;**6** AI模式&emsp; |
| &emsp;暗&emsp; | &emsp;高&emsp; | &emsp;**7** 前跳&emsp; | &emsp;**8** 空&emsp; | &emsp;**9** 平地行走&emsp; |
| &emsp;亮&emsp; | &emsp;低&emsp; | &emsp;**10** 空&emsp; | &emsp;**11** 空&emsp; | &emsp;**12** 空&emsp; |
| &emsp;亮&emsp; | &emsp;中&emsp; | &emsp;**13** 空&emsp; | &emsp;**14** 空&emsp; | &emsp;**15** 空&emsp; |
| &emsp;亮&emsp; | &emsp;高&emsp; | &emsp;**16** 空&emsp; | &emsp;**17** 空&emsp; | &emsp;**18** 空&emsp; |

| 紧急制动 | 模式 | 姿态 | 自起 / 灯光 |
| --- | --- | --- | --- |
| H 高 + D：蹲下→被动 | C（H 低/中）：进入模式 | G 高 0.5 秒：站立 | H 低 0.5 秒：自起 |
| 不等待动作退出 | D（H 低/中）：正常休眠 | G 低 0.5 秒：蹲下 | H 高 + C：白灯开关 |

图传仅保留 native 接口，默认 `fpv_control_enabled:=false`，网页与遥控器 A 键均不启动图传。
SDK 图传中间层就绪后，可用 `fpv_control_enabled:=true` 启用，调用 `fpv_sky`；
程序与配置路径分别由 `fpv_executable`、`fpv_config` 指定。网页摄像头预览和图传不能同时开启。
网页预览使用 K3 H.264 硬件编码与 MediaMTX WebRTC，需要安装 `ffmpeg` 和
[MediaMTX riscv64 v1.20.1](https://github.com/Fitz8863/spacemit-mediamtx/releases/tag/v1.20.1-riscv64)，
将 `mediamtx` 放入 `/usr/local/bin/`；打开、关闭摄像头时自动启停，使用 TCP 8889 和 UDP 8189。
C、D 每次按下（通道状态变化）触发一次；H 高 + D 优先于同帧其他按键，4 号位也可紧急制动。
正常休眠只等待当前模式已有的退出或恢复流程，例如招手放爪及坐下起身恢复；
普通站立、行走等没有退出流程的模式直接蹲下并进入被动。
H 高 + D 可以中断正常休眠的退出等待；自起仅在被动/蹲下时可用。这里的蹲下是配置姿态过渡，不是 3 号坐下策略。
电机、IMU 和推理故障仍由原有保护逻辑优先处理。

SBUS 通道与遥控器实体控件的默认对应关系如下（通道号从 1 开始）：

| 通道 | 遥控器功能 | 取值/方向 |
| --- | --- | --- |
| CH1 | 右摇杆横向 | 282 左、1002 中、1722 右；旋转 |
| CH2 | 右摇杆纵向 | 282 上、1002 中、1722 下；步态参数调节 |
| CH3 | 左摇杆纵向 | 1722 上、1002 中、282 下；前进/后退 |
| CH4 | 左摇杆横向 | 282 左、1002 中、1722 右；横移 |
| CH5 (E) | 三档拨杆 | 282 左、1002 中、1722 右 |
| CH6 (F) | 三档拨杆 | 282 左、1002 中、1722 右 |
| CH7 (A) | 两态按键 | 暗 282、亮 1722；图传开关（默认禁用） |
| CH8 (B) | 两态按键 | 暗 282、亮 1722 |
| CH9 (C) | 两态按键 | 282/1722；只看跳变 |
| CH10 (D) | 两态按键 | 282/1722；只看跳变 |
| CH11 (G) | 三档滑轮 | 282 左、1002 中、1722 右 |
| CH12 (H) | 三档滑轮 | 282 左、1002 中、1722 右 |

以上是 Go2 应用层的默认映射，不是 `rc_receiver` 通用 SBUS 驱动的固定定义；实际通道可通过
`sbus_teleop_node` 的 `*_channel` 参数覆盖。

行走姿态保护由 `walking_tilt_protection:=true/false` 选择；开启后，行走模式中
`|Roll| > 40°` 或 `|Pitch| > 60°` 时先蹲下再进入被动保护。默认由本机配置的 `robot.walking_tilt_protection` 决定。

### AP 热点模式

先停止当前程序，再在板端执行：

```bash
cd /root/spacemit_robot
nohup env GO2_NETWORK_MODE=ap bash application/ros2/unitree-go2/scripts/start-go2.sh \
  > /tmp/go2-ap.log 2>&1 < /dev/null &
echo $! > /tmp/go2-ap.pid
```

热点名称来自本机配置的 `robot.ap_ssid`，未填写时为 `Go2-<robot.id>`。
默认密码 `go2control`，连接后打开 [热点网页](http://10.0.91.250:8082/)。
停止后台程序：

```bash
kill -INT "$(cat /tmp/go2-ap.pid)"
```

正常停止后恢复之前的 Wi-Fi。已安装服务时也可用 `scripts/wifi.sh`、`scripts/ap.sh` 保存模式并重启服务。

### 通信域与状态话题

主控、网页和 SBUS 使用本机配置的 `robot.ros_domain_id`。同一网络上的各机器狗应使用不同的
身份和通信域。诊断终端加载本机通信域：

```bash
export ROS_DOMAIN_ID=$(python3 -c 'import os,yaml; print(yaml.safe_load(open(os.environ.get("GO2_LOCAL_CONFIG", "/etc/unitree-go2/robot.yaml")))["robot"]["ros_domain_id"])')
```

### 打印 12 个关节侧角度

Go2 主程序运行后，另开一个板端终端执行：

```bash
cd ~/spacemit_robot
source /opt/ros/humble/setup.bash
source ~/spacemit_robot/output/staging/setup.bash
# 先按上文加载本机 ROS_DOMAIN_ID
ros2 run unitree_go2 print_joint_angles.py
```

脚本只订阅 `/joint_states`，不会发送电机命令。默认每 `0.5` 秒用中文打印一次
12 个电机的关节侧位置，单位为度；按 `Ctrl+C` 停止。可以通过 ROS 参数修改
打印间隔，例如每秒打印一次：

```bash
ros2 run unitree_go2 print_joint_angles.py --ros-args -p interval_sec:=1.0
```

### 终端切换

先在一个终端启动 Go2 主节点：

```bash
cd ~/spacemit_robot
source /opt/ros/humble/setup.bash
source ~/spacemit_robot/output/staging/setup.bash
ros2 launch unitree_go2 go2.launch.py \
  backend:=serial \
  sbus:=false \
  web:=false
```

再开第二个终端，加载环境后每次选择一条模式命令执行：

```bash
cd ~/spacemit_robot
source /opt/ros/humble/setup.bash
source ~/spacemit_robot/output/staging/setup.bash
# 先按上文加载本机 ROS_DOMAIN_ID

# 从下面选择一条执行
ros2 service call /go2/set_mode unitree_go2/srv/SetMode '{mode: stand}'
# ros2 service call /go2/set_mode unitree_go2/srv/SetMode '{mode: wave}'
# ros2 service call /go2/set_mode unitree_go2/srv/SetMode '{mode: terrain}'
# ros2 service call /go2/set_mode unitree_go2/srv/SetMode '{mode: forward_jump}'
# ros2 service call /go2/set_mode unitree_go2/srv/SetMode '{mode: sit}'
# ros2 service call /go2/set_mode unitree_go2/srv/SetMode '{mode: crouch}'
# ros2 service call /go2/set_mode unitree_go2/srv/SetMode '{mode: rollover}'
# 紧急制动（H 高 + D）：直接蹲下后进入被动
ros2 service call /go2/set_mode unitree_go2/srv/SetMode '{mode: passive_direct}'
# 正常休眠（D）：有退出流程的模式先退出，再蹲下后进入被动
ros2 service call /go2/set_mode unitree_go2/srv/SetMode '{mode: passive}'
```

策略参数在 `config/policies.yaml` 中维护，默认读取安装目录中的配置；参考动作路径相对于配置文件解析。
启动时可用 `policy_config:=/absolute/path/policies.yaml` 指定配置文件；
修改源码目录的配置后，需要重新安装包或显式指定该文件。

## 首次构建

SDK 需注册 `application/ros2/unitree-go2` 项目和 `k3-com260-unitree-go2` 目标；
目标启用本应用及 RL、motor、IMU、rc_receiver、LED 组件。

模型的发布目录为 [SpacemiT 模型库中的 unitree-go2/](https://archive.spacemit.com/spacemit-ai/model_zoo/rl/unitree-go2/)。
在 K3 的 SDK 根目录执行：

```bash
cd ~/spacemit_robot
python3 application/ros2/unitree-go2/scripts/download_models_go2.py
```

脚本按 `config/model_manifest.yaml` 下载当前模型并校验大小及 SHA-256，保存到
`application/ros2/unitree-go2/policy/`。普通部署无需训练仓库或训练环境；策略参数、参考动作和许可声明由本代码仓库提供。

离线部署时，从维护者获取与当前代码版本配套的 `unitree-go2.tar.gz`，在 K3 上保存为
`~/Downloads/unitree-go2.tar.gz`。

在 K3 的 SDK 根目录解压并校验：

```bash
cd ~/spacemit_robot
mkdir -p application/ros2/unitree-go2/policy
tar -xzf ~/Downloads/unitree-go2.tar.gz --strip-components=1 -C application/ros2/unitree-go2/policy
python3 application/ros2/unitree-go2/scripts/download_models_go2.py --check
```

`policy/` 下应直接放置 ONNX 文件，校验成功显示 `Verified 5 files`。
校验值不匹配时使用与当前代码配套的模型包，不通过改文件名替换不同策略。
校验通过后构建并安装：

```bash
cd ~/spacemit_robot
source build/envsetup.sh
lunch k3-com260-unitree-go2
m
```

模型默认从 `output/staging/share/unitree_go2/policy/` 加载。后续更新时，覆盖源码目录的
`policy/` 并校验，再进入 `application/ros2/unitree-go2` 执行 `mm -j4` 重新安装。
本机标定配置好后，按[直接运行](#直接运行)启动。
开发者也可使用下载脚本的 `--training-repo` 参数，从本地训练归档准备权重。

| 代码中的腿（丝印） | 新板大核 | 新板小核 | 现有软件设备 / 服务 |
| --- | --- | --- | --- |
| FR 右前（LB，J18） | UART8 | R.UART2 | `/dev/ttyS8` / `go2-motor-rl` |
| FL 左前（RB，J19） | UART5 | R.UART5 | `/dev/ttyS5` / `go2-motor-rr` |
| RR 右后（LF，J16） | UART6 | R.UART0 | `/dev/ttyS6` / `go2-motor-fl` |
| RL 左后（RF，J17） | UART3 | R.UART3 | `/dev/ttyS3` / `go2-motor-fr` |
| IMU，U50 | SSP1 BMI270 | 大核 IIO BMI270 | `/dev/iio:device0`，同步轮询 |
| SBUS，J22 | UART7 RX，板载反相 | 同左 | `/dev/ttyS7` |

电机设备和服务沿用板级支持包既有命名，其 `fr/fl/rr/rl` 后缀不是修正后的逻辑腿位。
当前 DTB 没有生成 `/dev/go2/*` 别名，`new_bigcore` 直接使用识别到的 UART、IIO 和
SBUS 节点。本机设备路径可在 `/etc/unitree-go2/robot.yaml` 的 `go2.ros__parameters` 中覆盖；
公共板型定义由 `config/board_profiles.yaml` 维护。
切换新板后，按实际 BMI270 安装方向核对 `imu_to_body_matrix` 和
`imu_to_body_quaternion_wxyz`，并完成急停、电机通信和 IMU 方向验证。

## Mock 联调

不连接电机时，把启动命令中的 `backend:=serial` 改为：

```bash
ros2 launch unitree_go2 go2.launch.py board_profile:=legacy_usb backend:=mock web:=true web_port:=8082
```

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 [LICENSE](LICENSE) 文件为准。
第三方模型保留原许可，见 [NOTICE](NOTICE) 和 [模型声明](MODEL_SOURCES.md)。
