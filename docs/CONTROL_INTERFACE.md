# TCP / ROS 2 控制接口

## 模型和通道

默认配置 `tools/dephy_exo.json` 选择 `myolegs22 + DephyExoBoot_L1`，通道顺序 `[right,left]`。服务启动后按执行器、关节名称查询 ID 和 qpos/qvel 地址，并检查传动、增益、范围、关闭状态及 hinge 类型。

六个状态关节为 `hip_flexion_r/l`、`knee_angle_r/l`、`ankle_angle_r/l`。角度 rad，角速度 rad/s。配置化接口当前支持两个独立、直接关节传动、固定增益且无 bias/activation dynamics 的设备执行器；复杂伺服、滤波、肌腱传动或新人体模型需要额外适配。

## 不能混淆的三种量

| 协议 Gym 外骨骼动作 | 实际 MuJoCo ctrl | 当前设备输出力矩 |
|---:|---:|---:|
| +1 | 0 | 0 N·m，关闭 |
| +0.9 | −0.05 | −5 N·m |
| 0 | −0.5 | −50 N·m |
| −1 | −1 | −100 N·m |

仅对当前经验证的 gain=100、gear=1、无偏置直接传动成立：`tau_joint = gain × gear × ctrl`。Gym 动作不是 N·m，`actuator_force` 与设备关节力矩也要结合传动验证；不能把总 `qfrc_actuator` 当成设备单独力矩。负号表示模型关节坐标负方向，未据此认定跖屈/背屈。

网络只能设置两个设备通道，服务构造完整动作向量。默认 22 个肌肉 Gym 动作均为 0，在此环境映射为 excitation=0.5，**不是零肌肉刺激或稳定行走策略**。设备关闭则使用 +1；看门狗不把动作 0 当关闭。

## TCP 请求

只监听 `127.0.0.1`，逐行 UTF-8 JSON，每条请求/响应一行；协议版本 1，请求 seq 在连接内严格递增。

```json
{"version":1,"seq":1,"cmd":"reset"}
{"version":1,"seq":2,"cmd":"step","epoch":1,"action":[0.9,1.0],"steps":1}
{"version":1,"seq":3,"cmd":"ping"}
{"version":1,"seq":4,"cmd":"stop"}
```

`epoch=1` 只是示例，必须使用最新 reset 响应中的真实 epoch；steps 默认允许 1..10。ping 只读，不积分，也不续控制看门狗。客户端断线之后需要重新 reset。

响应重点字段：

| 字段 | 含义 |
|---|---|
| `version, seq, ok, error` | 协议、请求匹配、成功或错误码 |
| `steps_completed` | 实际执行的控制步数 |
| `state.status, epoch, sim_time_s` | 服务状态、当前控制代次、物理时间 |
| `requested_action, clipped_action` | 请求与限幅后的设备动作 |
| `env_step_action` | 实际传给 env.step 的完整动作；安全关闭后仍是历史 step 记录 |
| `effective_exo_action, control_source` | 当前生效动作与安全关闭等来源 |
| `actual_ctrl, actuator_force` | 真实 MuJoCo 数据，不是命令回显 |
| `device_joint_torque_Nm` | 已验证直接传动的设备关节输出 |
| `joints` | 按名称的 `angle_rad`、`velocity_rad_s` |
| `terminated, truncated` | 环境终止或回合截断 |
| `instance_id, viewer` | 同一仿真实例身份、窗口显示快照与共享状态 |

输入格式、必需字段、维度、有限数、版本、序号和 epoch 均校验。超时、断线、终止、异常、停止均触发关闭，不继续推进终止环境。看门狗使用单调时钟，在控制步间和 socket 轮询时检查，不是硬实时保障。

## ROS 2 接口

- `/myoassist/command`：`std_msgs/msg/String`，data 为上述 JSON 请求。
- `/myoassist/state`：`std_msgs/msg/String`，data 为实际 TCP 响应，额外含 `source`、`command_seq`。
- 桥接内部 TCP seq 包含定时 ping，与 ROS 用户 `command_seq` 不相同。
- 周期查询默认 0.1 s，仅发布当前状态，不自行推进仿真。
- 桥接不自动重放失败控制，不自动 reset；可重连读取状态，重新控制仍需显式 reset。

手动发布 reset 示例（先启动服务和桥接）：

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=42
ros2 topic pub --once /myoassist/command std_msgs/msg/String \
  '{"data":"{\"version\":1,\"seq\":1,\"cmd\":\"reset\"}"}'
```

后续 step 需要在看门狗时限内使用真实 epoch、增加 seq。初次演示推荐 `bash scripts/run_ros_demo.sh`，它自动等待回复、处理序号/epoch，并核验实际输出，避免手工发指令的时序问题。
