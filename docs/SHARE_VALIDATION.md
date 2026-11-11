# 本次分享版的验证记录

日期：2026-11-11。测试从独立分享目录启动，读取其中的环境、TCP、ROS 和配置源码；没有用原项目目录替代缺失代码。

## 已实际通过

| 项目 | 条件 | 结果 |
|---|---|---|
| 本地导入路径检查 | Python 3.11，检查 compose / MyoAssistLegBase / exo_device 来自分享目录 | 通过 |
| TCP / 安全测试 | 真实独立服务进程，默认 Dephy 配置 | 13/13 通过 |
| ROS 2 + 实时窗口 | Humble 系统 Python 3.10 与仿真 Python 3.11 分离，独立测试域 | 通过 |
| ROS 用户演示 | 复位、关闭、右侧 −5、关闭、左侧 −5、关闭 | 真实输出与同实例窗口核对通过 |
| 三组短时动力学对照 | 3 组 × 2 次，每组 12 步 | 相同初态 / 肌肉输入 / 步数；重复全状态最大差 0；有非零关节响应 |
| MS-Human-700 分享运行器 | 全身、下肢各 10 步，既有兼容环境 | 真实 Actor、0.20 s；与既有基线前缀逐元素一致 |

精选结构化结果见 [ROS 验证摘要](../results/ros2_validation_summary.json)、[MS-Human-700 分享检查](../results/share_validation.json)。演示图见 [ROS 操作说明](ROS2_SIMULATION.md)。完整日志保留在本地验证目录，不随仓库发布个人路径及大日志。

## 新环境安装

本次另在分享目录创建独立 `.my_venv`，实际执行 `bash scripts/setup_sim.sh`，重新下载并安装固定依赖，Python 3.11.17、MuJoCo 3.4.0、dm-control 1.0.36、Myosuite 2.8.4、myo-sim 0.2.3、assist-sim 1.0.0、myoassist-terrains 1.0.0 的导入检查通过。

在新环境中再次运行 TCP 13 项测试、ROS 2 窗口及用户演示、三组动力学对照，全部通过。测试未设置外部仿真解释器，自动使用分享目录新建的 `.my_venv`。系统 ROS 2 仍使用已安装的 Humble / Python 3.10；本次没有全新安装 ROS 操作系统。

原项目 `.my_venv` 与相邻 MyoSuite `.venv` 未更新。完整包版本见 [simulation_environment.json](../results/simulation_environment.json)。

## 未在本次重新执行

- MS-Human-700 全量新环境安装及大检查点重新下载；提供固定版本、SHA256 和获取脚本，实际推理在已有兼容环境中核验。
- 其他操作系统、其他显卡或远程无图形桌面的窗口兼容性。
- MS-Human-700 的 ROS 2 控制、长时正常步态、助力有效性与硬件闭环。

源码原样复制及测试解释器选择的变更说明见 [runtime_source_provenance.json](../results/runtime_source_provenance.json)。没有训练或修改原始模型、官方权重、肌肉参数和控制映射。
